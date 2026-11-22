#include "global.h"

// ============================================================================
// armorpaint-mcp â€” in-app bridge (plugin half of docs/PROTOCOL.md v1)
//
// Drop this file into ArmorPaint's plugins folder and enable it in the Plugins
// tab. It watches a spool directory for request files written by the Python MCP
// server, executes one per frame, and writes replies back with a two-file
// commit.
//
// WHY A FILE MAILBOX: minic exposes no inbound sockets (the only network
// bindings are HTTPS GET downloads), so files are the only transport a plugin
// can offer. See docs/PROTOCOL.md.
//
// ---------------------------------------------------------------------------
// THE OP VOCABULARY IS THE SERVER'S, NOT THIS FILE'S
//
// Every `op` string below is an `ap_*` tool from armorpaint_mcp/server.py with
// the `ap_` prefix stripped, and every argument name below is exactly what
// server.py's _build_wire_args() emits for that tool. That pairing IS the
// contract. A rename on either side does not fail loudly: an unknown op comes
// back as `unsupported`, and an unknown ARGUMENT is simply absent, so an op
// like material_set_channels answers `ok` while changing nothing. If you add or
// rename an op here, change server.py in the same commit, and vice versa.
//
// ---------------------------------------------------------------------------
// SPOOL LAYOUT (all paths use forward slashes; backslash escapes truncate
// minic string literals)
//
//   <data>/mcp_spool/req/<id>.json     server -> plugin   (server writes atomically)
//   <data>/mcp_spool/res/<id>.json     plugin -> server   (body)
//   <data>/mcp_spool/res/<id>.done     plugin -> server   (commit marker: byte length)
//   <data>/mcp_spool/heartbeat.json    plugin -> server   (~1 Hz liveness)
//   <data>/mcp_spool/bridge.lock       plugin -> server   (written once at start)
//
// <data> is data_path(), i.e. "./data/" relative to ArmorPaint's working
// directory. The resolved path is logged to the console at start and is
// reported by get_app_info.
//
// ---------------------------------------------------------------------------
// REQUEST SHAPE â€” READ THIS BEFORE CHANGING THE SERVER
//
// Requests are parsed with json_parse_to_map(), whose behaviour is fixed and
// hostile (iron_json.c:297). It must be fed a FLAT, COMPACT object whose values
// are all JSON strings:
//
//   {"v":"1","id":"42","op":"select_tool","a_tool":"10"}
//
//   * every value comes back as a char* (numbers and booleans included);
//   * nested objects are FLATTENED into the same map -- the "args" key itself
//     disappears, so nested arg names share the envelope namespace. The "a_"
//     prefix keeps them apart. arg() also accepts a bare name so a server that
//     literally sends PROTOCOL.md's {"args":{...}} still works;
//   * the ENVELOPE key "op" is therefore read with any_map_get DIRECTLY, never
//     through arg(). Going through arg() would let a request carrying an
//     argument named `op` (legal: server.py's _ARG_RE accepts it) send "a_op"
//     and hijack the dispatcher;
//   * ARRAYS CORRUPT THE REST OF THE PARSE. Never send one. List-shaped args
//     (stroke points) arrive as delimited strings;
//   * key detection is s[end]==':' -- no space may sit before a colon, so the
//     server must emit compact separators (json.dumps(o, separators=(",",":"))).
//
// REPLY SHAPE (parsed by Python, so it may nest freely):
//
//   {"v":1,"id":"42","ok":true,"result":{...},"elapsed_ms":12}
//   {"v":1,"id":"42","ok":false,"error":{"code":"...","message":"..."},"elapsed_ms":1}
//
// Result/error objects are built with the json_encode_* family; the envelope
// around them is assembled with string(). That split is forced: json_encode_key
// is NOT a bound function and json_encode_begin_object() takes no key, so the
// json_encode_* family physically cannot emit a nested object under a key. It
// also never escapes string values, hence jesc() on every dynamic string below.
//
// ---------------------------------------------------------------------------
// DIALECT CONSTRAINTS OBSERVED HERE (see docs/MINIC_DIALECT_AND_API.md)
//   * main() is the LAST function; anything after it is never registered.
//   * 23 functions + main, against a silent hard cap of 32. Every operation is
//     an else-if ARM inside dispatch(), never its own function.
//   * no switch, no ternary, no casts, no i++ inside an expression, no #define,
//     no fixed-size local arrays (they leak from a shared 512-slot pool).
//   * && and || do NOT short-circuit, so every null check is its own nested if.
//   * minic evaluates EVERY else-if CONDITION even after one has matched
//     (minic.c:1592) though it does skip the bodies. The conditions here are
//     pure string_equals calls, so that is ~55 strcmps and no side effects.
//     Never put a side effect in a dispatcher condition.
//   * ~29 KB of the 8 MB arena per SCRIPT function call, rewound only at the
//     on_update boundary => ~280 script calls per frame. The list and stroke
//     caps below exist to keep the worst arm well inside that.
// ============================================================================

char *BRIDGE_VERSION = "1.0.0";
int   ENVELOPE_V     = 1;

// Per-frame script-call budget. 48 stroke points cost 48*3 to_float calls; 64
// list items cost 64*2 jesc calls. Both leave wide margin under ~280.
// MAX_STROKE_POINTS must equal MAX_STROKE_POINTS in armorpaint_mcp/server.py.
int MAX_STROKE_POINTS = 48;
int MAX_LIST_ITEMS    = 64;

// viewport_save_texture_to_file() does NOT exist on a stock ArmorPaint. It is
// added by patch/apply_viewport_patch.py. Calling an unregistered function is a
// minic runtime error that aborts the whole handler (minic.c:939), so this has
// to be a flag rather than a try: set it to 1 only after applying the patch and
// rebuilding.
int HAVE_VIEWPORT_PATCH = 0;

void        *plugin;
// Newer ArmorPaint (2026-11+) UI widgets take bool* instead of ui_handle_t*,
// and ui_handle_create() is no longer exposed to plugins.
bool  panel_open    = false;
bool  enabled_ui    = true;

char *spool_root;
char *dir_req;
char *dir_res;
char *path_heartbeat;
char *path_lock;

int   enabled       = 1;
int   busy          = 0;
float poll_interval = 0.05;
float poll_accum    = 0.0;
float hb_accum      = 0.0;
int   req_count     = 0;
int   err_count     = 0;
char *last_op       = "-";
char *last_id       = "-";
char *last_error    = "-";

// capture_viewport's render target, kept between calls.
//
// gpu_create_render_target is bound, but NO destructor is: gpu_delete_texture
// exists in the engine (iron.h:854) and is simply absent from minic's binding
// table, so a plugin can never free a target it allocates. Allocating one per
// request would leak ~4 MB of VRAM per 1024x1024 capture, and a see -> adjust
// -> see loop is the whole point of the op. Reusing one target makes the steady
// state free. Changing the capture SIZE still abandons the previous target, so
// callers should keep the size stable within a session.
void *capture_tex;
int   capture_w = 0;
int   capture_h = 0;

// Pre-formatted panel lines. on_ui runs every frame the Plugins tab is open and
// string() allocates from a heap nothing ever collects, so formatting there
// would leak steadily for as long as the tab is visible. These are rebuilt only
// when the heartbeat ticks (~1 Hz and around each request).
char *ui_stats   = "requests 0 / errors 0";
char *ui_version = "-";

// dispatch() return channel: the arms return the inner JSON object and set r_ok.
int r_ok = 1;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// JSON-escape a string. json_encode_string() dumps its value verbatim between
// quotes, so an unescaped Windows path or a quote in a name would produce a
// response the server cannot parse. Also flattens control characters, which
// JSON forbids raw. Never returns NULL -- json_encode_string(NULL) would strlen
// a null pointer inside the host.
char *jesc(char *s) {
	if (s == NULL) {
		return "";
	}
	char *r = string_replace_all(s, "\\", "\\\\");
	r       = string_replace_all(r, "\"", "\\\"");
	r       = string_replace_all(r, "\n", " ");
	r       = string_replace_all(r, "\r", " ");
	r       = string_replace_all(r, "\t", " ");
	return r;
}

// Convert a path to the spelling Iron's Windows helpers actually accept.
//
// Iron is inconsistent about separators on Windows: functions that go through
// Win32 directly (iron_file_save_bytes -> _wfopen) accept '/' happily, but two
// of them do not, and BOTH FAIL SILENTLY:
//
//   iron_delete_file  -- shells  del /f "<path>"  to cmd.exe, where '/' is the
//                        switch prefix (iron_file.c:394, iron_system.c:596)
//   iron_file_exists  -- opens via IRON_FILE_TYPE_ASSET (iron_file.c:385)
//
// MEASURED on ArmorPaint 1.0 / Windows 11:
//   iron_delete_file  "E:/x/f"->fail  "E:\\x\\f"->ok   "./x/f"->fail
//   iron_file_exists  "E:/x/f"->0     "E:\\x\\f"->1    "./x/f"->1
//
// Relative paths are fine for exists() but not for delete(), and backslashes
// satisfy both, so normalising unconditionally is the one safe rule. The rest of
// the plugin keeps '/' because minic string literals treat '\' as an escape
// introducer and an unrecognised escape truncates the literal -- so the
// conversion lives here, at the boundary, and nowhere else.
char *win_path(char *p) {
	if (p == NULL) {
		return NULL;
	}
	return string_replace_all(p, "/", "\\");
}

// Delete a file. ALWAYS use this rather than iron_delete_file() -- see win_path().
//
// This is more load-bearing than it looks: an undeleted request file is re-read
// every frame forever, so the bridge answers request #1 on repeat and NEVER SEES
// request #2. Observed before the fix: 1625 replays of the first request while
// four later ones sat unread in req/ and timed out.
void del_file(char *p) {
	if (p == NULL) {
		return;
	}
	iron_delete_file(win_path(p));
}

// Does a file exist? ALWAYS use this rather than iron_file_exists() -- see win_path().
// A false negative here makes an agent believe its own export never happened.
int file_here(char *p) {
	if (p == NULL) {
		return 0;
	}
	return iron_file_exists(win_path(p));
}

// There is no atoi binding anywhere in the table, so integers arrive as text and
// are converted here. Unparseable input yields 0, never an error.
int to_int(char *s) {
	if (s == NULL) {
		return 0;
	}
	int len = string_length(s);
	int i   = 0;
	int neg = 0;
	int c   = 0;
	if (len > 0) {
		c = char_code_at(s, 0);
		if (c == 45) { // '-'
			neg = 1;
			i   = 1;
		}
		else if (c == 43) { // '+'
			i = 1;
		}
	}
	int n = 0;
	while (i < len) {
		c = char_code_at(s, i);
		if (c < 48) {
			break;
		}
		if (c > 57) {
			break;
		}
		n = n * 10 + (c - 48);
		i = i + 1;
	}
	if (neg) {
		n = -n;
	}
	return n;
}

// Same story for floats. Plain decimal only: sign, digits, optional fraction.
// Exponent notation is not accepted -- server.py's _fmt_float never emits it.
float to_float(char *s) {
	if (s == NULL) {
		return 0.0;
	}
	int   len   = string_length(s);
	int   i     = 0;
	int   c     = 0;
	float sign  = 1.0;
	float whole = 0.0;
	float frac  = 0.0;
	float scale = 1.0;
	if (len > 0) {
		c = char_code_at(s, 0);
		if (c == 45) { // '-'
			sign = -1.0;
			i    = 1;
		}
		else if (c == 43) { // '+'
			i = 1;
		}
	}
	while (i < len) {
		c = char_code_at(s, i);
		if (c < 48) {
			break;
		}
		if (c > 57) {
			break;
		}
		whole = whole * 10.0 + (c - 48);
		i     = i + 1;
	}
	if (i < len) {
		if (char_code_at(s, i) == 46) { // '.'
			i = i + 1;
			while (i < len) {
				c = char_code_at(s, i);
				if (c < 48) {
					break;
				}
				if (c > 57) {
					break;
				}
				scale = scale * 10.0;
				frac  = frac * 10.0 + (c - 48);
				i     = i + 1;
			}
		}
	}
	return sign * (whole + frac / scale);
}

// Booleans arrive as the literal text "true"/"false" (server.py encode_value).
// to_int returns 0 for BOTH of those, so they need their own conversion.
int to_bool(char *s) {
	if (s == NULL) {
		return 0;
	}
	if (string_equals(s, "true")) {
		return 1;
	}
	if (string_equals(s, "1")) {
		return 1;
	}
	return 0;
}

// Fetch an argument. Prefers the "a_" prefixed key; falls back to the bare name
// so a server sending PROTOCOL.md's nested "args" object (which json_parse_to_map
// flattens into the top level) still works. NEVER use this for envelope keys.
char *arg(void *m, char *name) {
	if (m == NULL) {
		return NULL;
	}
	if (name == NULL) {
		return NULL;
	}
	char *v = any_map_get(m, string("a_%s", name));
	if (v != NULL) {
		return v;
	}
	return any_map_get(m, name);
}

int arg_i(void *m, char *name, int dflt) {
	char *v = arg(m, name);
	if (v == NULL) {
		return dflt;
	}
	if (string_length(v) < 1) {
		return dflt;
	}
	if (string_equals(v, "true")) {
		return 1;
	}
	if (string_equals(v, "false")) {
		return 0;
	}
	return to_int(v);
}

float arg_f(void *m, char *name, float dflt) {
	char *v = arg(m, name);
	if (v == NULL) {
		return dflt;
	}
	if (string_length(v) < 1) {
		return dflt;
	}
	return to_float(v);
}

// Errors are values, never crashes. Codes are PROTOCOL.md's set:
// no_project, bad_args, not_found, unsupported, app_busy, internal.
char *fail(char *code, char *msg) {
	r_ok       = 0;
	err_count  = err_count + 1;
	last_error = string("%s: %s", code, msg);
	json_encode_begin();
	json_encode_string("code", jesc(code));
	json_encode_string("message", jesc(msg));
	return json_encode_end();
}

char *ok_empty() {
	json_encode_begin();
	return json_encode_end(); // "{}"
}

// An id becomes a filename, so a hostile one ("../../x") would write outside the
// spool. Accept only [0-9A-Za-z._-], reject dot-runs and a leading dot.
int id_ok(char *s) {
	if (s == NULL) {
		return 0;
	}
	int len = string_length(s);
	if (len < 1) {
		return 0;
	}
	if (len > 64) {
		return 0;
	}
	int c    = 0;
	int good = 0;
	int bad  = 0;
	for (int k = 0; k < len; ++k) {
		c    = char_code_at(s, k);
		good = 0;
		if (c >= 48) {
			if (c <= 57) {
				good = 1; // 0-9
			}
		}
		if (c >= 65) {
			if (c <= 90) {
				good = 1; // A-Z
			}
		}
		if (c >= 97) {
			if (c <= 122) {
				good = 1; // a-z
			}
		}
		if (c == 45) {
			good = 1; // '-'
		}
		if (c == 46) {
			good = 1; // '.'
		}
		if (c == 95) {
			good = 1; // '_'
		}
		if (!good) {
			bad = 1;
			break;
		}
	}
	if (bad) {
		return 0;
	}
	if (char_code_at(s, 0) == 46) {
		return 0;
	}
	if (string_index_of(s, "..") >= 0) {
		return 0;
	}
	return 1;
}

// Screen a filesystem path before it reaches a binding.
//
// iron_create_directory() is not a syscall: on every platform it builds a shell
// command line and hands it to system() (iron_file.c:353). A path containing a
// double quote closes the quoting, and everything after it executes. The server
// already refuses quotes (server.py _UNSAFE_STR_RE) but a request is just a
// file -- anything able to write into the spool bypasses the server entirely,
// so the check has to exist on this side too.
//
// The length cap is the same call's other hazard: it copies into a 1024-byte
// stack buffer with strcpy/strcat and no bounds check.
int path_ok(char *p) {
	if (p == NULL) {
		return 0;
	}
	int len = string_length(p);
	if (len < 1) {
		return 0;
	}
	if (len > 900) {
		return 0;
	}
	if (string_index_of(p, "\"") >= 0) {
		return 0;
	}
	if (string_index_of(p, "\n") >= 0) {
		return 0;
	}
	if (string_index_of(p, "\r") >= 0) {
		return 0;
	}
	return 1;
}

// Screen a request body before json_parse_to_map() ever sees it.
//
// WHY THIS EXISTS: jsmn_parse returns a NEGATIVE token count for malformed
// input, and load_tokens (iron_json.c:270) feeds that straight into
// malloc(sizeof(jsmntok_t) * count) -- which yields NULL for a negative size --
// and then writes through it. A truncated write, an empty file or any stray
// .json dropped into req/ would therefore take ArmorPaint down mid-paint. The
// host offers no "is this valid" call, so the check has to happen here.
//
// This is a proxy, not a parser. It catches the realistic failure modes and, as
// a bonus, rejects JSON arrays outright: json_parse_to_map skips an array's
// opening token and then consumes its elements as key/value pairs, silently
// corrupting everything after it (iron_json.c:311).
//
// Returns 1 = usable, 0 = malformed, -1 = contains an array.
int json_sane(char *s) {
	if (s == NULL) {
		return 0;
	}
	int len = string_length(s);
	if (len < 2) {
		return 0;
	}
	if (len > 8192) {
		return 0; // far larger than any op this bridge defines
	}
	if (char_code_at(s, 0) != 123) { // '{'
		return 0;
	}
	if (char_code_at(s, len - 1) != 125) { // '}'
		return 0;
	}
	int depth = 0;
	int instr = 0;
	int esc   = 0;
	int c     = 0;
	int bad   = 0;
	for (int q = 0; q < len; ++q) {
		c = char_code_at(s, q);
		if (esc) {
			esc = 0;
		}
		else if (instr) {
			if (c == 92) { // backslash
				esc = 1;
			}
			else if (c == 34) { // '"'
				instr = 0;
			}
		}
		else if (c == 34) {
			instr = 1;
		}
		else if (c == 123) {
			depth = depth + 1;
		}
		else if (c == 125) {
			depth = depth - 1;
			if (depth < 0) {
				bad = 1;
				break;
			}
		}
		else if (c == 91) { // '[' outside a string
			bad = 2;
			break;
		}
		else if (c == 93) { // ']' outside a string
			bad = 2;
			break;
		}
	}
	if (bad == 2) {
		return -1;
	}
	if (bad) {
		return 0;
	}
	if (instr) {
		return 0; // unterminated string: the write was almost certainly torn
	}
	if (depth != 0) {
		return 0;
	}
	return 1;
}

int dir_count(char *path) {
	any_array_t *fa = file_read_directory(path);
	if (fa == NULL) {
		return 0;
	}
	int n = fa->length;
	array_free(fa); // frees the backing buffer; the name strings are separate
	free(fa);
	return n;
}

// Join up to `max` entries of a buffer/length array into one pipe-delimited,
// escaped string. Names go out this way rather than as a JSON array because
// json_encode_string_array() cannot escape its elements, and one stray quote
// there would make the whole reply unparseable. A stray '|' only costs an extra
// split entry, which degrades gracefully.
//
// The parameter is any_array_t but string_array_t is passed to it too: every
// *_array_t in this codebase is {void *buffer; int length; int capacity;}
// (types.h:781, minic_api.c:331) and minic stamps the declared deref type onto
// the parameter, so the re-typing is exact rather than lucky.
char *sa_names(any_array_t *a, int max) {
	if (a == NULL) {
		return "";
	}
	int n = a->length;
	if (n > max) {
		n = max;
	}
	char *acc = "";
	char *s;
	for (int i = 0; i < n; ++i) {
		s = a->buffer[i];
		if (s != NULL) {
			if (i > 0) {
				acc = string("%s|%s", acc, jesc(s));
			}
			else {
				acc = jesc(s);
			}
		}
	}
	return acc;
}

// Emit an object's readable state into the reply currently being encoded.
// Rotation goes out as the raw quaternion: there is no quat->euler binding, and
// inventing one here would be a lossy guess the caller could not check.
void emit_object(object_t *o) {
	if (o == NULL) {
		return;
	}
	json_encode_string("name", jesc(o->name));
	json_encode_bool("visible", o->visible);
	json_encode_bool("is_empty", o->is_empty);
	transform_t *t = o->transform;
	if (t != NULL) {
		json_encode_f32("loc_x", t->loc.x);
		json_encode_f32("loc_y", t->loc.y);
		json_encode_f32("loc_z", t->loc.z);
		json_encode_f32("rot_x", t->rot.x);
		json_encode_f32("rot_y", t->rot.y);
		json_encode_f32("rot_z", t->rot.z);
		json_encode_f32("rot_w", t->rot.w);
		json_encode_f32("scale_x", t->scale.x);
		json_encode_f32("scale_y", t->scale.y);
		json_encode_f32("scale_z", t->scale.z);
		json_encode_string("rotation_format", "quaternion xyzw");
	}
}

// Walk a "x,y;x,y;..." (or "x,y,z;..." when is_world) point list and paint it.
// The ';' between points and the ',' within one are server.py _points()'s
// format; a JSON array cannot cross this wire because it corrupts
// json_parse_to_map.
//
// Returns the number of points painted, or -1 if the list is longer than
// MAX_STROKE_POINTS. The over-length case is detected by a counting pass BEFORE
// anything is painted: silently truncating would leave a half-drawn stroke on
// the user's model, and painting first and erroring afterwards is worse than
// either. The counting pass is char_code_at only -- host calls, no arena frames.
int do_stroke(char *pts, int is_world) {
	if (pts == NULL) {
		return 0;
	}
	int len = string_length(pts);
	if (len < 1) {
		return 0;
	}
	int i   = 0;
	int c   = 0;
	int cnt = 1;
	while (i < len) {
		c = char_code_at(pts, i);
		if (c == 59) { // ';'
			cnt = cnt + 1;
		}
		i = i + 1;
	}
	if (cnt > MAX_STROKE_POINTS) {
		return -1;
	}

	int   pos  = 0;
	int   n    = 0;
	int   sp   = 0;
	int   c1   = 0;
	int   c2   = 0;
	int   tlen = 0;
	char *tok;
	while (pos < len) {
		sp = string_index_of_pos(pts, ";", pos);
		if (sp < 0) {
			sp = len;
		}
		if (sp > pos) {
			tok  = substring(pts, pos, sp);
			tlen = string_length(tok);
			c1   = string_index_of(tok, ",");
			if (c1 > 0) {
				if (is_world) {
					c2 = string_index_of_pos(tok, ",", c1 + 1);
					if (c2 > c1) {
						script_paint_world(to_float(substring(tok, 0, c1)), to_float(substring(tok, c1 + 1, c2)),
						                   to_float(substring(tok, c2 + 1, tlen)));
						n = n + 1;
					}
				}
				else {
					script_paint(to_float(substring(tok, 0, c1)), to_float(substring(tok, c1 + 1, tlen)));
					n = n + 1;
				}
			}
		}
		pos = sp + 1;
	}
	if (n > 0) {
		script_paint_end(); // a stroke that is never ended stays open in the tool
	}
	return n;
}

// Rewritten roughly once per second. `t` is sys_time(): seconds since app start,
// monotonic WITHIN A RUN and unrelated to the wall clock -- liveness is judged
// by t advancing between two reads, never by comparing it to time.time().
//
// There is no rename binding, so this single file is written truncating and a
// reader can catch it torn. That is unavoidable and harmless: a torn heartbeat
// fails to parse and the server simply reads again a moment later.
void write_heartbeat() {
	project_t *pr   = script_get_project();
	char      *pver = "";
	if (pr != NULL) {
		if (pr->version != NULL) {
			pver = pr->version;
		}
	}
	json_encode_begin();
	json_encode_i32("v", ENVELOPE_V);
	json_encode_string("pid_hint", "armorpaint");
	json_encode_string("bridge_version", BRIDGE_VERSION);
	// The app's own version string is not bound. project_t.version is the
	// nearest reachable thing: the .arm format version stamped by this build.
	json_encode_string("app_version", jesc(pver));
	json_encode_string("app_title", jesc(sys_title()));
	json_encode_f32("t", sys_time());
	json_encode_string("project", jesc(project_filepath_get()));
	json_encode_string("spool", jesc(spool_root));
	json_encode_bool("busy", busy);
	json_encode_bool("enabled", enabled);
	json_encode_i32("requests", req_count);
	json_encode_i32("errors", err_count);
	json_encode_f32("poll_interval", poll_interval);
	char *body = json_encode_end();
	iron_file_save_bytes(path_heartbeat, sys_string_to_buffer(body), 0);

	// Refresh the panel's counter line here rather than in on_ui, which draws at
	// frame rate. This is the natural throttle point: ~1 Hz plus once either side
	// of every request.
	ui_stats = string("requests %d / errors %d", req_count, err_count);
}

// TWO-FILE COMMIT (PROTOCOL.md). A plugin cannot write atomically -- there is no
// rename or move binding in the whole binding table and iron_file_save_bytes is
// a truncating fopen("wb"). So: write the body, let that call RETURN (the file
// is closed), and only then write a tiny marker holding the body's byte length.
// The server ignores any res/<id>.json without a matching .done, so a visible
// marker proves a complete body; the length lets it detect a torn one. Writing
// the marker first, or in the same call, defeats the entire scheme.
void reply(char *id, int okflag, char *inner, int ms) {
	char *body;
	if (okflag) {
		body = string("{\"v\":%d,\"id\":\"%s\",\"ok\":true,\"result\":%s,\"elapsed_ms\":%d}", ENVELOPE_V, jesc(id), inner, ms);
	}
	else {
		body = string("{\"v\":%d,\"id\":\"%s\",\"ok\":false,\"error\":%s,\"elapsed_ms\":%d}", ENVELOPE_V, jesc(id), inner, ms);
	}
	iron_file_save_bytes(string("%s/%s.json", dir_res, id), sys_string_to_buffer(body), 0);
	// Marker LAST, in its own call. string_length() is a byte count (strlen),
	// sys_string_to_buffer() writes exactly that many bytes (iron_system.c:1189)
	// and iron_file_save_bytes with length 0 writes bytes->length
	// (iron_file.c:413), so the number the server compares against the file size
	// is exact.
	iron_file_save_bytes(string("%s/%s.done", dir_res, id), sys_string_to_buffer(i32_to_string(string_length(body))), 0);
}

// ---------------------------------------------------------------------------
// THE DISPATCHER â€” one function, one else-if arm per operation.
//
// minic silently DROPS the 33rd function in a script (minic.c:2054, no else
// branch, no diagnostic), so operations must never become functions.
//
// Every arm validates its arguments before touching a binding: minic has no
// exceptions and dereferences pointers unchecked, so one bad request would
// otherwise take down the user's session mid-paint.
//
// Op names and argument names are server.py's. See the header comment.
// ---------------------------------------------------------------------------
char *dispatch(void *m) {
	// Envelope key, read DIRECTLY -- not through arg(). See the header comment.
	char *op = any_map_get(m, "op");
	if (op == NULL) {
		return fail("bad_args", "missing 'op'");
	}
	last_op = op;

	context_t        *c  = script_get_context();
	project_t        *pr = script_get_project();
	config_t         *cf = script_get_config();
	slot_material_t  *mt;
	ui_node_canvas_t *cv;
	ui_node_t        *nd;
	ui_node_t        *nd2;
	object_t         *ob;
	mesh_object_t    *mo;
	any_array_t      *ar;
	char             *s1;
	char             *s2;
	char             *acc;
	int               i1;
	int               i2;
	int               i3;
	int               i4;
	int               n;

	// ---- bridge & session -------------------------------------------------
	if (string_equals(op, "ping")) {
		json_encode_begin();
		json_encode_string("bridge_version", BRIDGE_VERSION);
		json_encode_i32("envelope_v", ENVELOPE_V);
		json_encode_f32("t", sys_time());
		json_encode_string("app_title", jesc(sys_title()));
		json_encode_string("project", jesc(project_filepath_get()));
		if (pr == NULL) {
			json_encode_null("app_version");
		}
		else {
			json_encode_string("app_version", jesc(pr->version));
		}
		json_encode_bool("busy", busy);
		json_encode_bool("enabled", enabled);
		json_encode_i32("requests", req_count);
		json_encode_i32("errors", err_count);
		return json_encode_end();
	}
	else if (string_equals(op, "get_app_info")) {
		json_encode_begin();
		json_encode_string("app_title", jesc(sys_title()));
		json_encode_i32("window_w", sys_w());
		json_encode_i32("window_h", sys_h());
		json_encode_i32("window_x", sys_x());
		json_encode_i32("window_y", sys_y());
		json_encode_string("data_path", jesc(data_path()));
		json_encode_string("spool", jesc(spool_root));
		json_encode_string("bridge_version", BRIDGE_VERSION);
		json_encode_i32("envelope_v", ENVELOPE_V);
		json_encode_f32("t", sys_time());
		json_encode_f32("poll_interval", poll_interval);
		json_encode_i32("max_stroke_points", MAX_STROKE_POINTS);
		json_encode_i32("max_list_items", MAX_LIST_ITEMS);
		json_encode_bool("viewport_patch", HAVE_VIEWPORT_PATCH);
		json_encode_i32("requests", req_count);
		json_encode_i32("errors", err_count);
		json_encode_string("last_op", jesc(last_op));
		json_encode_string("last_error", jesc(last_error));
		if (pr == NULL) {
			json_encode_null("format_version");
		}
		else {
			json_encode_string("format_version", jesc(pr->version));
		}
		return json_encode_end();
	}
	else if (string_equals(op, "bridge_set_enabled")) {
		s1 = arg(m, "enabled");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'enabled'");
		}
		i1      = to_bool(s1);
		enabled = i1;
		enabled_ui = i1; // keep the Plugins-tab checkbox in sync
		json_encode_begin();
		json_encode_bool("enabled", enabled);
		// Disabling stops on_update from polling AND from resetting the idle
		// counter, so the app is allowed to sleep again. Re-enabling from the
		// server is therefore impossible: use the Plugins tab toggle.
		json_encode_bool("reenable_needs_ui", 1);
		return json_encode_end();
	}
	else if (string_equals(op, "bridge_set_poll_ms")) {
		i1 = arg_i(m, "value", -1);
		if (i1 < 5) {
			return fail("bad_args", "value must be 5..1000 ms");
		}
		if (i1 > 1000) {
			return fail("bad_args", "value must be 5..1000 ms");
		}
		poll_interval = i1 / 1000.0;
		json_encode_begin();
		json_encode_f32("poll_interval", poll_interval);
		return json_encode_end();
	}
	else if (string_equals(op, "console_write")) {
		s1 = arg(m, "text");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'text'");
		}
		s2 = arg(m, "level");
		if (s2 == NULL) {
			s2 = "log";
		}
		if (string_equals(s2, "error")) {
			console_error(s1);
		}
		else if (string_equals(s2, "info")) {
			console_info(s1);
		}
		else {
			console_log(s1);
		}
		json_encode_begin();
		json_encode_string("level", jesc(s2));
		return json_encode_end();
	}
	else if (string_equals(op, "show_message")) {
		s1 = arg(m, "text");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'text'");
		}
		i1 = arg_i(m, "modal", 0);
		if (i1) {
			s2 = arg(m, "title");
			if (s2 == NULL) {
				s2 = "ArmorPaint MCP";
			}
			// A modal blocks ArmorPaint's UI thread, which also stalls this
			// bridge until the user dismisses it. That is the documented
			// behaviour of the tool, not a bug.
			ui_box_show_message(s2, s1, 0);
		}
		else {
			script_show_message(s1, arg_f(m, "seconds", 4.0));
		}
		json_encode_begin();
		json_encode_bool("modal", i1);
		return json_encode_end();
	}
	else if (string_equals(op, "quit")) {
		// iron_stop() only clears the run-loop flag (iron_system.c:209), so this
		// frame finishes normally and handle_one still commits the reply below.
		script_quit();
		json_encode_begin();
		json_encode_bool("quitting", 1);
		return json_encode_end();
	}

	// ---- project ----------------------------------------------------------
	else if (string_equals(op, "project_new")) {
		script_project_new();
		return ok_empty();
	}
	else if (string_equals(op, "project_open")) {
		s1 = arg(m, "path");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'path'");
		}
		if (!path_ok(s1)) {
			return fail("bad_args", "'path' is empty, over-long, or contains a quote or newline");
		}
		if (!file_here(s1)) {
			return fail("not_found", string("no such file: %s", s1));
		}
		script_project_open(s1);
		json_encode_begin();
		json_encode_string("filepath", jesc(project_filepath_get()));
		return json_encode_end();
	}
	else if (string_equals(op, "project_save")) {
		s1 = project_filepath_get();
		if (s1 == NULL) {
			return fail("no_project", "project has never been saved; use project_save_as");
		}
		if (string_equals(s1, "")) {
			return fail("no_project", "project has never been saved; use project_save_as");
		}
		project_save(0);
		json_encode_begin();
		json_encode_string("filepath", jesc(s1));
		json_encode_bool("deferred", 1);
		json_encode_string("note", "project_save only QUEUES the write (sys_notify_on_next_frame, project.c:39-61); the file appears one frame later. This reply is not proof the save landed -- confirm with fs_stat.");
		return json_encode_end();
	}
	else if (string_equals(op, "project_save_as")) {
		s1 = arg(m, "path");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'path'");
		}
		if (!path_ok(s1)) {
			return fail("bad_args", "'path' is empty, over-long, or contains a quote or newline");
		}
		// "Save as" is filepath_set + save; there is no dedicated binding.
		project_filepath_set(s1);
		// project_filepath_set() is the ONLY thing standing between us and a modal
		// file picker: project_save() with an empty filepath calls project_save_as(),
		// which opens iron_save_dialog and blocks the UI thread -- and with it the
		// bridge -- until a human clicks something. Never call project_save() here
		// without confirming the path took.
		s2 = project_filepath_get();
		if (s2 == NULL) {
			return fail("internal", "project_filepath_set did not take; refusing to call project_save because an empty filepath opens a modal file dialog that would block the bridge");
		}
		if (string_equals(s2, "")) {
			return fail("internal", "project_filepath_set did not take; refusing to call project_save because an empty filepath opens a modal file dialog that would block the bridge");
		}
		project_save(0);
		json_encode_begin();
		json_encode_string("filepath", jesc(s2));
		// NO "exists" field here. project_save() is deferred by a frame, so a
		// same-frame file_here() is false for every first save and true only when
		// something was ALREADY at that path -- i.e. it reports the state BEFORE
		// this save. Measured: first save_as -> false, one round trip later -> true,
		// second save_as to the same path -> true. An agent reading that field would
		// conclude its own save had failed.
		json_encode_bool("deferred", 1);
		json_encode_string("note", "the write is queued for the next frame; confirm with fs_stat rather than trusting this reply");
		return json_encode_end();
	}
	else if (string_equals(op, "project_get_info")) {
		json_encode_begin();
		json_encode_string("filepath", jesc(project_filepath_get()));
		json_encode_string("basepath", jesc(project_basepath_get()));
		if (pr == NULL) {
			json_encode_null("format_version");
		}
		else {
			json_encode_string("format_version", jesc(pr->version));
			json_encode_i32("is_bgra", pr->is_bgra);
			json_encode_string("envmap", jesc(pr->envmap));
			// Newer ArmorPaint: project_t envmap fields are not exposed to plugins;
			// the live angle is on the context, the live strength is unreachable.
			json_encode_null("envmap_strength");
			if (c != NULL) {
				json_encode_f32("envmap_angle", c->envmap_angle);
			}
			json_encode_f32("camera_fov", pr->camera_fov);
			if (pr->assets == NULL) {
				json_encode_i32("asset_count", 0);
			}
			else {
				json_encode_i32("asset_count", pr->assets->length);
			}
		}
		if (c != NULL) {
			if (c->material != NULL) {
				if (c->material->canvas != NULL) {
					json_encode_string("active_material", jesc(c->material->canvas->name));
				}
			}
		}
		mo = context_main_object();
		if (mo != NULL) {
			if (mo->base != NULL) {
				json_encode_string("main_object", jesc(mo->base->name));
			}
		}
		return json_encode_end();
	}
	else if (string_equals(op, "project_list_texture_assets")) {
		if (pr == NULL) {
			return fail("no_project", "no project runtime");
		}
		if (pr->assets == NULL) {
			return fail("no_project", "project has no asset list");
		}
		n = pr->assets->length;
		json_encode_begin();
		json_encode_i32("count", n);
		if (n > MAX_LIST_ITEMS) {
			n = MAX_LIST_ITEMS;
		}
		json_encode_i32("returned", n);
		json_encode_string("names", sa_names(pr->assets, MAX_LIST_ITEMS));
		json_encode_string("delimiter", "|");
		json_encode_bool("live", 1);
		return json_encode_end();
	}
	else if (string_equals(op, "project_list_scripts")) {
		if (pr == NULL) {
			return fail("no_project", "no project runtime");
		}
		json_encode_begin();
		json_encode_bool("live", 0);
		json_encode_string("caveat", "project_t.script_datas is written only at save/load: empty in a fresh project and stale after this session's changes");
		if (pr->script_datas == NULL) {
			json_encode_i32("count", 0);
			json_encode_i32("returned", 0);
			json_encode_string("names", "");
		}
		else {
			n = pr->script_datas->length;
			json_encode_i32("count", n);
			if (n > MAX_LIST_ITEMS) {
				n = MAX_LIST_ITEMS;
			}
			json_encode_i32("returned", n);
			json_encode_string("names", sa_names(pr->script_datas, MAX_LIST_ITEMS));
		}
		json_encode_string("delimiter", "|");
		return json_encode_end();
	}

	// ---- import / export --------------------------------------------------
	else if (string_equals(op, "import_asset")) {
		s1 = arg(m, "path");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'path'");
		}
		if (!path_ok(s1)) {
			return fail("bad_args", "'path' is empty, over-long, or contains a quote or newline");
		}
		if (!file_here(s1)) {
			return fail("not_found", string("no such file: %s", s1));
		}
		// Dispatched by extension inside ArmorPaint: texture, mesh, or .arm material.
		script_import_asset(s1, 0);
		json_encode_begin();
		json_encode_string("path", jesc(s1));
		if (pr != NULL) {
			if (pr->assets != NULL) {
				json_encode_i32("asset_count", pr->assets->length);
			}
		}
		return json_encode_end();
	}
	else if (string_equals(op, "import_envmap")) {
		s1 = arg(m, "path");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'path'");
		}
		if (!path_ok(s1)) {
			return fail("bad_args", "'path' is empty, over-long, or contains a quote or newline");
		}
		if (!file_here(s1)) {
			return fail("not_found", string("no such file: %s", s1));
		}
		script_import_asset(s1, 1); // hdr_as_envmap
		json_encode_begin();
		json_encode_string("path", jesc(s1));
		if (pr != NULL) {
			json_encode_string("envmap", jesc(pr->envmap));
			json_encode_null("envmap_strength");
		}
		return json_encode_end();
	}
	else if (string_equals(op, "set_envmap_params")) {
		if (c == NULL) {
			return fail("no_context", "no context runtime");
		}
		s1 = arg(m, "strength");
		if (s1 != NULL) {
			// Newer ArmorPaint keeps the live strength on scene_world, which is
			// not exposed to plugins (project_t.envmap_strength is not either).
			return fail("unsupported", "envmap strength is not reachable from plugins on this ArmorPaint build; only 'angle' can be set");
		}
		s2 = arg(m, "angle");
		if (s2 != NULL) {
			c->envmap_angle = to_float(s2);
		}
		c->ddirty = 2; // the viewport will not repaint on its own
		json_encode_begin();
		json_encode_null("envmap_strength");
		json_encode_f32("envmap_angle", c->envmap_angle);
		return json_encode_end();
	}
	else if (string_equals(op, "export_textures")) {
		s1 = arg(m, "directory");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'directory' (export_texture_run takes a DIRECTORY, not a filename)");
		}
		if (!path_ok(s1)) {
			return fail("bad_args", "'directory' is empty, over-long, or contains a quote or newline");
		}
		iron_create_directory(s1);
		if (!iron_is_directory(s1)) {
			return fail("bad_args", string("not a directory and could not be created: %s", s1));
		}
		i2 = dir_count(s1);
		// The only binding in the table that writes real image files to disk.
		// Filenames come from the last export-dialog name (else "untitled") plus
		// the active preset's per-channel suffixes; format and bit depth live on
		// unregistered fields and are NOT settable from a plugin (8-bit PNG).
		export_texture_run(s1, 0);
		i3 = dir_count(s1);
		json_encode_begin();
		json_encode_string("directory", jesc(s1));
		json_encode_i32("bake_material", 0);
		json_encode_i32("files_before", i2);
		json_encode_i32("files_after", i3);
		json_encode_i32("files_added", i3 - i2);
		ar = file_read_directory(s1);
		if (ar != NULL) {
			json_encode_string("files", sa_names(ar, MAX_LIST_ITEMS));
			json_encode_string("delimiter", "|");
			array_free(ar);
			free(ar);
		}
		json_encode_string("note", "names derive from the last export-dialog name or 'untitled'; the format is not settable from a plugin");
		return json_encode_end();
	}
	else if (string_equals(op, "export_material_bake")) {
		s1 = arg(m, "directory");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'directory'");
		}
		if (!path_ok(s1)) {
			return fail("bad_args", "'directory' is empty, over-long, or contains a quote or newline");
		}
		iron_create_directory(s1);
		if (!iron_is_directory(s1)) {
			return fail("bad_args", string("not a directory and could not be created: %s", s1));
		}
		i2 = dir_count(s1);
		export_texture_run(s1, 1); // bake_material
		i3 = dir_count(s1);
		json_encode_begin();
		json_encode_string("directory", jesc(s1));
		json_encode_i32("bake_material", 1);
		json_encode_i32("files_before", i2);
		json_encode_i32("files_after", i3);
		json_encode_i32("files_added", i3 - i2);
		ar = file_read_directory(s1);
		if (ar != NULL) {
			json_encode_string("files", sa_names(ar, MAX_LIST_ITEMS));
			json_encode_string("delimiter", "|");
			array_free(ar);
			free(ar);
		}
		return json_encode_end();
	}
	else if (string_equals(op, "export_mesh")) {
		s1 = arg(m, "path");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'path' (the extension is added: <path>.obj)");
		}
		if (!path_ok(s1)) {
			return fail("bad_args", "'path' is empty, over-long, or contains a quote or newline");
		}
		script_export_mesh(s1);
		s2 = string("%s.obj", s1);
		json_encode_begin();
		json_encode_string("path", jesc(s2));
		json_encode_bool("exists", file_here(s2));
		return json_encode_end();
	}
	else if (string_equals(op, "export_material")) {
		s1 = arg(m, "path");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'path' (should end in .arm)");
		}
		if (!path_ok(s1)) {
			return fail("bad_args", "'path' is empty, over-long, or contains a quote or newline");
		}
		script_export_material(s1);
		json_encode_begin();
		json_encode_string("path", jesc(s1));
		json_encode_bool("exists", file_here(s1));
		return json_encode_end();
	}

	// ---- filesystem (as the ArmorPaint process sees it) --------------------
	else if (string_equals(op, "fs_list")) {
		s1 = arg(m, "path");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'path'");
		}
		if (!path_ok(s1)) {
			return fail("bad_args", "'path' is empty, over-long, or contains a quote or newline");
		}
		if (!iron_is_directory(s1)) {
			return fail("not_found", string("not a directory: %s", s1));
		}
		ar = file_read_directory(s1);
		if (ar == NULL) {
			return fail("internal", string("could not list %s", s1));
		}
		n   = ar->length;
		acc = sa_names(ar, MAX_LIST_ITEMS);
		// The entry strings come from string_split (iron_string.c:173), which
		// allocates each one separately, so they outlive the array header.
		array_free(ar);
		free(ar);
		json_encode_begin();
		json_encode_string("path", jesc(s1));
		json_encode_i32("count", n);
		if (n > MAX_LIST_ITEMS) {
			n = MAX_LIST_ITEMS;
		}
		json_encode_i32("returned", n);
		json_encode_string("names", acc);
		json_encode_string("delimiter", "|");
		return json_encode_end();
	}
	else if (string_equals(op, "fs_stat")) {
		s1 = arg(m, "path");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'path'");
		}
		if (!path_ok(s1)) {
			return fail("bad_args", "'path' is empty, over-long, or contains a quote or newline");
		}
		i1 = 0;
		if (starts_with(s1, "/")) {
			i1 = 1;
		}
		if (string_length(s1) > 1) {
			if (char_code_at(s1, 1) == 58) { // "C:" style
				i1 = 1;
			}
		}
		json_encode_begin();
		json_encode_string("path", jesc(s1));
		json_encode_bool("exists", file_here(s1));
		json_encode_bool("is_directory", iron_is_directory(s1));
		json_encode_bool("is_absolute", i1);
		return json_encode_end();
	}
	else if (string_equals(op, "fs_mkdir")) {
		s1 = arg(m, "path");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'path'");
		}
		if (!path_ok(s1)) {
			return fail("bad_args", "'path' is empty, over-long, or contains a quote or newline");
		}
		iron_create_directory(s1);
		json_encode_begin();
		json_encode_string("path", jesc(s1));
		json_encode_bool("is_directory", iron_is_directory(s1));
		json_encode_string("note", "intermediate directories are NOT created; make each parent first");
		return json_encode_end();
	}

	// ---- introspection ----------------------------------------------------
	else if (string_equals(op, "get_context")) {
		if (c == NULL) {
			return fail("internal", "no context");
		}
		json_encode_begin();
		json_encode_i32("tool", c->tool);
		json_encode_i32("viewport_mode", c->viewport_mode);
		json_encode_i32("xray", c->xray);
		json_encode_i32("brush_blending", c->brush_blending);
		json_encode_f32("brush_radius", c->brush_radius);
		json_encode_f32("brush_opacity", c->brush_opacity);
		json_encode_f32("brush_hardness", c->brush_hardness);
		json_encode_f32("brush_scale", c->brush_scale);
		json_encode_f32("brush_angle", c->brush_angle);
		json_encode_i32("ddirty", c->ddirty);
		json_encode_i32("pdirty", c->pdirty);
		// context_t.layer and .brush are untyped void* -- slot_layer_t is not a
		// registered struct, so only their non-nullness is knowable.
		json_encode_bool("has_layer", c->layer != NULL);
		json_encode_bool("has_brush", c->brush != NULL);
		if (c->material != NULL) {
			if (c->material->canvas != NULL) {
				json_encode_string("material", jesc(c->material->canvas->name));
			}
		}
		if (c->paint_object != NULL) {
			if (c->paint_object->base != NULL) {
				json_encode_string("paint_object", jesc(c->paint_object->base->name));
			}
		}
		return json_encode_end();
	}
	else if (string_equals(op, "get_config")) {
		if (cf == NULL) {
			return fail("internal", "no config");
		}
		json_encode_begin();
		json_encode_i32("window_w", cf->window_w);
		json_encode_i32("window_h", cf->window_h);
		json_encode_f32("window_scale", cf->window_scale);
		json_encode_f32("rp_supersample", cf->rp_supersample);
		json_encode_i32("layer_res", cf->layer_res);
		json_encode_i32("undo_steps", cf->undo_steps);
		json_encode_f32("camera_fov", cf->camera_fov);
		json_encode_i32("brush_live", cf->brush_live);
		json_encode_i32("node_previews", cf->node_previews);
		json_encode_i32("material_live", cf->material_live);
		json_encode_null("workspace");
		json_encode_null("workflow");
		json_encode_string("keymap", jesc(cf->keymap));
		json_encode_string("theme", jesc(cf->theme));
		json_encode_string("recent_projects", sa_names(cf->recent_projects, MAX_LIST_ITEMS));
		json_encode_string("plugins", sa_names(cf->plugins, MAX_LIST_ITEMS));
		json_encode_string("delimiter", "|");
		return json_encode_end();
	}
	else if (string_equals(op, "set_config")) {
		if (cf == NULL) {
			return fail("internal", "no config");
		}
		// Presence, not a sentinel: workspace, workflow and the three live-*
		// toggles are all legitimately 0, so "absent" has to mean "the key is
		// not in the map" rather than "the value is some magic number".
		s1 = arg(m, "window_w");
		if (s1 != NULL) {
			cf->window_w = to_int(s1);
		}
		s1 = arg(m, "window_h");
		if (s1 != NULL) {
			cf->window_h = to_int(s1);
		}
		s1 = arg(m, "undo_steps");
		if (s1 != NULL) {
			cf->undo_steps = to_int(s1);
		}
		s1 = arg(m, "layer_res");
		if (s1 != NULL) {
			cf->layer_res = to_int(s1);
		}
		// workspace/workflow were removed from config_t in newer ArmorPaint.
		if (arg(m, "workspace") != NULL) {
			return fail("unsupported", "'workspace' no longer exists in this ArmorPaint build's config");
		}
		if (arg(m, "workflow") != NULL) {
			return fail("unsupported", "'workflow' no longer exists in this ArmorPaint build's config");
		}
		s1 = arg(m, "window_scale");
		if (s1 != NULL) {
			cf->window_scale = to_float(s1);
		}
		s1 = arg(m, "rp_supersample");
		if (s1 != NULL) {
			cf->rp_supersample = to_float(s1);
		}
		s1 = arg(m, "camera_fov");
		if (s1 != NULL) {
			cf->camera_fov = to_float(s1);
		}
		s1 = arg(m, "brush_live");
		if (s1 != NULL) {
			cf->brush_live = to_bool(s1);
		}
		s1 = arg(m, "node_previews");
		if (s1 != NULL) {
			cf->node_previews = to_bool(s1);
		}
		s1 = arg(m, "material_live");
		if (s1 != NULL) {
			cf->material_live = to_bool(s1);
		}
		// The map's value strings are substrings of the request text and the map
		// is dropped at the end of this frame, so copy before handing the host a
		// pointer it will hold indefinitely.
		s1 = arg(m, "keymap");
		if (s1 != NULL) {
			cf->keymap = string_copy(s1);
		}
		s1 = arg(m, "theme");
		if (s1 != NULL) {
			cf->theme = string_copy(s1);
		}
		json_encode_begin();
		json_encode_i32("window_w", cf->window_w);
		json_encode_i32("window_h", cf->window_h);
		json_encode_f32("window_scale", cf->window_scale);
		json_encode_f32("rp_supersample", cf->rp_supersample);
		json_encode_i32("layer_res", cf->layer_res);
		json_encode_i32("undo_steps", cf->undo_steps);
		json_encode_f32("camera_fov", cf->camera_fov);
		json_encode_i32("brush_live", cf->brush_live);
		json_encode_i32("node_previews", cf->node_previews);
		json_encode_i32("material_live", cf->material_live);
		json_encode_null("workspace");
		json_encode_null("workflow");
		json_encode_string("keymap", jesc(cf->keymap));
		json_encode_string("theme", jesc(cf->theme));
		json_encode_string("note", "keymap and theme are stored but ArmorPaint applies them only on reload");
		return json_encode_end();
	}
	else if (string_equals(op, "get_main_object")) {
		mo = context_main_object();
		if (mo == NULL) {
			return fail("no_project", "no main paint object");
		}
		if (mo->base == NULL) {
			return fail("internal", "main object has no base");
		}
		json_encode_begin();
		emit_object(mo->base);
		return json_encode_end();
	}
	else if (string_equals(op, "get_object")) {
		s1 = arg(m, "name");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'name'");
		}
		ob = script_get_object(s1);
		if (ob == NULL) {
			return fail("not_found", string("no object named %s", s1));
		}
		json_encode_begin();
		emit_object(ob);
		return json_encode_end();
	}

	// ---- objects & meshes -------------------------------------------------
	else if (string_equals(op, "shape_list")) {
		// script_shape_list() returns project_default_mesh_list, a host-owned
		// global (minic_impl.c:702). NEVER free it.
		json_encode_begin();
		json_encode_string("names", sa_names(script_shape_list(), MAX_LIST_ITEMS));
		json_encode_string("delimiter", "|");
		return json_encode_end();
	}
	else if (string_equals(op, "shape_add")) {
		s1 = arg(m, "name");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'name'");
		}
		// The host validates the name against its own list and returns NULL for
		// an unknown shape or an empty project (minic_impl.c:707).
		ob = script_shape_add(s1);
		if (ob == NULL) {
			return fail("bad_args", string("unknown shape '%s', or there is no paint object to add it to", s1));
		}
		json_encode_begin();
		emit_object(ob);
		return json_encode_end();
	}
	else if (string_equals(op, "object_duplicate")) {
		s1 = arg(m, "name");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'name'");
		}
		ob = script_get_object(s1);
		if (ob == NULL) {
			return fail("not_found", string("no object named %s", s1));
		}
		ob = script_object_duplicate(ob);
		if (ob == NULL) {
			return fail("bad_args", string("'%s' is not a mesh object and cannot be duplicated", s1));
		}
		json_encode_begin();
		emit_object(ob);
		return json_encode_end();
	}
	else if (string_equals(op, "object_set_transform")) {
		s1 = arg(m, "name");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'name'");
		}
		ob = script_get_object(s1);
		if (ob == NULL) {
			return fail("not_found", string("no object named %s", s1));
		}
		transform_t *tr = ob->transform;
		if (tr == NULL) {
			return fail("internal", "object has no transform");
		}
		s2 = arg(m, "loc_x");
		if (s2 != NULL) {
			tr->loc.x = to_float(s2);
			tr->loc.y = arg_f(m, "loc_y", tr->loc.y);
			tr->loc.z = arg_f(m, "loc_z", tr->loc.z);
			tr->loc.w = 1.0;
		}
		s2 = arg(m, "rot_x");
		if (s2 != NULL) {
			// server.py sends XYZ euler in RADIANS. iron composes its quaternion
			// YZX (quat_from_euler, iron_math.c:404) and that function is not
			// bound, so the formula is reproduced exactly -- any other ordering
			// would silently disagree with what the app itself would produce.
			float ex  = to_float(s2);
			float ey  = arg_f(m, "rot_y", 0.0);
			float ez  = arg_f(m, "rot_z", 0.0);
			float h1  = ex / 2.0;
			float h2  = ey / 2.0;
			float h3  = ez / 2.0;
			float q1  = cosf(h1);
			float p1  = sinf(h1);
			float q2  = cosf(h2);
			float p2  = sinf(h2);
			float q3  = cosf(h3);
			float p3  = sinf(h3);
			tr->rot.x = p1 * q2 * q3 + q1 * p2 * p3;
			tr->rot.y = q1 * p2 * q3 + p1 * q2 * p3;
			tr->rot.z = q1 * q2 * p3 - p1 * p2 * q3;
			tr->rot.w = q1 * q2 * q3 - p1 * p2 * p3;
		}
		s2 = arg(m, "scale_x");
		if (s2 != NULL) {
			tr->scale.x = to_float(s2);
			tr->scale.y = arg_f(m, "scale_y", tr->scale.y);
			tr->scale.z = arg_f(m, "scale_z", tr->scale.z);
			tr->scale.w = 1.0;
		}
		transform_build_matrix(tr);
		if (c != NULL) {
			c->ddirty = 2;
		}
		json_encode_begin();
		emit_object(ob);
		return json_encode_end();
	}
	else if (string_equals(op, "object_set_visible")) {
		s1 = arg(m, "name");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'name'");
		}
		s2 = arg(m, "visible");
		if (s2 == NULL) {
			return fail("bad_args", "missing 'visible'");
		}
		ob = script_get_object(s1);
		if (ob == NULL) {
			return fail("not_found", string("no object named %s", s1));
		}
		ob->visible = to_bool(s2);
		if (c != NULL) {
			c->ddirty = 2;
		}
		json_encode_begin();
		emit_object(ob);
		return json_encode_end();
	}
	else if (string_equals(op, "append_mesh")) {
		s1 = arg(m, "path");
		if (s1 != NULL) {
			if (!path_ok(s1)) {
				return fail("bad_args", "'path' is empty, over-long, or contains a quote or newline");
			}
			if (!file_here(s1)) {
				return fail("not_found", string("no such file: %s", s1));
			}
			script_append_mesh(s1);
			json_encode_begin();
			json_encode_string("path", jesc(s1));
			json_encode_string("source", "file");
			return json_encode_end();
		}
		s2 = arg(m, "obj_data");
		if (s2 == NULL) {
			return fail("bad_args", "give exactly one of 'path' or 'obj_data'");
		}
		// A real newline cannot cross this wire -- the request JSON is not
		// escape-decoded on this side -- so server.py encodes OBJ line breaks as
		// '|' and they are restored here. Wavefront OBJ never contains a '|'.
		script_append_mesh_obj(string_replace_all(s2, "|", "\n"));
		json_encode_begin();
		json_encode_string("source", "inline");
		json_encode_i32("bytes", string_length(s2));
		return json_encode_end();
	}

	// ---- materials --------------------------------------------------------
	else if (string_equals(op, "material_get_active")) {
		if (c == NULL) {
			return fail("internal", "no context");
		}
		mt = c->material;
		if (mt == NULL) {
			return fail("no_project", "no active material");
		}
		json_encode_begin();
		json_encode_i32("id", mt->id);
		if (mt->canvas != NULL) {
			json_encode_string("name", jesc(mt->canvas->name));
			if (mt->canvas->nodes != NULL) {
				json_encode_i32("node_count", mt->canvas->nodes->length);
			}
			if (mt->canvas->links != NULL) {
				json_encode_i32("link_count", mt->canvas->links->length);
			}
		}
		// Reported under server.py's PAINT_CHANNELS names, not slot_material_t's
		// abbreviations, so what comes back matches what set_channels takes.
		json_encode_bool("base", mt->paint_base);
		json_encode_bool("opacity", mt->paint_opac);
		json_encode_bool("occlusion", mt->paint_occ);
		json_encode_bool("roughness", mt->paint_rough);
		json_encode_bool("metallic", mt->paint_met);
		json_encode_bool("normal", mt->paint_nor);
		json_encode_bool("height", mt->paint_height);
		json_encode_bool("emission", mt->paint_emis);
		json_encode_bool("subsurface", mt->paint_subs);
		return json_encode_end();
	}
	else if (string_equals(op, "material_list")) {
		if (pr == NULL) {
			return fail("no_project", "no project runtime");
		}
		json_encode_begin();
		json_encode_bool("live", 0);
		json_encode_string("caveat", "project_t.material_nodes is written only at save/load: null before the first save and blind to materials created this session; use material_get_active for ground truth");
		// project_t.material_nodes is an untyped void*. Every *_array_t in this
		// codebase shares {buffer,length,capacity}, so re-typing it as
		// any_array_t reads the right offsets (MINIC_DIALECT_AND_API.md 2.1).
		ar = pr->material_nodes;
		if (ar == NULL) {
			json_encode_i32("count", 0);
			json_encode_i32("returned", 0);
			json_encode_string("names", "");
		}
		else {
			n = ar->length;
			json_encode_i32("count", n);
			if (n > MAX_LIST_ITEMS) {
				n = MAX_LIST_ITEMS;
			}
			json_encode_i32("returned", n);
			acc = "";
			for (int mi = 0; mi < n; ++mi) {
				ui_node_canvas_t *cvi = ar->buffer[mi];
				if (cvi != NULL) {
					if (mi > 0) {
						acc = string("%s|%s", acc, jesc(cvi->name));
					}
					else {
						acc = jesc(cvi->name);
					}
				}
			}
			json_encode_string("names", acc);
		}
		json_encode_string("delimiter", "|");
		return json_encode_end();
	}
	else if (string_equals(op, "material_create")) {
		s1 = arg(m, "name");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'name'");
		}
		if (string_length(s1) < 1) {
			return fail("bad_args", "empty 'name'");
		}
		mt = script_material_create(s1);
		if (mt == NULL) {
			return fail("internal", "material could not be created");
		}
		json_encode_begin();
		json_encode_i32("id", mt->id);
		if (mt->canvas != NULL) {
			json_encode_string("name", jesc(mt->canvas->name));
		}
		return json_encode_end();
	}
	else if (string_equals(op, "material_select")) {
		s1 = arg(m, "name");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'name'");
		}
		// A material's identity is its canvas name (minic_impl.c:177).
		mt = script_get_material(s1);
		if (mt == NULL) {
			return fail("not_found", string("no material named %s", s1));
		}
		script_material_set(mt);
		json_encode_begin();
		json_encode_i32("id", mt->id);
		json_encode_string("name", jesc(s1));
		return json_encode_end();
	}
	else if (string_equals(op, "material_delete")) {
		s1 = arg(m, "name");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'name'");
		}
		mt = script_get_material(s1);
		if (mt == NULL) {
			return fail("not_found", string("no material named %s", s1));
		}
		script_material_delete(mt);
		return ok_empty();
	}
	else if (string_equals(op, "material_assign")) {
		s1 = arg(m, "object");
		s2 = arg(m, "material");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'object'");
		}
		if (s2 == NULL) {
			return fail("bad_args", "missing 'material'");
		}
		ob = script_get_object(s1);
		if (ob == NULL) {
			return fail("not_found", string("no paint object named %s", s1));
		}
		mt = script_get_material(s2);
		if (mt == NULL) {
			return fail("not_found", string("no material named %s", s2));
		}
		script_object_set_material(ob, mt);
		return ok_empty();
	}
	else if (string_equals(op, "material_set_channels")) {
		if (c == NULL) {
			return fail("internal", "no context");
		}
		mt = c->material;
		if (mt == NULL) {
			return fail("no_project", "no active material");
		}
		// The KEYS are server.py's PAINT_CHANNELS, not slot_material_t's field
		// spelling. They differ (opacity/paint_opac, occlusion/paint_occ,
		// roughness/paint_rough, ...) and reading the wrong name here is silent:
		// the op answers ok and changes nothing.
		s1 = arg(m, "base");
		if (s1 != NULL) {
			mt->paint_base = to_bool(s1);
		}
		s1 = arg(m, "opacity");
		if (s1 != NULL) {
			mt->paint_opac = to_bool(s1);
		}
		s1 = arg(m, "occlusion");
		if (s1 != NULL) {
			mt->paint_occ = to_bool(s1);
		}
		s1 = arg(m, "roughness");
		if (s1 != NULL) {
			mt->paint_rough = to_bool(s1);
		}
		s1 = arg(m, "metallic");
		if (s1 != NULL) {
			mt->paint_met = to_bool(s1);
		}
		s1 = arg(m, "normal");
		if (s1 != NULL) {
			mt->paint_nor = to_bool(s1);
		}
		s1 = arg(m, "height");
		if (s1 != NULL) {
			mt->paint_height = to_bool(s1);
		}
		s1 = arg(m, "emission");
		if (s1 != NULL) {
			mt->paint_emis = to_bool(s1);
		}
		s1 = arg(m, "subsurface");
		if (s1 != NULL) {
			mt->paint_subs = to_bool(s1);
		}
		json_encode_begin();
		json_encode_bool("base", mt->paint_base);
		json_encode_bool("opacity", mt->paint_opac);
		json_encode_bool("occlusion", mt->paint_occ);
		json_encode_bool("roughness", mt->paint_rough);
		json_encode_bool("metallic", mt->paint_met);
		json_encode_bool("normal", mt->paint_nor);
		json_encode_bool("height", mt->paint_height);
		json_encode_bool("emission", mt->paint_emis);
		json_encode_bool("subsurface", mt->paint_subs);
		return json_encode_end();
	}
	else if (string_equals(op, "material_update")) {
		script_material_update();
		return ok_empty();
	}

	// ---- material nodes ---------------------------------------------------
	else if (string_equals(op, "node_list")) {
		if (c == NULL) {
			return fail("internal", "no context");
		}
		if (c->material == NULL) {
			return fail("no_project", "no active material");
		}
		cv = c->material->canvas;
		if (cv == NULL) {
			return fail("no_project", "the active material has no canvas");
		}
		if (cv->nodes == NULL) {
			return fail("internal", "the canvas has no node array");
		}
		n = cv->nodes->length;
		if (n > MAX_LIST_ITEMS) {
			n = MAX_LIST_ITEMS;
		}
		// "id,TYPE,name,x,y;..." -- a delimited string for the same reason as
		// the other lists, and bounded because string() leaks every intermediate.
		acc = "";
		for (int ni = 0; ni < n; ++ni) {
			ui_node_t *ndi = cv->nodes->buffer[ni];
			if (ndi != NULL) {
				acc = string("%s%d,%s,%s,%f,%f;", acc, ndi->id, jesc(ndi->type), jesc(ndi->name), ndi->x, ndi->y);
			}
		}
		json_encode_begin();
		json_encode_i32("count", cv->nodes->length);
		json_encode_i32("returned", n);
		json_encode_string("nodes", acc);
		json_encode_string("node_format", "id,type,name,x,y;");
		// Graph topology. ui_node_link_t is its own registered struct -- reading a
		// link through ui_node_t would silently read the wrong offsets.
		if (cv->links != NULL) {
			i1 = cv->links->length;
			if (i1 > MAX_LIST_ITEMS) {
				i1 = MAX_LIST_ITEMS;
			}
			acc = "";
			for (int li = 0; li < i1; ++li) {
				ui_node_link_t *lk = cv->links->buffer[li];
				if (lk != NULL) {
					acc = string("%s%d,%d,%d,%d;", acc, lk->from_id, lk->from_socket, lk->to_id, lk->to_socket);
				}
			}
			json_encode_i32("link_count", cv->links->length);
			json_encode_i32("links_returned", i1);
			json_encode_string("links", acc);
			json_encode_string("link_format", "from_id,from_socket,to_id,to_socket;");
		}
		return json_encode_end();
	}
	else if (string_equals(op, "node_add")) {
		s1 = arg(m, "type");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'type'");
		}
		if (string_length(s1) < 1) {
			return fail("bad_args", "empty 'type'");
		}
		if (c == NULL) {
			return fail("internal", "no context");
		}
		if (c->material == NULL) {
			return fail("no_project", "no active material");
		}
		// Returns NULL for an unknown type, which is the only validation
		// available -- the legal type names are not enumerable at runtime.
		nd = script_material_create_node_at(s1, arg_f(m, "x", 0.0), arg_f(m, "y", 0.0));
		if (nd == NULL) {
			return fail("bad_args", string("unknown node type: %s", s1));
		}
		json_encode_begin();
		json_encode_i32("id", nd->id);
		json_encode_string("type", jesc(nd->type));
		json_encode_string("name", jesc(nd->name));
		json_encode_f32("x", nd->x);
		json_encode_f32("y", nd->y);
		if (nd->inputs != NULL) {
			json_encode_i32("inputs", nd->inputs->length);
		}
		if (nd->outputs != NULL) {
			json_encode_i32("outputs", nd->outputs->length);
		}
		return json_encode_end();
	}
	else if (string_equals(op, "node_remove")) {
		i1 = arg_i(m, "id", -1);
		if (i1 < 0) {
			return fail("bad_args", "missing or negative 'id'");
		}
		nd = script_material_get_node_id(i1);
		if (nd == NULL) {
			return fail("not_found", string("no node with id %d", i1));
		}
		if (string_equals(nd->type, "OUTPUT_MATERIAL_PBR")) {
			return fail("unsupported", "the PBR output node cannot be removed");
		}
		script_material_remove_node(nd);
		return ok_empty();
	}
	else if (string_equals(op, "node_connect")) {
		i1 = arg_i(m, "from_id", -1);
		i2 = arg_i(m, "from_socket", -1);
		i3 = arg_i(m, "to_id", -1);
		i4 = arg_i(m, "to_socket", -1);
		if (i1 < 0) {
			return fail("bad_args", "missing 'from_id'");
		}
		if (i3 < 0) {
			return fail("bad_args", "missing 'to_id'");
		}
		if (i2 < 0) {
			return fail("bad_args", "missing 'from_socket'");
		}
		if (i4 < 0) {
			return fail("bad_args", "missing 'to_socket'");
		}
		nd = script_material_get_node_id(i1);
		if (nd == NULL) {
			return fail("not_found", string("no node with id %d", i1));
		}
		nd2 = script_material_get_node_id(i3);
		if (nd2 == NULL) {
			return fail("not_found", string("no node with id %d", i3));
		}
		// script_material_connect() no-ops silently on an out-of-range socket,
		// so the range is checked here to report bad_args instead.
		if (nd->outputs == NULL) {
			return fail("bad_args", "the source node has no outputs");
		}
		if (i2 >= nd->outputs->length) {
			return fail("bad_args", string("from_socket %d out of range", i2));
		}
		if (nd2->inputs == NULL) {
			return fail("bad_args", "the target node has no inputs");
		}
		if (i4 >= nd2->inputs->length) {
			return fail("bad_args", string("to_socket %d out of range", i4));
		}
		script_material_connect(nd, i2, nd2, i4);
		return ok_empty();
	}
	else if (string_equals(op, "node_disconnect")) {
		i1 = arg_i(m, "to_id", -1);
		i2 = arg_i(m, "to_socket", -1);
		if (i1 < 0) {
			return fail("bad_args", "missing 'to_id'");
		}
		if (i2 < 0) {
			return fail("bad_args", "missing 'to_socket'");
		}
		nd = script_material_get_node_id(i1);
		if (nd == NULL) {
			return fail("not_found", string("no node with id %d", i1));
		}
		script_material_disconnect(nd, i2);
		return ok_empty();
	}
	else if (string_equals(op, "node_set_value")) {
		// One op, four kinds -- server.py's ap_node_set_value. Four separate ops
		// would work too, but the tool surface is the contract and the tool is
		// singular.
		s1 = arg(m, "kind");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'kind' (float, color, vector or button)");
		}
		i1 = arg_i(m, "id", -1);
		if (i1 < 0) {
			return fail("bad_args", "missing 'id'");
		}
		nd = script_material_get_node_id(i1);
		if (nd == NULL) {
			return fail("not_found", string("no node with id %d", i1));
		}
		i3 = arg_i(m, "is_input", 1);
		if (string_equals(s1, "button")) {
			i2 = arg_i(m, "button", -1);
			if (i2 < 0) {
				return fail("bad_args", "kind 'button' needs 'button'");
			}
			if (nd->buttons == NULL) {
				return fail("bad_args", "the node has no buttons");
			}
			if (i2 >= nd->buttons->length) {
				return fail("bad_args", string("button %d out of range", i2));
			}
			script_material_set_button(nd, i2, arg_f(m, "value", 0.0));
			return ok_empty();
		}
		i2 = arg_i(m, "socket", -1);
		if (i2 < 0) {
			return fail("bad_args", "missing 'socket'");
		}
		// Range-check against the side actually being written: the setters index
		// the socket array without a bounds check of their own.
		if (i3) {
			if (nd->inputs == NULL) {
				return fail("bad_args", "the node has no input sockets");
			}
			if (i2 >= nd->inputs->length) {
				return fail("bad_args", string("input socket %d out of range", i2));
			}
		}
		else {
			if (nd->outputs == NULL) {
				return fail("bad_args", "the node has no output sockets");
			}
			if (i2 >= nd->outputs->length) {
				return fail("bad_args", string("output socket %d out of range", i2));
			}
		}
		if (string_equals(s1, "float")) {
			script_material_set_float(nd, i3, i2, arg_f(m, "value", 0.0));
			return ok_empty();
		}
		else if (string_equals(s1, "color")) {
			// 'alpha', not 'a' -- server.py sends the fourth component spelled out.
			script_material_set_color(nd, i3, i2, arg_f(m, "r", 0.0), arg_f(m, "g", 0.0), arg_f(m, "b", 0.0), arg_f(m, "alpha", 1.0));
			return ok_empty();
		}
		else if (string_equals(s1, "vector")) {
			script_material_set_vector(nd, i3, i2, arg_f(m, "x", 0.0), arg_f(m, "y", 0.0), arg_f(m, "z", 0.0));
			return ok_empty();
		}
		return fail("bad_args", string("unknown kind '%s'; expected float, color, vector or button", s1));
	}

	// ---- painting & viewport ----------------------------------------------
	else if (string_equals(op, "select_tool")) {
		i1 = arg_i(m, "tool", -1);
		if (i1 < 0) {
			return fail("bad_args", "tool must be 0..13 (brush..bake)");
		}
		if (i1 > 13) {
			return fail("bad_args", "tool must be 0..13 (brush..bake)");
		}
		context_select_tool(i1);
		json_encode_begin();
		if (c != NULL) {
			json_encode_i32("tool", c->tool); // read back, do not assume
		}
		return json_encode_end();
	}
	else if (string_equals(op, "set_display_channel")) {
		i1 = arg_i(m, "mode", -99);
		if (i1 < -1) {
			return fail("bad_args", "mode must be -1..15");
		}
		if (i1 > 15) {
			return fail("bad_args", "mode must be -1..15");
		}
		context_set_viewport_mode(i1);
		json_encode_begin();
		if (c != NULL) {
			json_encode_i32("viewport_mode", c->viewport_mode);
		}
		return json_encode_end();
	}
	else if (string_equals(op, "set_brush")) {
		if (c == NULL) {
			return fail("internal", "no context");
		}
		// Brush parameters are context_t fields, not bindings. Presence rather
		// than a sentinel: brush_angle may legitimately hold any value.
		s1 = arg(m, "radius");
		if (s1 != NULL) {
			c->brush_radius = to_float(s1);
		}
		s1 = arg(m, "opacity");
		if (s1 != NULL) {
			c->brush_opacity = to_float(s1);
		}
		s1 = arg(m, "hardness");
		if (s1 != NULL) {
			c->brush_hardness = to_float(s1);
		}
		s1 = arg(m, "scale");
		if (s1 != NULL) {
			c->brush_scale = to_float(s1);
		}
		s1 = arg(m, "angle");
		if (s1 != NULL) {
			c->brush_angle = to_float(s1);
		}
		s1 = arg(m, "blending");
		if (s1 != NULL) {
			c->brush_blending = to_int(s1);
		}
		json_encode_begin();
		json_encode_f32("radius", c->brush_radius);
		json_encode_f32("opacity", c->brush_opacity);
		json_encode_f32("hardness", c->brush_hardness);
		json_encode_f32("scale", c->brush_scale);
		json_encode_f32("angle", c->brush_angle);
		json_encode_i32("blending", c->brush_blending);
		return json_encode_end();
	}
	else if (string_equals(op, "paint_stroke")) {
		s1 = arg(m, "points");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'points' (\"x,y;x,y;...\", normalised screen coords)");
		}
		if (c == NULL) {
			return fail("internal", "no context");
		}
		if (c->layer == NULL) {
			return fail("no_project", "no layer is selected; script_paint would no-op");
		}
		n = do_stroke(s1, 0);
		if (n < 0) {
			return fail("bad_args", string("too many points; this bridge paints at most %d in the single frame a stroke runs in", MAX_STROKE_POINTS));
		}
		if (n < 1) {
			return fail("bad_args", "no parseable points; expected \"x,y;x,y;...\"");
		}
		json_encode_begin();
		json_encode_i32("points", n);
		json_encode_i32("max_points", MAX_STROKE_POINTS);
		return json_encode_end();
	}
	else if (string_equals(op, "paint_stroke_world")) {
		s1 = arg(m, "points");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'points' (\"x,y,z;x,y,z;...\", world coords)");
		}
		if (c == NULL) {
			return fail("internal", "no context");
		}
		if (c->layer == NULL) {
			return fail("no_project", "no layer is selected; script_paint_world would no-op");
		}
		n = do_stroke(s1, 1);
		if (n < 0) {
			return fail("bad_args", string("too many points; this bridge paints at most %d in the single frame a stroke runs in", MAX_STROKE_POINTS));
		}
		if (n < 1) {
			return fail("bad_args", "no parseable points; expected \"x,y,z;x,y,z;...\"");
		}
		json_encode_begin();
		json_encode_i32("points", n);
		json_encode_i32("max_points", MAX_STROKE_POINTS);
		return json_encode_end();
	}
	else if (string_equals(op, "fill_layer")) {
		// The binding table contains exactly ONE layer operation. slot_layer_t is
		// not a registered struct and project_t.layer_datas is NULL in a live
		// session, so create/delete/rename/reorder/mask/opacity/blend are all
		// genuinely unreachable -- which is why there is no tool for them.
		if (c == NULL) {
			return fail("internal", "no context");
		}
		if (c->layer == NULL) {
			return fail("no_project", "no layer is selected");
		}
		if (c->material == NULL) {
			return fail("no_project", "no active material");
		}
		script_fill_layer(); // fills the selected layer with the active material
		return ok_empty();
	}
	else if (string_equals(op, "capture_to_project")) {
		if (pr == NULL) {
			return fail("no_project", "viewport capture needs an open project");
		}
		if (c == NULL) {
			return fail("internal", "no context");
		}
		i1 = arg_i(m, "width", 1024);
		i2 = arg_i(m, "height", 1024);
		if (i1 < 16) {
			return fail("bad_args", "width must be 16..4096");
		}
		if (i1 > 4096) {
			return fail("bad_args", "width must be 16..4096");
		}
		if (i2 < 16) {
			return fail("bad_args", "height must be 16..4096");
		}
		if (i2 > 4096) {
			return fail("bad_args", "height must be 16..4096");
		}
		// The upstream recipe is make_tilesheet.c:38-58. It copies the render
		// path's "last" target, i.e. the frame ALREADY RENDERED. A handler runs
		// inline and cannot wait for a re-render, so the UI-hiding handshake
		// (capturing_screenshot plus a two-frame settle) cannot complete inside
		// one request -- this captures whatever was last drawn.
		void *tex = gpu_create_render_target(i1, i2, GPU_TEXTURE_FORMAT_RGBA32);
		if (tex == NULL) {
			return fail("internal", "gpu_create_render_target returned null");
		}
		viewport_capture_screenshot_to(tex, 0.0, 0.0, i1, i2);
		viewport_save_texture(tex);
		c->capturing_screenshot = 0;
		c->ddirty               = 2;
		json_encode_begin();
		json_encode_i32("width", i1);
		json_encode_i32("height", i2);
		json_encode_string("destination", "project packed asset (/packed/screenshotN.png)");
		json_encode_string("note", "the pixels live inside the project and are persisted only when the .arm is saved; nothing outside ArmorPaint can read them");
		json_encode_string("settle", "none: taken from the previously rendered frame, so the UI overlay state is whatever was last drawn");
		return json_encode_end();
	}
	else if (string_equals(op, "capture_viewport")) {
		if (!HAVE_VIEWPORT_PATCH) {
			// Do NOT attempt the call. An unregistered function is a minic
			// runtime error (minic.c:939) that aborts the whole handler, so
			// feature detection has to be a flag rather than a try.
			return fail("unsupported",
			            "viewport_save_texture_to_file is not in this build's binding table. Apply patch/apply_viewport_patch.py to an ArmorPaint checkout, rebuild, set HAVE_VIEWPORT_PATCH = 1 at the top of this plugin, and reload it. Stock fallbacks: capture_to_project (in-project only) or export_textures (real files, but flat textures rather than the shaded view).");
		}
		if (pr == NULL) {
			return fail("no_project", "viewport capture needs an open project");
		}
		s1 = arg(m, "path");
		if (s1 == NULL) {
			return fail("bad_args", "missing 'path'");
		}
		if (!path_ok(s1)) {
			return fail("bad_args", "'path' is empty, over-long, or contains a quote or newline");
		}
		i1 = arg_i(m, "width", 1024);
		i2 = arg_i(m, "height", 1024);
		if (i1 < 16) {
			return fail("bad_args", "width must be 16..4096");
		}
		if (i1 > 4096) {
			return fail("bad_args", "width must be 16..4096");
		}
		if (i2 < 16) {
			return fail("bad_args", "height must be 16..4096");
		}
		if (i2 > 4096) {
			return fail("bad_args", "height must be 16..4096");
		}
		// Reuse the cached target; only a size change reallocates. See the
		// capture_tex declaration for why this cannot be freed instead.
		if (capture_tex == NULL || capture_w != i1 || capture_h != i2) {
			capture_tex = gpu_create_render_target(i1, i2, GPU_TEXTURE_FORMAT_RGBA32);
			capture_w   = i1;
			capture_h   = i2;
		}
		if (capture_tex == NULL) {
			capture_w = 0;
			capture_h = 0;
			return fail("internal", "gpu_create_render_target returned null");
		}
		viewport_capture_screenshot_to(capture_tex, 0.0, 0.0, i1, i2);
		viewport_save_texture_to_file(capture_tex, s1);
		if (c != NULL) {
			c->capturing_screenshot = 0;
			c->ddirty               = 2;
		}
		json_encode_begin();
		json_encode_string("path", jesc(s1));
		json_encode_i32("width", i1);
		json_encode_i32("height", i2);
		json_encode_bool("exists", file_here(s1));
		return json_encode_end();
	}

	return fail("unsupported", string("unknown op: %s", op));
}

// ---------------------------------------------------------------------------
// Request lifecycle
// ---------------------------------------------------------------------------
void handle_one(char *name) {
	int nlen = string_length(name);
	if (nlen < 6) {
		return; // ".json" with no id at all; substring below would go negative
	}
	char *id   = substring(name, 0, nlen - 5); // strip ".json"
	char *path = string("%s/%s", dir_req, name);

	// The id becomes the reply filename, so it is validated before anything
	// else. An unusable one is deleted rather than retried forever.
	if (!id_ok(id)) {
		del_file(path);
		err_count  = err_count + 1;
		last_error = string("bad_args: unusable request id in %s", name);
		return;
	}

	buffer_t *blob = data_get_blob(path);
	if (blob == NULL) {
		// A miss is cached as a NULL entry, which data_delete_blob will not
		// remove -- harmless, since a NULL entry reads back as a miss and the
		// file is reloaded next time.
		del_file(path);
		err_count  = err_count + 1;
		last_error = string("internal: could not read %s", name);
		return;
	}

	// Copy the bytes out FIRST. This ordering is not stylistic: data_delete_blob
	// frees blob->buffer and blob itself (engine.c:2000), so converting after the
	// eviction would read freed memory.
	char *text = sys_buffer_to_string(blob);

	// MANDATORY. data_get_blob memoises by path in data_cached_blobs
	// (engine.c:1879) with no expiry. Skip this and the bridge answers the first
	// request seen at a given filename correctly and then silently replays those
	// same bytes for every later request that reuses the name.
	data_delete_blob(path);

	// Consume BEFORE executing. If a handler ever took the app down, a request
	// still sitting on disk would be replayed into the user's project on the next
	// start -- re-running a destructive op they never asked for twice.
	del_file(path);

	req_count = req_count + 1;
	last_id   = id;

	// Screen before parsing: a malformed body would crash the host parser, not
	// merely fail. See json_sane().
	int sane = json_sane(text);
	if (sane < 0) {
		reply(id, 0, fail("bad_args", "request contains a JSON array; json_parse_to_map corrupts on arrays -- send list args as a delimited string"), 0);
		return;
	}
	if (sane < 1) {
		reply(id, 0, fail("bad_args", "body is not a well-formed compact JSON object (<= 8192 bytes)"), 0);
		return;
	}

	// json_parse_to_map flattens nested objects, skips arrays and returns every
	// value as a char*. See the header comment for what the server must send.
	void *m = json_parse_to_map(text);

	float t0 = sys_time();
	busy     = 1;
	// Publish busy=1 before dispatching. Handlers run inline on the render
	// thread, so a slow one (export, bake) blocks for its whole duration and
	// this is the only way the server can tell "working" from "wedged".
	write_heartbeat();

	r_ok        = 1;
	char *inner = dispatch(m);

	// A minic runtime error inside dispatch -- a call this build does not export,
	// a parse error in an arm -- aborts THAT function and returns 0 without
	// touching this one, because every call gets a fresh env (minic.c:823).
	// Without this guard the reply body would contain a literal `null` where the
	// result object belongs and r_ok would still say true. Turn it into an error
	// the caller can act on; the real message is already in the console.
	if (inner == NULL) {
		r_ok       = 0;
		err_count  = err_count + 1;
		last_error = "internal: handler aborted";
		inner      = "{\"code\":\"internal\",\"message\":\"the handler aborted; look in the ArmorPaint console for a '<plugin>.c:<line>: error:' line\"}";
	}

	busy   = 0;
	int ms = (sys_time() - t0) * 1000.0;
	reply(id, r_ok, inner, ms);
	write_heartbeat(); // clear busy promptly rather than waiting for the 1 Hz tick
}

void on_update() {
	// The enable check is the ONLY statement permitted above the idle-sleep
	// reset. A disabled bridge deliberately lets ArmorPaint fall asleep again --
	// that is the whole point of the toggle, since staying awake costs full-rate
	// rendering forever.
	if (!enabled) {
		return;
	}

	// FIRST statement of the live path, and nothing may be inserted above it.
	// base_update() returns before iron_update() -- which dispatches this
	// callback -- once paused_frames exceeds 3 (Windows background) or 120
	// (idle). BOTH gates read that same counter and this call zeroes it
	// (iron.h:1011), so a per-frame reset holds both open and live control of an
	// unfocused ArmorPaint works. The background gate tolerates only THREE
	// consecutive missed frames, so putting the poll throttle above this would
	// let the app sleep and the bridge would go permanently deaf.
	iron_delay_idle_sleep();

	float dt = sys_real_delta();

	hb_accum = hb_accum + dt;
	if (hb_accum >= 1.0) {
		hb_accum = 0.0;
		write_heartbeat();
	}

	poll_accum = poll_accum + dt;
	if (poll_accum < poll_interval) {
		return;
	}
	poll_accum = 0.0;

	// Cheap pre-filter: iron_read_directory fills a shared static buffer and
	// allocates nothing, so an idle poll costs one directory scan and one strstr.
	// Only when a .json is actually present do we pay for the array listing,
	// whose per-entry strings are never freed.
	char *listing = iron_read_directory(dir_req);
	if (listing == NULL) {
		return;
	}
	if (string_index_of(listing, ".json") < 0) {
		return;
	}

	any_array_t *files = file_read_directory(dir_req);
	if (files == NULL) {
		return;
	}
	int   fn   = files->length;
	char *pick = NULL;
	char *nm;
	for (int fi = 0; fi < fn; ++fi) {
		nm = files->buffer[fi];
		if (nm != NULL) {
			if (ends_with(nm, ".json")) {
				pick = nm;
				break;
			}
		}
	}
	// pick points at a string_split-allocated name, not into this buffer, so it
	// stays valid after the array header and buffer are released.
	array_free(files);
	free(files);

	if (pick == NULL) {
		return;
	}

	// AT MOST ONE request per frame. This is a memory-safety rule, not only a
	// latency one: every script call burns ~29 KB of the 8 MB context arena and
	// the arena is rewound only here, at the host->script boundary, giving about
	// 280 calls per frame. Draining a backlog in one frame would overflow it and
	// corrupt the heap, because minic_alloc has no bounds check.
	handle_one(pick);
}

// Drawn ONLY while the Plugins tab is visible (tab_plugins.c:27), so it can show
// state but must never be the only way to observe it -- everything here is also
// in heartbeat.json and get_app_info.
void on_ui() {
	if (ui_panel(&panel_open, "MCP Bridge", 0, 0, 0)) {
		// The checkbox owns the flag. bridge_set_enabled writes enabled_ui too,
		// so the widget and the server never disagree.
		ui_check(&enabled_ui, "Enabled", "");
		enabled = enabled_ui;
		if (enabled) {
			ui_text("status: listening", UI_ALIGN_LEFT, 0);
		}
		else {
			ui_text("status: stopped, app may sleep", UI_ALIGN_LEFT, 0);
		}
		// Everything below draws a PRE-BUILT string -- no string() at frame rate.
		ui_text(ui_stats, UI_ALIGN_LEFT, 0);
		ui_row2();
		ui_text("last op", UI_ALIGN_LEFT, 0);
		ui_text(last_op, UI_ALIGN_LEFT, 0);
		ui_row2();
		ui_text("last id", UI_ALIGN_LEFT, 0);
		ui_text(last_id, UI_ALIGN_LEFT, 0);
		ui_text("last error", UI_ALIGN_LEFT, 0);
		ui_text(last_error, UI_ALIGN_LEFT, 0);
		ui_separator(4, 0);
		ui_text("spool", UI_ALIGN_LEFT, 0);
		ui_text(spool_root, UI_ALIGN_LEFT, 0);
		ui_text(ui_version, UI_ALIGN_LEFT, 0);
		if (ui_button("Open spool folder", UI_ALIGN_CENTER, "")) {
			file_start(spool_root);
		}
		if (ui_button("Clear error", UI_ALIGN_CENTER, "")) {
			last_error = "-";
		}
	}
}

void on_delete() {
	enabled = 0;
	busy    = 0;
	// Leave a final heartbeat saying so, so the server reports "bridge stopped"
	// rather than "bridge crashed" when t stops advancing.
	write_heartbeat();
	del_file(path_lock);
}

// main MUST be last: minic's registration pass stops dead at main (minic.c:2051)
// and any function defined below it is never registered.
void main() {
	plugin       = plugin_create();
	enabled_ui   = enabled;

	// data_path() is "./data" plus the platform separator, which is a backslash
	// on Windows. Normalise it: backslashes inside minic string literals are
	// escape sequences and any unrecognised one truncates the string at that
	// point, so every path this plugin handles stays forward-slashed.
	char *base = string_replace_all(data_path(), "\\", "/");
	spool_root = string("%smcp_spool", base);
	dir_req    = string("%s/req", spool_root);
	dir_res    = string("%s/res", spool_root);

	// Created parent-first: iron_create_directory does not build intermediates.
	iron_create_directory(spool_root);
	iron_create_directory(dir_req);
	iron_create_directory(dir_res);

	path_heartbeat = string("%s/heartbeat.json", spool_root);
	path_lock      = string("%s/bridge.lock", spool_root);
	ui_version     = string("bridge %s", BRIDGE_VERSION);

	// Drain anything still sitting in req/ from a previous run.
	//
	// The server refuses to write a request unless it can already see a live
	// heartbeat, so nothing found here at start can belong to a live session --
	// these are requests whose server was killed before its own timeout could
	// unlink them. Without this they would be executed now, replaying a
	// destructive op nobody asked for (project_open discarding unsaved work,
	// say) minutes or days after it was abandoned.
	any_array_t *stale = file_read_directory(dir_req);
	if (stale != NULL) {
		int   sn = stale->length;
		char *snm;
		for (int si = 0; si < sn; ++si) {
			snm = stale->buffer[si];
			if (snm != NULL) {
				if (ends_with(snm, ".json")) {
					del_file(string("%s/%s", dir_req, snm));
				}
			}
		}
		array_free(stale);
		free(stale);
	}

	json_encode_begin();
	json_encode_i32("v", ENVELOPE_V);
	json_encode_string("bridge_version", BRIDGE_VERSION);
	json_encode_string("spool", jesc(spool_root));
	json_encode_f32("started_t", sys_time());
	iron_file_save_bytes(path_lock, sys_string_to_buffer(json_encode_end()), 0);

	write_heartbeat();

	// The spool path is relative to ArmorPaint's working directory, so log it:
	// this line is how a user tells the server where to look.
	console_info(string("armorpaint-mcp bridge %s listening on %s", BRIDGE_VERSION, spool_root));

	plugin_notify_on_ui(plugin, on_ui);
	plugin_notify_on_update(plugin, on_update);
	plugin_notify_on_delete(plugin, on_delete);
}
