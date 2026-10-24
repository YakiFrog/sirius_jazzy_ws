#!/usr/bin/env python3
"""SiriusLauncher ブラウザ版（遠隔/Tailscale対応）。

デスクトップ版(sirius_launcher.py)と同じエイリアス群(bash_alias2.sh)を読み、
ブラウザから 起動 / 停止 / 状態確認 / プリセット / ログ閲覧 ができる。
Home / Terminal / Monitor / 設定を切り替える専用コンソール。
ホスト監視は共有キャッシュから読み出し、既存の起動・対話入力経路を使う。

起動:
  python3 web_server.py --port 8090 [--password xxx]
アクセス:
  http://<このPCのTailscale IP>:8090
"""
import argparse
import codecs
from functools import lru_cache
import os
import pty
import re
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import psutil
from flask import Flask, jsonify, request, make_response

from alias_parser import parse_bash_aliases
from process_manager import ProcessManager
from command_session import session_request
from resource_monitor import ResourceMonitor
from tf_monitor import TFMonitor

RESOURCES = ResourceMonitor()
TF_MONITOR = TFMonitor()

app = Flask(__name__)
LOCK = threading.Lock()
MANAGERS = {}
BUTTONS = {}
GROUPS = []
PRESETS = []
ALIAS_FILE = ""
PASSWORD = ""
DEFAULT_DOMAIN = ""
HOME = os.path.expanduser("~")
RESTART_EXIT_CODE = 99


def log_path(name):
    # デスクトップ版(process_manager.py)と共有: /tmp/sirius_launcher_<name>.log
    return f"/tmp/sirius_launcher_{name}.log"


def pid_path(name):
    # デスクトップ版(process_manager.py)と共有: /tmp/sirius_launcher_<name>.pid
    return f"/tmp/sirius_launcher_{name}.pid"


ERR_KEYWORDS = ("[ERROR]", "[FATAL]", "Traceback (most recent call last):")


def has_error(name):
    try:
        with open(log_path(name), "r", errors="ignore") as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(max(0, size - 65536))
            content = f.read()
        return any(k in content for k in ERR_KEYWORDS)
    except FileNotFoundError:
        return False


TERMS = {}
TERMS_LOCK = threading.Lock()
TERM_MAX_CHARS = 200000

ANSI_RE = re.compile(
    r"\x1b(?:\[[0-9;?]*[ -/]*[@-~]"      # CSI (色・カーソル・bracketed paste等)
    r"|\][^\x07\x1b]*(?:\x07|\x1b\\)"     # OSC (タイトル等)
    r"|[@-Z\\-_])"                        # 2文字エスケープ
)


def clean_term_text(s):
    s = ANSI_RE.sub("", s)
    s = s.replace("\r\n", "\n").replace("\r", "\n")
    return s


class TeleopBridge:
    """十字キーパッド → /controller (ControllerInput) のブリッジ。

    rclpyヘルパー(teleop_bridge.py)を遅延起動し、stdinパイプにキーコードを流す。
    sirius_controller_node（scnエイリアス）が起動中ならリモート操作できる。
    """

    KEYS = {65, 66, 67, 68, 44, 46}  # 矢印4種 + 並進停止(,) + 旋回停止(.)

    def __init__(self, script, log_path):
        self.script = script
        self.log_path = log_path
        self.lock = threading.Lock()
        self.proc = None
        self.domain = None
        self.error = ""

    def _tail(self):
        try:
            with open(self.log_path, "r", errors="ignore") as f:
                f.seek(0, os.SEEK_END)
                size = f.tell()
                f.seek(max(0, size - 2000))
                return f.read().strip().replace("\n", " | ")[-300:]
        except OSError:
            return "(ログなし)"

    def _spawn(self, domain):
        ws = Path(self.script).resolve().parents[2]
        cmd = (
            "source /opt/ros/jazzy/setup.bash 2>/dev/null; "
            f"source '{ws}/install/setup.bash' 2>/dev/null; "
            f"exec python3 '{self.script}'"
        )
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(domain)
        env.setdefault("LANG", "C.UTF-8")
        logf = open(self.log_path, "ab", buffering=0)
        self.proc = subprocess.Popen(
            ["bash", "-c", cmd], stdin=subprocess.PIPE,
            stdout=logf, stderr=subprocess.STDOUT, env=env)
        self.domain = str(domain)
        self.error = ""
        time.sleep(0.3)
        if self.proc.poll() is not None:
            self.error = "ブリッジ起動失敗: " + self._tail()
            raise RuntimeError(self.error)

    def ensure(self, domain):
        with self.lock:
            if (self.proc is None or self.proc.poll() is not None
                    or self.domain != str(domain)):
                try:
                    self._spawn(str(domain))
                except Exception as e:  # noqa: BLE001
                    self.error = str(e)
                    return False
            return True

    def send(self, key, domain):
        with self.lock:
            if (self.proc is None or self.proc.poll() is not None
                    or self.domain != str(domain)):
                try:
                    self._spawn(str(domain))
                except Exception as e:  # noqa: BLE001
                    self.error = str(e)
                    return False
            try:
                self.proc.stdin.write(f"{key}\n".encode())
                self.proc.stdin.flush()
                self.error = ""
                return True
            except (BrokenPipeError, OSError):
                self.error = "ブリッジに書き込めません（ログ参照）"
                return False

    def status(self):
        with self.lock:
            alive = self.proc is not None and self.proc.poll() is None
            return {"alive": alive, "pid": (self.proc.pid if alive else 0),
                    "domain": self.domain or "", "error": self.error}


TELEOP = TeleopBridge(
    str(Path(__file__).resolve().parent / "teleop_bridge.py"),
    "/tmp/sirius_launcher_teleop_bridge.log",
)


class TermSession:
    """ブラウザのターミナルタブ（PTY）。対話シェル/任意コマンドを実行する。

    出力はANSIエスケープを除去して保存する（<pre>表示用）。オフセットは
    生バイト列基準の単調増加値で、切り詰め後も再送・欠落しない。
    """

    def __init__(self, name, cmd="", cwd=None):
        self.name = name
        self.text = ""       # raw (エスケープ含む)
        self.total = 0       # これまでに受信したraw文字数の累計（オフセット基準）
        self.alive = True
        self.lock = threading.Lock()
        self.decoder = codecs.getincrementaldecoder("utf-8")("replace")
        env = os.environ.copy()
        env["TERM"] = "xterm-256color"
        env.setdefault("LANG", "C.UTF-8")
        pid, fd = pty.fork()
        if pid == 0:  # child
            try:
                if cwd:
                    os.chdir(cwd)
                if cmd:
                    os.execvpe("bash", ["bash", "-lc", cmd], env)
                os.execvpe("bash", ["bash", "-i"], env)
            except Exception:  # noqa: BLE001
                os._exit(127)
        self.pid = pid
        self.fd = fd
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def _read_loop(self):
        while True:
            try:
                data = os.read(self.fd, 4096)
            except OSError:
                break
            if not data:
                break
            chunk = self.decoder.decode(data)
            if not chunk:
                continue
            with self.lock:
                self.text += chunk
                self.total += len(chunk)
                if len(self.text) > TERM_MAX_CHARS:
                    self.text = self.text[-TERM_MAX_CHARS:]
        self.alive = False

    def read_since(self, offset):
        with self.lock:
            base = self.total - len(self.text)
            if offset < base or offset > self.total:
                offset = max(base, self.total - 20000)
            raw = self.text[offset - base:]
            return clean_term_text(raw), self.total

    def write(self, data):
        try:
            os.write(self.fd, data.encode("utf-8", "replace"))
            return True
        except OSError:
            return False

    def close(self):
        try:
            os.killpg(os.getpgid(self.pid), signal.SIGTERM)
        except OSError:
            try:
                os.kill(self.pid, signal.SIGTERM)
            except OSError:
                pass
        try:
            os.close(self.fd)
        except OSError:
            pass
        self.alive = False


def alias_signature(cmd):
    """エイリアスのコマンドから「同一プログラム」を識別する短い署名を作る。

    例: 'ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8766'
        -> 'ros2 launch foxglove_bridge foxglove_bridge_launch.xml'
        'bash .../start_3d_lio.sh sim' -> 'start_3d_lio.sh'
    """
    s = cmd.strip()
    s = re.sub(r"^cd [^&]+&&\s*", "", s)
    s = re.sub(r"^source [^&]+&&\s*", "", s)
    s = s.replace("src && ", "")
    s = re.sub(r"^bash\s+", "", s)
    toks = s.split()
    for i, t in enumerate(toks):
        if i > 0 and toks[i - 1] == "ros2" and t in ("launch", "run"):
            parts = toks[i + 1:i + 3]
            return " ".join(["ros2", t] + parts)
    m = re.search(r"([\w.\-]+\.(?:sh|py))", s)
    if m:
        return m.group(1)
    last = s.split("&&")[-1].strip()
    lt = last.split()
    return " ".join(lt[:2]) if lt else ""


_SIG_CACHE = {}
_VAR_CACHE = {}


def alias_signatures():
    """エイリアス名 -> 基本署名（同一プログラム判定・二重起動防止用）。"""
    global _SIG_CACHE, _VAR_CACHE
    if not _SIG_CACHE:
        for n, c in BUTTONS.items():
            base = alias_signature(c)
            var = base
            m = re.search(r"\b(sim|real)\b", c)
            if base and m and not base.startswith('ros2 launch '):
                var = base + " " + m.group(1)
            _SIG_CACHE[n] = base
            _VAR_CACHE[n] = var
    return _SIG_CACHE


def alias_variant_signatures():
    """エイリアス名 -> 詳細署名（sim/realを区別。表示の特定用）。"""
    alias_signatures()
    return _VAR_CACHE


def best_alias_for(cmd):
    """cmdに一致する最も具体的な署名を持つエイリアス名を返す。"""
    best = ""
    best_len = 0
    actual_params = launch_parameters(cmd)
    for n, var in alias_variant_signatures().items():
        if not var or len(var) < 6 or var not in cmd:
            continue
        expected = launch_parameters(BUTTONS[n]) if var.startswith('ros2 launch ') else {}
        actual = actual_params
        if var == 'ros2 launch sirius_description display.launch.py':
            # This launch starts RViz by default; a TF-only process must never
            # be attributed to an alias which expects that default RViz window.
            expected = {'rviz': 'true', **expected}
            actual = {'rviz': 'true', **actual_params}
        if any(actual.get(key) != value for key, value in expected.items()):
            continue
        # TF-only and RViz launches share a launch file. Explicit parameters
        # identify the variant regardless of their order in the process argv.
        specificity = len(var) + sum(len(key)+len(value)+3 for key, value in expected.items())
        if specificity > best_len:
            best, best_len = n, specificity
    return best


@lru_cache(maxsize=1024)
def launch_parameters(command):
    """Literal ROS launch arguments; shell-expanded values cannot identify a variant."""
    return {key: value.strip("\"'")
            for key, value in re.findall(r'(?<!\S)([\w/.-]+):=([^\s]+)', command)
            if '$' not in value and '~' not in value}


def process_matches_alias(name, command):
    """Use the same identity for display, duplicate detection and external stop."""
    matched = best_alias_for(command)
    return bool(matched and (matched == name or BUTTONS.get(matched) == BUTTONS.get(name)))


# コマンド文字列に現れない固定ポート（launch既定値など）。このポートは
# そのプログラム専用なので、占有されていれば古いプロセスを停止してよい。
KNOWN_PORTS = {
    "foxglove": [8766],
    "rosbridge": [9090],
    "sirius_launcher_web": [8090],
}


def alias_ports(cmd, name=""):
    """エイリアスが使うポート番号（コマンド内の明示 + 既知マップ）。"""
    ports = set(KNOWN_PORTS.get(name, []))
    for m in re.finditer(r"(?:port[:=]\s*|--port[= ]\s*)(\d{2,5})", cmd or ""):
        ports.add(int(m.group(1)))
    return sorted(ports)


def _self_ancestors():
    """自分自身の祖先PID集合（Terminatorやbash等。停止対象から除外する）。"""
    pids = set()
    try:
        p = psutil.Process(os.getpid())
        while True:
            p = p.parent()
            if p is None:
                break
            pids.add(p.pid)
    except psutil.Error:
        pass
    return pids


def listening_pids(port):
    """portをLISTENしているPID一覧。"""
    pids = set()
    try:
        for c in psutil.net_connections(kind="inet"):
            if (c.status == psutil.CONN_LISTEN and c.laddr
                    and getattr(c.laddr, "port", None) == port and c.pid):
                pids.add(c.pid)
    except (psutil.AccessDenied, OSError):
        try:
            out = subprocess.check_output(
                ["ss", "-ltnp", f"sport = :{port}"], text=True, stderr=subprocess.DEVNULL)
            for m in re.finditer(r"pid=(\d+)", out):
                pids.add(int(m.group(1)))
        except Exception:  # noqa: BLE001
            pass
    return pids


def proc_rows():
    """対象プロセス（ROS/sirius関連）を pid/cmd/age で列挙する。"""
    keywords = ("ros2", "/opt/ros/", "sirius", "fast_lio", "fastlio", "laser_mapping",
                "robot_state_publisher", "gz ", "gazebo", "rviz", "terminator",
                "nav2", "slam_toolbox", "rosbridge", "foxglove", "ros_tcp", "theta")
    out = []
    me = os.getpid()
    now = time.time()
    for p in psutil.process_iter(["pid", "cmdline", "create_time"]):
        try:
            pid = p.info["pid"]
            if pid == me:
                continue
            cmdline = p.info["cmdline"] or []
            if not cmdline:
                continue
            joined = " ".join(cmdline)
            if "sirius_launcher" in joined or "web_server.py" in joined:
                continue
            if not any(k in joined for k in keywords):
                continue
            age = int(now - (p.info["create_time"] or now))
            out.append({"pid": pid, "cmd": joined[:300], "age": age})
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    out.sort(key=lambda d: d["pid"])
    return out


def annotate_rows(rows):
    """各行にエイリアス名・年齢表示・重複/エラーフラグを付ける。"""
    for r in rows:
        r["alias"] = best_alias_for(r["cmd"])
        r["error"] = has_error(r["alias"]) if r["alias"] else False
        a = r["age"]
        r["age_txt"] = f"{a // 3600}h{(a % 3600) // 60}m" if a >= 3600 else f"{a // 60}m{a % 60}s"
    groups = {}
    for r in rows:
        if r["alias"]:
            groups.setdefault(r["alias"], []).append(r)
    dups = []
    for alias, rs in groups.items():
        if len(rs) > 1:
            for r in rs:
                r["dup"] = True
            dups.append({"alias": alias, "count": len(rs), "pids": [x["pid"] for x in rs]})
    dups.sort(key=lambda d: d["count"], reverse=True)
    return rows, dups


def external_matches():
    """外部起動プロセスをエイリアスへ対応付ける（エイリアス名 -> [pid]）。"""
    res = {}
    for r in proc_rows():
        n = best_alias_for(r["cmd"])
        if n:
            res.setdefault(n, []).append(r["pid"])
    return res


def ps_snapshot():
    rows = proc_rows()
    return annotate_rows(rows)


def save_presets_to_file(path, presets, backup=True):
    """bash_alias2.sh の先頭プリセット領域のみ書き換える（以降は保持）。"""
    with open(path, encoding="utf-8") as f:
        lines = f.read().split("\n")
    head_index = len(lines)
    for i, line in enumerate(lines):
        s = line.strip()
        if s.startswith("# TAB:") or s.startswith("# GROUP:"):
            head_index = i
            break
    tail = lines[head_index:]
    block = []
    for name, items in presets:
        block.append(f"# PRESET: {name}")
        block.append(f"# PRESET_ITEMS: {','.join(items)}")
        block.append("")
    if backup:
        import shutil
        shutil.copy(path, path + ".bak")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(block + tail))


def load_aliases(path):
    global BUTTONS, GROUPS, PRESETS
    groups, presets, descs = parse_bash_aliases(path, include_group_descriptions=True)
    BUTTONS = {}
    _SIG_CACHE.clear()
    _VAR_CACHE.clear()
    GROUPS = []
    for gname, items in groups.items():
        its = []
        for it in items:
            name, cmd, desc = it[0], it[1], it[2]
            sub = it[3] if len(it) > 3 else ""
            its.append({"name": name, "desc": desc, "subgroup": sub})
            BUTTONS[name] = cmd
        GROUPS.append({"name": gname, "desc": descs.get(gname, ""), "items": its})
    PRESETS = [{"name": n, "items": i} for n, i in presets]


def get_manager(name):
    """エイリアス名ごとの ProcessManager を取得（アプリ版と同じ起動方式を共有）。"""
    m = MANAGERS.get(name)
    if m is None:
        m = ProcessManager(name, BUTTONS.get(name, ""))
        MANAGERS[name] = m
    return m


def running_pid(name):
    """Web版・アプリ版どちらが起動したものでも、稼働中ならPIDを返す。"""
    m = get_manager(name)
    if m.is_running():
        return m.pid_file_content
    return None


def is_running(name):
    return get_manager(name).is_running()


def scan_pid_files():
    """アプリ版/Web版が起動したプロセスを拾うため /tmp のPIDファイルを走査。"""
    names = set()
    prefix = "sirius_launcher_"
    suffix = ".pid"
    try:
        for fn in os.listdir("/tmp"):
            if fn.startswith(prefix) and fn.endswith(suffix):
                names.add(fn[len(prefix):-len(suffix)])
    except OSError:
        pass
    return names


def alias_actually_running(name):
    """pidは生きているが中身が別物/残骸、という偽「起動中」を除外して判定する。"""
    m = get_manager(name)
    if not m.is_running():
        return False
    sig = alias_signature(BUTTONS.get(name, ""))
    if not sig or len(sig) < 6:
        return True
    return any(process_matches_alias(name, row["cmd"]) for row in proc_rows())


def start_one(name, domain=""):
    if name not in BUTTONS:
        return False, "unknown alias"
    m = get_manager(name)
    if m.is_running():
        if alias_actually_running(name):
            return True, "already running"
        # 古いPIDファイルが残っているだけ（プロセスは別物/残骸）→掃除して起動する
        print(f"古いPIDファイルを掃除: {name}")
        try:
            if os.path.exists(m.pid_file):
                os.remove(m.pid_file)
        except OSError:
            pass
        m.pid_file_content = None
    # 外部起動も含めて同一プログラムが動いていれば二重起動しない
    sig = alias_signature(BUTTONS.get(name, ""))
    if sig and len(sig) >= 6:
        for row in proc_rows():
            if process_matches_alias(name, row["cmd"]):
                return True, f"already running (external pid {row['pid']})"
    # 専用ポートが使用中なら古いプロセスを停止してから起動する（例: foxgloveの8766）
    ports = alias_ports(BUTTONS.get(name, ""), name)
    if ports:
        skip = _self_ancestors() | {os.getpid()}
        for port in ports:
            for pid in listening_pids(port):
                if pid in skip:
                    continue
                print(f"ポート競合解消: port {port} の PID {pid} を停止")
                if _kill_pid_tree(pid):
                    time.sleep(0.4)
    use_domain = str(domain).strip() or str(DEFAULT_DOMAIN).strip()
    ok = m.launch_web(use_domain)
    if not ok:
        return False, "launch failed"
    return True, "started"


def stop_one(name):
    m = get_manager(name)
    if not m.is_running():
        # 外部起動分（ランチャー経由でない）は署名一致で停止する
        sig = alias_signature(BUTTONS.get(name, ""))
        if sig and len(sig) >= 6:
            targets = [r["pid"] for r in proc_rows() if process_matches_alias(name, r["cmd"])]
            if targets:
                for pid in targets:
                    _kill_pid_tree(pid)
                return True, "stopped (external)"
        if os.path.exists(m.pid_file):
            try:
                os.remove(m.pid_file)
            except OSError:
                pass
        return False, "not started"
    result = m.stop()
    if result:
        # アプリ版と同様に、少し待ってから強制終了しタブを閉じる
        time.sleep(1.5)
        m.force_kill_tree(result["pids"], result["terminator_pid"])
        m.close_terminator_tab(result["terminator_pid"])
    return True, "stopped"


def auth_ok():
    if not PASSWORD:
        return True
    return request.cookies.get("pw") == PASSWORD or request.args.get("pw") == PASSWORD


def render_page():
    return (Path(__file__).with_name('launcher.html')).read_text(encoding='utf-8')


def render_shell():
    return (Path(__file__).with_name('shell.html')).read_text(encoding='utf-8')


PAGE = render_page()

LOGIN = """<!doctype html><html><head><meta charset="utf-8"><title>login</title></head>
<body style="font-family:sans-serif"><h3>SiriusLauncher Web ログイン</h3>
<form method="get" action="/console"><input name="pw" type="password" placeholder="password">
<button>ログイン</button></form></body></html>"""


@app.route("/")
def index():
    resp = make_response(render_shell())
    if PASSWORD and request.args.get("pw") == PASSWORD:
        resp.set_cookie("pw", PASSWORD, max_age=7 * 24 * 3600)
    resp.headers["Cache-Control"] = "no-store"
    return resp


@app.route("/console")
def console():
    if PASSWORD and request.args.get("pw") == PASSWORD:
        resp = make_response(render_page())
        resp.set_cookie("pw", PASSWORD, max_age=7 * 24 * 3600)
    elif auth_ok():
        resp = make_response(render_page())
    else:
        return LOGIN
    resp.headers["Cache-Control"] = "no-store"
    return resp


@app.route("/api/health")
def api_health():
    return jsonify({"service": "sirius-launcher", "ui_version": "console-2"})


@app.route("/api/state")
def api_state():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    return jsonify({"groups": GROUPS, "presets": PRESETS, "default_domain": DEFAULT_DOMAIN,
                    "console": {"autostart": (Path(os.environ.get("XDG_CONFIG_HOME") or str(Path.home()/".config"))/"autostart/sirius-console.desktop").exists()}})


@app.route("/api/status")
def api_status():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    out = {}
    with LOCK:
        for name in set(MANAGERS) | scan_pid_files():
            pid = running_pid(name)
            if pid is None:
                continue
            out[name] = {"running": True, "pid": pid, "returncode": None,
                         "error": has_error(name), "domain": "", "source": "launcher"}
        # 外部（別ターミナル等）で起動済みのものを署名一致で拾う
        for name, pids in external_matches().items():
            if name not in out:
                out[name] = {"running": True, "pid": pids[0], "returncode": None,
                             "error": has_error(name), "domain": "",
                             "source": "external", "pids": pids}
    return jsonify(out)


@app.route("/api/start", methods=["POST"])
def api_start():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    body = request.json or {}
    name = body.get("name", "")
    domain = body.get("domain", "")
    with LOCK:
        ok, msg = start_one(name, domain)
    return jsonify({} if ok else {"error": msg})


@app.route("/api/stop", methods=["POST"])
def api_stop():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    name = (request.json or {}).get("name", "")
    with LOCK:
        ok, msg = stop_one(name)
    return jsonify({} if ok else {"error": msg})


@app.route("/api/stop_all", methods=["POST"])
def api_stop_all():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    with LOCK:
        for name in list(MANAGERS):
            stop_one(name)
    return jsonify({})


@app.route("/api/restart_server", methods=["POST", "GET"])
def api_restart_server():
    """Webサーバー自身を再起動する。

    起動ラッパー(スーパーバイザ)経由なら exit code 99 で終了して再起動させる。
    直接起動されている場合は execv で自己置換を試みる。
    """
    if not auth_ok():
        return jsonify({"error": "auth"}), 401

    supervised = os.environ.get("SIRIUS_WEB_SUPERVISED") == "1"
    argv = list(sys.argv)

    def _restart():
        time.sleep(0.5)
        if supervised:
            os._exit(RESTART_EXIT_CODE)
        try:
            os.execv(sys.executable, [sys.executable] + argv)
        except Exception as error:  # noqa: BLE001
            print(f"自己再起動に失敗: {error}")

    threading.Thread(target=_restart, daemon=True).start()
    return jsonify({"restarting": True})


@app.route("/api/preset", methods=["POST"])
def api_preset():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    name = (request.json or {}).get("name", "")
    domain = (request.json or {}).get("domain", "")
    items = next((p["items"] for p in PRESETS if p["name"] == name), None)
    if items is None:
        return jsonify({"error": "unknown preset"}), 404
    with LOCK:
        for n in items:
            start_one(n, domain)
            time.sleep(0.5)
    return jsonify({})


@app.route("/api/restart", methods=["POST"])
def api_restart():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    body = request.json or {}
    name, domain = body.get("name", ""), body.get("domain", "")
    with LOCK:
        stop_one(name)
        time.sleep(0.4)
        ok, msg = start_one(name, domain)
    return jsonify({} if ok else {"error": msg})


@app.route("/api/presets/save", methods=["POST"])
def api_presets_save():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    body = request.json or {}
    raw = body.get("presets", [])
    clean = []
    for p in raw:
        name = str(p.get("name", "")).strip()
        items = [a for a in p.get("items", []) if a in BUTTONS]
        if name and items:
            clean.append((name, items))
    try:
        with LOCK:
            save_presets_to_file(ALIAS_FILE, clean)
            load_aliases(ALIAS_FILE)
    except Exception as error:  # noqa: BLE001
        return jsonify({"error": str(error)}), 500
    return jsonify({"presets": PRESETS})


@app.route("/api/log")
def api_log():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    name = request.args.get("name", "")
    if name not in BUTTONS:
        return jsonify({"error": "unknown alias"}), 404
    interactive = session_request(name).get("ok", False)
    try:
        with open(log_path(name), "r", errors="ignore") as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(max(0, size - 20000))
            return jsonify({"log": clean_term_text(f.read()), "interactive": interactive})
    except FileNotFoundError:
        return jsonify({"log": "(ログなし)", "interactive": interactive})


@app.route("/api/input", methods=["POST"])
def api_input():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    body = request.json or {}
    name, data = body.get("name", ""), body.get("data", "")
    if not isinstance(name, str) or name not in BUTTONS:
        return jsonify({"error": "unknown alias"}), 404
    if not isinstance(data, str) or len(data.encode("utf-8")) > 4096:
        return jsonify({"error": "入力は4096バイトまでです"}), 400
    result = session_request(name, data)
    return jsonify(result), 200 if result.get("ok") else 409


@app.route("/api/teleop", methods=["POST"])
def api_teleop():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    body = request.json or {}
    key = body.get("key")
    domain = str(body.get("domain") or DEFAULT_DOMAIN)
    if key == "stop":
        ok = TELEOP.send(44, domain)
        time.sleep(0.15)
        ok = TELEOP.send(46, domain) and ok
    elif isinstance(key, int) and not isinstance(key, bool) and key in TeleopBridge.KEYS:
        ok = TELEOP.send(key, domain)
    else:
        return jsonify({"error": "invalid key"}), 400
    st = TELEOP.status()
    st["ok"] = ok
    return jsonify(st), 200 if ok else 502


@app.route("/api/teleop/status")
def api_teleop_status():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    return jsonify(TELEOP.status())


@app.route("/api/tf")
def api_tf():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    response = jsonify(TF_MONITOR.snapshot(DEFAULT_DOMAIN))
    response.headers["Cache-Control"] = "no-store"
    return response


@app.route("/api/resources")
def api_resources():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    data = RESOURCES.snapshot()
    if data.get("available") and request.args.get("summary") == "1":
        data.pop("processes", None)
        data.pop("history", None)
    elif data.get("available"):
        data["processes"] = [{**row, "alias": best_alias_for(row["command"]) if row["related"] else None}
                             for row in data["processes"]]
    response = jsonify(data)
    response.headers["Cache-Control"] = "no-store"
    return response


@app.route("/api/ps")
def api_ps():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    rows, dups = ps_snapshot()
    return jsonify({"processes": rows, "duplicates": dups})


def _kill_pid_tree(pid):
    try:
        proc = psutil.Process(pid)
    except psutil.Error:
        return False
    try:
        procs = proc.children(recursive=True) + [proc]
        for pr in procs:
            try:
                pr.terminate()
            except psutil.Error:
                pass
        _, alive = psutil.wait_procs(procs, timeout=3)
        for pr in alive:
            try:
                pr.kill()
            except psutil.Error:
                pass
    except psutil.Error:
        return False
    return True


@app.route("/api/ps/kill", methods=["POST"])
def api_ps_kill():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    pid = int((request.json or {}).get("pid", 0))
    if not _kill_pid_tree(pid):
        return jsonify({"error": "no such process"}), 404
    return jsonify({})


@app.route("/api/ps/kill_many", methods=["POST"])
def api_ps_kill_many():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    pids = (request.json or {}).get("pids", [])
    killed = [int(p) for p in pids if _kill_pid_tree(int(p))]
    return jsonify({"killed": killed})


@app.route("/api/term/list")
def api_term_list():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    with TERMS_LOCK:
        return jsonify({"terms": [{"name": n, "alive": s.alive, "pid": s.pid}
                                  for n, s in TERMS.items()]})


@app.route("/api/term/open", methods=["POST"])
def api_term_open():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    body = request.json or {}
    cmd = str(body.get("cmd", "")).strip()
    name = str(body.get("name", "")).strip() or (
        ("cmd" if cmd else "shell") + "_" + str(time.time_ns()))
    with TERMS_LOCK:
        existing = TERMS.get(name)
        if existing and existing.alive:
            return jsonify({"error": "exists", "name": name}), 400
        TERMS[name] = TermSession(name, cmd)
    return jsonify({"name": name})


@app.route("/api/term/read")
def api_term_read():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    name = request.args.get("name", "")
    offset = int(request.args.get("offset", 0) or 0)
    with TERMS_LOCK:
        s = TERMS.get(name)
    if s is None:
        return jsonify({"error": "no such term"}), 404
    data, new_offset = s.read_since(offset)
    return jsonify({"data": data, "offset": new_offset, "alive": s.alive})


@app.route("/api/term/write", methods=["POST"])
def api_term_write():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    body = request.json or {}
    name = str(body.get("name", ""))
    data = str(body.get("data", ""))
    with TERMS_LOCK:
        s = TERMS.get(name)
    if s is None:
        return jsonify({"error": "no such term"}), 404
    return jsonify({"ok": s.write(data)})


@app.route("/api/term/close", methods=["POST"])
def api_term_close():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    name = str((request.json or {}).get("name", ""))
    with TERMS_LOCK:
        s = TERMS.pop(name, None)
    if s:
        s.close()
    return jsonify({})


def main():
    global ALIAS_FILE, PASSWORD, DEFAULT_DOMAIN
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--alias-file",
                    default=str(Path.home() / "sirius_jazzy_ws" / "bash" / "bash_alias2.sh"))
    ap.add_argument("--password", default="")
    ap.add_argument("--ros-domain", default=os.environ.get("ROS_DOMAIN_ID", "53"),
                    help="起動コマンドに渡すROS_DOMAIN_ID（既定: 環境変数 or 53）")
    args = ap.parse_args()
    ALIAS_FILE = args.alias_file
    PASSWORD = args.password
    DEFAULT_DOMAIN = args.ros_domain
    if not os.path.exists(ALIAS_FILE):
        print(f"エイリアスファイルがありません: {ALIAS_FILE}")
        return 1
    load_aliases(ALIAS_FILE)
    RESOURCES.start()
    threading.Thread(target=TELEOP.ensure, args=(DEFAULT_DOMAIN,), daemon=True).start()
    print(f"SiriusLauncher Web: http://{args.host}:{args.port}  "
          f"({len(BUTTONS)} aliases, {len(GROUPS)} groups, {len(PRESETS)} presets)")
    if PASSWORD:
        print("パスワード保護: 有効")
    app.run(host=args.host, port=args.port, threaded=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
