#!/usr/bin/env python3
"""SiriusLauncher ブラウザ版（遠隔/Tailscale対応）。

デスクトップ版(sirius_launcher.py)と同じエイリアス群(bash_alias2.sh)を読み、
ブラウザから 起動 / 停止 / 状態確認 / プリセット / ログ閲覧 ができる。
レイアウトはデスクトップ版に合わせる:
  左カラム = 上:プリセット / 下:起動中、右 = グループのタブ。

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


PAGE = """<!doctype html><html lang="ja"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SiriusLauncher Web</title>
<style>
*{box-sizing:border-box}
body{font-family:system-ui,sans-serif;margin:0;height:100vh;display:flex;flex-direction:column;background:#0f1115;color:#e6e6e6}
header{display:flex;align-items:center;gap:10px;padding:8px 14px;background:#1b2027}
header h1{font-size:15px;margin:0;flex:1}
button{cursor:pointer}
.hbtn{background:#232a33;color:#e6e6e6;border:1px solid #333;border-radius:6px;padding:6px 10px;font-size:12px}
.hbtn.stop{background:#5a1f1f;color:#fff;border:none}
.body{flex:1;display:flex;min-height:0}
.sidebar{width:340px;min-width:160px;display:flex;flex-direction:column;border-right:1px solid #24303a;min-height:0}
.gutter{flex:0 0 6px;background:#1b2027;cursor:col-resize}
.gutter:hover{background:#2f6fed}
.sidebar.closed+.gutter{display:none}
.panel{display:flex;flex-direction:column;min-height:0}
.panel.presets{flex:0 0 55%}
.panel.running{flex:1 1 auto}
.hgutter{flex:0 0 6px;background:#1b2027;cursor:row-resize}
.hgutter:hover{background:#2f6fed}
.ptitle{padding:6px 10px;font-weight:bold;font-size:13px;background:#141920;color:#cfd6df}
.plist{overflow:auto;padding:6px 8px;display:flex;flex-direction:column;gap:4px}
.preset{background:#3a2f6f;color:#fff;border-radius:6px;padding:7px 10px;font-size:13px;text-align:left;border:none}
.empty{color:#7f8a99;font-size:12px;padding:2px 12px 8px}
.ptitle{cursor:pointer;user-select:none;display:flex;align-items:center;gap:6px}
.ptitle .arr{transition:transform .15s;font-size:10px;color:#8a93a0}
.ptitle.closed .arr{transform:rotate(-90deg)}
.sec{margin:6px 0 10px}
.sec-h{display:flex;align-items:center;gap:6px;cursor:pointer;color:#9fb0c0;font-size:12px;border-bottom:1px solid #24303a;padding:4px 0;user-select:none}
.sec-h .arr{transition:transform .15s;font-size:10px;color:#8a93a0}
.sec-h.closed .arr{transform:rotate(-90deg)}
.hidden{display:none!important}
.btn.err{background:#3a1f1f;border-color:#a33}
.badge{display:inline-block;min-width:16px;height:16px;line-height:16px;border-radius:8px;background:#dc3545;color:#fff;font-size:10px;text-align:center;padding:0 4px;margin-left:6px;vertical-align:middle}
.badge.hide{display:none}
.modal{position:fixed;inset:0;background:rgba(0,0,0,.5);display:none;z-index:20;align-items:flex-start;justify-content:center}
.modal.open{display:flex}
.modalbox{background:#141920;border:1px solid #333;border-radius:8px;margin-top:4vh;padding:14px;width:min(900px,92vw);max-height:88vh;overflow:auto}
.prow{display:flex;gap:6px;align-items:center;margin:6px 0}
.prow input{flex:1;background:#0f1115;color:#e6e6e6;border:1px solid #333;border-radius:6px;padding:6px 8px;font-size:12px}
.prow input.nm{flex:0 0 220px}
.main{flex:1;display:flex;flex-direction:column;min-width:0;min-height:0}
.tabs{display:flex;gap:4px;flex-wrap:wrap;padding:8px 12px;background:#141920}
 .tab{padding:5px 10px;border-radius:6px;background:#232a33;font-size:13px}
 .tab.active{background:#2f6fed}
 .cmdbar{display:flex;gap:6px;align-items:stretch;padding:8px 12px;background:#141920;border-bottom:1px solid #24303a}
 .cmdwrap{flex:1;position:relative;display:flex;min-width:0}
 #cmdin{flex:1;background:#0f1115;color:#e6e6e6;border:1px solid #333;border-radius:6px;padding:7px 10px;font-family:monospace;font-size:13px;outline:none}
 #cmdin:focus{border-color:#2f6fed}
 .sugg{position:absolute;left:0;right:0;top:calc(100% + 3px);background:#1b2027;border:1px solid #333;border-radius:6px;max-height:320px;overflow:auto;z-index:26;box-shadow:0 8px 24px rgba(0,0,0,.5)}
 .si{padding:6px 10px;cursor:pointer;display:flex;gap:8px;align-items:baseline;font-size:12px;border-bottom:1px solid #20262e}
 .si:last-child{border-bottom:none}
 .si .nm{font-weight:600;color:#e6e6e6;font-family:monospace}
 .si .d{color:#8a93a0;flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
 .si .g{color:#6b7686;font-size:10px;white-space:nowrap}
 .si.active{background:#243a5e}
 .si .mv{color:#7be3a2;font-size:10px;white-space:nowrap}
 .cmdhint{color:#7f8a99;font-size:11px;align-self:center;white-space:nowrap}
 @media(max-width:1100px){.cmdhint{display:none}}
.view{flex:1;overflow:auto;padding:12px}
.desc{color:#9aa4b2;font-size:12px;margin:2px 0 8px}
.sidebar.closed{display:none}
.desc-toggle{font-size:11px;color:#6ea8fe;cursor:pointer;margin-left:8px;font-weight:normal}
.sub{color:#7f8a99;font-size:12px;margin:12px 0 4px;border-bottom:1px solid #24303a;padding-bottom:3px}
.grid{display:flex;flex-direction:column;gap:6px;align-items:stretch}
.btn{padding:8px 12px;border-radius:6px;background:#232a33;border:1px solid #333;width:100%;font-size:13px;text-align:left;color:#e6e6e6}
.btn.run{background:#17442a;border-color:#2c7a4b}
.btn .st{display:block;font-size:11px;color:#9aa4b2}
.btn.run .st{color:#7be3a2}
.btn .d{display:block;font-size:11px;color:#8a93a0;margin-top:2px}
.ritem{display:flex;align-items:center;gap:6px;background:#173040;border-radius:6px;padding:5px 8px;font-size:12px}
.ritem .n{flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.xbtn{background:#5a1f1f;color:#fff;border:none;border-radius:5px;padding:3px 7px;font-size:11px}
.xbtn.gray{background:#2a323c}
.log{position:fixed;inset:5%;background:#0b0e12;border:1px solid #333;border-radius:8px;display:none;flex-direction:column;overflow:hidden;z-index:20}
.log.open{display:flex}
.logbar{display:flex;align-items:center;gap:10px;padding:10px;border-bottom:1px solid #24303a;flex-wrap:wrap}
#logtitle{font-weight:bold}#logstate{flex:1;font-size:12px;color:#9aa4b2}
#logout{flex:1;min-height:0;overflow:auto;margin:0;padding:10px;font-family:monospace;font-size:12px;white-space:pre-wrap;overflow-wrap:anywhere}
.log .tin input{min-width:0}.log button:disabled{opacity:.4;cursor:default}
.term{display:none;position:fixed;left:0;right:0;bottom:0;height:44vh;background:#0b0e12;border-top:2px solid #2f6fed;z-index:15;flex-direction:column}
.term.open{display:flex}
.tbar{display:flex;gap:4px;padding:6px 8px;background:#141920;align-items:center;flex-wrap:wrap}
.ttab{padding:4px 8px;border-radius:6px;background:#232a33;font-size:12px;cursor:pointer;display:flex;gap:8px;align-items:center}
.ttab.active{background:#2f6fed}
.ttab .x{color:#ff9a9a;font-weight:bold}
#termout{flex:1;overflow:auto;margin:0;padding:8px 10px;font-family:monospace;font-size:12px;white-space:pre-wrap;word-break:break-all}
.tin{display:flex;border-top:1px solid #24303a}
.tin input{flex:1;background:#0f1115;color:#e6e6e6;border:none;padding:8px 10px;font-family:monospace;font-size:12px;outline:none}
.proc{font-family:monospace;font-size:12px;margin-top:6px}
.proc .r{display:flex;gap:8px;align-items:center;border-bottom:1px solid #223;padding:5px 2px}
.proc .r.dup{background:#2a1f1f;border-left:3px solid #a33;padding-left:6px}
.proc .r.err{border-left:3px solid #dc3545;padding-left:6px}
.proc .r .p{color:#9aa4b2;flex:0 0 70px}
.proc .r .c{flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.btn.err .st{color:#ff9a9a;font-weight:bold}
.ritem.err{background:#2a1f1f;border-left:3px solid #a33}
#errcnt{cursor:pointer;font-size:12px;color:#ff8080;font-weight:bold;display:none}
#errcnt.on{display:inline}
button{transition:transform .05s,filter .05s,opacity .1s}
button:active{transform:translateY(1px);filter:brightness(1.3)}
button.busy{opacity:.55;pointer-events:none}
#toast{position:fixed;top:52px;right:16px;background:#17442a;color:#e6ffe9;border:1px solid #2c7a4b;border-radius:8px;padding:10px 14px;font-size:13px;z-index:30;display:none;max-width:60vw}
#toast.bad{background:#5a1f1f;border-color:#a33;color:#ffe9e9}
.hbtn.on{background:#1d3a5f;border-color:#2f6fed}
.teleop{position:fixed;right:16px;bottom:16px;z-index:25;background:#141920;border:1px solid #2a3444;border-radius:10px;padding:10px 12px;box-shadow:0 6px 24px #0008;display:none}
.teleop.open{display:block}
.thead{display:flex;align-items:center;gap:8px;font-size:12px;color:#9aa4b2;margin-bottom:8px}
.tstate{color:#7f8a99}.tstate.ok{color:#7fe08a}.tstate.warn{color:#ffb86b}
.tgrid{display:grid;grid-template-columns:repeat(3,46px);grid-template-rows:repeat(3,38px);gap:4px;justify-content:center}
.tk{background:#1a2230;border:1px solid #2a3444;color:#e6e6e6;border-radius:8px;font-size:15px;cursor:pointer;user-select:none;touch-action:none}
.tk.on{background:#2f6fed;border-color:#2f6fed}
.tk.up{grid-area:1/2}.tk.left{grid-area:2/1}.tk.stop{grid-area:2/2;background:#3a1d1d;border-color:#a33;font-size:12px}
.tk.stop.on{background:#a33;border-color:#a33}
.tk.right{grid-area:2/3}.tk.down{grid-area:3/2}
.thint{font-size:11px;color:#7f8a99;margin-top:6px;text-align:center;max-width:170px}
.tchk{display:flex;justify-content:center;font-size:11px;color:#9aa4b2;margin-top:7px;cursor:pointer;user-select:none}
.tchk input{margin-right:5px}
.loghint{font-size:11px;color:#ffb86b;padding:2px 12px 0}
</style></head><body>
<header>
 <h1>SiriusLauncher Web</h1>
 <span id="cnt" style="color:#9aa4b2;font-size:12px"></span>
 <span id="errcnt" onclick="openErrors()" title="エラー一覧を表示"></span>
 <label style="font-size:12px;color:#9aa4b2">ROS_DOMAIN</label>
 <input id="domain" type="number" min="0" max="232" style="width:76px;background:#0f1115;color:#e6e6e6;border:1px solid #333;border-radius:6px;padding:5px 8px">
 <button class="hbtn" id="sidebarBtn" onclick="toggleSidebar()">◀ 左パネル</button>
 <button class="hbtn" id="termBtn" onclick="toggleTerm()">⌨ ターミナル</button>
 <button class="hbtn" id="teleopBtn" onclick="toggleTeleop()">⭕ テレオペ</button>
 <button class="hbtn" onclick="openProcs()">⧉ プロセス</button>
 <button class="hbtn stop" onclick="stopAll()">全て停止</button>
 <button class="hbtn" onclick="restartServer()">⟳ サーバー再起動</button>
 <button class="hbtn" onclick="load()">再読み込み</button>
</header>
<div class="body">
 <div class="sidebar">
  <div class="panel presets">
   <div class="ptitle" id="pt_presets"><span class="arr">▼</span>プリセット<button class="xbtn gray" id="pedit" style="margin-left:auto">✏ 編集</button></div>
   <div class="plist" id="presets"></div>
  </div>
  <div class="hgutter" id="hgutter"></div>
  <div class="panel running">
   <div class="ptitle" id="pt_running"><span class="arr">▼</span>起動中 <span id="runhint" style="color:#7f8a99;font-weight:normal"></span></div>
   <div class="plist" id="running"><div class="empty">起動中のプログラムはありません</div></div>
  </div>
 </div>
 <div class="gutter" id="gutter"></div>
 <div class="main">
  <div class="cmdbar">
   <div class="cmdwrap">
    <input id="cmdin" placeholder="コマンド / エイリアス名を入力 → Enter（例: lio3d_sim、ros2 topic list）" autocomplete="off" spellcheck="false">
    <div class="sugg hidden" id="sugg"></div>
   </div>
   <button class="hbtn" id="cmdrun" onclick="runCmd()" title="Enterでも実行できます">▶ 実行</button>
   <span class="cmdhint">エイリアス名＝管理起動 / それ以外＝新ターミナル（パネルは開かない）</span>
  </div>
  <div class="tabs" id="tabs"></div>
  <div class="view" id="view"></div>
 </div>
</div>
<div class="modal" id="pmodal"><div class="modalbox">
 <h3 style="margin-top:0">プリセット編集</h3>
 <div style="color:#9aa4b2;font-size:12px;margin-bottom:6px">各行: 名前 / アイテム（エイリアス名のカンマ区切り）</div>
 <div id="prows"></div>
 <button class="hbtn" onclick="addPresetRow('','')">＋プリセット追加</button>
 <div style="margin-top:12px">
  <button class="hbtn" style="background:#2f6fed;color:#fff;border:none" onclick="savePresets()">保存</button>
  <button class="hbtn" onclick="closePresets()">閉じる</button>
 </div>
 <div id="perr" style="color:#ff8080;margin-top:6px;font-size:12px"></div>
</div></div>
<datalist id="aliaslist"></datalist>
<div class="term" id="term">
 <div class="tbar" id="termtabs"></div>
 <pre id="termout"></pre>
 <div class="tin"><input id="termin" placeholder="コマンドを入力してEnter（例: src && ros2 topic list）" onkeydown="termInput(event)"></div>
</div>
<div class="modal" id="pmodal2"><div class="modalbox">
 <h3 style="margin-top:0">起動中のプロセス（外部起動も含む）</h3>
 <div class="desc">ROS/sirius関連プロセスを表示します。「kill」で停止できます（子孫も停止）。</div>
 <div class="proc" id="procrows"></div>
 <div style="margin-top:12px"><button class="hbtn" onclick="openProcs()">再読み込み</button>
 <button class="hbtn" onclick="closeProcs()">閉じる</button></div>
</div></div>
<div class="modal" id="emodal"><div class="modalbox">
 <h3 style="margin-top:0;color:#ff8080">⚠ エラーが出ているプロセス</h3>
 <div class="desc">ログ末尾に [ERROR]/[FATAL]/Traceback を含むプロセスです。「ログ」で内容を確認し、「停止」で止められます。</div>
 <div class="proc" id="errrows"></div>
 <div style="margin-top:12px"><button class="hbtn" onclick="openErrors()">再読み込み</button>
 <button class="hbtn" onclick="closeErrors()">閉じる</button></div>
</div></div>
<div class="log" id="log" role="dialog" aria-label="コマンドのログと入力">
 <div class="logbar"><span id="logtitle"></span><span id="logstate"></span>
  <button class="hbtn" id="loginterrupt" onclick="sendLogControl()" disabled>Ctrl+C</button>
  <button class="hbtn" id="logclose" onclick="closeLog()">閉じる</button></div>
 <pre id="logout"></pre>
 <div class="loghint" id="loghint"></div>
 <form class="tin" onsubmit="sendLogInput(event)">
  <input id="login" aria-label="コマンドへの入力" placeholder="番号やパスを入力してEnter" onkeydown="logKey(event)" disabled autocomplete="off">
  <button class="hbtn" id="logsend" type="submit" disabled>送信 / Enter</button>
 </form>
</div>
<div id="toast"></div>
<div class="teleop" id="teleop">
 <div class="thead"><span>⭕ テレオペ（/controller）</span><span class="tstate" id="tstate">待機</span><button class="xbtn gray" onclick="toggleTeleop(false)">✕</button></div>
 <div class="tgrid">
  <button class="tk up" data-k="65" title="前進 +0.1 m/s">▲</button>
  <button class="tk left" data-k="68" title="左旋回 +0.157 rad/s">◀</button>
  <button class="tk stop" data-k="stop" title="停止（並進・旋回をリセット）">■ 停止</button>
  <button class="tk right" data-k="67" title="右旋回 -0.157 rad/s">▶</button>
  <button class="tk down" data-k="66" title="後退 -0.1 m/s">▼</button>
 </div>
 <div class="tchk"><label><input type="checkbox" id="tkdm" checked> 離したら停止</label></div>
 <div class="thint">押している間で加減速 / 矢印・Spaceキーでも可（scn起動中のみ有効）</div>
</div>
<script>
let DATA=null,TAB=0,COLLAPSED={};
async function api(p,body){const o={method:body?'POST':'GET',headers:{'Content-Type':'application/json'}};if(body)o.body=JSON.stringify(body);const r=await fetch(p,o);return r.json();}
async function load(){DATA=await api('/api/state');renderTabs();render();refresh();
 const dEl=document.getElementById('domain');
 dEl.value=(localStorage.getItem('rosd')!==null?localStorage.getItem('rosd'):(DATA.default_domain||''));
 dEl.addEventListener('change',()=>localStorage.setItem('rosd',dEl.value));
 [['pt_presets','presets'],['pt_running','running']].forEach(([tid,cid])=>{const t=document.getElementById(tid);const c=document.getElementById(cid);
  t.onclick=()=>{c.classList.toggle('hidden');t.classList.toggle('closed');};});
 const dl=document.getElementById('aliaslist');dl.innerHTML='';DATA.groups.forEach(g=>g.items.forEach(it=>{const o=document.createElement('option');o.value=it.name;dl.appendChild(o);}));
 buildCmdIndex();
 const pe=document.getElementById('pedit');if(pe)pe.onclick=(e)=>{e.stopPropagation();openPresets();};}
function openPresets(){const rows=document.getElementById('prows');rows.innerHTML='';
 DATA.presets.forEach(p=>addPresetRow(p.name,p.items.join(',')));
 document.getElementById('perr').textContent='';
 document.getElementById('pmodal').classList.add('open');}
function closePresets(){document.getElementById('pmodal').classList.remove('open');}
function addPresetRow(name,items){const rows=document.getElementById('prows');const d=document.createElement('div');d.className='prow';
 d.innerHTML='<input class="nm" placeholder="プリセット名" list="aliaslist" value="'+(name||'')+'">'+
  '<input placeholder="アイテム（エイリアス名をカンマ区切り）" list="aliaslist" value="'+(items||'')+'">'+
  '<button class="xbtn" onclick="this.parentNode.remove()">削除</button>';
 rows.appendChild(d);}
async function savePresets(){const rows=Array.prototype.slice.call(document.querySelectorAll('#prows .prow'));
 const presets=rows.map(r=>{const ins=r.querySelectorAll('input');return {name:ins[0].value.trim(),items:ins[1].value.split(',').map(s=>s.trim()).filter(Boolean)};}).filter(p=>p.name&&p.items.length);
 const res=await api('/api/presets/save',{presets});
 if(res.error){document.getElementById('perr').textContent='保存失敗: '+res.error;return;}
 DATA.presets=res.presets;renderTabs();document.getElementById('perr').textContent='保存しました';closePresets();}
function dom(){const v=document.getElementById('domain').value;localStorage.setItem('rosd',v);return v;}
function renderTabs(){const t=document.getElementById('tabs');t.innerHTML='';
 DATA.groups.forEach((g,i)=>{const d=document.createElement('div');d.className='tab'+(i==TAB?' active':'');d.textContent=g.name;const bd=document.createElement('span');bd.className='badge hide';bd.id='badge_'+i;d.appendChild(bd);d.onclick=()=>{TAB=i;renderTabs();render();};t.appendChild(d);});
 const p=document.getElementById('presets');p.innerHTML='';
 DATA.presets.forEach(pr=>{const b=document.createElement('button');b.className='preset';b.textContent='▶ '+pr.name;b.onclick=()=>runPreset(pr.name,b);p.appendChild(b);});}
function makeBtn(it){const b=document.createElement('button');b.className='btn';b.id='btn_'+it.name;
 b.innerHTML='<b>'+it.name+'</b><span class="st">...</span><span class="d">'+(it.desc||'')+'</span>';
 b.onclick=()=>toggle(it.name,b);return b;}
function render(){const g=DATA.groups[TAB];const v=document.getElementById('view');
 v.innerHTML='<h3 style="font-size:14px;margin:0">'+g.name+(g.desc?'<span class="desc-toggle" id="dscbtn">[説明]</span>':'')+'</h3><div class="desc'+(COLLAPSED['desc:'+TAB]?' hidden':'')+'" id="gdesc">'+(g.desc||'')+'</div><div id="sections"></div>';
 const db=document.getElementById('dscbtn');if(db)db.onclick=()=>{const d=document.getElementById('gdesc');d.classList.toggle('hidden');COLLAPSED['desc:'+TAB]=d.classList.contains('hidden');};
 const secs=document.getElementById('sections');
 const plain=g.items.filter(it=>!it.subgroup);
 if(plain.length){const body=document.createElement('div');body.className='grid';plain.forEach(it=>body.appendChild(makeBtn(it)));secs.appendChild(body);}
 const order=[],map={};
 g.items.forEach(it=>{if(!it.subgroup)return;const k=it.subgroup;if(!map[k]){map[k]=[];order.push(k);}map[k].push(it);});
 order.forEach(k=>{const key='sec:'+g.name+':'+k;const hid=!!COLLAPSED[key];
  const sec=document.createElement('div');sec.className='sec';
  const h=document.createElement('div');h.className='sec-h'+(hid?' closed':'');h.innerHTML='<span class="arr">▼</span>'+k;
  const body=document.createElement('div');body.className='grid'+(hid?' hidden':'');
  map[k].forEach(it=>body.appendChild(makeBtn(it)));
  h.onclick=()=>{const closed=!h.classList.contains('closed');h.classList.toggle('closed',closed);body.classList.toggle('hidden',closed);COLLAPSED[key]=closed;};
  sec.appendChild(h);sec.appendChild(body);secs.appendChild(sec);});}
async function refresh(){if(!DATA)return;const st=await api('/api/status');
 let errTotal=0;
 document.querySelectorAll('.btn').forEach(b=>{const n=b.id.replace('btn_','');const s=st[n];
  const run=s&&s.running, err=s&&s.error, op=OPS[n];
  b.dataset.running=run?'1':'0';
  b.querySelector('.st').textContent=op?('⏳ '+op)
    :(err?('⚠ エラー'+(run?(' / 起動中 pid '+s.pid):''))
    :(run?('起動中'+(s.source==='external'?'(外部)':'')+' pid '+s.pid):'停止'));
  b.classList.toggle('busy',!!op);
  b.classList.toggle('run',!!run&&!err&&!op);b.classList.toggle('err',!!err&&!op);});
 const errs=Object.entries(st).filter(([n,s])=>s.error);errTotal=errs.length;
 const ec=document.getElementById('errcnt');
 if(ec){ec.textContent=errTotal?('⚠ エラー: '+errTotal+'件'):'';ec.classList.toggle('on',errTotal>0);}
 DATA.groups.forEach((g,i)=>{let c=0;g.items.forEach(it=>{const s=st[it.name];if(s&&s.error)c++;});
  const bd=document.getElementById('badge_'+i);if(bd){bd.textContent=c;bd.classList.toggle('hide',c===0);}});
 const run=Object.entries(st).filter(([n,s])=>s.running);
 SCN_RUN=run.some(([n])=>n==='scn');
 document.getElementById('cnt').textContent='起動中: '+run.length+' 件';
 document.getElementById('runhint').textContent=run.length?('('+run.length+')'):'';
 const r=document.getElementById('running');
 r.innerHTML=run.length?run.map(([n,s])=>{const op=OPS[n];const mark=op?'⏳ ':(s.error?'⚠ ':'');
  return '<div class="ritem'+((s.error||op)?' err':'')+'"><span class="n">'+mark+n+' ('+s.pid+(s.source==='external'?', 外部':'')+')</span>'+
   (op?('<span class="st" style="color:#ffb86b">'+op+'</span>'):'<button class="xbtn gray" onclick="restartIt(\\''+n+'\\',this)">再起動</button><button class="xbtn" onclick="stopIt(\\''+n+'\\',this)">停止</button><button class="xbtn gray" onclick="showLog(\\''+n+'\\')">ログ</button>')+'</div>';}).join(''):'<div class="empty">起動中のプログラムはありません</div>';}
async function openErrors(){const st=await api('/api/status');const errs=Object.entries(st).filter(([n,s])=>s.error);const box=document.getElementById('errrows');
 box.innerHTML=errs.length?errs.map(([n,s])=>'<div class="r dup"><span class="p">'+s.pid+'</span><span class="c">⚠ '+esc(n)+(s.source==='external'?' (外部)':'')+'</span><button class="xbtn gray" onclick="showLog(\\''+n+'\\')">ログ</button><button class="xbtn" onclick="stopIt(\\''+n+'\\',this).then(openErrors)">停止</button></div>').join(''):'<div class="empty">エラーはありません</div>';
 document.getElementById('emodal').classList.add('open');}
function closeErrors(){document.getElementById('emodal').classList.remove('open');}
const OPS={};let toastTimer=null;
function toast(msg,bad){const t=document.getElementById('toast');t.textContent=msg;t.classList.toggle('bad',!!bad);t.style.display='block';
 clearTimeout(toastTimer);toastTimer=setTimeout(()=>t.style.display='none',4000);}
async function finishOp(n,msg){delete OPS[n];let bad=false;try{const st=await api('/api/status');const s=st[n];bad=!!(s&&s.error);}catch(e){}
 toast(msg+(bad?' ⚠ エラーが出ています（ログ参照）':''),bad);refresh();}
async function restartIt(n,el){if(OPS[n])return;OPS[n]='再起動中…';if(el)el.classList.add('busy');refresh();
 try{await api('/api/restart',{name:n,domain:dom()});}finally{await finishOp(n,'再起動: '+n);}}
async function toggle(n,b){if(OPS[n])return;const cur=!!(b&&b.dataset.running==='1');
 OPS[n]=cur?'停止中…':'起動中…';if(b)b.classList.add('busy');refresh();
 try{if(cur){await api('/api/stop',{name:n});await finishOp(n,'停止: '+n);}
 else{const res=await api('/api/start',{name:n,domain:dom()});if(res.error){delete OPS[n];toast('起動できません: '+n+' ('+res.error+')',true);refresh();return;}await finishOp(n,'起動: '+n);await showLog(n);}}
 catch(e){delete OPS[n];toast('操作に失敗: '+n,true);refresh();}}
async function stopIt(n,el){if(OPS[n])return;OPS[n]='停止中…';if(el)el.classList.add('busy');refresh();
 try{await api('/api/stop',{name:n});}finally{await finishOp(n,'停止: '+n);}}
async function stopAll(){if(!confirm('全て停止しますか？'))return;await api('/api/stop_all',{});refresh();}
async function restartServer(){if(!confirm('Webサーバー（SiriusLauncher Web）を再起動しますか？\\n起動中のプロセスは維持されます。'))return;
 try{await api('/api/restart_server',{});}catch(e){}
 setTimeout(()=>location.reload(),3000);}
async function runPreset(n,el){if(el&&el.classList.contains('busy'))return;if(el){el.classList.add('busy');el.textContent='▶ 実行中… '+n;}
 try{await api('/api/preset',{name:n,domain:dom()});}finally{if(el){el.classList.remove('busy');el.textContent='▶ '+n;}toast('プリセット実行: '+n);refresh();}}
function toggleSidebar(){const s=document.querySelector('.sidebar');s.classList.toggle('closed');document.getElementById('sidebarBtn').textContent=s.classList.contains('closed')?'▶ 左パネル':'◀ 左パネル';}
(function(){const gutter=document.getElementById('gutter'),sb=document.querySelector('.sidebar');
 const saved=localStorage.getItem('sbw');if(saved)sb.style.width=saved+'px';
 let drag=false;
 gutter.addEventListener('mousedown',e=>{drag=true;e.preventDefault();document.body.style.userSelect='none';});
 window.addEventListener('mousemove',e=>{if(!drag)return;sb.style.width=Math.max(160,Math.min(760,e.clientX))+'px';});
 window.addEventListener('mouseup',()=>{if(!drag)return;drag=false;document.body.style.userSelect='';localStorage.setItem('sbw',parseInt(sb.style.width));});
})();
(function(){const g=document.getElementById('hgutter'),pr=document.querySelector('.panel.presets'),sb=document.querySelector('.sidebar');
 const saved=localStorage.getItem('prh');if(saved)pr.style.flex='0 0 '+saved+'px';
 let drag=false;
 g.addEventListener('mousedown',e=>{drag=true;e.preventDefault();document.body.style.userSelect='none';});
 window.addEventListener('mousemove',e=>{if(!drag)return;const r=sb.getBoundingClientRect();let h=Math.max(80,Math.min(r.height-80,e.clientY-r.top));pr.style.flex='0 0 '+h+'px';});
 window.addEventListener('mouseup',()=>{if(!drag)return;drag=false;document.body.style.userSelect='';localStorage.setItem('prh',parseInt(pr.style.flex.split(' ')[2]));});
})();
let LOG_NAME=null,LOG_REQUEST=0,LOG_FOCUS=false,LOG_BUSY=false;
async function showLog(n){LOG_NAME=n;LOG_FOCUS=true;document.getElementById('logtitle').textContent=n;
 document.getElementById('loghint').textContent=(n==='scn')?'KeyOp操作: A=前進 B=後退 C=右旋回 D=左旋回 / ,=並進リセット .=旋回リセット（入力してEnter）':'';
 document.getElementById('logout').textContent='';document.getElementById('login').value='';
 document.getElementById('log').classList.add('open');await pollLog();}
function closeLog(){LOG_NAME=null;LOG_REQUEST++;document.getElementById('log').classList.remove('open');}
async function pollLog(){if(!LOG_NAME)return;const name=LOG_NAME,requestId=++LOG_REQUEST;
 try{const j=await api('/api/log?name='+encodeURIComponent(name));if(LOG_NAME!==name||requestId!==LOG_REQUEST)return;
 const out=document.getElementById('logout'),bottom=out.scrollHeight-out.clientHeight-out.scrollTop<40;
 out.textContent=j.log||j.error||'';if(bottom)out.scrollTop=out.scrollHeight;
 ['login','logsend','loginterrupt'].forEach(id=>document.getElementById(id).disabled=!j.interactive);
 document.getElementById('logstate').textContent=j.interactive?'入力可能（Enterで送信）':'入力端末なし：終了済み、または更新前の起動。入力する場合はこのコマンドを再起動してください。';
 if(j.interactive&&LOG_FOCUS){document.getElementById('login').focus();LOG_FOCUS=false;}}
 catch(e){if(LOG_NAME===name)document.getElementById('logstate').textContent='接続を再試行しています…';}}
async function writeLog(data){if(!LOG_NAME||LOG_BUSY)return false;const name=LOG_NAME;LOG_BUSY=true;
 try{const r=await api('/api/input',{name:name,data:data});if(!r.ok){toast(r.error||'入力を送信できません',true);return false;}return true;}
 catch(e){toast('入力を送信できませんでした',true);return false;}finally{LOG_BUSY=false;pollLog();}}
async function sendLogInput(e){e.preventDefault();const inp=document.getElementById('login');if(inp.disabled)return;
 const name=LOG_NAME,value=inp.value;if(await writeLog(value+'\\n')){if(LOG_NAME===name&&inp.value===value)inp.value='';}}
function sendLogControl(){writeLog(String.fromCharCode(3));}
function logKey(e){if(e.isComposing||e.keyCode===229){if(e.key==='Enter')e.preventDefault();return;}
 if(e.ctrlKey&&e.key.toLowerCase()==='c'&&!window.getSelection().toString()&&e.target.selectionStart===e.target.selectionEnd){e.preventDefault();sendLogControl();}}
const TERM={sessions:[],active:null,offsets:{}};
function toggleTerm(){const t=document.getElementById('term');t.classList.toggle('open');
 document.getElementById('termBtn').textContent=t.classList.contains('open')?'⌨ ターミナル ▼':'⌨ ターミナル';
 if(t.classList.contains('open')){if(!TERM.sessions.length){newTerm('');}else if(TERM.active){selectTerm(TERM.active);}else{selectTerm(TERM.sessions[0]);}}}
async function newTerm(cmd){const r=await api('/api/term/open',{cmd:cmd||''});if(r.error&&r.error!=='exists'){alert('起動できません: '+r.error);return;}const n=r.name;if(!TERM.sessions.includes(n))TERM.sessions.push(n);selectTerm(n);}
function selectTerm(n){if(!n)return;if(!TERM.sessions.includes(n))TERM.sessions.push(n);TERM.active=n;TERM.offsets[n]=0;renderTermTabs();document.getElementById('termout').textContent='';pollTerm();}
function renderTermTabs(){const b=document.getElementById('termtabs');b.innerHTML='';
 TERM.sessions.forEach(n=>{const d=document.createElement('div');d.className='ttab'+(n===TERM.active?' active':'');const s=document.createElement('span');s.textContent=n;const x=document.createElement('span');x.className='x';x.textContent='×';x.title='閉じる';x.onclick=(e)=>{e.stopPropagation();closeTerm(n);};d.appendChild(s);d.appendChild(x);d.onclick=()=>selectTerm(n);b.appendChild(d);});
 const a=document.createElement('button');a.className='hbtn';a.textContent='＋ シェル';a.title='新しいターミナル（対話シェル）';a.onclick=()=>newTerm('');b.appendChild(a);
 const c=document.createElement('button');c.className='hbtn';c.textContent='✕ 閉じる';c.onclick=()=>toggleTerm();b.appendChild(c);}
async function closeTerm(n){await api('/api/term/close',{name:n});TERM.sessions=TERM.sessions.filter(x=>x!==n);delete TERM.offsets[n];if(TERM.active===n){TERM.active=TERM.sessions[TERM.sessions.length-1]||null;}renderTermTabs();document.getElementById('termout').textContent='';if(TERM.active)selectTerm(TERM.active);}
async function pollTerm(){if(!TERM.active)return;if(!document.getElementById('term').classList.contains('open'))return;
 const r=await api('/api/term/read?name='+encodeURIComponent(TERM.active)+'&offset='+(TERM.offsets[TERM.active]||0));
 if(r.error)return;TERM.offsets[TERM.active]=r.offset;if(r.data){const o=document.getElementById('termout');const atBottom=(o.scrollHeight-o.clientHeight-o.scrollTop)<40;o.textContent+=r.data;if(atBottom)o.scrollTop=o.scrollHeight;}}
function termInput(e){if(e.key!=='Enter')return;if(!TERM.active)return;const inp=document.getElementById('termin');const v=inp.value;inp.value='';api('/api/term/write',{name:TERM.active,data:v+'\\n'});setTimeout(pollTerm,150);}
async function openProcs(){const j=await api('/api/ps');const rows=(j.processes||[]);const dups=(j.duplicates||[]);const box=document.getElementById('procrows');
 let html='';
 if(dups.length){html+='<div class="desc" style="color:#ffb86b">⚠ 重複（同じプログラムが複数動いています。通常は不要なので停止推奨）</div>';
  dups.forEach(d=>{html+='<div class="r dup"><span class="p">'+d.count+'個</span><span class="c">'+esc(d.alias)+'  (pids: '+d.pids.join(', ')+')</span><button class="xbtn" onclick="killMany('+JSON.stringify(d.pids)+')">重複を停止</button></div>';});
  html+='<div class="desc" style="margin-top:10px">その他のプロセス</div>';}
 html+=rows.length?rows.map(p=>'<div class="r'+(p.dup?' dup':'')+(p.error?' err':'')+'"><span class="p">'+p.pid+'</span><span class="c">'+(p.error?'⚠ ':'')+esc((p.alias?('['+p.alias+'] '):'')+p.cmd)+'</span><span class="p" style="flex:0 0 74px;text-align:right">'+p.age_txt+'</span><button class="xbtn gray" onclick="killProc('+p.pid+')">kill</button></div>').join(''):'<div class="empty">ROS/sirius関連のプロセスはありません</div>';
 box.innerHTML=html;
 document.getElementById('pmodal2').classList.add('open');}
function closeProcs(){document.getElementById('pmodal2').classList.remove('open');}
async function killProc(pid){if(!confirm('PID '+pid+' を停止しますか？（子孫も停止）'))return;await api('/api/ps/kill',{pid:pid});openProcs();refresh();}
async function killMany(pids){if(!confirm('重複プロセス '+pids.length+' 個を停止しますか？（子孫も停止）'))return;await api('/api/ps/kill_many',{pids:pids});openProcs();refresh();}
// ---- コマンドバー（エイリアス名は管理起動 / それ以外は新ターミナルで実行） ----
let CMDS=[],SUGG=[],SEL=-1,HIST=[],HISTIX=-1;
try{HIST=JSON.parse(localStorage.getItem('cmdhist')||'[]');if(!Array.isArray(HIST))HIST=[];}catch(e){HIST=[];}
function buildCmdIndex(){CMDS=[];DATA.groups.forEach(g=>g.items.forEach(it=>{if(!CMDS.some(c=>c.name===it.name))CMDS.push({name:it.name,desc:it.desc||'',group:g.name});}));}
function matchCmds(q){const s=q.trim().toLowerCase();if(!s)return CMDS.slice(0,12);
 const starts=[],has=[];CMDS.forEach(c=>{const n=c.name.toLowerCase();
  if(n.startsWith(s))starts.push(c);else if(n.includes(s)||(c.desc||'').toLowerCase().includes(s))has.push(c);});
 return starts.concat(has).slice(0,12);}
function hideSugg(){document.getElementById('sugg').classList.add('hidden');SEL=-1;}
function renderSugg(){const box=document.getElementById('sugg');const inp=document.getElementById('cmdin');
 if(document.activeElement!==inp){box.classList.add('hidden');return;}
 SUGG=matchCmds(inp.value);if(!SUGG.length){box.classList.add('hidden');return;}
 if(SEL>=SUGG.length)SEL=SUGG.length-1;
 box.innerHTML=SUGG.map((c,i)=>'<div class="si'+(i===SEL?' active':'')+'" title="'+esc(c.name+' — '+(c.desc||''))+'" onmousedown="pickSugg(event,'+i+')"><span class="nm">'+esc(c.name)+'</span><span class="d">'+esc(c.desc)+'</span><span class="g">'+esc(c.group)+'</span><span class="mv" style="'+(i===SEL?'':'visibility:hidden')+'">▸ Enter</span></div>').join('');
 box.classList.remove('hidden');}
function pickSugg(e,i){e.preventDefault();const c=SUGG[i];if(!c)return;const inp=document.getElementById('cmdin');inp.value=c.name;hideSugg();runCmd();}
function cmdHistory(dir){const inp=document.getElementById('cmdin');if(!HIST.length)return;
 HISTIX+=dir;
 if(HISTIX<0){HISTIX=-1;inp.value='';return;}
 if(HISTIX>=HIST.length)HISTIX=HIST.length-1;
 inp.value=HIST[HISTIX]||'';}
function cmdKey(e){if(e.isComposing||e.keyCode===229)return;
 const open=!document.getElementById('sugg').classList.contains('hidden');
 if(e.key==='ArrowDown'){e.preventDefault();if(!open){cmdHistory(1);return;}SEL=Math.min(SEL+1,SUGG.length-1);renderSugg();return;}
 if(e.key==='ArrowUp'){e.preventDefault();if(!open){cmdHistory(-1);return;}SEL=Math.max(SEL-1,0);renderSugg();return;}
 if(e.key==='Tab'){e.preventDefault();if(open){const c=SUGG[SEL>=0?SEL:0];if(c){document.getElementById('cmdin').value=c.name;hideSugg();}}return;}
 if(e.key==='Enter'){e.preventDefault();if(open&&SEL>=0&&SUGG[SEL])document.getElementById('cmdin').value=SUGG[SEL].name;runCmd();return;}
 if(e.key==='Escape'){hideSugg();return;}}
function pushHist(v){HIST=[v].concat(HIST.filter(h=>h!==v)).slice(0,50);HISTIX=-1;try{localStorage.setItem('cmdhist',JSON.stringify(HIST));}catch(e){}}
async function startByName(n){if(OPS[n])return;
 try{const st=await api('/api/status');const s=st[n];
  if(s&&s.running){toast(n+' は既に起動中です'+(s.error?'（⚠ エラーあり）':''));await showLog(n);return;}}
 catch(e){}
 OPS[n]='起動中…';refresh();
 try{const res=await api('/api/start',{name:n,domain:dom()});
  if(res.error){delete OPS[n];toast('起動できません: '+n+' ('+res.error+')',true);refresh();return;}
  await finishOp(n,'起動: '+n);await showLog(n);}
 catch(e){delete OPS[n];toast('操作に失敗: '+n,true);refresh();}}
async function runInTerminal(cmd){const name='cmd_'+Date.now();
 const r=await api('/api/term/open',{cmd:'',name:name});
 let n=name;if(r.error){if(r.error==='exists'){n=r.name||name;}else{toast('ターミナルを開けません: '+r.error,true);return;}}
 if(!TERM.sessions.includes(n))TERM.sessions.push(n);
 if(document.getElementById('term').classList.contains('open'))selectTerm(n);else renderTermTabs();
 toast('ターミナルで実行: '+cmd+'（⌨ ターミナルで確認）');
 setTimeout(()=>{api('/api/term/write',{name:n,data:cmd+'\\n'});setTimeout(pollTerm,250);},150);}
async function runCmd(){const inp=document.getElementById('cmdin');const v=inp.value.trim();if(!v)return;
 const alias=CMDS.find(c=>c.name===v);hideSugg();inp.value='';pushHist(v);
 if(alias)await startByName(alias.name);else await runInTerminal(v);}
(function(){const ci=document.getElementById('cmdin');
 ci.addEventListener('input',()=>{SEL=-1;renderSugg();});
 ci.addEventListener('focus',()=>{SEL=-1;renderSugg();});
 ci.addEventListener('keydown',cmdKey);
 document.addEventListener('mousedown',e=>{if(!e.target.closest('.cmdwrap'))hideSugg();});})();
function esc(s){return (s||'').replace(/[&<>"]/g,function(c){return {'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c];});}
// ---- テレオペ（十字キーパッド → /controller） ----
let SCN_RUN=false,TK_TIMER=null,TK_KEY=null;
const TK_MAP={ArrowUp:65,ArrowDown:66,ArrowLeft:68,ArrowRight:67};
function deadmanOn(){const c=document.getElementById('tkdm');return !c||c.checked;}
function toggleTeleop(force){const p=document.getElementById('teleop');
 const open=(force!==undefined)?force:!p.classList.contains('open');
 p.classList.toggle('open',open);
 document.getElementById('teleopBtn').classList.toggle('on',open);
 if(open)teleopPoll();
 try{localStorage.setItem('teleop',open?'1':'0');}catch(e){}}
async function teleopSend(k){try{const r=await api('/api/teleop',{key:k,domain:dom()});
 if(!r.ok)toast(r.error||'テレオペ送信に失敗しました',true);teleopState(r);}catch(e){toast('テレオペ送信に失敗しました',true);}}
function teleopState(r){const s=document.getElementById('tstate');if(!s)return;
 let cls='',txt='';
 if(r&&r.error){cls='warn';txt='⚠ '+r.error;}
 else if(SCN_RUN){if(r&&r.alive){cls='ok';txt='操作可能';}else{txt='scn起動中・ブリッジ待機';}}
 else if(r&&r.alive){cls='ok';txt='ブリッジ稼働'+(r.domain?(' (D'+r.domain+')'):'');}
 else{txt='待機（キー操作で起動 / scn未起動）';}
 s.className='tstate '+cls;s.textContent=txt;}
async function teleopPoll(){if(!document.getElementById('teleop').classList.contains('open'))return;
 try{teleopState(await api('/api/teleop/status'));}catch(e){}}
function startHold(k,el){stopHold();el.classList.add('on');teleopSend(k);TK_KEY=k;TK_TIMER=setInterval(()=>teleopSend(k),120);}
function stopHold(){if(TK_TIMER){clearInterval(TK_TIMER);TK_TIMER=null;}
 if(TK_KEY!==null){const k=TK_KEY;TK_KEY=null;document.querySelectorAll('.tk').forEach(b=>b.classList.remove('on'));
  if(k!=='stop'&&deadmanOn())teleopSend('stop');}}
function teleopKeyGuard(e){if(!document.getElementById('teleop').classList.contains('open'))return false;
 if(e.isComposing||e.keyCode===229)return false;
 const t=e.target;return !(t&&t.closest&&t.closest('input,textarea,select,[contenteditable]'));}
(function(){document.querySelectorAll('#teleop .tk').forEach(b=>{
 b.addEventListener('pointerdown',e=>{e.preventDefault();const dk=b.dataset.k;startHold(dk==='stop'?'stop':parseInt(dk,10),b);});
 b.addEventListener('pointerleave',stopHold);
 b.addEventListener('contextmenu',e=>e.preventDefault());});
 window.addEventListener('pointerup',stopHold);window.addEventListener('pointercancel',stopHold);
 document.addEventListener('keydown',e=>{
  if(!teleopKeyGuard(e))return;
  if(e.key===' '){e.preventDefault();if(!e.repeat)teleopSend('stop');return;}
  const k=TK_MAP[e.key];if(k!==undefined){e.preventDefault();teleopSend(k);}});
 document.addEventListener('keyup',e=>{
  if(!teleopKeyGuard(e))return;
  if(TK_MAP[e.key]!==undefined&&deadmanOn())teleopSend('stop');});
 const dm=document.getElementById('tkdm');
 dm.checked=localStorage.getItem('tkdm')!=='0';
 dm.addEventListener('change',()=>{try{localStorage.setItem('tkdm',dm.checked?'1':'0');}catch(e){}});
 toggleTeleop(localStorage.getItem('teleop')==='1');})();
load();setInterval(refresh,2000);setInterval(pollTerm,700);setInterval(pollLog,700);setInterval(teleopPoll,2000);
</script></body></html>"""

LOGIN = """<!doctype html><html><head><meta charset="utf-8"><title>login</title></head>
<body style="font-family:sans-serif"><h3>SiriusLauncher Web ログイン</h3>
<form method="get" action="/"><input name="pw" type="password" placeholder="password">
<button>ログイン</button></form></body></html>"""


@app.route("/")
def index():
    if PASSWORD and request.args.get("pw") == PASSWORD:
        resp = make_response(PAGE)
        resp.set_cookie("pw", PASSWORD, max_age=7 * 24 * 3600)
        return resp
    return PAGE if auth_ok() else LOGIN


@app.route("/api/state")
def api_state():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    return jsonify({"groups": GROUPS, "presets": PRESETS, "default_domain": DEFAULT_DOMAIN})


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
        ("cmd" if cmd else "shell") + "_" + str(int(time.time())))
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
    threading.Thread(target=TELEOP.ensure, args=(DEFAULT_DOMAIN,), daemon=True).start()
    print(f"SiriusLauncher Web: http://{args.host}:{args.port}  "
          f"({len(BUTTONS)} aliases, {len(GROUPS)} groups, {len(PRESETS)} presets)")
    if PASSWORD:
        print("パスワード保護: 有効")
    app.run(host=args.host, port=args.port, threaded=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
