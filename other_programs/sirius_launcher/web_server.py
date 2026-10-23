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
import fcntl
import struct
import termios
from functools import lru_cache
import json
import os
import pty
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path
from urllib.parse import urlsplit

import psutil
from flask import Flask, jsonify, request, make_response
from https_proxy import LocalHTTPSProxy

from alias_parser import parse_bash_aliases
from process_manager import ProcessManager
from command_session import session_request
from resource_monitor import ResourceMonitor
from tf_monitor import TFMonitor
from tf_readiness import tf_readiness
from topic_monitor import TopicMonitor
from process_health import LogHealth, battery_health, readiness_health, readiness_pattern
from input_wait import InputWaitMonitor
from emergency_stop_command import send_emergency_stop
from teleop_process import TeleopBridge
from teleop_control import VelocityControl
from documentation_notes import register_documentation_notes
from file_browser import register_file_browser
from face_bridge import register_face
from ear_control import register_ear
from hotspot_status import register_hotspot
from research_archive import register_research_archive
from preset_metadata import read_preset_metadata, preset_presentation

RESOURCES = ResourceMonitor()
TF_MONITOR = TFMonitor()
TOPIC_MONITOR = TopicMonitor()
EMERGENCY_STOP_LOCK = threading.Lock()

app = Flask(__name__)
app.wsgi_app = LocalHTTPSProxy(app.wsgi_app)
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
VISUALIZATION_LAYOUT = Path(__file__).with_name("sirius_visualization_layout.json")
VISUALIZATION_LAYOUTS_DIR = Path(__file__).with_name("sirius_visualization_layouts")
VISUALIZATION_VIEWER_PORT = 8091
VISUALIZATION_BRIDGE_PORT = 8766
LAYOUTS_LOCK = threading.Lock()
LAUNCH_SPECS_FILE = Path("/tmp/sirius_launcher_launch_specs.json")
LAUNCH_SPECS = {}
LAUNCH_SPECS_LOCK = threading.Lock()
LAUNCH_SPECS_LOADED = False


def log_path(name):
    # デスクトップ版(process_manager.py)と共有: /tmp/sirius_launcher_<name>.log
    return f"/tmp/sirius_launcher_{name}.log"


def pid_path(name):
    # デスクトップ版(process_manager.py)と共有: /tmp/sirius_launcher_<name>.pid
    return f"/tmp/sirius_launcher_{name}.pid"


LOG_HEALTH = LogHealth()
INPUT_WAIT = InputWaitMonitor()


def program_health(name, uptime=None):
    required = readiness_pattern(name)
    health = (LOG_HEALTH.snapshot(log_path(name), required=required) if required
              else LOG_HEALTH.snapshot(log_path(name)))
    if name == 'sirius_ble_battery':
        health = battery_health(health, TF_MONITOR.snapshot(DEFAULT_DOMAIN))
    else:
        health = readiness_health(health, name, uptime)
    return health


def has_error(name, uptime=None):
    return program_health(name, uptime)['state'] == 'error'


def process_uptime(pid):
    try:
        started = psutil.Process(int(pid)).create_time()
    except (psutil.Error, OSError, TypeError, ValueError):
        return None
    return max(0, int(time.time() - started))


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

    def read_since(self, offset, raw=False):
        raw_output = raw
        with self.lock:
            base = self.total - len(self.text)
            if offset < base or offset > self.total:
                offset = max(base, self.total - 20000)
            raw = self.text[offset - base:]
            return (raw if raw_output else clean_term_text(raw)), self.total

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
_SCRIPT_SIG_CACHE = None


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


def _script_paths(command):
    """エイリアス内の実行可能な .sh パスを返す。"""
    try:
        parts = shlex.split(command)
    except ValueError:
        parts = command.split()
    for part in parts:
        if not part.endswith(".sh"):
            continue
        path = os.path.expanduser(os.path.expandvars(part))
        if os.path.isfile(path):
            yield path


def alias_script_signatures():
    """ラッパー .sh の中の ros2 launch/run を、外部起動プロセス照合用に拾う。

    start_roboteq.sh のように bash ラッパーが exec で消える場合でも、
    ``ros2 launch roboteq_ros2_driver ...`` から roboteq を特定できる。
    """
    global _SCRIPT_SIG_CACHE
    if _SCRIPT_SIG_CACHE is None:
        cache = {}
        pattern = re.compile(
            r"\s*(?:exec\s+)?ros2\s+(launch|run)\s+"
            r"([^\s;&|'\"$]+)\s+([^\s;&|'\"$]+)")
        for name, command in BUTTONS.items():
            signatures = set()
            for path in _script_paths(command):
                try:
                    text = Path(path).read_text(encoding="utf-8", errors="ignore")
                except OSError:
                    continue
                for line in text.splitlines():
                    if line.lstrip().startswith("#"):
                        continue
                    match = pattern.match(line)
                    if match:
                        signatures.add(
                            f"ros2 {match.group(1)} {match.group(2)} {match.group(3)}")
            if signatures:
                cache[name] = tuple(sorted(signatures))
        _SCRIPT_SIG_CACHE = cache
    return _SCRIPT_SIG_CACHE


def best_alias_for(cmd):
    """cmdに一致する最も具体的な署名を持つエイリアス名を返す。"""
    best = ""
    best_len = 0
    actual_params = launch_parameters(cmd)
    for n, var in alias_variant_signatures().items():
        candidates = [(var, True)]
        candidates.extend((sig, False) for sig in alias_script_signatures().get(n, ()))
        for sig, from_alias in candidates:
            if not sig or len(sig) < 6 or sig not in cmd:
                continue
            expected = {}
            if from_alias and sig.startswith('ros2 launch '):
                expected = launch_parameters(BUTTONS[n])
            actual = actual_params
            if from_alias and sig == 'ros2 launch sirius_description display.launch.py':
                # This launch starts RViz by default; a TF-only process must never
                # be attributed to an alias which expects that default RViz window.
                expected = {'rviz': 'true', **expected}
                actual = {'rviz': 'true', **actual_params}
            if any(actual.get(key) != value for key, value in expected.items()):
                continue
            # TF-only and RViz launches share a launch file. Explicit parameters
            # identify the variant regardless of their order in the process argv.
            specificity = len(sig) + sum(len(key)+len(value)+3 for key, value in expected.items())
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
        r["error"] = has_error(r["alias"], r["age"]) if r["alias"] else False
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


def session_owner_map(sessions=None):
    """command_session とその子孫の PID -> セッション名。"""
    sessions = managed_sessions() if sessions is None else sessions
    owners = {}
    for name, infos in sessions.items():
        for info in infos:
            owners[info["pid"]] = name
            try:
                for child in psutil.Process(info["pid"]).children(recursive=True):
                    owners[child.pid] = name
            except psutil.Error:
                pass
    return owners


def classify_process_rows(rows, aliases, managed_pids, tracked_aliases, owners):
    """Monitor用に、管理下・隠れ・無関係を判定した行を返す。

    aliases: PID -> 起動可能コマンドのエイリアス名
    managed_pids: LauncherのPIDファイルまたはセッション管理下のPID集合
    tracked_aliases: エイリアス名 -> Launcherが把握しているPID（未起動ならNone）
    """
    out = []
    for row in rows:
        pid = row.get("pid")
        alias = aliases.get(pid) or None
        managed = pid in managed_pids
        hidden = bool(alias and not managed)
        related = bool(row.get("related") or alias or managed)
        hidden_kind = ""
        if hidden:
            hidden_kind = ("duplicate" if tracked_aliases.get(alias) is not None
                           else "external")
        unrelated = bool((row.get("command") or "").strip()) and not related
        out.append({**row, "alias": alias, "session": owners.get(pid, ""),
                    "managed": managed, "hidden": hidden, "hidden_kind": hidden_kind,
                    "related": related, "unrelated": unrelated})
    return out


def enrich_resource_processes(rows):
    """現在の全プロセスを、起動可能コマンドとの照合結果で分類する。"""
    sessions = managed_sessions()
    owners = session_owner_map(sessions)
    aliases = {row.get("pid"): best_alias_for(row.get("command") or "")
               for row in rows}
    tracked = {}
    managed = set(owners)
    try:
        managed.add(os.getpid())
        managed.update(child.pid for child in
                       psutil.Process(os.getpid()).children(recursive=True))
    except psutil.Error:
        pass
    for alias in {name for name in aliases.values() if name}:
        try:
            pid = running_pid(alias)
        except (psutil.Error, OSError):
            pid = None
        tracked[alias] = pid
        if pid is None:
            continue
        managed.add(pid)
        try:
            managed.update(child.pid for child in
                           psutil.Process(pid).children(recursive=True))
        except psutil.Error:
            pass
    return classify_process_rows(rows, aliases, managed, tracked, owners)


def external_matches(sessions=None, owners=None):
    """外部起動プロセスをエイリアスへ対応付ける（エイリアス名 -> [pid]）。

    command_session の子孫は、外部起動ではなくそのセッションのプログラムとして
    扱う。これにより同じプロセスが外部起動名とセッション名で二重表示されない。
    """
    sessions = managed_sessions() if sessions is None else sessions
    owners = session_owner_map(sessions) if owners is None else owners
    res = {}
    for r in proc_rows():
        if r["pid"] in owners:
            continue
        n = best_alias_for(r["cmd"])
        if n:
            res.setdefault(n, []).append(r["pid"])
    return res


def ps_snapshot():
    rows = proc_rows()
    return annotate_rows(rows)


def save_presets_to_file(path, presets, backup=True, metadata=None):
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
    metadata = read_preset_metadata(path) if metadata is None else metadata
    block = []
    for name, items in presets:
        block.append(f"# PRESET: {name}")
        block.append(f"# PRESET_ITEMS: {','.join(items)}")
        settings = preset_presentation(name, items, metadata)
        block.append('# PRESET_UI: ' + json.dumps(settings, ensure_ascii=False))
        block.append("")
    if backup:
        import shutil
        shutil.copy(path, path + ".bak")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(block + tail))


def load_aliases(path):
    global BUTTONS, GROUPS, PRESETS, _SCRIPT_SIG_CACHE
    groups, presets, descs = parse_bash_aliases(path, include_group_descriptions=True)
    BUTTONS = {}
    _SIG_CACHE.clear()
    _VAR_CACHE.clear()
    _SCRIPT_SIG_CACHE = None
    GROUPS = []
    for gname, items in groups.items():
        its = []
        for it in items:
            name, cmd, desc = it[0], it[1], it[2]
            sub = it[3] if len(it) > 3 else ""
            its.append({"name": name, "desc": desc, "subgroup": sub})
            BUTTONS[name] = cmd
        GROUPS.append({"name": gname, "desc": descs.get(gname, ""), "items": its})
    metadata = read_preset_metadata(path)
    PRESETS = [{"name": n, "items": i, **preset_presentation(n, i, metadata)} for n, i in presets]


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
    return get_manager(name).is_running() or bool(managed_sessions().get(name))


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


def managed_sessions():
    """Web起動の command_session.py を、PIDファイルに依存せず名前で検出する。

    PIDファイルが消えた/古い場合でも、Roboteqなどの起動中セッションを
    状態表示・二重起動防止・停止の対象にする。
    """
    sessions = {}
    now = time.time()
    for proc in psutil.process_iter(["pid", "cmdline", "create_time"]):
        try:
            cmdline = proc.info["cmdline"] or []
            if not any(os.path.basename(str(part)) == "command_session.py"
                       for part in cmdline):
                continue
            name = ""
            for i, part in enumerate(cmdline):
                part = str(part)
                if part == "--name" and i + 1 < len(cmdline):
                    name = str(cmdline[i + 1])
                    break
                if part.startswith("--name="):
                    name = part.split("=", 1)[1]
                    break
            if not name or name not in BUTTONS:
                continue
            age = int(now - (proc.info.get("create_time") or now))
            sessions.setdefault(name, []).append({"pid": proc.info["pid"], "age": age})
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    for infos in sessions.values():
        infos.sort(key=lambda info: info["pid"])
    return sessions


def alias_actually_running(name):
    """pidは生きているが中身が別物/残骸、という偽「起動中」を除外して判定する。"""
    m = get_manager(name)
    if not m.is_running():
        return False
    sig = alias_signature(BUTTONS.get(name, ""))
    if not sig or len(sig) < 6:
        return True
    return any(process_matches_alias(name, row["cmd"]) for row in proc_rows())


def _load_launch_specs_locked():
    global LAUNCH_SPECS, LAUNCH_SPECS_LOADED
    if LAUNCH_SPECS_LOADED:
        return
    LAUNCH_SPECS_LOADED = True
    try:
        data = json.loads(LAUNCH_SPECS_FILE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return
    if isinstance(data, dict):
        LAUNCH_SPECS = {
            str(name): value for name, value in data.items()
            if isinstance(value, dict)
        }


def _save_launch_specs_locked():
    try:
        LAUNCH_SPECS_FILE.parent.mkdir(parents=True, exist_ok=True)
        temp = LAUNCH_SPECS_FILE.with_name(LAUNCH_SPECS_FILE.name + ".tmp")
        temp.write_text(json.dumps(LAUNCH_SPECS, ensure_ascii=False, indent=2),
                        encoding="utf-8")
        os.replace(temp, LAUNCH_SPECS_FILE)
    except OSError as error:
        print(f"起動入力の保存に失敗: {error}")


def record_launch_spec(name, domain, command):
    if not name:
        return
    with LAUNCH_SPECS_LOCK:
        _load_launch_specs_locked()
        LAUNCH_SPECS[name] = {
            "domain": str(domain).strip(),
            "command": str(command),
            "updated": int(time.time()),
        }
        _save_launch_specs_locked()


def launch_specs_snapshot():
    with LAUNCH_SPECS_LOCK:
        _load_launch_specs_locked()
        return {name: dict(value) for name, value in LAUNCH_SPECS.items()}


def program_identity(name):
    pid = running_pid(name)
    if pid is None:
        sessions = managed_sessions()
        if sessions.get(name):
            pid = sessions[name][0]['pid']
        else:
            pids = external_matches(sessions, session_owner_map(sessions)).get(name, [])
            pid = pids[0] if pids else None
    return process_identity(pid)


def process_identity(pid):
    if pid is None:
        return None
    try:
        return [int(pid), psutil.Process(int(pid)).create_time()]
    except (ValueError, psutil.NoSuchProcess, psutil.AccessDenied):
        return None


def record_preset_origin(name, preset):
    identity = program_identity(name)
    if identity is None:
        return
    with LAUNCH_SPECS_LOCK:
        _load_launch_specs_locked()
        spec = LAUNCH_SPECS.setdefault(name, {})
        origins = spec.get('presets', []) if spec.get('identity') == identity else []
        spec.update(identity=identity, presets=list(dict.fromkeys([*origins, preset])))
        _save_launch_specs_locked()


def attach_preset_origins(status):
    specs = launch_specs_snapshot()
    for name, state in status.items():
        spec = specs.get(name, {})
        identity = process_identity(state.get('pid'))
        state['started_by_presets'] = (spec.get('presets', [])
                                      if identity is not None and spec.get('identity') == identity else [])


def running_program_names():
    """Return only launcher-managed programs that are currently alive."""
    sessions = managed_sessions()
    external = external_matches(sessions, session_owner_map(sessions))
    names = set(sessions) | set(external)
    for name in set(MANAGERS) | scan_pid_files():
        if name in BUTTONS and running_pid(name) is not None:
            names.add(name)
    return sorted(name for name in names if name in BUTTONS)


def restart_all_programs(domain="", restore=False):
    names = running_program_names()
    if not names:
        return {"count": 0, "stopped": [], "started": [], "restored": [], "failed": []}

    specs = launch_specs_snapshot() if restore else {}
    plans = []
    for name in names:
        spec = specs.get(name, {})
        if restore and spec:
            plan_domain = str(spec.get("domain", "")).strip()
            plan_command = str(spec.get("command", "")).strip() or BUTTONS.get(name, "")
        else:
            plan_domain = str(domain).strip()
            plan_command = BUTTONS.get(name, "")
        plans.append((name, plan_domain, plan_command, bool(restore and spec)))

    stopped = []
    failed = []
    ready = set()
    for name, _, _, _ in plans:
        ok, message = stop_one(name)
        if ok:
            stopped.append(name)
            ready.add(name)
        elif message == "not started":
            ready.add(name)
        else:
            failed.append({"name": name, "action": "stop", "error": message})
        time.sleep(0.1)
    time.sleep(0.4)

    started = []
    restored = []
    for name, plan_domain, plan_command, was_restored in plans:
        if name not in ready:
            continue
        ok, message = start_one(name, plan_domain, command=plan_command)
        if ok:
            started.append(name)
            if was_restored:
                restored.append(name)
            time.sleep(0.3)
        else:
            failed.append({"name": name, "action": "start", "error": message})
    return {
        "count": len(names),
        "stopped": stopped,
        "started": started,
        "restored": restored,
        "failed": failed,
    }


def start_one(name, domain="", command=None):
    if name not in BUTTONS:
        return False, "unknown alias"
    launch_command = str(command).strip() if command is not None else BUTTONS.get(name, "")
    if not launch_command:
        return False, "empty command"
    m = get_manager(name)
    m.command = launch_command
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
    # PIDファイルが失われたWebセッションも、名前が一致すれば起動済みとみなす
    sessions = managed_sessions()
    if sessions.get(name):
        return True, f"already running (session pid {sessions[name][0]['pid']})"
    # 外部起動も含めて同一プログラムが動いていれば二重起動しない
    sig = alias_signature(launch_command)
    if sig and len(sig) >= 6:
        for row in proc_rows():
            if process_matches_alias(name, row["cmd"]):
                return True, f"already running (external pid {row['pid']})"
    # 専用ポートが使用中なら古いプロセスを停止してから起動する（例: foxgloveの8766）
    ports = alias_ports(launch_command, name)
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
    record_launch_spec(name, use_domain, launch_command)
    return True, "started"


def stop_one(name):
    m = get_manager(name)
    sessions = managed_sessions().get(name, [])
    if m.is_running():
        result = m.stop()
        if result:
            # 同じ名前の別セッションが残っている場合も停止する
            for info in sessions:
                if psutil.pid_exists(info["pid"]):
                    _kill_pid_tree(info["pid"])
            # アプリ版と同様に、少し待ってから強制終了しタブを閉じる
            time.sleep(1.5)
            m.force_kill_tree(result["pids"], result["terminator_pid"])
            m.close_terminator_tab(result["terminator_pid"])
            return True, "stopped"
    if sessions:
        # PIDファイルが失われたWebセッション（例: 古いRoboteq）を停止
        stopped = False
        for info in sessions:
            if _kill_pid_tree(info["pid"]):
                stopped = True
        if stopped:
            m.pid_file_content = None
            if os.path.exists(m.pid_file):
                try:
                    os.remove(m.pid_file)
                except OSError:
                    pass
            return True, "stopped (session)"
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


def auth_ok():
    if not PASSWORD:
        return True
    return request.cookies.get("pw") == PASSWORD or request.args.get("pw") == PASSWORD


register_documentation_notes(app, auth_ok)
register_file_browser(app, auth_ok)
register_face(app, auth_ok)
register_ear(app, auth_ok, domain=lambda: DEFAULT_DOMAIN)
register_hotspot(app, auth_ok)
register_research_archive(app, auth_ok)


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


def visualization_defaults():
    """Browser-facing defaults; both services are configured independently."""
    parsed = urlsplit(request.url_root)
    host = parsed.hostname or "127.0.0.1"
    if ":" in host and not host.startswith("["):
        host = f"[{host}]"
    scheme = parsed.scheme or "http"
    viewer_url = (os.environ.get("SIRIUS_LICHTBLICK_URL")
                  or f"{scheme}://{host}:{8443 if scheme == 'https' else VISUALIZATION_VIEWER_PORT}/")
    bridge_scheme = "wss" if scheme == "https" else "ws"
    bridge_url = (os.environ.get("SIRIUS_FOXGLOVE_WS")
                  or f"{bridge_scheme}://{host}:{8767 if scheme == 'https' else VISUALIZATION_BRIDGE_PORT}")
    return {
        "viewer_url": viewer_url,
        "bridge_url": bridge_url,
        "layout_url": request.url_root.rstrip("/") + "/api/visualization/layout",
        "viewer_version": os.environ.get("SIRIUS_LICHTBLICK_VERSION", "1.29.1"),
    }


def _layout_id_valid(layout_id):
    return bool(re.fullmatch(r"[A-Za-z0-9_-]{1,64}", str(layout_id or "")))


def _layout_url(layout_id):
    return request.url_root.rstrip("/") + "/api/visualization/layouts/" + layout_id


def _layout_response(body):
    resp = make_response(body)
    resp.mimetype = "application/json"
    resp.headers["Access-Control-Allow-Origin"] = "*"
    resp.headers["Cross-Origin-Resource-Policy"] = "cross-origin"
    resp.headers["Cache-Control"] = "no-store"
    return resp


def _saved_layout_path(layout_id):
    return VISUALIZATION_LAYOUTS_DIR / (layout_id + ".json")


def _read_saved_layout(layout_id):
    try:
        data = json.loads(_saved_layout_path(layout_id).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    if not isinstance(data, dict) or not isinstance(data.get("layout"), dict):
        return None
    return data


def _saved_layout_entries():
    entries = []
    try:
        paths = sorted(VISUALIZATION_LAYOUTS_DIR.glob("*.json"))
    except OSError:
        paths = []
    for path in paths:
        layout_id = path.stem
        if not _layout_id_valid(layout_id):
            continue
        data = _read_saved_layout(layout_id)
        if not data:
            continue
        entries.append({
            "id": layout_id,
            "name": str(data.get("name") or layout_id),
            "builtin": False,
            "url": _layout_url(layout_id),
            "updated": data.get("updated"),
        })
    entries.sort(key=lambda entry: entry["name"].lower())
    return entries


def _builtin_layout_entry():
    return {
        "id": "default",
        "name": "標準",
        "builtin": True,
        "url": _layout_url("default"),
        "updated": None,
    }


@app.route("/api/visualization/config")
def api_visualization_config():
    return jsonify(visualization_defaults())


@app.route("/api/visualization/layout")
def api_visualization_layout():
    """Default Lichtblick layout; CORS is required for layoutUrl in the iframe."""
    try:
        body = VISUALIZATION_LAYOUT.read_text(encoding="utf-8")
    except OSError:
        return jsonify({"error": "layout unavailable"}), 503
    return _layout_response(body)


@app.route("/api/visualization/layouts")
def api_visualization_layouts():
    with LAYOUTS_LOCK:
        layouts = [_builtin_layout_entry()] + _saved_layout_entries()
    return jsonify({"layouts": layouts, "default": "default"})


@app.route("/api/visualization/layouts/<layout_id>")
def api_visualization_named_layout(layout_id):
    if layout_id == "default":
        try:
            body = VISUALIZATION_LAYOUT.read_text(encoding="utf-8")
        except OSError:
            return jsonify({"error": "layout unavailable"}), 503
        return _layout_response(body)
    if not _layout_id_valid(layout_id):
        return jsonify({"error": "invalid layout id"}), 400
    with LAYOUTS_LOCK:
        data = _read_saved_layout(layout_id)
    if not data:
        return jsonify({"error": "layout not found"}), 404
    return _layout_response(json.dumps(data["layout"], ensure_ascii=False))


@app.route("/api/visualization/layouts", methods=["POST"])
def api_visualization_layouts_save():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    if request.content_length and request.content_length > 2_000_000:
        return jsonify({"error": "layout is too large"}), 413
    body = request.json or {}
    name = str(body.get("name", "")).strip()
    layout = body.get("layout")
    if not name or len(name) > 80:
        return jsonify({"error": "name is required (max 80 chars)"}), 400
    if not isinstance(layout, dict):
        return jsonify({"error": "layout must be a JSON object"}), 400
    layout_id = str(body.get("id", "")).strip() or uuid.uuid4().hex[:12]
    if layout_id == "default" or not _layout_id_valid(layout_id):
        return jsonify({"error": "invalid layout id"}), 400
    saved = {
        "name": name,
        "updated": int(time.time()),
        "layout": layout,
    }
    try:
        with LAYOUTS_LOCK:
            VISUALIZATION_LAYOUTS_DIR.mkdir(parents=True, exist_ok=True)
            path = _saved_layout_path(layout_id)
            temp = path.with_name(path.name + ".tmp")
            temp.write_text(json.dumps(saved, ensure_ascii=False, indent=2),
                            encoding="utf-8")
            os.replace(temp, path)
    except OSError as error:
        return jsonify({"error": str(error)}), 500
    return jsonify({"layout": {
        "id": layout_id,
        "name": name,
        "builtin": False,
        "url": _layout_url(layout_id),
        "updated": saved["updated"],
    }})


@app.route("/api/visualization/layouts/<layout_id>", methods=["DELETE"])
def api_visualization_layout_delete(layout_id):
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    if layout_id == "default" or not _layout_id_valid(layout_id):
        return jsonify({"error": "cannot delete layout"}), 400
    with LAYOUTS_LOCK:
        path = _saved_layout_path(layout_id)
        try:
            path.unlink()
        except FileNotFoundError:
            return jsonify({"error": "layout not found"}), 404
        except OSError as error:
            return jsonify({"error": str(error)}), 500
    return jsonify({"deleted": layout_id})


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
        sessions = managed_sessions()
        for name in set(MANAGERS) | scan_pid_files():
            pid = running_pid(name)
            if pid is None:
                continue
            uptime = process_uptime(pid)
            health = program_health(name, uptime)
            out[name] = {"running": True, "pid": pid, "returncode": None,
                         "error": health['state'] == 'error', "health": health,
                         "domain": "", "source": "launcher", "uptime": uptime}
        # PIDファイルが無いWebセッションも稼働中として返す（外部起動より優先）
        for name, infos in sessions.items():
            if name in BUTTONS and name not in out:
                uptime = process_uptime(infos[0]["pid"])
                health = program_health(name, uptime)
                out[name] = {"running": True, "pid": infos[0]["pid"], "returncode": None,
                             "error": health['state'] == 'error', "health": health,
                             "domain": "", "source": "session",
                             "pids": [info["pid"] for info in infos],
                             "uptime": uptime}
        # 外部（別ターミナル等）で起動済みのものを署名一致で拾う。
        # セッションの子孫は二重表示しない。
        for name, pids in external_matches(
                sessions, session_owner_map(sessions)).items():
            if name not in out:
                uptime = process_uptime(pids[0])
                health = program_health(name, uptime)
                out[name] = {"running": True, "pid": pids[0], "returncode": None,
                             "error": health['state'] == 'error', "health": health,
                             "domain": "", "source": "external", "pids": pids,
                             "uptime": uptime}
    for name, state in out.items():
        # The PTY owner stays an ancestor of its child across Web server restarts.
        # Older sessions keep working through the read-only /proc fallback.
        wait = session_request(name).get('input_wait') if name in sessions else None
        if not isinstance(wait, dict) or 'state' not in wait:
            wait = INPUT_WAIT.snapshot(state['pid'], log_path(name), interactive=name in sessions)
        state['input_wait'] = wait
        state['input_waiting'] = wait['state'] in {'waiting', 'possible'}
    attach_preset_origins(out)
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
        names = (set(MANAGERS) | scan_pid_files() | set(managed_sessions())
                 | set(external_matches()))
        for name in sorted(names):
            stop_one(name)
    return jsonify({})


@app.route("/api/restart_all", methods=["POST"])
def api_restart_all():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    body = request.json or {}
    domain = str(body.get("domain", ""))
    restore = bool(body.get("restore"))
    with LOCK:
        result = restart_all_programs(domain, restore)
    return jsonify(result)


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
    started, skipped, failed, seen = [], [], [], set()
    with LOCK:
        for n in items:
            if n in seen:
                skipped.append(n)
                continue
            seen.add(n)
            ok, msg = start_one(n, domain)
            if ok:
                record_preset_origin(n, name)
            if ok and msg.startswith("already running"):
                skipped.append(n)
            elif ok:
                started.append(n)
                time.sleep(0.5)
            else:
                failed.append({"name": n, "error": msg})
    return jsonify({"started": started, "skipped": skipped, "failed": failed})


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
    if not isinstance(raw, list) or any(not isinstance(p, dict) for p in raw):
        return jsonify({"error": "プリセットの形式が不正です"}), 400
    clean = []
    metadata = read_preset_metadata(ALIAS_FILE)
    names = set()
    for p in raw:
        name = str(p.get("name", "")).strip()
        category = p.get('category', metadata.get(name, {}).get('category', ''))
        note = p.get('note', metadata.get(name, {}).get('note', ''))
        if not isinstance(note, str) or len(note) > 2000:
            return jsonify({'error': 'メモは2000文字までで入力してください'}), 400
        if (len(name) > 200 or '\n' in name or '\r' in name or name in names
                or not isinstance(category, str) or len(category) > 60
                or '\n' in category or '\r' in category
                or ('visible' in p and type(p['visible']) is not bool)
                or not isinstance(p.get('items', []), list)):
            return jsonify({"error": "名前は重複なしで200文字、用途は60文字までの1行で入力してください"}), 400
        if not name or not p.get('items'):
            return jsonify({'error': 'プリセット名と起動するプログラムを1つ以上設定してください'}), 400
        unknown = [a for a in p['items'] if not isinstance(a, str) or a not in BUTTONS]
        if unknown:
            return jsonify({'error': '登録されていないプログラムが含まれています。一覧から選び直してください'}), 400
        items = list(dict.fromkeys(p['items']))
        if name and items:
            clean.append((name, items))
            names.add(name)
            metadata[name] = {'category': category, 'visible': p.get('visible', metadata.get(name, {}).get('visible', True)), 'note': note}
    try:
        with LOCK:
            save_presets_to_file(ALIAS_FILE, clean, metadata=metadata)
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


@app.route('/api/emergency_stop', methods=['POST'])
def api_emergency_stop():
    if not auth_ok():
        return jsonify({'error': 'auth'}), 401
    body = request.get_json(silent=True) or {}
    if not isinstance(body, dict):
        return jsonify({'error': 'invalid body'}), 400
    active = body.get('active')
    if type(active) is not bool:
        return jsonify({'error': 'active must be boolean'}), 400
    if not EMERGENCY_STOP_LOCK.acquire(blocking=False):
        return jsonify({'error': '停止指令を送信中です'}), 409
    try:
        if not active:
            if body.get('confirmed') is not True:
                return jsonify({'error': '解除には確認が必要です'}), 400
            telemetry = TF_MONITOR.snapshot(DEFAULT_DOMAIN)
            driver = (telemetry.get('battery') or {}).get('driver') or {}
            age = driver.get('age_seconds')
            if (not telemetry.get('available') or driver.get('state') != 'live'
                    or not isinstance(age, (int, float)) or not 0 <= age <= 5
                    or age+max(0, time.time()-telemetry.get('timestamp', 0)) > 5
                    or driver.get('emergency_stop') is not True):
                return jsonify({'error': '電子停止の現在の状態を確認できないため解除しません'}), 409
        result = send_emergency_stop(active, DEFAULT_DOMAIN)
        return jsonify(result), 200 if result.get('sent') else 502
    finally:
        EMERGENCY_STOP_LOCK.release()


@app.route("/api/teleop", methods=["POST"])
def api_teleop():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    body = request.get_json(silent=True)
    if not isinstance(body, dict):
        return jsonify({"error": "invalid request"}), 400
    action = body.get('action')
    session, sequence = body.get('session'), body.get('seq')
    if (action not in ('arm', 'heartbeat', 'key', 'disable', 'stop')
            or not isinstance(session, str) or not re.fullmatch(r'[a-zA-Z0-9_-]{16,80}', session)
            or type(sequence) is not int or sequence < 1):
        return jsonify({"error": "invalid action, session or sequence"}), 400
    if action == 'key' and (type(body.get('key')) is not int or body['key'] not in VelocityControl.KEYS):
        return jsonify({"error": "invalid key"}), 400
    if 'route_mode' in body and (action != 'arm' or body['route_mode'] not in ('auto', 'mux')):
        return jsonify({'error': 'invalid route mode'}), 400
    domain = str(body.get('domain') or DEFAULT_DOMAIN or '53')
    if not domain.isdigit() or not 0 <= int(domain) <= 232:
        return jsonify({"error": "invalid ROS domain"}), 400
    result = TELEOP.command({k: body[k] for k in ('action', 'session', 'seq', 'key', 'route_mode') if k in body}, domain)
    result = {**TELEOP.status(session), **result}
    return jsonify(result), 200 if result.get('ok') else 409


@app.route("/api/teleop/status")
def api_teleop_status():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    response = jsonify(TELEOP.status(request.args.get('session')))
    response.headers['Cache-Control'] = 'no-store'
    return response


@app.route("/api/tf")
def api_tf():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    snapshot = TF_MONITOR.snapshot(DEFAULT_DOMAIN)
    snapshot['readiness'] = tf_readiness(snapshot)
    response = jsonify(snapshot)
    response.headers["Cache-Control"] = "no-store"
    return response


@app.route("/api/topics")
def api_topics():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    response = jsonify(TOPIC_MONITOR.snapshot(DEFAULT_DOMAIN))
    response.headers["Cache-Control"] = "no-store"
    return response


@app.route("/api/topics/measure", methods=["POST"])
def api_topics_measure():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    body = request.json or {}
    result = TOPIC_MONITOR.measure(DEFAULT_DOMAIN, body.get("topic"), body.get("duration"))
    status = result.pop("status", 200)
    return jsonify(result), status


@app.route("/api/battery")
def api_battery():
    if not auth_ok():
        return jsonify({"error": "auth"}), 401
    telemetry = TF_MONITOR.snapshot(DEFAULT_DOMAIN)
    data = dict(telemetry.get("battery") or {"state": "waiting"})
    if not telemetry.get("available"):
        data["state"] = "receiver_error" if telemetry.get("error") else "connecting"
        data["receiver_error"] = telemetry.get("error", "")
        for source in ("battery", "driver", "details"):
            if isinstance(data.get(source), dict):
                data[source] = {**data[source], "state": data["state"]}
    response = jsonify({**data, "domain": telemetry.get("domain", DEFAULT_DOMAIN)})
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
        data["processes"] = enrich_resource_processes(data["processes"])
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
    data, new_offset = s.read_since(offset, raw=request.args.get('raw') == '1')
    return jsonify({"data": data, "offset": new_offset, "alive": s.alive})


@app.route('/api/program_terminal/read')
def api_program_terminal_read():
    if not auth_ok():
        return jsonify({'error': 'auth'}), 401
    name = request.args.get('name', '')
    if name not in BUTTONS:
        return jsonify({'error': 'unknown alias'}), 404
    try:
        offset = max(0, int(request.args.get('offset', 0)))
    except ValueError:
        return jsonify({'error': 'invalid offset'}), 400
    interactive = session_request(name).get('ok', False)
    try:
        with open(log_path(name), 'rb') as stream:
            stream.seek(0, os.SEEK_END)
            size = stream.tell()
            reset = offset > size or size - offset > 524288
            start = max(0, size - 131072) if reset or offset == 0 else offset
            stream.seek(start)
            payload = stream.read(131072)
            decoder = codecs.getincrementaldecoder('utf-8')('replace')
            text = decoder.decode(payload, final=False)
            pending, _ = decoder.getstate()
            return jsonify(data=text, offset=start+len(payload)-len(pending),
                           reset=reset, interactive=interactive)
    except FileNotFoundError:
        return jsonify(data='', offset=0, reset=offset != 0, interactive=interactive)


@app.route('/api/term/resize', methods=['POST'])
def api_term_resize():
    if not auth_ok():
        return jsonify({'error': 'auth'}), 401
    body = request.json or {}
    cols, rows = body.get('cols'), body.get('rows')
    if type(cols) is not int or type(rows) is not int or not (2 <= cols <= 500 and 2 <= rows <= 300):
        return jsonify({'error': 'invalid terminal size'}), 400
    with TERMS_LOCK:
        session = TERMS.get(str(body.get('name', '')))
        if session is None:
            return jsonify({'error': 'no such term'}), 404
        try:
            fcntl.ioctl(session.fd, termios.TIOCSWINSZ, struct.pack('HHHH', rows, cols, 0, 0))
        except OSError:
            return jsonify({'error': 'terminal closed'}), 409
    return jsonify(ok=True)


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
    with LAUNCH_SPECS_LOCK:
        _load_launch_specs_locked()
    RESOURCES.start()
    threading.Thread(target=TELEOP.ensure, args=(DEFAULT_DOMAIN,), daemon=True).start()
    print(f"SiriusLauncher Web: http://{args.host}:{args.port}  "
          f"({len(BUTTONS)} aliases, {len(GROUPS)} groups, {len(PRESETS)} presets)")
    if PASSWORD:
        print("パスワード保護: 有効")
    app.config['SIRIUS_LISTEN_PORT'] = args.port
    app.run(host=args.host, port=args.port, threaded=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
