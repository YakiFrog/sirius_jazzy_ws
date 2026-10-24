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
import os
import sys
import threading
import time
from pathlib import Path

from flask import Flask, jsonify, request, make_response

from alias_parser import parse_bash_aliases
from process_manager import ProcessManager

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


def start_one(name, domain=""):
    if name not in BUTTONS:
        return False, "unknown alias"
    m = get_manager(name)
    if m.is_running():
        return True, "already running"
    use_domain = str(domain).strip() or str(DEFAULT_DOMAIN).strip()
    old_domain = os.environ.get("ROS_DOMAIN_ID")
    if use_domain:
        os.environ["ROS_DOMAIN_ID"] = use_domain
    try:
        # アプリ版と同じく Terminator の新規タブで起動する
        ok = m.launch()
    finally:
        if use_domain:
            if old_domain is None:
                os.environ.pop("ROS_DOMAIN_ID", None)
            else:
                os.environ["ROS_DOMAIN_ID"] = old_domain
    if not ok:
        return False, "launch failed"
    return True, "started"


def stop_one(name):
    m = get_manager(name)
    if not m.is_running():
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
.hidden{display:none}
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
.log{position:fixed;inset:5%;background:#0b0e12;border:1px solid #333;border-radius:8px;overflow:auto;padding:12px;font-family:monospace;font-size:12px;white-space:pre-wrap;display:none;z-index:10}
.logclose{position:fixed;right:calc(5% + 6px);top:calc(5% + 6px);display:none;z-index:11}
</style></head><body>
<header>
 <h1>SiriusLauncher Web</h1>
 <span id="cnt" style="color:#9aa4b2;font-size:12px"></span>
 <label style="font-size:12px;color:#9aa4b2">ROS_DOMAIN</label>
 <input id="domain" type="number" min="0" max="232" style="width:76px;background:#0f1115;color:#e6e6e6;border:1px solid #333;border-radius:6px;padding:5px 8px">
 <button class="hbtn" id="sidebarBtn" onclick="toggleSidebar()">◀ 左パネル</button>
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
<div class="log" id="log"></div>
<button class="xbtn logclose" id="logclose" onclick="closeLog()">閉じる</button>
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
 DATA.presets.forEach(pr=>{const b=document.createElement('button');b.className='preset';b.textContent='▶ '+pr.name;b.onclick=()=>runPreset(pr.name);p.appendChild(b);});}
function makeBtn(it){const b=document.createElement('button');b.className='btn';b.id='btn_'+it.name;
 b.innerHTML='<b>'+it.name+'</b><span class="st">...</span><span class="d">'+(it.desc||'')+'</span>';
 b.onclick=()=>toggle(it.name);return b;}
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
 document.querySelectorAll('.btn').forEach(b=>{const n=b.id.replace('btn_','');const s=st[n];
  const run=s&&s.running, err=s&&s.error;
  b.querySelector('.st').textContent=run?('起動中 pid '+s.pid):(err?'エラー':'停止');
  b.classList.toggle('run',!!run);b.classList.toggle('err',!!err);});
 DATA.groups.forEach((g,i)=>{let c=0;g.items.forEach(it=>{const s=st[it.name];if(s&&(s.running||s.error))c++;});
  const bd=document.getElementById('badge_'+i);if(bd){bd.textContent=c;bd.classList.toggle('hide',c===0);}});
 const run=Object.entries(st).filter(([n,s])=>s.running);
 document.getElementById('cnt').textContent='起動中: '+run.length+' 件';
 document.getElementById('runhint').textContent=run.length?('('+run.length+')'):'';
 const r=document.getElementById('running');
 r.innerHTML=run.length?run.map(([n,s])=>'<div class="ritem"><span class="n">'+n+' ('+s.pid+')</span><button class="xbtn gray" onclick="restartIt(\\''+n+'\\')">再起動</button><button class="xbtn" onclick="stopIt(\\''+n+'\\')">停止</button><button class="xbtn gray" onclick="showLog(\\''+n+'\\')">ログ</button></div>').join(''):'<div class="empty">起動中のプログラムはありません</div>';}
async function restartIt(n){await api('/api/restart',{name:n,domain:dom()});refresh();}
async function toggle(n){const st=await api('/api/status');const r=st[n]&&st[n].running;
 if(r){await api('/api/stop',{name:n});}else{const res=await api('/api/start',{name:n,domain:dom()});if(res.error)alert(res.error);}refresh();}
async function stopIt(n){await api('/api/stop',{name:n});refresh();}
async function stopAll(){if(!confirm('全て停止しますか？'))return;await api('/api/stop_all',{});refresh();}
async function restartServer(){if(!confirm('Webサーバー（SiriusLauncher Web）を再起動しますか？\\n起動中のプロセスは維持されます。'))return;
 try{await api('/api/restart_server',{});}catch(e){}
 setTimeout(()=>location.reload(),3000);}
async function runPreset(n){await api('/api/preset',{name:n,domain:dom()});refresh();}
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
async function showLog(n){const j=await api('/api/log?name='+encodeURIComponent(n));document.getElementById('log').textContent=j.log;document.getElementById('log').style.display='block';document.getElementById('logclose').style.display='block';}
function closeLog(){document.getElementById('log').style.display='none';document.getElementById('logclose').style.display='none';}
load();setInterval(refresh,2000);
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
                         "error": has_error(name), "domain": ""}
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
    try:
        with open(log_path(name), "r", errors="ignore") as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(max(0, size - 20000))
            return jsonify({"log": f.read()})
    except FileNotFoundError:
        return jsonify({"log": "(ログなし)"})


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
    print(f"SiriusLauncher Web: http://{args.host}:{args.port}  "
          f"({len(BUTTONS)} aliases, {len(GROUPS)} groups, {len(PRESETS)} presets)")
    if PASSWORD:
        print("パスワード保護: 有効")
    app.run(host=args.host, port=args.port, threaded=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
