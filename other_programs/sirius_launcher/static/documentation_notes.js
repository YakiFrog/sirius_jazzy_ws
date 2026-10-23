'use strict';
window.GUIDE_MODE='manual';
const NOTES={ready:false,preparing:null,list:[],id:null,revision:null,dirty:false,busy:false,preview:true,autoOpen:true};
const NOTE_DRAFT='sirius-guide-note-draft-v1';
const NOTE_TEMPLATE='# 起動するプログラム\n\n- [ ] （プログラム名を記入）\n- [ ] （プログラム名を記入）\n- [ ] （プログラム名を記入）\n\n## 起動後の確認\n\n- [ ] LiDAR・IMUのトピックが出ている\n- [ ] カメラの準備が完了している\n- [ ] TF Treeに競合がない\n\n## 記録・終了\n\n1. 記録コマンドと保存先：\n2. 終了時に確認すること：\n\n## 実験メモ\n\n';
const noteTools=document.createElement('div');noteTools.className='guide-document-tools';
noteTools.innerHTML='<div class="segmented" role="group" aria-label="ドキュメントの種類"><button id="guide-tab-manual" class="active" type="button" aria-pressed="true" onclick="setGuideMode(\'manual\')">操作ガイド</button><button id="guide-tab-notes" type="button" aria-pressed="false" onclick="setGuideMode(\'notes\')">自分のメモ</button></div><button class="hbtn" id="guide-note-new" type="button" onclick="newGuideNote()">＋ メモを作成</button>';
document.querySelector('.guide-header').after(noteTools);
const noteHost=document.createElement('section');noteHost.id='guide-notes';noteHost.hidden=true;noteHost.setAttribute('aria-label','自分のメモ');noteTools.after(noteHost);
function noteStatus(message,error=false){const e=document.getElementById('note-status');if(e){e.textContent=message;e.classList.toggle('note-error',error);}}
async function notesApi(path,body,method){
  const response=await fetch('/api/documentation/notes'+path,{method:method||'GET',headers:{'Content-Type':'application/json'},body:body?JSON.stringify(body):undefined,cache:'no-store',signal:AbortSignal.timeout(10000)});
  const data=await response.json();if(!response.ok)throw new Error(data.error==='auth'?'ログインが必要です':data.error||'メモを読み書きできません');return data;
}
function noteFields(){return {id:NOTES.id,revision:NOTES.revision,title:document.getElementById('note-title').value,body:document.getElementById('note-body').value};}
function saveNoteDraft(){
  NOTES.dirty=true;try{localStorage.setItem(NOTE_DRAFT,JSON.stringify(noteFields()));noteStatus('未保存 · この端末に下書き保存');}catch(error){noteStatus('未保存 · 端末への下書き保存ができません',true);}
  if(NOTES.preview)renderNotePreview();
}
function clearNoteDraft(){try{localStorage.removeItem(NOTE_DRAFT);}catch(error){}}
function noteBusy(busy){
  NOTES.busy=busy;document.querySelectorAll('#note-title,#note-body,#note-form button').forEach(e=>e.disabled=busy);
  document.getElementById('note-delete').disabled=busy||!NOTES.id;
}
function showNote(note,dirty=false){
  NOTES.id=note.id||null;NOTES.revision=note.revision||null;NOTES.dirty=dirty;
  document.getElementById('note-title').value=note.title||'';document.getElementById('note-body').value=note.body||'';
  document.getElementById('note-heading').textContent=NOTES.id?'メモを編集':'新しいメモ';
  noteStatus(dirty?'下書きを復元しました · 未保存':note.updated_at?'保存済み · '+new Date(note.updated_at).toLocaleString('ja-JP'):'未保存');
  noteBusy(NOTES.busy);setNotePreview(!dirty);renderNoteList();
}
async function prepareGuideNotes(){
  if(NOTES.ready)return;if(NOTES.preparing)return NOTES.preparing;
  NOTES.preparing=(async()=>{
    const response=await fetch('/static/documentation_notes.html',{signal:AbortSignal.timeout(8000)});
    if(!response.ok)throw new Error('メモ画面を読み込めません');noteHost.innerHTML=await response.text();NOTES.ready=true;
    document.getElementById('note-form').addEventListener('submit',saveGuideNote);
    for(const id of ['note-title','note-body'])document.getElementById(id).addEventListener('input',saveNoteDraft);
    let draft=null;try{draft=JSON.parse(localStorage.getItem(NOTE_DRAFT));}catch(error){}
    const valid=draft&&typeof draft.title==='string'&&typeof draft.body==='string';showNote(valid?draft:{},!!valid);
  })();
  try{await NOTES.preparing;}finally{NOTES.preparing=null;}
}
async function setGuideMode(mode){
  window.GUIDE_MODE=mode==='notes'?'notes':'manual';const notes=GUIDE_MODE==='notes';
  noteHost.hidden=!notes;document.getElementById('guide-content').hidden=notes;
  for(const name of ['manual','notes']){const b=document.getElementById('guide-tab-'+name);b.classList.toggle('active',name===GUIDE_MODE);b.setAttribute('aria-pressed',String(name===GUIDE_MODE));}
  document.getElementById('guide-no-results').hidden=true;
  if(notes){
    try{await prepareGuideNotes();loadGuideNotes();}catch(error){if(!NOTES.ready){noteHost.textContent=error.message;}else noteStatus(error.message,true);}
  }else filterDocumentation();
}
async function loadGuideNotes(){
  try{await prepareGuideNotes();const query=document.getElementById('guide-search').value;const data=await notesApi('?q='+encodeURIComponent(query));if(query!==document.getElementById('guide-search').value)return;NOTES.list=data.notes;renderNoteList();if(NOTES.autoOpen){NOTES.autoOpen=false;if(!NOTES.id&&!NOTES.dirty&&NOTES.list.length)await openGuideNote(NOTES.list[0].id);}}
  catch(error){noteStatus(error.message,true);}
}
function renderNoteList(){
  if(!NOTES.ready)return;const list=document.getElementById('notes-list');list.replaceChildren();
  const visible=NOTES.list;
  for(const n of visible){
    const button=document.createElement('button');button.type='button';button.dataset.noteId=n.id;button.className='note-list-item';button.classList.toggle('selected',n.id===NOTES.id);
    const title=document.createElement('strong');title.textContent=n.title;const summary=document.createElement('span');summary.textContent=n.excerpt||'本文なし';
    const date=document.createElement('small');date.textContent=new Date(n.updated_at).toLocaleString('ja-JP');button.append(title,summary,date);button.onclick=()=>openGuideNote(n.id);const row=document.createElement('div');row.setAttribute('role','listitem');row.append(button);list.append(row);
  }
  if(!visible.length){const empty=document.createElement('p');empty.className='note-help';empty.textContent=document.getElementById('guide-search').value.trim()?'検索に一致するメモがありません':'メモはまだありません。「メモを作成」から書き始められます。';list.append(empty);}
}
let noteSearchTimer;
document.getElementById('guide-search').addEventListener('input',()=>{if(GUIDE_MODE==='notes'){clearTimeout(noteSearchTimer);noteSearchTimer=setTimeout(loadGuideNotes,250);}});
function canLeaveNote(){return !NOTES.busy&&(!NOTES.dirty||confirm('未保存の内容があります。破棄して別のメモを開きますか？'));}
async function newGuideNote(template=false){
  NOTES.autoOpen=false;
  await setGuideMode('notes');if(!NOTES.ready||!canLeaveNote())return;
  clearNoteDraft();showNote({title:template?'地図撮りの起動手順':'',body:template?NOTE_TEMPLATE:''});
  setNotePreview(false);if(template)saveNoteDraft();document.getElementById('note-title').focus();
}
async function openGuideNote(id){
  if(id===NOTES.id||!canLeaveNote())return;noteBusy(true);
  try{const data=await notesApi('/'+id);clearNoteDraft();showNote(data.note);if(matchMedia('(max-width:900px)').matches)document.getElementById('note-form').scrollIntoView({block:'start',behavior:'smooth'});}catch(error){noteStatus(error.message,true);}finally{noteBusy(false);}
}
async function saveGuideNote(event){
  event.preventDefault();if(NOTES.busy)return;const values=noteFields();noteBusy(true);noteStatus('保存中…');
  try{const data=await notesApi(values.id?'/'+values.id:'',values,values.id?'PUT':'POST');clearNoteDraft();showNote(data.note);await loadGuideNotes();}
  catch(error){noteStatus(error.message,true);}finally{noteBusy(false);}
}
function duplicateGuideNote(){
  if(NOTES.busy)return;const value=noteFields();showNote({title:(value.title||'メモ')+'（コピー）',body:value.body});setNotePreview(false);saveNoteDraft();
}
async function deleteGuideNote(){
  if(NOTES.busy||!NOTES.id||!confirm('「'+document.getElementById('note-title').value+'」を削除しますか？'))return;
  noteBusy(true);try{await notesApi('/'+NOTES.id,{revision:NOTES.revision},'DELETE');clearNoteDraft();showNote({});await loadGuideNotes();noteStatus('削除しました');}
  catch(error){noteStatus(error.message,true);}finally{noteBusy(false);}
}
function renderNotePreview(){
  const preview=document.getElementById('note-preview'),body=document.getElementById('note-body').value;
  try{
    const html=marked.parse(body,{gfm:true,breaks:false});
    preview.innerHTML=DOMPurify.sanitize(html,{USE_PROFILES:{html:true},FORBID_TAGS:['style','form','button','textarea','select'],FORBID_ATTR:['id','name','style','autofocus','contenteditable']});
    preview.querySelectorAll('input').forEach(e=>{if(e.type==='checkbox')e.disabled=true;else e.remove();});
    preview.querySelectorAll('a').forEach(a=>{a.target='_blank';a.rel='noopener noreferrer';});
    if(!body)preview.textContent=NOTES.id?'本文はまだありません。「編集する」から追加できます。':'一覧からメモを選ぶか、「メモを作成」から書き始めてください。';
  }catch(error){preview.textContent=body;}
}
function setNotePreview(on){
  NOTES.preview=!!on;document.getElementById('note-body-label').hidden=!!on;document.getElementById('note-preview').hidden=!on;
  document.getElementById('note-form').classList.toggle('is-reading',!!on);
  document.getElementById('note-title-field').hidden=!!on;document.getElementById('note-heading').hidden=!!on;
  const title=document.getElementById('note-reading-title');title.hidden=!on;title.textContent=document.getElementById('note-title').value||'メモを選択';
  document.getElementById('note-save').hidden=!!on&&!NOTES.dirty;document.getElementById('note-delete').hidden=!!on;
  document.querySelector('#note-form .note-actions').hidden=!!on&&!NOTES.id&&!document.getElementById('note-title').value&&!document.getElementById('note-body').value;
  document.querySelector('#note-form .note-help').hidden=!!on;
  for(const [id,active] of [['note-edit-tab',!on],['note-preview-tab',!!on]]){const b=document.getElementById(id);b.classList.toggle('active',active);b.setAttribute('aria-pressed',String(active));}
  if(on)renderNotePreview();
}
function exportGuideNote(){
  const value=noteFields(),file=new Blob(['# '+(value.title||'メモ')+'\n\n'+value.body],{type:'text/markdown;charset=utf-8'});
  const link=document.createElement('a'),url=URL.createObjectURL(file);link.href=url;link.download=(value.title||'メモ').replace(/[\\/:*?"<>|\x00-\x1f]/g,'_')+'.md';link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
}
