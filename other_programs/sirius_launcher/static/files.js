'use strict';
(()=>{
 const state={path:'',data:null,favorites:[],selected:null,checked:new Set(),sequence:0,previewSequence:0,controller:null,previewController:null,imageURL:null,initialized:false};
 const el=id=>document.getElementById(id),node=(tag,cls,text)=>{const n=document.createElement(tag);if(cls)n.className=cls;if(text!==undefined)n.textContent=text;return n;};
 const fileGlyph = name => `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><use href="/static/file-entry-icons.svg#${name}"/></svg>`;
 function fileIcon(directory) {
  const icon = node('span', 'file-entry-icon' + (directory ? ' file-entry-icon-folder' : ''));
  icon.innerHTML = fileGlyph(directory ? 'folder' : 'file');
  return icon;
 }
 el('files-up').innerHTML = fileGlyph('up');
 el('files-new-folder').innerHTML = fileGlyph('add-folder') + '<span>新規フォルダ</span>';
 const actionPaths = {
  copy: '<rect x="8" y="8" width="12" height="12" rx="2"/><path d="M16 8V6a2 2 0 0 0-2-2H6a2 2 0 0 0-2 2v8a2 2 0 0 0 2 2h2"/>',
  move: '<path d="M3 6a2 2 0 0 1 2-2h4l2 3h8a2 2 0 0 1 2 2v9a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2zM9 13h7m-2.5-2.5L16 13l-2.5 2.5"/>',
  rename: '<path d="m4 16-1 5 5-1L20 8l-4-4L4 16Zm10-10 4 4"/>',
  trash: '<path d="M4 7h16M9 7V4h6v3m3 0-1 13H7L6 7m4 4v5m4-5v5"/>',
  refresh: '<path d="M4 10a8 8 0 1 1 1 7m-1-12v5h5"/>',
 };
 function actionLabel(button, action, label){
  button.classList.add('files-icon-action');
  button.innerHTML = `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${actionPaths[action]}</svg>`;
  button.append(node('span','',label));
 }
 actionLabel(el('files-copy-selected'),'copy','複製');
 actionLabel(el('files-move-selected'),'move','移動');
 actionLabel(el('files-trash'),'trash','ゴミ箱');
 actionLabel(el('files-refresh'),'refresh','更新');
 const bytes=n=>{if(n===null||n===undefined)return '—';for(const [v,u]of [[1e9,'GB'],[1e6,'MB'],[1e3,'KB']])if(n>=v)return (n/v).toFixed(1)+' '+u;return n+' B';};
 async function fileApi(path,body){const response=await fetch(path,{method:body?'POST':'GET',headers:{'Content-Type':'application/json'},body:body?JSON.stringify(body):undefined,cache:'no-store',signal:body?undefined:AbortSignal.timeout(8000)});const result=await response.json();if(!response.ok)throw Error(result.error==='auth'?'ログインが必要です':result.error||'HTTP '+response.status);return result;}
 const editButton=node('button','hbtn','編集');editButton.id='files-edit';
 const mapButton=node('button','hbtn','Open in Map Editor');
 mapButton.type='button';mapButton.hidden=true;
 mapButton.onclick=()=>{if(state.selected)window.openMapInEditor?.(state.selected);};
 el('files-download').before(mapButton);editButton.type='button';editButton.hidden=true;el('files-download').before(editButton);
 editButton.onclick=async()=>{const originalPath=state.selected;if(!state.textDocument)return;editButton.disabled=true;try{const document=await fileApi(url('text',originalPath));if(state.selected!==originalPath)return;if(!document.editable)throw Error(document.edit_note||'このファイルは編集できません');state.textDocument=document;openHostFileEditor(document,(text,result)=>{if(state.selected!==originalPath)return;state.textDocument={...document,text,revision:result.revision,size:result.size};el('files-preview-content').replaceChildren(node('pre','files-text',text));const entry=state.data?.entries.find(e=>e.path===originalPath);if(entry){entry.size=result.size;entry.modified=Date.now()/1000;rows();}});}catch(error){message(error.message,true);}finally{editButton.disabled=false;}};
 const url=(endpoint,path)=>'/api/files/'+endpoint+'?path='+encodeURIComponent(path);
 function message(text,error=false){el('files-status').textContent=text;el('files-status').classList.toggle('files-error',error);}
 function favorites(){const box=el('files-favorites');box.replaceChildren();box.hidden=!state.favorites.length;
  for(const favorite of state.favorites){const item=node('span','files-favorite'),open=node('button','',favorite.label);open.type='button';open.title=favorite.path;open.onclick=()=>navigate(favorite.path);const remove=node('button','files-favorite-remove','×');remove.type='button';remove.title='お気に入りから削除';remove.setAttribute('aria-label',favorite.label+'をお気に入りから削除');remove.onclick=()=>saveFavorite(favorite.path,true);item.append(open,remove);box.append(item);}
  const active=state.favorites.some(f=>f.path===state.path);el('files-star').textContent=active?'★':'☆';el('files-star').setAttribute('aria-pressed',String(active));el('files-star').title=active?'お気に入りから削除':'このフォルダをお気に入りに登録';
 }
 async function saveFavorite(path,remove){try{const result=await fileApi('/api/files/favorites',{action:remove?'remove':'add',path});if(result.error)throw Error(result.error);state.favorites=result.favorites;favorites();}catch(error){message(error.message,true);}}
 function clearPreview(){mapButton.hidden=true;state.textDocument=null;editButton.hidden=true;state.previewSequence++;state.previewController?.abort();if(state.imageURL)URL.revokeObjectURL(state.imageURL);state.imageURL=null;state.selected=null;
  const empty=node('div','files-preview-empty');empty.append(fileIcon(false),node('p','','ファイルを選択するとプレビューできます'));
  el('files-preview-content').replaceChildren(empty);el('files-preview-title').textContent='プレビュー';el('files-download').hidden=true;el('files-zoom').hidden=true;}
 function crumbs(){const box=el('files-crumbs');box.replaceChildren();let path='';for(const part of state.path.split('/').filter(Boolean)){path+='/'+part;const value=path,button=node('button','',part);button.type='button';button.onclick=()=>navigate(value);if(value===state.path)button.setAttribute('aria-current','location');box.append(button);}el('files-path').value=state.path;}
 function visibleEntries(){const search=el('files-search').value.toLocaleLowerCase();return (state.data?.entries||[]).filter(e=>e.name.toLocaleLowerCase().includes(search));}
 function selection(){const count=state.checked.size;el('files-selected-count').textContent=count+'件選択';el('files-move-selected').disabled=el('files-copy-selected').disabled=!count;el('files-clear-selected').disabled=!count;const visible=visibleEntries(),selected=visible.filter(e=>state.checked.has(e.path)).length;el('files-select-all').checked=visible.length>0&&selected===visible.length;el('files-select-all').indeterminate=selected>0&&selected<visible.length;}
 function rows(){const box=el('files-rows');box.replaceChildren();if(!state.data)return;const search=el('files-search').value;const entries=visibleEntries();if(el('files-sort').value==='modified')entries.sort((a,b)=>Number(b.directory)-Number(a.directory)||b.modified-a.modified||a.name.localeCompare(b.name));
  for(const entry of entries){const button=node('button','files-row'+(state.selected===entry.path?' selected':''));button.type='button';button.title=entry.path;button.append(fileIcon(entry.directory),node('span','files-name',entry.name+(entry.symlink?' ↗':'')),node('span','files-size',bytes(entry.size)),node('time','files-time',new Date(entry.modified*1000).toLocaleString('ja-JP',{month:'2-digit',day:'2-digit',hour:'2-digit',minute:'2-digit'})));button.onclick=()=>entry.directory?navigate(entry.path):preview(entry);const item=node('div','files-entry'),checkLabel=node('label','files-entry-check'),check=node('input'),more=node('button','files-more','⋯');more.type='button';more.setAttribute('aria-label',entry.name+'の操作');more.title='複製・名前変更・移動・削除';more.onclick=()=>entryActions(entry);check.type='checkbox';check.checked=state.checked.has(entry.path);check.setAttribute('aria-label',entry.name+'を選択');check.onchange=()=>{if(check.checked)state.checked.add(entry.path);else state.checked.delete(entry.path);selection();};checkLabel.append(check);item.append(checkLabel,button,more);box.append(item);}
  if(!entries.length)box.append(node('p','files-empty',search?'一致する項目はありません':'このフォルダは空です'));el('files-count').textContent=entries.length+'件'+(state.data.truncated?'（先頭5,000件）':'');selection();
 }
 async function navigate(path){const sequence=++state.sequence;state.controller?.abort();state.controller=new AbortController();message('読み込み中…');try{const response=await fetch(url('list',path)+'&hidden='+(el('files-hidden').checked?'1':'0'),{signal:state.controller.signal,cache:'no-store'});const data=await response.json();if(!response.ok||data.error)throw Error(data.error||'フォルダを開けません');if(sequence!==state.sequence)return;if(state.path!==data.path)state.checked.clear();else{const available=new Set(data.entries.map(e=>e.path));state.checked=new Set([...state.checked].filter(path=>available.has(path)));}state.path=data.path;state.data=data;state.favorites=data.favorites;clearPreview();crumbs();favorites();rows();el('files-up').disabled=!data.parent;el('files-star').disabled=false;const bookmarks=el('files-bookmarks');bookmarks.replaceChildren();for(const bookmark of data.bookmarks){const button=node('button','hbtn',bookmark.label);button.type='button';button.classList.add('files-icon-action');button.prepend(fileIcon(true));button.onclick=()=>navigate(bookmark.path);const active=data.path===bookmark.path||data.path.startsWith(bookmark.path+'/');button.toggleAttribute('data-active',active&&(bookmark.path!==data.bookmarks.at(-1)?.path||data.path===bookmark.path));bookmarks.append(button);}message(data.truncated?'項目が多いため先頭5,000件を表示しています':'');state.initialized=true;}catch(error){if(error.name!=='AbortError')message(error.message,true);}}
 async function preview(entry){clearPreview();state.selected=entry.path;
 mapButton.hidden=!(window.openMapInEditor && /[/]maps_waypoints[/](maps|map_archive)[/].*\.(yaml|yml|pgm|colored\.json)$/i.test(entry.path));
 rows();el('files-preview-title').textContent=entry.name;el('files-download').href=url('download',entry.path);el('files-download').hidden=false;const content=el('files-preview-content');content.replaceChildren(node('p','muted','読み込み中…'));const sequence=state.previewSequence;state.previewController=new AbortController();try{
  if(entry.kind==='image'){const response=await fetch(url('preview',entry.path),{signal:state.previewController.signal,cache:'no-store'});if(!response.ok)throw Error((await response.json()).error||'画像を表示できません');const blob=await response.blob();if(sequence!==state.previewSequence)return;state.imageURL=URL.createObjectURL(blob);const image=node('img','files-image');image.src=state.imageURL;image.alt=entry.name+'のプレビュー';const stage=node('div','files-image-stage');stage.append(image);const info=node('p','files-image-info',response.headers.get('X-Image-Width')+' × '+response.headers.get('X-Image-Height')+' px · '+bytes(entry.size));content.replaceChildren(info,stage);el('files-zoom').hidden=false;el('files-zoom').textContent='拡大表示';el('files-zoom').onclick=()=>{stage.classList.toggle('expanded');el('files-zoom').textContent=stage.classList.contains('expanded')?'全体表示':'拡大表示';};
  }else if(entry.kind==='text'){const response=await fetch(url('text',entry.path),{signal:state.previewController.signal,cache:'no-store'});const result=await response.json();if(!response.ok||result.error)throw Error(result.error||'表示できません');if(sequence!==state.previewSequence)return;content.replaceChildren(node('pre','files-text',result.text));state.textDocument={...result,name:entry.name};editButton.hidden=!result.editable;if(result.edit_note)content.prepend(node('p','muted',result.edit_note));if(result.truncated)content.prepend(node('p','muted','先頭512 KBを表示しています。全文はダウンロードできます。'));
  }else{content.replaceChildren(node('p','files-empty',bytes(entry.size)+' · この形式はダウンロードして確認できます。'));}
 }catch(error){if(error.name!=='AbortError'&&sequence===state.previewSequence)content.replaceChildren(node('p','files-error',error.message));}}
 el('files-path-form').onsubmit=e=>{e.preventDefault();navigate(el('files-path').value);};el('files-refresh').onclick=()=>navigate(state.path);el('files-up').onclick=()=>state.data?.parent&&navigate(state.data.parent);el('files-star').onclick=()=>saveFavorite(state.path,state.favorites.some(f=>f.path===state.path));el('files-search').oninput=rows;el('files-sort').onchange=rows;el('files-hidden').onchange=()=>navigate(state.path);

 const dialog=node('dialog','files-dialog');dialog.setAttribute('aria-label','ファイル操作');document.body.append(dialog);let operationBusy=false,pickerSequence=0;
 dialog.addEventListener('cancel',event=>{if(operationBusy)event.preventDefault();});
 function modal(title){pickerSequence++;dialog.replaceChildren();const heading=node('h2','',title);dialog.append(heading);if(!dialog.open)dialog.showModal();return dialog;}
 function cancelButton(){const cancel=node('button','hbtn','キャンセル');cancel.type='button';cancel.onclick=()=>dialog.close();return cancel;}
 function errorArea(){const error=node('p','files-error');error.setAttribute('role','alert');dialog.append(error);return error;}
 async function perform(body,error) {
  if(operationBusy)return;
  operationBusy=true;
  const controls=[...dialog.querySelectorAll('button:not([disabled]),input:not([disabled])')];
  controls.forEach(control=>control.disabled=true);
  const copying=['copy','copy_many'].includes(body.action);
  error.classList.remove('files-error');
  error.textContent=copying?'複製中…':'処理中…';
  try {
   const result=await fileApi('/api/files/operate',body);
   if(result.error)throw Error(result.error);
   dialog.close();
   if(copying)state.checked.clear();
   await navigate(copying?body.destination:state.path);
   if(result.partial){
    const count=(copying?result.copied:result.moved).length;
    message((copying?'複製完了 ':'移動完了 ')+count+'件・未完了 '+result.remaining.length+'件：'+result.error_message,true);
   }else{
    message(result.warning||({
     copy:'「'+result.copied?.[0]?.path.split('/').at(-1)+'」を作成しました',
     copy_many:result.copied?.length+'件を複製しました',move_many:result.moved?.length+'件を移動しました',
     mkdir:'フォルダを作成しました',rename:'名前を変更しました',move:'移動しました',trash:'ゴミ箱へ移動しました',
    })[body.action]);
   }
  }catch(e){
   error.classList.add('files-error');
   error.textContent=e.message;
  }finally{
   operationBusy=false;
   controls.forEach(control=>control.disabled=false);
  }
 }
 function entryActions(entry) {
  modal('ファイル操作');
  const source=node('div','files-copy-source');
  const label=node('div','files-source-name');
  label.append(node('strong','',entry.name),node('p','files-operation-path',entry.path));
  source.append(fileIcon(entry.directory),label);
  dialog.append(source);
  const actions=node('div','files-operation-actions');
  for(const [action,label]of [['copy','複製'],['rename','名前を変更'],['move','別のフォルダへ移動'],['trash','ゴミ箱へ移動']]){
   const button=node('button','hbtn'+(action==='trash'?' files-danger':''));
   button.type='button';
   actionLabel(button,action,label);
   button.onclick=()=>action==='copy'?copyEntry(entry):editEntry(action,entry);
   actions.append(button);
  }
  const footer=node('div','files-operation-buttons');
  footer.append(cancelButton());
  dialog.append(actions,footer);
 }
 function defaultCopyName(entry) {
  let suffix=entry.directory&&!entry.symlink?'':(/\.[^.]+$/.exec(entry.name)||[''])[0];
  if(entry.name===suffix)suffix='';
  const encoder=new TextEncoder();
  if(encoder.encode(suffix).length>249)suffix='';
  const ending='-copy'+suffix;
  const characters=Array.from(suffix?entry.name.slice(0,-suffix.length):entry.name);
  while(encoder.encode(characters.join('')+ending).length>255)characters.pop();
  return characters.join('')+ending;
 }
 function copyEntry(entry) {
  const batch=Boolean(entry.entries?.length>1);
  const source=batch?null:entry.entries?.[0]||entry;
  modal(batch?'選択した項目を複製':source.directory?'フォルダを複製':'ファイルを複製');
  const summary=node('div','files-copy-source');
  const sourceLabel=node('div','files-source-name');
  sourceLabel.append(node('strong','',batch?entry.entries.length+'件を複製':source.name));
  if(batch)sourceLabel.append(node('p','files-operation-path','同じフォルダでは名前に -copy を追加します。'));
  else sourceLabel.append(node('p','files-operation-path',source.path));
  summary.append(fileIcon(batch||source.directory),sourceLabel);
  dialog.append(summary);
  const form=node('form','files-operation-form');
  const destinationLabel=node('label','','複製先フォルダ');
  const destination=node('input');
  destination.type='text';destination.value=state.path;destination.required=true;
  destinationLabel.append(destination);
  const picker=node('div','files-folder-picker');
  form.append(destinationLabel,picker);
  let name;
  if(!batch){
   const nameLabel=node('label','','複製後の名前');
   name=node('input');name.type='text';name.value=defaultCopyName(source);name.required=true;name.maxLength=255;
   nameLabel.append(name);
   form.append(nameLabel);
  }
  const error=node('p','files-operation-status');error.setAttribute('role','status');
  const footer=node('div','files-operation-buttons');
  const submit=node('button','primary-button');
  submit.type='submit';actionLabel(submit,'copy','複製する');
  footer.append(cancelButton(),submit);
  form.append(error,footer);
  form.onsubmit=event=>{
   event.preventDefault();
   const payload=batch?{action:'copy_many',paths:entry.entries.map(e=>e.path)}:{action:'copy',path:source.path,name:name.value};
   perform({...payload,destination:destination.value},error);
  };
  dialog.append(form);
  folderPicker(picker,destination,state.path,entry);
  if(name){name.focus();name.select();}
  else destination.focus();
 }
 function editEntry(action,entry){modal(action==='mkdir'?'新規フォルダ':action==='rename'?'名前を変更':action==='move'?'フォルダを選んで移動':'ゴミ箱へ移動');dialog.append(node('p','files-operation-path',entry?.path||state.path));if(entry?.entries)dialog.append(node('p','',entry.entries.length+'件をまとめて移動します。'));
  if(action==='trash'){dialog.append(node('p','',entry.name+'をゴミ箱へ移動しますか？'));dialog.append(node('p','muted','後からゴミ箱で元に戻せます。この操作ではディスクの空き容量は増えません。'));const error=errorArea(),buttons=node('div','files-operation-buttons'),confirm=node('button','hbtn files-danger','ゴミ箱へ移動');confirm.type='button';confirm.onclick=()=>perform({action,path:entry.path},error);buttons.append(cancelButton(),confirm);dialog.append(buttons);return;}
  const form=node('form','files-operation-form'),label=node('label','',action==='move'?'移動先フォルダ':'名前'),input=node('input');input.type='text';input.required=true;input.value=action==='mkdir'?'新しいフォルダ':action==='rename'?entry.name:state.path;input.setAttribute('aria-label',action==='move'?'移動先フォルダ':'名前');label.append(input);form.append(label);dialog.append(form);
  if(action==='move'){const picker=node('div','files-folder-picker');form.append(picker);folderPicker(picker,input,state.path,entry);}
  if(action==='rename')form.append(node('p','muted','このファイル名を参照しているYAML・起動設定などは自動更新されません。'));
  const error=node('p','files-error');error.setAttribute('role','alert');const buttons=node('div','files-operation-buttons'),submit=node('button','hbtn',action==='mkdir'?'作成':action==='rename'?'変更':'ここへ移動');submit.type='submit';buttons.append(cancelButton(),submit);form.append(error,buttons);form.onsubmit=e=>{e.preventDefault();perform(action==='mkdir'?{action,parent:state.path,name:input.value}:action==='rename'?{action,path:entry.path,name:input.value}:entry.entries?{action:'move_many',paths:entry.entries.map(e=>e.path),destination:input.value}:{action,path:entry.path,destination:input.value},error);};input.focus();if(action!=='move')input.select();
 }
 async function folderPicker(box,input,path,entry){const seq=++pickerSequence;box.replaceChildren(node('p','muted','フォルダを読み込み中…'));try{const data=await fileApi(url('list',path));if(data.error)throw Error(data.error);if(seq!==pickerSequence||!dialog.open)return;input.value=data.path;box.replaceChildren();const quick=node('div','files-picker-shortcuts');for(const bookmark of [...data.bookmarks,...state.favorites]){const b=node('button','',bookmark.label);b.type='button';b.onclick=()=>folderPicker(box,input,bookmark.path,entry);quick.append(b);}box.append(quick,node('p','files-operation-path',data.path));const list=node('div','files-picker-list');if(data.parent){const up=node('button','files-picker-folder');up.innerHTML=fileGlyph('up')+'<span>親フォルダ</span>';up.type='button';up.onclick=()=>folderPicker(box,input,data.parent,entry);list.append(up);}for(const folder of data.entries.filter(row=>row.directory)){const b=node('button','files-picker-folder');b.append(fileIcon(true),node('span','',folder.name));b.type='button';const blocked=(entry.entries||[entry]).some(e=>e.directory&&!e.symlink&&(folder.path===e.path||folder.path.startsWith(e.path+'/')));b.disabled=blocked;b.onclick=()=>folderPicker(box,input,folder.path,entry);list.append(b);}box.append(list);}catch(e){if(seq===pickerSequence)box.replaceChildren(node('p','files-error',e.message));}}
 async function showTrash(){modal('Filesのゴミ箱');dialog.append(node('p','muted','この画面から削除したファイル・フォルダを元に戻せます。'));const list=node('div','files-trash-list');dialog.append(list,cancelButton());try{const data=await fileApi('/api/files/trash');if(data.error)throw Error(data.error);if(!dialog.open)return;if(!data.entries.length)list.append(node('p','muted','ゴミ箱は空です'));for(const entry of data.entries){const row=node('div','files-trash-item'),details=node('div');details.append(node('strong','',entry.name),node('p','files-operation-path',entry.original),node('small','muted',entry.deleted));const restore=node('button','hbtn','元に戻す');restore.type='button';restore.onclick=async()=>{restore.disabled=true;try{const result=await fileApi('/api/files/restore',{id:entry.id});if(result.error)throw Error(result.error);row.remove();if(!list.children.length)list.append(node('p','muted','ゴミ箱は空です'));await navigate(state.path);}catch(e){let error=row.querySelector('.files-error');if(!error){error=node('p','files-error');details.append(error);}error.textContent=e.message;restore.disabled=false;}};row.append(details,restore);list.append(row);}}catch(e){list.append(node('p','files-error',e.message));}}
 el('files-select-all').onchange=()=>{for(const entry of visibleEntries()){if(el('files-select-all').checked)state.checked.add(entry.path);else state.checked.delete(entry.path);}rows();};el('files-clear-selected').onclick=()=>{state.checked.clear();rows();};const checkedEntries=()=>state.data?.entries.filter(e=>state.checked.has(e.path))||[];
 el('files-move-selected').onclick=()=>{const entries=checkedEntries();if(entries.length)editEntry('move',{entries,path:entries.map(e=>e.name).join('、')});};
 el('files-copy-selected').onclick=()=>{const entries=checkedEntries();if(entries.length)copyEntry({entries});};
 el('files-new-folder').onclick=()=>editEntry('mkdir');el('files-trash').onclick=showTrash;
 window.fileBrowserPageChanged=()=>{if(!state.initialized)navigate('');};
})();
