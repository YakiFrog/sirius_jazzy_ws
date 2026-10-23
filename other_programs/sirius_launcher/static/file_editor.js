'use strict';
(()=>{
 const dialog=document.createElement('dialog');dialog.className='files-editor';dialog.id='files-editor';dialog.setAttribute('aria-labelledby','files-editor-title');
 dialog.innerHTML='<header class="files-editor-heading"><div><h2 id="files-editor-title">ファイルを編集</h2><p id="files-editor-path"></p></div><button type="button" class="hbtn" id="files-editor-close" aria-label="編集画面を閉じる">×</button></header><div class="files-editor-toolbar"><span id="files-editor-dirty">変更なし</span><button type="button" class="hbtn" id="files-editor-format">JSONを整形</button><button type="button" class="hbtn" id="files-editor-save" disabled>保存</button></div><textarea id="files-editor-text" aria-label="ファイルの内容" spellcheck="false" wrap="off"></textarea><footer class="files-editor-footer"><p id="files-editor-status" role="status" aria-live="polite"></p><a id="files-editor-backup" hidden>保存前の版をダウンロード</a><small>Ctrl＋S / ⌘S で保存</small></footer>';
 document.body.append(dialog);
 const $=id=>document.getElementById(id),text=$('files-editor-text');let current=null,busy=false,savedCallback=null;
 const dirty=()=>!!current&&text.value!==current.original;
 function message(value,error=false){$('files-editor-status').textContent=value;$('files-editor-status').classList.toggle('files-error',error);}
 function paint(){const changed=dirty();$('files-editor-save').disabled=busy||!changed;$('files-editor-save').textContent=busy?'保存中…':'保存';$('files-editor-close').disabled=busy;$('files-editor-format').disabled=busy;text.readOnly=busy;$('files-editor-dirty').textContent=changed?'未保存の変更あり':'変更なし';$('files-editor-dirty').classList.toggle('changed',changed);}
 function close(){if(busy)return;if(dirty()&&!confirm('編集内容が未保存です。変更を破棄して閉じますか？'))return;dialog.close();current=null;savedCallback=null;}
 function checkJson(){if(current?.json)JSON.parse(text.value);}
 async function save(){if(busy||!dirty())return;try{checkJson();}catch(error){message('JSONの形式を確認してください：'+error.message,true);return;}
  busy=true;paint();message('保存中…');const sent=text.value;
  try{const response=await fetch('/api/files/text',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({path:current.path,revision:current.revision,text:sent}),signal:AbortSignal.timeout(15000)});const result=await response.json();if(!response.ok)throw Error(result.error==='auth'?'ログインが必要です':result.error||'保存できません');current.revision=result.revision;current.original=sent;message('保存しました。保存前の内容もバックアップ済みです。');const backup=$('files-editor-backup');backup.href='/api/files/download?path='+encodeURIComponent(result.backup_path);backup.hidden=false;if(savedCallback)savedCallback(sent,result);}
  catch(error){message(error.name==='TimeoutError'?'保存結果を確認できません。入力内容は残っています。':error.message,true);}
  finally{busy=false;paint();}
 }
 window.openHostFileEditor=(document,onSaved)=>{if(!document?.editable||busy)return;if(dialog.open&&dirty()&&!confirm('編集中の変更を破棄して、別のファイルを開きますか？'))return;if(typeof DRIVE!=='undefined'&&DRIVE.enabled)driveDisable();current={path:document.path,revision:document.revision,json:document.name.toLowerCase().endsWith('.json')};text.value=document.text;current.original=text.value;savedCallback=onSaved;$('files-editor-title').textContent=document.name;$('files-editor-path').textContent=document.path;$('files-editor-format').hidden=!current.json;$('files-editor-backup').hidden=true;message('');paint();if(!dialog.open)dialog.showModal();text.focus();};
 text.addEventListener('input',()=>{message('');paint();});$('files-editor-close').onclick=close;$('files-editor-save').onclick=save;
 $('files-editor-format').onclick=()=>{try{checkJson();text.value=JSON.stringify(JSON.parse(text.value),null,2)+'\n';message('整形しました。保存するとファイルに反映されます。');paint();}catch(error){message('JSONの形式を確認してください：'+error.message,true);}};
 dialog.addEventListener('cancel',event=>{event.preventDefault();close();});
 dialog.addEventListener('keydown',event=>{event.stopPropagation();if((event.ctrlKey||event.metaKey)&&event.key.toLowerCase()==='s'){event.preventDefault();save();}else if(event.target===text&&event.key==='Tab'&&!event.shiftKey&&!event.ctrlKey&&!event.metaKey){event.preventDefault();text.setRangeText('  ',text.selectionStart,text.selectionEnd,'end');paint();}});
 window.addEventListener('beforeunload',event=>{if(dialog.open&&(dirty()||busy)){event.preventDefault();event.returnValue='';}});
})();
