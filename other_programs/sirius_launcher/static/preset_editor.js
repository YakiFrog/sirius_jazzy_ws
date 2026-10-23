'use strict';
const PresetEditor=(()=>{
  const rows=document.getElementById('prows'),modal=document.getElementById('pmodal');
  let baseline='',saving=false;
  const categories=['共通','地図生成','自律移動','自己位置推定','センサー','シミュレーション','その他'];
  const el=(tag,cls,text)=>{const e=document.createElement(tag);if(cls)e.className=cls;if(text!==undefined)e.textContent=text;return e;};
  const button=(text,label,action)=>{const b=el('button','hbtn',text);b.type='button';b.setAttribute('aria-label',label);b.onclick=action;return b;};
  function field(text,input){const label=el('label','preset-field');label.append(el('span','',text),input);return label;}
  function catalog(){return DATA.groups.flatMap(group=>group.items.map(item=>({...item,group:group.name})));}
  function categoryValue(row){const select=row.querySelector('.preset-category');return select.value==='__custom__'?row.querySelector('.preset-new-category').value.trim():select.value;}
  function collect(){return [...rows.querySelectorAll('.prow')].map(row=>({name:row.querySelector('.nm').value.trim(),items:[...row._items],category:categoryValue(row),visible:row.querySelector('.preset-visible input').checked,note:row.querySelector('.preset-memo').value}));}
  function add(name='',items='',category='',visible=true,note='',expand=true){
    const row=el('details','prow preset-editor-row');row._items=[...new Set(Array.isArray(items)?items:String(items).split(',').map(s=>s.trim()).filter(Boolean))];
    const summary=el('summary','preset-editor-summary');
    const drag=button('','ドラッグして並べ替え',e=>e.preventDefault());drag.className='pdrag';drag.title='ドラッグして並べ替え';
    drag.innerHTML='<svg viewBox="0 0 10 18" fill="currentColor" aria-hidden="true"><circle cx="2.5" cy="3" r="1.3"/><circle cx="7.5" cy="3" r="1.3"/><circle cx="2.5" cy="9" r="1.3"/><circle cx="7.5" cy="9" r="1.3"/><circle cx="2.5" cy="15" r="1.3"/><circle cx="7.5" cy="15" r="1.3"/></svg>';
    const title=el('strong','preset-row-title'),count=el('span','preset-row-count');summary.append(drag,title,count,el('span','preset-row-expand','⌄'));
    const content=el('div','preset-editor-content'),grid=el('div','preset-editor-fields');
    const nameInput=el('input','nm');nameInput.value=name;nameInput.maxLength=200;nameInput.placeholder='例：屋外の地図撮り';
    const select=el('select','preset-category');
    const choices=['',...new Set([...categories,...DATA.presets.map(p=>p.category).filter(Boolean),category].filter((v,i)=>i===0||v))];
    choices.push('__custom__');for(const value of choices){const option=el('option','',value==='__custom__'?'＋新しい用途…':value==='共通'?'共通（すべての用途）':value||'自動分類');option.value=value;select.append(option);}select.value=category||'';
    const newCategory=el('input','preset-new-category');newCategory.hidden=true;newCategory.maxLength=60;newCategory.placeholder='新しい用途名';newCategory.setAttribute('aria-label','新しい用途名');
    const categoryField=field('用途',select);categoryField.append(newCategory);
    const visibleLabel=el('label','preset-visible'),visibleInput=el('input');visibleInput.type='checkbox';visibleInput.checked=visible!==false;visibleInput.setAttribute('aria-label','クイックスタートに表示');visibleLabel.append(visibleInput,el('span','','クイックスタートに表示'));
    const memo=el('textarea','preset-memo');memo.rows=3;memo.maxLength=2000;memo.value=note||'';memo.placeholder='起動の前提や注意点、使う場面など';
    grid.append(field('セット名',nameInput),categoryField,visibleLabel);content.append(grid,field('メモ（2000文字まで）',memo));
    const planTitle=el('h4','','起動するプログラム'),plan=el('ol','preset-selected-programs');content.append(planTitle,plan);
    const chooseTitle=el('h4','','プログラムを選んで追加'),tools=el('div','preset-picker-tools');
    const search=el('input','preset-program-search');search.type='search';search.placeholder='名前・説明を検索';search.setAttribute('aria-label','追加するプログラムを検索');
    const group=el('select','preset-program-group');group.setAttribute('aria-label','プログラムのカテゴリ');
    const all=el('option','','すべてのカテゴリ');all.value='';group.append(all);
    DATA.groups.forEach(g=>{const option=el('option','',g.name);option.value=g.name;group.append(option);});
    tools.append(search,group);const list=el('div','preset-program-picker');list.setAttribute('role','group');list.setAttribute('aria-label','選択できるプログラム');content.append(chooseTitle,tools,list);
    const actions=el('div','preset-editor-row-actions');
    actions.append(button('セットを削除','このセットを削除',()=>{if(confirm('「'+(nameInput.value||'新しいセット')+'」を削除しますか？'))row.remove();}));content.append(actions);
    function summaryText(){title.textContent=nameInput.value.trim()||'新しいセット';count.textContent=row._items.length+'件';}
    function drawPlan(){
      plan.replaceChildren();summaryText();planTitle.textContent='起動するプログラム（'+row._items.length+'件・上から順）';
      const programs=new Map(catalog().map(p=>[p.name,p]));
      row._items.forEach((alias,index)=>{
        const item=programs.get(alias),li=el('li'),copy=el('div','preset-program-copy');copy.append(el('code','',alias),el('small','',item?.desc||'現在の定義にないプログラムです'));
        const controls=el('div','preset-order-controls');
        for(const [text,delta] of [['↑',-1],['↓',1]]){const move=button(text,alias+'を'+(delta<0?'上':'下')+'へ',()=>{const other=index+delta;[row._items[index],row._items[other]]=[row._items[other],row._items[index]];drawPlan();plan.children[other]?.querySelector('button:not(:disabled)')?.focus();});move.disabled=index+delta<0||index+delta>=row._items.length;controls.append(move);}
        controls.append(button('外す',alias+'をセットから外す',()=>{row._items.splice(index,1);drawPlan();drawChoices();}));li.append(copy,controls);plan.append(li);
      });
      if(!row._items.length)plan.append(el('li','empty','下の一覧からプログラムを選んでください。'));
    }
    function drawChoices(){
      const active=document.activeElement?.dataset.alias,query=search.value.trim().toLocaleLowerCase();list.replaceChildren();
      const found=catalog().filter(p=>(!group.value||p.group===group.value)&&(!query||(p.name+' '+p.desc).toLocaleLowerCase().includes(query)));
      found.forEach(item=>{const label=el('label','preset-program-option'),check=el('input');check.type='checkbox';check.checked=row._items.includes(item.name);check.dataset.alias=item.name;check.setAttribute('aria-label',item.name+'を選択');
        const copy=el('span','preset-program-copy');copy.append(el('code','',item.name),el('small','',item.desc||item.group));label.append(check,copy);list.append(label);
        check.onchange=()=>{if(check.checked){if(!row._items.includes(item.name))row._items.push(item.name);}else row._items=row._items.filter(n=>n!==item.name);drawPlan();drawChoices();};});
      if(!found.length)list.append(el('p','empty','一致するプログラムがありません。'));
      if(active)[...list.querySelectorAll('input')].find(input=>input.dataset.alias===active)?.focus();
    }
    search.oninput=drawChoices;group.onchange=drawChoices;nameInput.oninput=summaryText;
    select.onchange=()=>{newCategory.hidden=select.value!=='__custom__';if(!newCategory.hidden)newCategory.focus();};
    row.append(summary,content);rows.append(row);drawPlan();
    row.addEventListener('toggle',()=>{if(row.open){[...rows.children].filter(other=>other!==row).forEach(other=>other.open=false);drawChoices();}});
    row.open=expand;return row;
  }
  function open(){rows.replaceChildren();for(const preset of DATA.presets)add(preset.name,preset.items,preset.category,preset.visible,preset.note,false);baseline=JSON.stringify(collect());document.getElementById('perr').textContent='';modal.classList.add('open');if(rows.firstElementChild)rows.firstElementChild.open=true;}
  function close(){if(saving)return;if(JSON.stringify(collect())!==baseline&&!confirm('保存していない変更を破棄して閉じますか？'))return;modal.classList.remove('open');}
  async function save(){
    if(saving)return;const presets=collect(),names=new Set(),known=new Set(catalog().map(p=>p.name));
    for(let i=0;i<presets.length;i++){
      const p=presets[i];let error='';
      if(!p.name)error='セット名を入力してください。';else if(names.has(p.name))error='同じセット名が重複しています。';else if(!p.items.length)error='プログラムを1つ以上選んでください。';else if(p.items.some(name=>!known.has(name)))error='登録されていないプログラムを外して選び直してください。';else if(rows.children[i].querySelector('.preset-category').value==='__custom__'&&!p.category)error='新しい用途名を入力してください。';
      if(error){rows.children[i].open=true;rows.children[i].scrollIntoView({block:'nearest'});document.getElementById('perr').textContent=(p.name||'セット'+(i+1))+': '+error;return;}names.add(p.name);
    }
    saving=true;modal.setAttribute('aria-busy','true');rows.inert=true;
    const buttons=[...modal.querySelectorAll('button')].map(button=>[button,button.disabled]);buttons.forEach(([button])=>button.disabled=true);
    try{const res=await api('/api/presets/save',{presets});if(res.error)throw Error(res.error);DATA.presets=res.presets;baseline=JSON.stringify(collect());renderTabs();modal.classList.remove('open');toast('プリセットとメモを保存しました');}
    catch(error){document.getElementById('perr').textContent='保存失敗: '+error.message;}
    finally{saving=false;rows.inert=false;modal.removeAttribute('aria-busy');buttons.forEach(([button,disabled])=>button.disabled=disabled);}
  }
  document.addEventListener('keydown',event=>{if(event.key==='Escape'&&modal.classList.contains('open')){event.preventDefault();event.stopImmediatePropagation();close();}},true);
  return {open,close,add,save};
})();
