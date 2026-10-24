'use strict';
const TOPIC_VIEW={data:null,busy:false,selected:null,search:'',filter:'all',shape:'',lastPoll:0,measureBusy:false,watchBusy:false,watchTopic:null,watchSentAt:0,messageScroll:0};

function topicElement(tag,className,text){
  const element=document.createElement(tag);
  if(className)element.className=className;
  if(text!==undefined)element.textContent=text;
  return element;
}

function topicMeasurement(topic){
  return (TOPIC_VIEW.data&&TOPIC_VIEW.data.measurements||{})[topic]||null;
}

function topicWatch(){
  const watch=TOPIC_VIEW.data&&TOPIC_VIEW.data.watch;
  return watch&&watch.topic===TOPIC_VIEW.selected?watch:null;
}

function topicWatchState(watch){
  if(!watch)return '未表示';
  if(watch.state==='watching')return '受信中';
  if(watch.state==='starting')return '購読を開始しています';
  if(watch.state==='error')return 'エラー';
  return '停止中';
}

function topicRateLabel(topic){
  const measurement=topicMeasurement(topic);
  if(!measurement)return '—';
  if(measurement.state==='measuring')return '…';
  if(measurement.state==='error')return 'err';
  return measurement.rate_hz===null||measurement.rate_hz===undefined?'—':Number(measurement.rate_hz).toFixed(2);
}

function topicQoS(qos){
  if(!qos)return '';
  return [qos.reliability,qos.durability,qos.history,qos.depth!==undefined?'depth='+qos.depth:''].filter(Boolean).join(' · ');
}

function topicEndpointText(endpoint){
  const qos=topicQoS(endpoint.qos);
  return (endpoint.node||endpoint.name||'unknown')+(qos?' · '+qos:'');
}

function topicMatches(topic){
  const measurement=topicMeasurement(topic.name);
  if(TOPIC_VIEW.filter==='active'&&!topic.publisher_count&&!topic.subscriber_count)return false;
  if(TOPIC_VIEW.filter==='measured'&&(!measurement||measurement.state!=='done'))return false;
  const query=TOPIC_VIEW.search.trim().toLowerCase();
  if(!query)return true;
  const haystack=[topic.name,topic.type,...(topic.publishers||[]).map(endpoint=>endpoint.node),...(topic.subscribers||[]).map(endpoint=>endpoint.node)].join(' ').toLowerCase();
  return haystack.includes(query);
}

function topicShape(data){
  return JSON.stringify([
    data.available,
    (data.topics||[]).map(topic=>[topic.name,topic.type,topic.publisher_count,topic.subscriber_count]),
    Object.entries(data.measurements||{}).map(([name,measurement])=>[name,measurement.state,measurement.rate_hz,measurement.count])
  ]);
}

function renderTopicRows(){
  const rows=document.getElementById('topics-rows');
  if(!rows||!TOPIC_VIEW.data)return;
  const scroll=rows.scrollTop;
  const topics=(TOPIC_VIEW.data.topics||[]).filter(topicMatches);
  rows.replaceChildren();
  if(!topics.length){
    const empty=topicElement('div','topics-empty',TOPIC_VIEW.data.available?'条件に一致するトピックがありません':'ROSトピックを待っています…');
    rows.append(empty);
    return;
  }
  const fragment=document.createDocumentFragment();
  for(const topic of topics){
    const measurement=topicMeasurement(topic.name);
    const row=topicElement('div','topic-row');
    row.dataset.topic=topic.name;
    row.tabIndex=0;
    row.setAttribute('role','option');
    row.setAttribute('aria-selected',String(topic.name===TOPIC_VIEW.selected));
    row.classList.toggle('selected',topic.name===TOPIC_VIEW.selected);
    const name=topicElement('span','topic-name',topic.name);
    const type=topicElement('span','topic-type',topic.type||'—');
    type.title=topic.type||'';
    const rate=topicElement('span','topic-rate',topicRateLabel(topic.name));
    if(measurement&&measurement.state==='error')rate.classList.add('rate-error');
    if(measurement&&measurement.state==='measuring')rate.classList.add('rate-busy');
    const publishers=topicElement('span','topic-count',String(topic.publisher_count||0));
    const subscribers=topicElement('span','topic-count',String(topic.subscriber_count||0));
    const endpointSummary=[...(topic.publishers||[]).map(endpoint=>'pub: '+endpoint.node),...(topic.subscribers||[]).map(endpoint=>'sub: '+endpoint.node)].join('\n');
    row.title=endpointSummary||topic.name;
    row.append(name,type,rate,publishers,subscribers);
    row.onclick=()=>selectTopic(topic.name);
    row.onkeydown=event=>{if(event.key==='Enter'||event.key===' '){event.preventDefault();selectTopic(topic.name);}};
    fragment.append(row);
  }
  rows.append(fragment);
  rows.scrollTop=scroll;
}

function selectTopic(topic){
  if(TOPIC_VIEW.selected!==topic){
    TOPIC_VIEW.messageScroll=0;
    if(TOPIC_VIEW.watchTopic&&TOPIC_VIEW.watchTopic!==topic)stopTopicWatch(true);
  }
  TOPIC_VIEW.selected=topic;
  renderTopicRows();
  renderTopicDetails();
}

function topicEndpointSection(title,endpoints,emptyText){
  const section=topicElement('section','topics-endpoints');
  section.append(topicElement('h3','',title));
  if(!endpoints||!endpoints.length){
    section.append(topicElement('p','topics-help',emptyText));
    return section;
  }
  for(const endpoint of endpoints){
    const row=topicElement('div','topics-endpoint');
    row.append(topicElement('b','',endpoint.node||'unknown'));
    const detail=topicElement('span','',[endpoint.namespace, topicQoS(endpoint.qos)].filter(Boolean).join(' · '));
    if(endpoint.gid)detail.title='GID '+endpoint.gid;
    row.append(detail);
    section.append(row);
  }
  return section;
}

function topicMessageSection(topicName){
  const section=topicElement('section','topics-message');
  section.append(topicElement('h3','','メッセージ'));
  const watch=topicWatch();
  if(!watch){
    section.append(topicElement('p','topics-help','選択したトピックを読み取り専用で購読し、最新メッセージを表示します。画像・点群などの大きな配列は先頭のみ表示します。'));
    const button=topicElement('button','hbtn',TOPIC_VIEW.watchBusy?'開始中…':'最新メッセージを表示');
    button.disabled=TOPIC_VIEW.watchBusy;
    button.onclick=()=>startTopicWatch(topicName);
    section.append(button);
    return section;
  }
  const state=watch.state||'stopped';
  const status=topicElement('div','topics-message-status');
  const stateLabel=topicElement('span','topics-message-state',topicWatchState(watch));
  stateLabel.classList.add('state-'+state);
  status.append(stateLabel);
  if(watch.count)status.append(topicElement('span','','受信 '+watch.count+' 件'));
  if(watch.received_at){
    const received=new Date(watch.received_at*1000);
    status.append(topicElement('span','','最終受信 '+received.toLocaleTimeString('ja-JP',{hour12:false})+'.'+String(received.getMilliseconds()).padStart(3,'0')));
  }
  section.append(status);
  if(watch.error)section.append(topicElement('p','topics-help topics-message-error',watch.error));
  if(state==='stopped'&&watch.expired)section.append(topicElement('p','topics-help','一定時間が経過したため購読を停止しました。再表示すると再開します。'));
  if(watch.text){
    const pre=topicElement('pre','topics-message-text',watch.text);
    pre.id='topics-message-text';
    pre.tabIndex=0;
    section.append(pre);
    if(watch.truncated)section.append(topicElement('p','topics-help','長い配列・文字列・深い階層は省略しています。'));
  }else if(state==='watching'||state==='starting'){
    section.append(topicElement('p','topics-help','メッセージを待っています…'));
  }else if(!watch.error){
    section.append(topicElement('p','topics-help','メッセージはまだありません。'));
  }
  const actions=topicElement('div','topics-message-actions');
  if(state==='watching'||state==='starting'){
    const stop=topicElement('button','hbtn','停止');
    stop.disabled=TOPIC_VIEW.watchBusy;
    stop.onclick=()=>stopTopicWatch();
    actions.append(stop);
  }else{
    const start=topicElement('button','hbtn',watch.text?'再表示':'表示');
    start.disabled=TOPIC_VIEW.watchBusy;
    start.onclick=()=>startTopicWatch(topicName);
    actions.append(start);
  }
  if(watch.text){
    const copy=topicElement('button','hbtn','コピー');
    copy.onclick=async()=>{
      try{
        await navigator.clipboard.writeText(watch.text);
        copy.textContent='コピーしました';
        setTimeout(()=>{if(copy.isConnected)copy.textContent='コピー';},1500);
      }catch(error){
        toast('メッセージをコピーできませんでした',true);
      }
    };
    actions.append(copy);
  }
  section.append(actions);
  return section;
}

function renderTopicDetails(){
  const box=document.getElementById('topics-inspector');
  if(!box)return;
  const scroll=box.scrollTop;
  const previousMessage=document.getElementById('topics-message-text');
  const messageScroll=previousMessage?previousMessage.scrollTop:TOPIC_VIEW.messageScroll;
  box.replaceChildren();
  const topic=TOPIC_VIEW.selected;
  const data=TOPIC_VIEW.data;
  const row=data&&(data.topics||[]).find(item=>item.name===topic);
  if(!topic||!row){
    box.append(topicElement('h2','','トピックの詳細'));
    box.append(topicElement('p','topics-help','一覧からトピックを選択すると、配信周期・宛先・メッセージを表示します。'));
    return;
  }
  box.append(topicElement('h2','',row.name));
  box.append(topicElement('p','topics-type',row.type||'型未取得'));
  const measurement=topicMeasurement(row.name);
  const summary=topicElement('dl','topics-summary');
  const fields=[
    ['配信ノード',String(row.publisher_count||0)+' 件'],
    ['購読ノード',String(row.subscriber_count||0)+' 件'],
    ['ROSドメイン',data.domain||'—'],
    ['最終受信',data.timestamp?new Date(data.timestamp*1000).toLocaleTimeString('ja-JP',{hour12:false}):'—']
  ];
  for(const [key,value] of fields){
    summary.append(topicElement('dt','',key),topicElement('dd','',value));
  }
  box.append(summary);

  const rateSection=topicElement('section','topics-rate');
  rateSection.append(topicElement('h3','','配信周期'));
  let rateText='—',rateDetail='まだ測定していません。選択したトピックを約3秒受信して周期を計算します。';
  rateSection.classList.remove('rate-error','rate-busy');
  if(measurement){
    if(measurement.state==='measuring'){
      rateText='測定中…';
      rateDetail='メッセージを受信しています。';
      rateSection.classList.add('rate-busy');
    }else if(measurement.state==='error'){
      rateText='測定エラー';
      rateDetail=measurement.error||'周期を測定できませんでした。';
      rateSection.classList.add('rate-error');
    }else if(measurement.rate_hz!==null&&measurement.rate_hz!==undefined){
      rateText=Number(measurement.rate_hz).toFixed(2)+' Hz';
      rateDetail=measurement.count+' メッセージ / '+Number(measurement.duration).toFixed(1)+' 秒';
    }else{
      rateText='受信なし';
      rateDetail=measurement.error||'測定時間内にメッセージが届きませんでした。';
      rateSection.classList.add('rate-error');
    }
  }
  rateSection.append(topicElement('b','topics-rate-value',rateText));
  rateSection.append(topicElement('p','topics-help',rateDetail));
  const measureButton=topicElement('button','hbtn',measurement&&measurement.state==='measuring'?'測定中…':'周期を測定');
  measureButton.disabled=TOPIC_VIEW.measureBusy||(measurement&&measurement.state==='measuring');
  measureButton.onclick=()=>measureTopic(row.name);
  rateSection.append(measureButton);
  box.append(rateSection);

  box.append(topicMessageSection(row.name));

  box.append(topicEndpointSection('配信元（Publisher）',row.publishers,'このトピックを配信しているノードはありません。'));
  box.append(topicEndpointSection('購読先（Subscriber）',row.subscribers,'このトピックを購読しているノードはありません。'));
  box.scrollTop=scroll;
  const nextMessage=document.getElementById('topics-message-text');
  if(nextMessage)nextMessage.scrollTop=messageScroll;
}

async function measureTopic(topic){
  if(TOPIC_VIEW.measureBusy||!topic)return;
  TOPIC_VIEW.measureBusy=true;
  renderTopicDetails();
  try{
    await api('/api/topics/measure',{topic,duration:3});
    setTimeout(()=>pollTopics(true),250);
  }catch(error){
    toast('周期を測定できませんでした',true);
  }finally{
    TOPIC_VIEW.measureBusy=false;
    renderTopicDetails();
  }
}

async function startTopicWatch(topic){
  if(TOPIC_VIEW.watchBusy||!topic)return;
  TOPIC_VIEW.watchBusy=true;
  TOPIC_VIEW.watchTopic=topic;
  TOPIC_VIEW.watchSentAt=Date.now();
  TOPIC_VIEW.messageScroll=0;
  renderTopicDetails();
  try{
    await api('/api/topics/watch',{action:'start',topic,lease:10});
    await pollTopics(true);
  }catch(error){
    TOPIC_VIEW.watchTopic=null;
    toast('メッセージを表示できませんでした',true);
  }finally{
    TOPIC_VIEW.watchBusy=false;
    renderTopicDetails();
  }
}

async function stopTopicWatch(silent=false){
  if(!TOPIC_VIEW.watchTopic)return;
  TOPIC_VIEW.watchTopic=null;
  TOPIC_VIEW.watchSentAt=0;
  try{
    await api('/api/topics/watch',{action:'stop'});
  }catch(error){
    if(!silent)toast('メッセージ購読を停止できませんでした',true);
  }
  if(!silent)await pollTopics(true);
}

function renewTopicWatch(){
  const topic=TOPIC_VIEW.watchTopic;
  const watch=TOPIC_VIEW.data&&TOPIC_VIEW.data.watch;
  if(!topic||TOPIC_VIEW.watchBusy||!watch||watch.topic!==topic)return;
  if(watch.state!=='watching'&&watch.state!=='starting')return;
  if(Date.now()-TOPIC_VIEW.watchSentAt<5000)return;
  TOPIC_VIEW.watchSentAt=Date.now();
  api('/api/topics/watch',{action:'start',topic,lease:10}).catch(()=>{});
}

function renderTopicSummary(data){
  const counts=document.getElementById('topics-counts');
  if(!counts)return;
  counts.replaceChildren();
  const topics=data.topics||[];
  const publishers=topics.reduce((total,topic)=>total+(topic.publisher_count||0),0);
  const subscribers=topics.reduce((total,topic)=>total+(topic.subscriber_count||0),0);
  for(const text of [topics.length+' topics','配信 '+publishers,'購読 '+subscribers,'Domain '+(data.domain||'—')]){
    counts.append(topicElement('span','',text));
  }
}

async function pollTopics(force=false){
  if(TOPIC_VIEW.busy)return;
  if(!force&&Date.now()-TOPIC_VIEW.lastPoll<1200)return;
  TOPIC_VIEW.busy=true;
  TOPIC_VIEW.lastPoll=Date.now();
  const status=document.getElementById('topics-status');
  try{
    const data=await api('/api/topics');
    TOPIC_VIEW.data=data;
    const fresh=!!data.available;
    status.classList.toggle('stale',!fresh);
    status.textContent=fresh?'LIVE · '+new Date(data.timestamp*1000).toLocaleTimeString('ja-JP',{hour12:false}):data.error?'トピック受信エラー':'トピック受信を開始しています…';
    status.title=data.error||'';
    renderTopicSummary(data);
    const shape=topicShape(data);
    if(shape!==TOPIC_VIEW.shape){
      TOPIC_VIEW.shape=shape;
      if(TOPIC_VIEW.selected&&!(data.topics||[]).some(topic=>topic.name===TOPIC_VIEW.selected))TOPIC_VIEW.selected=null;
      renderTopicRows();
    }
    renderTopicDetails();
    renewTopicWatch();
  }catch(error){
    status.classList.add('stale');
    status.textContent='接続を再試行しています';
  }finally{
    TOPIC_VIEW.busy=false;
  }
}

function initTopicsPage(){
  if(document.getElementById('page-topics'))return;
  const style=document.createElement('link');
  style.rel='stylesheet';
  style.href='/static/topics.css';
  document.head.append(style);
  const nav=document.createElement('button');
  nav.className='nav-item';
  nav.dataset.page='topics';
  nav.title='Topic Monitor';
  nav.setAttribute('aria-label','Topic Monitor');
  nav.innerHTML='<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.65" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M4 6h16M4 11h16M4 16h16M4 21h16"/><circle cx="7" cy="6" r="1.2" fill="currentColor" stroke="none"/><circle cx="12" cy="11" r="1.2" fill="currentColor" stroke="none"/><circle cx="17" cy="16" r="1.2" fill="currentColor" stroke="none"/></svg><span>Topics</span>';
  nav.onclick=()=>{setPage('topics');setTimeout(()=>pollTopics(true),0);};
  const tfNav=document.querySelector('.nav-item[data-page="tf"]');
  if(tfNav)tfNav.after(nav);else document.querySelector('.rail-nav').append(nav);
  const section=document.createElement('section');
  section.className='page';
  section.id='page-topics';
  section.setAttribute('aria-label','Topic Monitor');
  section.innerHTML='<div class="topics-toolbar"><div class="topics-counts" id="topics-counts">—</div><span class="sample-status" id="topics-status">接続を待っています</span><div class="topics-tools"><input id="topics-search" placeholder="トピック・型・ノードを検索" aria-label="トピック検索"><div class="segmented" id="topics-filter" role="group" aria-label="トピック絞り込み"><button class="active" data-topic-filter="all">すべて</button><button data-topic-filter="active">配信あり</button><button data-topic-filter="measured">測定済み</button></div><button class="hbtn" id="topics-refresh">更新</button></div></div><div class="topics-layout"><article class="surface topics-list"><div class="topics-list-head"><span>トピック</span><span>型</span><span>Hz</span><span>Pub</span><span>Sub</span></div><div class="topics-rows" id="topics-rows" role="listbox" aria-label="ROSトピック一覧"><div class="topics-empty">トピックを待っています…</div></div></article><aside class="surface topics-inspector" id="topics-inspector"><h2>トピックの詳細</h2><p class="topics-help">一覧からトピックを選択すると、配信周期・宛先・メッセージを表示します。</p></aside></div><p class="topics-footnote">ROS 2グラフから約1秒ごとに一覧を更新します。Hzは選択したトピックを約3秒受信したときの実測値です。メッセージは読み取り専用で購読し、大きな配列は先頭のみ表示します。LichtblickのTopicsパネルでは購読中トピックのHzしか出ないため、全トピックの宛先確認はこの画面を使用してください。</p>';
  document.querySelector('main.workspace').append(section);
  if(typeof PAGE_LABELS==='object')PAGE_LABELS.topics='Topic Monitor';
  document.getElementById('topics-search').addEventListener('input',event=>{TOPIC_VIEW.search=event.target.value;renderTopicRows();});
  document.querySelectorAll('[data-topic-filter]').forEach(button=>{
    button.onclick=()=>{
      TOPIC_VIEW.filter=button.dataset.topicFilter;
      document.querySelectorAll('[data-topic-filter]').forEach(item=>item.classList.toggle('active',item===button));
      renderTopicRows();
    };
  });
  document.getElementById('topics-refresh').onclick=()=>pollTopics(true);
  if(location.hash==='#topics')pollTopics(true);
}

initTopicsPage();
setInterval(()=>{if((window.isPageVisible?isPageVisible('topics'):document.body.dataset.page==='topics'))pollTopics();},1500);
