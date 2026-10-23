'use strict';
let GUIDE_LOADING=null,GUIDE_READY=false;
function filterDocumentation(){
  const query=document.getElementById('guide-search').value.trim().toLocaleLowerCase();
  const terms=query.split(/\s+/).filter(Boolean);
  let count=0;
  document.querySelectorAll('.guide-section').forEach(section=>{
    const text=(section.textContent+' '+(section.dataset.keywords||'')).toLocaleLowerCase();
    const shown=terms.every(term=>text.includes(term));section.hidden=!shown;if(shown)count++;
    document.querySelectorAll('[data-guide-target="'+section.id+'"]').forEach(button=>button.hidden=!shown);
  });
  document.getElementById('guide-no-results').hidden=!GUIDE_READY||count>0||window.GUIDE_MODE==='notes';
}
async function initDocumentation(){
  if(GUIDE_READY||GUIDE_LOADING)return GUIDE_LOADING;
  const content=document.getElementById('guide-content');
  content.setAttribute('aria-busy','true');
  GUIDE_LOADING=(async()=>{
    try{
      const response=await fetch('/static/documentation.html',{signal:AbortSignal.timeout(8000)});
      if(!response.ok)throw new Error('HTTP '+response.status);
      content.innerHTML=await response.text();GUIDE_READY=true;filterDocumentation();
    }catch(error){
      content.replaceChildren();const message=document.createElement('p');message.className='guide-message';
      message.textContent='ガイドを読み込めませんでした。接続を確認して再試行してください。';
      const retry=document.createElement('button');retry.className='hbtn';retry.textContent='再試行';retry.onclick=initDocumentation;
      content.append(message,retry);
    }finally{content.setAttribute('aria-busy','false');GUIDE_LOADING=null;}
  })();
  return GUIDE_LOADING;
}
document.getElementById('guide-search').addEventListener('input',filterDocumentation);
document.getElementById('guide-content').addEventListener('click',event=>{
  const section=event.target.closest('[data-guide-target]');
  if(section){document.getElementById(section.dataset.guideTarget)?.scrollIntoView({block:'start',behavior:matchMedia('(prefers-reduced-motion:reduce)').matches?'instant':'smooth'});return;}
  const link=event.target.closest('[data-guide-page]');
  if(link&&PAGE_LABELS[link.dataset.guidePage])setPage(link.dataset.guidePage);
});
if(document.body.dataset.page==='docs')initDocumentation();
