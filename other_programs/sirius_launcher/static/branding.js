'use strict';
// The fullscreen shell owns the browser tab; keep its icon in sync with the app.
(function(){
  if(window.parent===window)return;
  try{
    const parentDocument=window.parent.document;
    for(const rel of ['icon','apple-touch-icon']){
      let link=parentDocument.querySelector('link[rel="'+rel+'"]');
      if(!link){link=parentDocument.createElement('link');link.rel=rel;parentDocument.head.append(link);}
      link.type='image/png';link.href='/static/branding/sirius-icon.png';
    }
  }catch(error){/* A separately hosted frame retains its own document icon. */}
})();
