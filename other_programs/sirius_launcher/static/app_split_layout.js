'use strict';
class AppSplitLayout {
 constructor(page='home'){this.pages=[page];this.focus=0;this.axis='row';this.ratio=.5;}
 get enabled(){return this.pages.length===2;}
 has(page){return this.enabled&&this.pages.includes(page);}
 select(page){const index=this.pages.indexOf(page);if(index>=0)this.focus=index;else this.pages[this.focus]=page;}
 configure(first,second,axis='row'){if(first===second)return false;this.pages=[first,second];this.axis=axis==='column'?'column':'row';this.ratio=.5;this.focus=0;return true;}
 split(page,edge,current){const other=page!==current?current:this.pages.find(p=>p!==page);if(!other)return false;const before=edge==='left'||edge==='top';this.configure(before?page:other,before?other:page,edge==='top'||edge==='bottom'?'column':'row');this.focus=before?0:1;return true;}
 collapse(page){this.pages=[this.pages.includes(page)?page:this.pages[this.focus]];this.focus=0;return this.pages[0];}
}
if(typeof module!=='undefined')module.exports=AppSplitLayout;
