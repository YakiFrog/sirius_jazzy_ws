'use strict';
class TerminalPanes {
 constructor(){this.root=null;this.active=null;this.primary=null;this.serial=0;}
 leaf(name){return {id:'pane-'+(++this.serial),name};}
 leaves(node=this.root){return !node?[]:node.name!==undefined?[node]:[...this.leaves(node.first),...this.leaves(node.second)];}
 find(id){return this.leaves().find(p=>p.id===id);}
 select(name){let pane=this.leaves().find(p=>p.name===name);if(!pane){pane=this.find(this.primary);if(pane)pane.name=name;else{pane=this.leaf(name);this.root=pane;this.primary=pane.id;}}this.active=pane.id;return pane;}
 replace(id,replacement,node=this.root){if(!node)return null;if(node.id===id)return replacement;if(node.name!==undefined)return node;node.first=this.replace(id,replacement,node.first);node.second=this.replace(id,replacement,node.second);return node;}
 remove(id){const prune=node=>{if(!node||node.id===id)return null;if(node.name!==undefined)return node;node.first=prune(node.first);node.second=prune(node.second);return node.first&&node.second?node:node.first||node.second;};this.root=prune(this.root);if(!this.find(this.active))this.active=this.leaves()[0]?.id||null;if(!this.find(this.primary))this.primary=this.leaves()[0]?.id||null;}
 drop(name,targetId,edge){const target=this.find(targetId),source=this.leaves().find(p=>p.name===name);if(!target||source===target||!['left','right','top','bottom','center'].includes(edge))return false;if(edge==='center'){if(source)source.name=target.name;target.name=name;this.active=target.id;return true;}if(source)this.remove(source.id);const pane=this.leaf(name),before=edge==='left'||edge==='top';this.root=this.replace(targetId,{id:'split-'+(++this.serial),axis:edge==='left'||edge==='right'?'row':'column',ratio:.5,first:before?pane:target,second:before?target:pane});this.active=pane.id;return true;}
 reset(name){this.root=name?this.leaf(name):null;this.active=this.root?.id||null;this.primary=this.active;}
}
class TerminalWorkspaces {
 constructor(){this.layouts=new Map();this.current=null;}
 ownerOf(name){for(const [owner,layout] of this.layouts)if(layout.leaves().some(p=>p.name===name))return owner;return null;}
 open(name){name=this.ownerOf(name)||name;if(!this.layouts.has(name)){const layout=new TerminalPanes();layout.reset(name);this.layouts.set(name,layout);}this.current=name;return this.layouts.get(name);}
 normalize(){const next=new Map();let current=null;for(const [owner,layout] of this.layouts){const leaves=layout.leaves();if(!leaves.length)continue;const key=leaves.some(p=>p.name===owner)?owner:leaves[0].name;next.set(key,layout);if(this.current===owner)current=key;}this.layouts=next;this.current=current;}
 drop(name,target,edge){const layout=this.layouts.get(this.current);if(!layout||!layout.drop(name,target,edge))return false;for(const other of this.layouts.values())if(other!==layout)for(const pane of other.leaves().filter(p=>p.name===name))other.remove(pane.id);this.normalize();return true;}
 tabs(sessions){const owners=new Map();for(const [owner,layout] of this.layouts)if(layout.leaves().length>1)for(const pane of layout.leaves())owners.set(pane.name,owner);const seen=new Set(),tabs=[];for(const session of sessions){const owner=owners.get(session)||session;if(seen.has(owner))continue;seen.add(owner);const layout=this.layouts.get(owner);tabs.push({name:owner,members:layout?layout.leaves().map(p=>p.name):[session]});}return tabs;}
 close(name){for(const layout of this.layouts.values())for(const pane of layout.leaves().filter(p=>p.name===name))layout.remove(pane.id);this.normalize();}
}
if(typeof module!=='undefined'){module.exports=TerminalPanes;module.exports.TerminalWorkspaces=TerminalWorkspaces;}
