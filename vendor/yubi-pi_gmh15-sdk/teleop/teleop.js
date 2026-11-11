/* MuJoCo server-rendered model picking. No motor write is sent by preview gestures. */
(()=>{
'use strict';
const owner=crypto.randomUUID(), hands={};let sides=['left','right'];
let config=null,deadman=false,networkGood=false,frame=null,heartbeatBusy=false;
async function request(side,action,extra={}){
 const r=await fetch('/api/teleop',{method:'POST',headers:{'Content-Type':'application/json','X-Teleop-Token':config.token},
  body:JSON.stringify({side,action,owner,deadman,...extra}),keepalive:action==='stop'});
 const value=await r.json();if(!r.ok)throw Error(value.error||`HTTP ${r.status}`);return value;
}
function stopAll(reason='已停止控制'){
 deadman=false;
 for(const side of sides){const h=hands[side];if(!h)continue;
  if(config&&!config.demo&&(h.active||h.pendingArm))request(side,'stop').catch(()=>{});
  h.active=false;h.pendingArm=false;h.message.textContent=reason;h.dirty=false;
 } updateButtons();
}
function updateButtons(){for(const h of Object.values(hands)){
 h.arm.disabled=!config||config.demo||config.motion_locked||!deadman||!networkGood||h.active||h.pendingArm||!h.valid;
 h.key.textContent=deadman?'保持键已按下':'保持键未按下';h.key.className=deadman?'deadman-on':'deadman-off';
}}
function syncControls(h){h.inputs.forEach((input,i)=>{input.value=h.target[i];h.values[i].textContent=h.target[i].toFixed(1)+'°';
 input.closest('label').classList.toggle('selected',i===h.selected)});
 h.selection.textContent=`${config.joint_names[h.selected]}：${h.target[h.selected].toFixed(1)}° · 上拖弯曲，下拖伸展`;
}
function edit(h,index,value){
 const [low,high]=config.limits_degrees[index];h.selected=index;h.target[index]=Math.max(low,Math.min(high,Number(value)));
 h.dirty=true;h.previewDirty=true;syncControls(h);
}
function snapshot(side){
 const h=hands[side],actual=frame?.hands?.[side];
 if(!actual?.connected||actual.joint_error||actual.age_ms>500||actual.joints_degrees?.length!==15){h.message.textContent='当前没有新鲜的关节反馈';return false}
 h.target=config.adjustable_indices_15.map(i=>actual.joints_degrees[i]);h.initialized=true;h.previewDirty=true;h.dirty=false;syncControls(h);return true;
}
function build(side){
 const root=document.getElementById(side),section=document.createElement('section');section.className='teleop';section.dataset.side=side;
 section.innerHTML=`<h3>拖拽遥操作 · ${side==='left'?'左手':'右手'}</h3><p>默认只改虚拟目标。按住空格，点击“启用实机”，再拖拽关节标记；松开空格即请求失能。</p>
 <div class="toolbar"><button class="snapshot">目标跟随当前反馈</button><button class="arm">启用实机（可能运动）</button><button class="stop">停止并失能</button></div>
 <div class="teleop-status" role="status">正在准备控制入口…</div><small class="key-state"></small>
 <div class="target-wrap"><img class="target-image" draggable="false" alt="可拖拽的 MuJoCo 关节目标预览"></div><div class="selection"></div>
 <div class="joint-controls"></div><p class="passive-note">15关节模型中可编辑11个关节目标。四个 DIP 在预览里保留反馈角；真手的耦合由 SDK 处理。目标画面不是实物反馈。</p>
 <small>目标缓变 ≤8°/s · 发送 ≤10 Hz · 心跳600 ms超时失能。软件停止不替代硬件断电。</small>`;
 root.insertBefore(section,root.lastElementChild);
 if(config.motion_locked&&!config.demo){
  section.querySelector('p').textContent='已连接真实反馈，实机运动已锁定。拖拽只更新下方 MuJoCo 目标预览。';
  section.querySelector('.arm').textContent='实机运动已锁定';
 }
 const h=hands[side]={side,section,target:config.limits_degrees.map(([a])=>a),selected:0,
 inputs:[],values:[],active:false,pendingArm:false,dirty:false,previewDirty:false,valid:false,initialized:false,sequence:0,
 arm:section.querySelector('.arm'),message:section.querySelector('.teleop-status'),key:section.querySelector('.key-state'),
 image:section.querySelector('.target-image'),selection:section.querySelector('.selection')};
 config.joint_names.forEach((name,i)=>{
  const label=document.createElement('label');label.className='joint-control';label.innerHTML=`<span>${i+1}. ${name}</span><output></output><input type="range" step="0.1" min="${config.limits_degrees[i][0]}" max="${config.limits_degrees[i][1]}" aria-label="${side} ${name}目标角度">`;
  const input=label.querySelector('input');h.inputs.push(input);h.values.push(label.querySelector('output'));
  input.oninput=()=>edit(h,i,input.value);section.querySelector('.joint-controls').append(label);
 });
 section.querySelector('.snapshot').onclick=()=>snapshot(side);
 section.querySelector('.stop').onclick=()=>{request(side,'stop').catch(e=>h.message.textContent=e.message);stopAll('已请求停止，请核对电机未使能')};
 h.arm.onclick=async()=>{
  if(!deadman||!snapshot(side))return;h.pendingArm=true;h.message.textContent='正在同步当前电机目标并启用…';updateButtons();
  try{await request(side,'arm')}catch(e){h.pendingArm=false;h.message.textContent=e.message;updateButtons()}
 };
 let drag=null;
 h.image.onpointerdown=e=>{
  const rect=h.image.getBoundingClientRect(),meta=frame?.media?.['target-'+side];
  const x=(e.clientX-rect.left)/rect.width*560,y=(e.clientY-rect.top)/rect.height*420;
  const markers=meta?.markers||[];let nearest=null;
  for(const m of markers){const distance=Math.hypot(m.x-x,m.y-y);if(distance<30&&(!nearest||distance<nearest.distance))nearest={...m,distance}}
  if(nearest)h.selected=nearest.index;
  if(!h.initialized)snapshot(side);
  drag={pointer:e.pointerId,y:e.clientY,start:h.target[h.selected],index:h.selected};
  h.image.setPointerCapture(e.pointerId);h.image.classList.add('dragging');syncControls(h);e.preventDefault();
 };
 h.image.onpointermove=e=>{if(drag?.pointer===e.pointerId)edit(h,drag.index,drag.start+(drag.y-e.clientY)*.3)};
 const release=()=>{drag=null;h.image.classList.remove('dragging')};h.image.onpointerup=release;
 h.image.onpointercancel=()=>{release();stopAll('指针操作取消，已请求停止')};
 let imageBusy=false;
 const refresh=()=>{if(!imageBusy&&!document.hidden){imageBusy=true;h.image.src=`/media/target/${side}.jpg?view=${typeof mediaViews!=='undefined'?mediaViews[side]:'oblique'}&t=${Date.now()}`}};
 h.image.onload=()=>imageBusy=false;h.image.onerror=()=>imageBusy=false;setInterval(refresh,250);refresh();syncControls(h);
}
async function poll(){
 try{
  const r=await fetch('/api/state',{cache:'no-store'});if(!r.ok)throw Error('HTTP '+r.status);frame=await r.json();networkGood=true;
  for(const side of sides){const h=hands[side],actual=frame.hands?.[side],control=frame.teleop?.[side]||{};
   h.valid=Boolean(actual?.connected&&!actual.joint_error&&actual.age_ms<500&&actual.joints_degrees?.length===15);
   if(!h.initialized&&h.valid)snapshot(side);
   const active=Boolean(control.active&&control.owner===owner);
   if(active&&!h.active&&control.sent_degrees){h.target=[...control.sent_degrees];h.dirty=false;h.previewDirty=true;syncControls(h)}
   h.active=active;
   if(!config.demo&&!h.valid)h.section.querySelector('.passive-note').textContent='当前没有有效关节正解。目标模型从参考姿态开始，四个 DIP 使用参考零位；它不表示真手当前关节角。';
   if(active||control.error||control.reason)h.message.textContent=(control.error?control.error+' · ':'')+(control.reason||'')+(active?` · 已发送${control.command_count}帧`:'');
   if(active||control.error)h.pendingArm=false;
   if(h.pendingArm&&!deadman)request(side,'stop').catch(()=>{});
  }
 }catch(e){networkGood=false;stopAll('连接中断，已停止发送；服务端看门狗将请求失能')}
 updateButtons();
}
async function flush(){
 for(const h of Object.values(hands)){
  if(h.busy)continue;h.busy=true;
  try{
   if(h.previewDirty){h.previewDirty=false;await request(h.side,'preview',{angles_degrees:h.target})}
   if(h.active&&deadman&&h.dirty){h.dirty=false;await request(h.side,'target',{sequence:++h.sequence,angles_degrees:h.target})}
  }catch(e){h.message.textContent=e.message;if(h.active)stopAll('控制请求失败，已请求停止')}
  finally{h.busy=false}
 }
}
async function heartbeat(){
 if(!deadman||heartbeatBusy||!config||config.demo)return;heartbeatBusy=true;
 try{for(const h of Object.values(hands))if(h.active)await request(h.side,'heartbeat')}
 catch(e){stopAll('心跳请求失败，已请求停止')}
 finally{heartbeatBusy=false}
}
addEventListener('keydown',e=>{if(e.code==='Escape'){stopAll('Esc 停止');return}
 if(e.code==='Space'&&!e.repeat){e.preventDefault();deadman=true;updateButtons()}});
addEventListener('keyup',e=>{if(e.code==='Space'){e.preventDefault();stopAll('保持键已释放，已请求停止')}});
addEventListener('blur',()=>stopAll('窗口失焦，已请求停止'));
addEventListener('pagehide',()=>stopAll('页面离开，已请求停止'));
document.addEventListener('visibilitychange',()=>{if(document.hidden)stopAll('页面隐藏，已请求停止')});
document.getElementById('pause')?.addEventListener('click',()=>stopAll('画面暂停/恢复，已请求停止'));
(async()=>{
 try{const r=await fetch('/api/teleop/config');config=await r.json();sides=config.hands||sides;
  if(config.demo){const note=document.createElement('div');note.className='teleop-demo';note.textContent='离线演示：使用已保存的关节样本，未连接实物。可以拖拽虚拟目标，实机启用已禁用。';document.querySelector('h1').after(note)}
  else if(config.motion_locked){const note=document.createElement('div');note.className='teleop-demo';note.textContent='真手已接入 · 运动锁定：设备、触觉和相机读取自实物，关节有效性以下方提示为准；拖拽仅改变目标模型，不能使能或驱动真手。';document.querySelector('h1').after(note)}
  sides.forEach(build);await poll();setInterval(poll,200);setInterval(flush,100);setInterval(heartbeat,150);
  window.__gmhTeleop={hands,get frame(){return frame},get config(){return {...config,token:'hidden'}}};
 }catch(e){document.querySelector('#network').textContent='遥操作入口加载失败：'+e.message}
})();
})();
