/* Button-controlled teleoperation. Space has no handler or control function. */
(()=>{
'use strict';
const owner=crypto.randomUUID(),hands={};let sides=[],config=null,frame=null,networkGood=false,heartbeatBusy=false;
async function request(side,action,extra={}){
 const r=await fetch('/api/teleop',{method:'POST',headers:{'Content-Type':'application/json','X-Teleop-Token':config.token},
  body:JSON.stringify({side,action,owner,control_enabled:Boolean(hands[side]?.wantControl),...extra}),keepalive:action==='stop'});
 const value=await r.json();if(!r.ok)throw Error(value.error||`HTTP ${r.status}`);return value;
}
function stopAll(reason='已请求停止并失能'){
 for(const h of Object.values(hands)){
  if(config&&!config.demo&&(h.active||h.wantControl||h.pendingArm||h.pendingInit))request(h.side,'stop').catch(()=>{});
  h.wantControl=false;h.active=false;h.pendingArm=false;h.pendingInit=false;h.pendingId=null;h.dirty=false;
  h.message.textContent=reason;
 }updateButtons();
}
function updateButtons(){for(const h of Object.values(hands)){
 const unavailable=!config||config.demo||config.motion_locked||!networkGood;
 const busy=h.pendingArm||h.pendingInit||h.initializing;
 h.arm.disabled=unavailable||busy||h.active||!h.valid||!h.initReady;
 h.init.disabled=unavailable||busy||h.active||!h.connected;
 h.key.textContent=busy?'正在处理操作…':h.active?'实机控制已开启 · 拖拽直接发送目标':'尚未使能 · 拖拽仅预览';
 h.key.className=h.active?'deadman-on':'deadman-off';
}}
function syncControls(h){h.inputs.forEach((input,i)=>{
 input.value=h.target[i];h.values[i].textContent=h.target[i].toFixed(1)+'°';input.closest('label').classList.toggle('selected',i===h.selected);
 });h.selection.textContent=`${config.joint_names[h.selected]}：${h.target[h.selected].toFixed(1)}° · 上拖弯曲，下拖伸展`;
}
const isPassive=i=>(config.passive_indices||[]).includes(i);
function edit(h,index,value){if(isPassive(index))return;const [low,high]=config.limits_degrees[index];h.selected=index;h.target[index]=Math.max(low,Math.min(high,Number(value)));h.dirty=true;h.previewDirty=true;syncControls(h)}
function snapshot(side){const h=hands[side],actual=frame?.hands?.[side];
 if(!h.valid||!actual){h.message.textContent='关节反馈未就绪；请先点击位置初始化（可能运动）';return false}
 h.target=config.adjustable_indices_15.map((i,k)=>isPassive(k)?0:actual.joints_degrees[i]);h.initialized=true;h.previewDirty=true;h.dirty=h.active;syncControls(h);return true;
}
function build(side){
 const section=document.createElement('section');section.className='teleop';section.dataset.side=side;
 section.innerHTML=`<h3>拖拽遥操作 · ${side==='right'?'右手':'左手'}</h3><p>点击“启用实机”后，拖拽模型或滑条即可控制；点击停止即请求失能。关节未就绪时，先单独点击位置初始化。</p>
<div class="toolbar"><button class="initialize">位置初始化（厂商默认速度，可能运动）</button><button class="snapshot">目标同步当前手势</button><button class="arm">启用实机控制（可能运动）</button><button class="stop">停止并失能</button></div>
 <div class="teleop-status" role="status">正在连接…</div><div class="init-diagnostics"></div><small class="key-state"></small>
 <div class="target-wrap"><img class="target-image" draggable="false" alt="可拖拽的 MuJoCo 关节目标预览"></div><div class="selection"></div>
 <div class="joint-controls"></div><p class="passive-note">7个电机关节可控：四指 MCP + 拇指 CMC/MCP/IP。四指 PIP/DIP 无独立电机，已隐藏并固定为0°。</p>
 <small>目标缓变 ≤8°/s · 发送 ≤10 Hz · 失焦、断线、隐藏页面或点击停止均请求失能；软件停止不替代硬件断电。</small>`;
 const root=document.getElementById(side);root.insertBefore(section,root.lastElementChild);
 const h=hands[side]={side,section,target:config.limits_degrees.map(()=>0),selected:0,inputs:[],values:[],
  active:false,wantControl:false,pendingArm:false,pendingInit:false,pendingId:null,initializing:false,
  dirty:false,previewDirty:false,connected:false,valid:false,initialized:false,sequence:0,
  arm:section.querySelector('.arm'),init:section.querySelector('.initialize'),message:section.querySelector('.teleop-status'),
  key:section.querySelector('.key-state'),image:section.querySelector('.target-image'),selection:section.querySelector('.selection')};
 if(config.motion_locked){h.arm.textContent='实机运动已锁定';section.querySelector('p').textContent='实机运动锁定；拖拽仅更新虚拟目标。'}
 config.joint_names.forEach((name,i)=>{
  const label=document.createElement('label');label.className='joint-control';
  label.innerHTML=`<span>${i+1}. ${name}</span><output></output><input type="range" step="0.1" min="${config.limits_degrees[i][0]}" max="${config.limits_degrees[i][1]}" aria-label="${side} ${name}目标角度">`;
  const input=label.querySelector('input');if(isPassive(i)){label.hidden=true;label.style.display='none';input.disabled=true}h.inputs.push(input);h.values.push(label.querySelector('output'));
  input.oninput=()=>edit(h,i,input.value);section.querySelector('.joint-controls').append(label);
 });
 section.querySelector('.snapshot').onclick=()=>snapshot(side);
 section.querySelector('.stop').onclick=()=>{request(side,'stop').catch(()=>{});stopAll()};
 h.init.onclick=async()=>{
  h.pendingInit=true;h.message.textContent='正在请求位置初始化；这一步可能运动，结束后会失能。';updateButtons();
  try{const response=await request(side,'initialize',{confirm_initialization:true});h.pendingId=response.request_id}
  catch(e){h.pendingInit=false;h.message.textContent=e.message;updateButtons()}
 };
 h.arm.onclick=async()=>{
  if(!snapshot(side))return;
  h.wantControl=true;h.pendingArm=true;h.message.textContent='正在同步当前目标并使能…';updateButtons();
  try{const response=await request(side,'arm');h.pendingId=response.request_id}
  catch(e){h.wantControl=false;h.pendingArm=false;h.message.textContent=e.message;updateButtons()}
 };
 let drag=null;
 h.image.onpointerdown=e=>{
  const rect=h.image.getBoundingClientRect(),meta=frame?.media?.['target-'+side];
  const x=(e.clientX-rect.left)/rect.width*560,y=(e.clientY-rect.top)/rect.height*420;let nearest=null;
  for(const marker of meta?.markers||[]){const distance=Math.hypot(marker.x-x,marker.y-y);if(distance<30&&(!nearest||distance<nearest.distance))nearest={...marker,distance}}
  if(nearest&&!isPassive(nearest.index))h.selected=nearest.index;if(!h.initialized&&h.valid)snapshot(side);
  drag={id:e.pointerId,y:e.clientY,start:h.target[h.selected],index:h.selected};h.image.setPointerCapture(e.pointerId);h.image.classList.add('dragging');syncControls(h);e.preventDefault();
 };
 h.image.onpointermove=e=>{if(drag?.id===e.pointerId)edit(h,drag.index,drag.start+(drag.y-e.clientY)*.3)};
 const release=()=>{drag=null;h.image.classList.remove('dragging')};h.image.onpointerup=release;
 h.image.onpointercancel=()=>{release();stopAll('指针取消，已请求停止')};
 let imageBusy=false;
 const refresh=()=>{if(!imageBusy&&!document.hidden){imageBusy=true;h.image.src=`/media/target/${side}.jpg?view=${typeof mediaViews!=='undefined'?mediaViews[side]:'oblique'}&t=${Date.now()}`}};
 h.image.onload=h.image.onerror=()=>imageBusy=false;setInterval(refresh,250);refresh();syncControls(h);
}
async function poll(){
 try{
  const r=await fetch('/api/state',{cache:'no-store'});if(!r.ok)throw Error('HTTP '+r.status);frame=await r.json();networkGood=true;
  for(const h of Object.values(hands)){
   const actual=frame.hands?.[h.side],control=frame.teleop?.[h.side]||{};
   h.connected=Boolean(actual?.connected&&actual.age_ms<1500);
   h.valid=Boolean(h.connected&&!actual.joint_error&&actual.age_ms<500&&actual.joints_degrees?.length===15);
   if(!h.initialized&&h.valid)snapshot(h.side);
   h.initReady=control.last_init_result===1;
   const active=Boolean(control.active&&control.owner===owner);
   if(active&&!h.active&&control.sent_degrees){h.target=[...control.sent_degrees];h.dirty=false;h.previewDirty=true;syncControls(h)}
   const previouslyActive=h.active;h.active=active;h.initializing=Boolean(control.initializing);
   if(h.pendingId&&control.last_request_id===h.pendingId){h.pendingArm=h.pendingInit=false;h.pendingId=null;h.wantControl=active}
   if(previouslyActive&&!active)h.wantControl=false;
   if(active)h.pendingArm=false;
   h.section.querySelector('.passive-note').textContent=h.valid?'7个电机关节可控：四指 MCP + 拇指 CMC/MCP/IP。四指 PIP/DIP 无独立电机，已隐藏并固定为0°。目标模型不是实物反馈。':'当前没有有效关节角，目标从参考姿态开始。请先点击位置初始化，完成后再启用控制。';
   if(control.error||control.reason)h.message.textContent=(control.error?control.error+' · ':'')+(control.reason||'')+(active?` · 已发送${control.command_count}帧`:'');
   const diagnostic=actual?.device_diagnostics,init=control.initialization_diagnostics;
   h.section.querySelector('.init-diagnostics').textContent=diagnostic?`固件 ${diagnostic.firmware_version} · 故障 ${diagnostic.faults?.length?diagnostic.faults.map(f=>f.error).join(', '):'无'}${init?` · 初始化返回 ${control.last_init_result} / ${init.duration_ms} ms`:''}`:'';
  }
 }catch(e){networkGood=false;stopAll('连接中断，停止发送；服务端看门狗将请求失能')}
 updateButtons();
}
async function flush(){for(const h of Object.values(hands)){
 if(h.busy)continue;h.busy=true;
 try{
  if(h.previewDirty){h.previewDirty=false;await request(h.side,'preview',{angles_degrees:h.target})}
  if(h.active&&h.wantControl&&h.dirty){h.dirty=false;await request(h.side,'target',{sequence:++h.sequence,angles_degrees:h.target})}
 }catch(e){h.message.textContent=e.message;if(h.active)stopAll('控制请求失败，已请求停止')}
 finally{h.busy=false}
}}
async function heartbeat(){
 if(heartbeatBusy||!config||config.demo)return;heartbeatBusy=true;
 try{for(const h of Object.values(hands))if(h.active&&h.wantControl)await request(h.side,'heartbeat')}
 catch(e){stopAll('心跳失败，已请求停止')}
 finally{heartbeatBusy=false}
}
addEventListener('keydown',e=>{if(e.code==='Escape')stopAll('Esc 停止')});
addEventListener('blur',()=>stopAll('窗口失焦，已请求停止'));
addEventListener('pagehide',()=>stopAll('页面离开，已请求停止'));
document.addEventListener('visibilitychange',()=>{if(document.hidden)stopAll('页面隐藏，已请求停止')});
document.getElementById('pause')?.addEventListener('click',()=>stopAll('画面暂停/恢复，已请求停止'));
(async()=>{
 try{
  config=await(await fetch('/api/teleop/config')).json();sides=config.hands||['left','right'];
  const note=document.createElement('div');note.className='teleop-demo';
  note.textContent=config.demo?'离线演示，不连接实物。':config.motion_locked?'实机运动已锁定，拖拽仅预览。':'按钮控制已开放：先初始化（如需），再点击启用；打开页面不会自动运动。';
  document.querySelector('h1').after(note);sides.forEach(build);await poll();setInterval(poll,200);setInterval(flush,100);setInterval(heartbeat,150);
  window.__gmhTeleop={hands,get frame(){return frame},get config(){return {...config,token:'hidden'}}};
 }catch(e){document.querySelector('#network').textContent='遥操作加载失败：'+e.message}
})();
})();
