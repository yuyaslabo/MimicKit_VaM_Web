// VaM VRHand + HMD input for MuJoCo WASM. No changes to PPO observation layout.
// Raw UDP (23 x float32): left pos3 quat xyzw4 trigger1, right same, HMD pos3 quat4.
// Python bridge sends the unchanged payload with four-byte ASCII header 'VRH1'.
export const vrSettings = {
  enabled:true, timeoutMs:500, delaySeconds:3, positionOffsetX:0,
  positionOffsetY:0.15, positionOffsetZ:0, verticalScale:0.8,
  kp:950, kd:40, maxForce:300, maxError:0.2, gravityComp:true,
  grabEnabled:true, grabDistance:0.2, grabKp:250, grabKd:15, grabMaxForce:500,
  grabPressThreshold:0.25, grabReleaseThreshold:0.10, leftGrab:'None', rightGrab:'None',
  leftDoubleEnabled:true, leftDoubleWindow:0.65,
  rightDoubleEnabled:true, rightDoubleWindow:0.65, rightDownThreshold:-0.65,
  rightPointDistance:5.0, facingMode:'HMD',
  showBodies:true, status:'Waiting for VaM UDP 9999', frames:0,
  hmdX:0,hmdY:0,hmdZ:0, leftTrigger:0, rightTrigger:0,
};
export const vrBodies = [
  {name:'web_vr_left', joint:'web_vr_left_free', color:0x347be8, qpos:-1,qvel:-1,body:-1},
  {name:'web_vr_right',joint:'web_vr_right_free',color:0xf0783f,qpos:-1,qvel:-1,body:-1},
  {name:'web_vr_torso',joint:'web_vr_torso_free',color:0x41aaaa,qpos:-1,qvel:-1,body:-1},
];
let latest=null, lastTimestamp=-Infinity, activeStart=null, wasActive=false;
// MuJoCo body IDs are mapped from the original 31-body MimicKit order.
// These state buffers are independent of the 3 auxiliary VR bodies.
const grabState={
  left:{pressed:false, bodyIndex:-1, lastUnlockedPress:null}, right:{pressed:false, bodyIndex:-1, lastUnlockedPress:null},
  previousPositions:null, previousIds:null, appliedBodies:new Set()
};
function clearGrabForces(data){
  if(data?.xfrc_applied)for(const id of grabState.appliedBodies){
    const start=id*6;
    for(let k=0;k<3;k++)data.xfrc_applied[start+k]=0;
  }
  grabState.appliedBodies.clear();
}
function resetGrab(data){
  clearGrabForces(data);
  for(const hand of ['left','right']){
    grabState[hand].pressed=false;grabState[hand].bodyIndex=-1;
  }
  grabState.previousPositions=null;grabState.previousIds=null;
  vrSettings.leftGrab='None';vrSettings.rightGrab='None';
  grabState.right.lastUnlockedPress=null;
  grabState.left.lastUnlockedPress=null;
}

let onLeftDoubleTrigger=null;
export function setLeftDoubleTriggerHandler(handler){onLeftDoubleTrigger=handler;}
export function getVRHMDPose(){
  if(!vrSettings.enabled || !wasActive || !latest || performance.now()-lastTimestamp>vrSettings.timeoutMs)return null;
  return {pos:latest.hmd.pos.slice(), quat:latest.hmd.quat.slice()};
}
// Newton right-hand double trigger: a fixed WORLD facing point, or live HMD.
let fixedFacingPoint=null;
export function getVRFacingPlayerPosition(){
  if(!vrSettings.enabled || !wasActive || !latest ||
     performance.now()-lastTimestamp>vrSettings.timeoutMs)return null;
  return fixedFacingPoint ? fixedFacingPoint.slice() : latest.hmd.pos.slice();
}
function rightDoubleTriggerAction(right,hmd){
  // Tracking quaternion is already in engine xyzw coordinates.
  // Matches Newton VRPhysicsMixin: controller LOCAL +X is pointing forward.
  const dir=rotate(right.quat,[1,0,0]);
  const length=Math.hypot(...dir);
  if(!(length>1e-8))return;
  const dz=dir[2]/length;
  if(dz<=vrSettings.rightDownThreshold){
    fixedFacingPoint=null;vrSettings.facingMode='HMD';
    console.log('[VR Facing] Right downward double trigger: LIVE HMD');
    return;
  }
  const horiz=Math.hypot(dir[0],dir[1]);
  if(horiz<0.1)return; // Nearly vertical: do not select a facing point.
  const distance=vrSettings.rightPointDistance;
  fixedFacingPoint=[hmd.pos[0]+distance*dir[0]/horiz,
                    hmd.pos[1]+distance*dir[1]/horiz,hmd.pos[2]];
  vrSettings.facingMode='WORLD fixed';
  console.log('[VR Facing] Right double trigger: WORLD point',fixedFacingPoint);
}
function normQ(q){const m=Math.hypot(...q);return m>1e-8?q.map(x=>x/m):[0,0,0,1];}
// Equivalent to _vam_pos_to_engine and _vam_quat_to_engine then VRTrackingMixin's q remap.
function transform(raw, isHmd){
  const [x,y,z]=raw.pos;
  const pos=[z-vrSettings.positionOffsetZ, -(x-vrSettings.positionOffsetX),
    (y-vrSettings.positionOffsetY)*vrSettings.verticalScale];
  const [qx,qy,qz,qw]=raw.quat;
  // intermediate = [-qx,-qz,-qy,qw]; final = [intermediate.y,-intermediate.x,intermediate.z,intermediate.w]
  const quat=normQ([-qz, qx, -qy, qw]);
  if(isHmd){quat[0]=0;quat[1]=0;return {pos,quat:normQ(quat),trigger:0};}
  return {pos,quat,trigger:raw.trigger};
}
export function acceptVRMessage(buffer){
  if(!(buffer instanceof ArrayBuffer)||buffer.byteLength!==96)return false;
  const dv=new DataView(buffer);
  if([0,1,2,3].map(i=>String.fromCharCode(dv.getUint8(i))).join('')!=='VRH1')return false;
  const f=Array.from({length:23},(_,i)=>dv.getFloat32(4+i*4,true));
  if(!f.every(Number.isFinite))return false;
  const read=(o,hasTrigger)=>({pos:f.slice(o,o+3),quat:f.slice(o+3,o+7),trigger:hasTrigger?f[o+7]:0});
  for(const o of [0,8,16])if(Math.hypot(...f.slice(o+3,o+7))<1e-8)return false;
  latest={left:transform(read(0,true),false),right:transform(read(8,true),false),hmd:transform(read(16,false),true)};
  lastTimestamp=performance.now();vrSettings.frames++;
  vrSettings.leftTrigger=latest.left.trigger;vrSettings.rightTrigger=latest.right.trigger;
  [vrSettings.hmdX,vrSettings.hmdY,vrSettings.hmdZ]=latest.hmd.pos;
  vrSettings.status='Receiving';return true;
}
// Insert three optional auxiliary bodies into the MJCF before MuJoCo compiles it.
export function addVRBodiesToXML(xmlText){
  const doc=new DOMParser().parseFromString(xmlText,'application/xml');
  if(doc.querySelector('parsererror'))throw new Error('[VR] invalid MJCF');
  const worldbody=doc.querySelector('worldbody');
  if(!worldbody)throw new Error('[VR] MJCF worldbody missing');
  function body(name,geomType,size,mass,rgba,contype='1',conaffinity='1'){
    const b=doc.createElement('body');b.setAttribute('name',name);b.setAttribute('pos','0 0 1000');
    const j=doc.createElement('freejoint');j.setAttribute('name',name+'_free');b.appendChild(j);
    const g=doc.createElement('geom');g.setAttribute('name',name+'_geom');g.setAttribute('type',geomType);
    g.setAttribute('size',size);g.setAttribute('mass',mass);g.setAttribute('rgba',rgba);
    g.setAttribute('contype',contype);g.setAttribute('conaffinity',conaffinity);
    g.setAttribute('friction','1 0.005 0.0001');
    b.appendChild(g);worldbody.appendChild(b);
  }
  body('web_vr_left','sphere','0.03','1','0.2 0.65 1 0.6');
  body('web_vr_right','sphere','0.03','1','1 0.4 0.2 0.6');
  body('web_vr_torso','capsule','0.08 0.125','8','0.3 0.8 0.8 0.4','1','2');
  return new XMLSerializer().serializeToString(doc);
}
export function bindVRBodies(mujoco,model){
  for(const b of vrBodies){
    const bi=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY.value,b.name);
    const ji=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT.value,b.joint);
    if(bi<0||ji<0)throw new Error('[VR] missing body/joint '+b.name);
    b.body=bi;b.qpos=model.jnt_qposadr[ji];b.qvel=model.jnt_dofadr[ji];
  }
}
function park(data,b){
  data.qpos.set([100+b.body*2,0,1000,1,0,0,0],b.qpos);
  data.qvel.fill(0,b.qvel,b.qvel+6);
  for(let i=0;i<6;i++)data.xfrc_applied[b.body*6+i]=0;
}
export function resetVR(data){resetGrab(data);for(const b of vrBodies)if(b.qpos>=0)park(data,b);activeStart=null;wasActive=false;}
function setRotation(data,b,quat){
  data.qpos.set([quat[3],quat[0],quat[1],quat[2]],b.qpos+3);
  data.qvel.fill(0,b.qvel+3,b.qvel+6);
}
function alignZ(axis){
  const d=Math.hypot(...axis);if(d<1e-8)return [0,0,0,1];
  const [x,y,z]=axis.map(v=>v/d);
  if(z>1-1e-8)return [0,0,0,1];
  if(z< -1+1e-8)return [1,0,0,0];
  return normQ([-y,x,0,1+z]);
}
export function stepVR(data,dt,bodyMap){
  if(!data?.xfrc_applied)return;
  for(const b of vrBodies)for(let i=0;i<6;i++)data.xfrc_applied[b.body*6+i]=0;
  clearGrabForces(data);
  if(!vrSettings.enabled || !latest || performance.now()-lastTimestamp>vrSettings.timeoutMs){
    vrSettings.status=!vrSettings.enabled?'Disabled':'No fresh UDP 9999';
    resetGrab(data);
    if(wasActive){for(const b of vrBodies)park(data,b);wasActive=false;}
    return;
  }
  if(activeStart===null)activeStart=performance.now();
  if(performance.now()-activeStart<vrSettings.delaySeconds*1000){
    vrSettings.status='Startup delay';resetGrab(data);for(const b of vrBodies)park(data,b);return;
  }
  const {left,right,hmd}=latest;
  for(const [index,target] of [[0,left],[1,right]]){
    const b=vrBodies[index],p=b.qpos,v=b.qvel;
    const dest=[target.pos[0],target.pos[1],target.pos[2]+0.5*9.81*(1/30)**2];
    if(!wasActive){data.qpos.set(dest,p);data.qvel.fill(0,v,v+6);}
    setRotation(data,b,target.quat);
    const err=dest.map((x,i)=>x-data.qpos[p+i]);
    const len=Math.hypot(...err),k=len>vrSettings.maxError&&vrSettings.maxError>0?vrSettings.maxError/len:1;
    let force=err.map((x,i)=>vrSettings.kp*x*k-vrSettings.kd*data.qvel[v+i]);
    const flen=Math.hypot(...force);
    if(vrSettings.maxForce>0 && flen>vrSettings.maxForce)force=force.map(x=>x*vrSettings.maxForce/flen);
    if(vrSettings.gravityComp)force[2]+=9.81;
    for(let i=0;i<3;i++)data.xfrc_applied[b.body*6+i]=force[i];
  }
  // Torso capsule: kinematic follow of upright HMD yaw, like VRBodyMixin.
  const fwd=rotate(hmd.quat,[1,0,0]);const yaw=Math.atan2(fwd[1],fwd[0]);
  const chest=[hmd.pos[0]-0.02*Math.cos(yaw),hmd.pos[1]-0.02*Math.sin(yaw),hmd.pos[2]-0.16];
  const waist=[chest[0],chest[1],chest[2]-0.25];
  const b=vrBodies[2];const mid=chest.map((x,i)=>(x+waist[i])/2);
  data.qpos.set(mid,b.qpos);setRotation(data,b,alignZ(waist.map((x,i)=>x-chest[i])));
  data.qvel.fill(0,b.qvel,b.qvel+6);wasActive=true;
  applyVRGrab(data,dt,bodyMap,left,right,hmd);
}

// Newton VRPhysicsMixin._apply_vrhand_drag port, excluding double trigger and facing/target commands.
// Forces act on the humanoid body COM in world coordinates (MuJoCo xfrc_applied).
// World linear velocity is estimated from data.xpos on successive physics ticks,
// avoiding cvel (which is a spatial quantity in a different reference frame).
function applyVRGrab(data,dt,bodyMap,left,right,hmd){
  if(!Array.isArray(bodyMap) || !bodyMap.length){
    resetGrab(data);return;
  }
  const positions=bodyMap.map(id=>[
    data.xpos[id*3],data.xpos[id*3+1],data.xpos[id*3+2]
  ]);
  const prev=grabState.previousPositions;
  const validPrev=prev && grabState.previousIds &&
    grabState.previousIds.length===bodyMap.length &&
    bodyMap.every((id,i)=>id===grabState.previousIds[i]);
  const velocities=positions.map((p,i)=>validPrev && dt>0
    ?p.map((v,j)=>(v-prev[i][j])/dt):[0,0,0]);
  grabState.previousPositions=positions;
  grabState.previousIds=bodyMap.slice();
  const press=vrSettings.grabPressThreshold, release=vrSettings.grabReleaseThreshold;
  for(const [name,handTarget,handBody] of [
    ['left',left,vrBodies[0]], ['right',right,vrBodies[1]]
  ]){
    const state=grabState[name],trigger=handTarget.trigger;
    if(trigger<=release){state.pressed=false;state.bodyIndex=-1;}
    else if(!state.pressed && trigger>=press){
      state.pressed=true;
      const handPos=Array.from(data.qpos.slice(handBody.qpos,handBody.qpos+3));
      let nearest=-1,minDist=Infinity;
      if(vrSettings.grabEnabled)for(let i=0;i<positions.length;i++){
        const p=positions[i];
        const distance=Math.hypot(p[0]-handPos[0],p[1]-handPos[1],p[2]-handPos[2]);
        if(distance<minDist){minDist=distance;nearest=i;}
      }
      // Newton locks only on a rising trigger edge and only within 0.20 m.
      state.bodyIndex=minDist<vrSettings.grabDistance?nearest:-1;
      if(state.bodyIndex>=0){
        state.lastUnlockedPress=null;
        console.log(`[VR Grab] ${name} LOCK body index=${state.bodyIndex} distance=${minDist.toFixed(3)}m`);
      }else if((name==='right' && vrSettings.rightDoubleEnabled) ||
                (name==='left' && vrSettings.leftDoubleEnabled)){
        // Newton: count a double-press only when neither press grabbed a bone.
        const now=performance.now();
        const previous=state.lastUnlockedPress;
        const windowMs=(name==='left'?vrSettings.leftDoubleWindow:vrSettings.rightDoubleWindow)*1000;
        if(previous!==null && now>previous && now-previous<=windowMs){
          state.lastUnlockedPress=null;
          if(name==='right')rightDoubleTriggerAction(right,hmd);
          else if(typeof onLeftDoubleTrigger==='function')onLeftDoubleTrigger();
        }else state.lastUnlockedPress=now;
      }
    }
    const index=state.bodyIndex;
    vrSettings[name+'Grab']=index>=0 ? `Body ${index}` : 'None';
    if(index<0)continue;
    const bodyId=bodyMap[index],p=positions[index],v=velocities[index];
    const handPos=Array.from(data.qpos.slice(handBody.qpos,handBody.qpos+3));
    // Newton scaled_trigger = (clamp(trigger, .3, 1) - .3) / .7
    const scale=(Math.max(0.3,Math.min(1,trigger))-0.3)/0.7;
    let force=p.map((x,i)=>(vrSettings.grabKp*(handPos[i]-x)-vrSettings.grabKd*v[i])*scale);
    const size=Math.hypot(...force);
    if(vrSettings.grabMaxForce>0 && size>vrSettings.grabMaxForce)
      force=force.map(x=>x*vrSettings.grabMaxForce/size);
    if(force.every(Number.isFinite)){
      for(let k=0;k<3;k++)data.xfrc_applied[bodyId*6+k]+=force[k];
      grabState.appliedBodies.add(bodyId);
    }
  }
}
function rotate(q,v){const [x,y,z,w]=q;const t=[2*(y*v[2]-z*v[1]),2*(z*v[0]-x*v[2]),2*(x*v[1]-y*v[0])];return [v[0]+w*t[0]+y*t[2]-z*t[1],v[1]+w*t[1]+z*t[0]-x*t[2],v[2]+w*t[2]+x*t[1]-y*t[0]];}
export function updateVRVisuals(model,data,sceneGroup,THREE){
  if(!sceneGroup)return;
  for(const b of vrBodies){
    if(!b.visual){
      const geometry=b.name.includes('torso')?new THREE.CapsuleGeometry(0.08,0.25,8,12):new THREE.SphereGeometry(0.03,16,12);
      b.visual=new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color:b.color,transparent:true,opacity:0.7}));
      // Three CapsuleGeometry is Y-up, MuJoCo capsule is Z-up.
      if(b.name.includes('torso'))geometry.rotateX(Math.PI/2);
      sceneGroup.add(b.visual);
    }
    b.visual.visible=vrSettings.showBodies && wasActive;
    if(!b.visual.visible)continue;
    b.visual.position.set(...data.xpos.slice(b.body*3,b.body*3+3));
    const q=data.xquat.slice(b.body*4,b.body*4+4);b.visual.quaternion.set(q[1],q[2],q[3],q[0]);
  }
}
