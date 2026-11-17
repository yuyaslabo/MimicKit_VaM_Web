// Newton TaskLocationEnv HMD target controller, adapted for browser MuJoCo coordinates.
// STAY bypasses obstacle checks; FOLLOW provides basic furniture avoidance.
// Newton's separate rescue/route-planner/wall-follow is intentionally NOT implemented.
export const targetSettings={
  mode:'STAY',minDistance:0.20,maxDistance:0.55,
  recoverDistance:1.0,recoverDeadzone:0.005,
  yawDeadzoneDeg:10,movingSpeedOn:0.35,movingSpeedOff:0.20,speedSmoothing:0.12,
  movingTargetDistance:0.50,
  avoidFurniture:true,avoidClearance:0.15,avoidRadius:1.0,avoidGrid:0.10,
  nominalDistance:0,moving:false,avoidStatus:'STAY',
};
let initialized=false, prevHmd=null,prevFacing=null,placementFacing=null;
let lastNominal=null,speedEMA=0, moving=false,needsMaxDistance=true;
const clamp=(x,lo,hi)=>Math.max(lo,Math.min(hi,x));
function normalize(v){const n=Math.hypot(...v);return n>1e-8?v.map(x=>x/n):[1,0];}
function cross(a,b){return a[0]*b[1]-a[1]*b[0];}
function rotate2(v,rad){const c=Math.cos(rad),s=Math.sin(rad);return [v[0]*c-v[1]*s,v[0]*s+v[1]*c];}
function distance(a,b){return Math.hypot(a[0]-b[0],a[1]-b[1]);}
function add(a,dir,scale){return [a[0]+dir[0]*scale,a[1]+dir[1]*scale];}
// Newton _avoid_update_target: the actor's FOLLOW goal is always in the
// agent-centered XY disk, even without furniture or when LOS is unobstructed.
function withinAgentRadius(root,point){
  const center=root.slice(0,2);
  const r=Math.max(0.05,targetSettings.avoidRadius);
  const d=distance(center,point);
  return d>r?add(center,[(point[0]-center[0])/d,(point[1]-center[1])/d],r):point.slice();
}
function hmdHeading(hmd){const [x,y,z,w]=hmd.quat;return normalize([1-2*(y*y+z*z),2*(x*y+w*z)]);}
function resetHistory(){initialized=false;prevHmd=null;prevFacing=null;placementFacing=null;lastNominal=null;speedEMA=0;moving=false;needsMaxDistance=true;targetSettings.moving=false;}
export function resetHMDTarget(){
  // A character reset must not unfreeze the user's selected STAY target.
  resetHistory();targetSettings.avoidStatus=targetSettings.mode==='STAY'?'STAY':'Awaiting HMD';
}
export function toggleHMDTarget(targetXY,hmd){
  if(targetSettings.mode==='STAY'){
    if(!hmd){console.warn('[HMD Target] FOLLOW requested without valid VR tracking; remaining STAY');return;}
    targetSettings.mode='FOLLOW';resetHistory();targetSettings.avoidStatus='FOLLOW';
    console.info('[HMD Target] LEFT double: FOLLOW; distance reset to MAX',targetSettings.maxDistance);
  }else{
    targetSettings.mode='STAY';resetHistory();targetSettings.avoidStatus='STAY';
    console.info('[HMD Target] LEFT double: STAY at WORLD XY',targetXY.x,targetXY.y);
  }
}
// Convex polygon collision with Euclidean clearance from each line segment.
function pointSegDist(p,a,b){
  const dx=b[0]-a[0],dy=b[1]-a[1],den=dx*dx+dy*dy;
  const t=den?clamp(((p[0]-a[0])*dx+(p[1]-a[1])*dy)/den,0,1):0;
  return Math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy);
}
function blocked(point,obstacles,clearance,rootZ){
  for(const shape of obstacles){
    if(shape.zmax<rootZ-0.80||shape.zmin>rootZ+0.70)continue;
    const [xmin,ymin,xmax,ymax]=shape.bounds;
    if(point[0]<xmin-clearance||point[0]>xmax+clearance||point[1]<ymin-clearance||point[1]>ymax+clearance)continue;
    const poly=shape.poly;if(poly.length<3)continue;
    let inside=true,near=false;
    for(let i=0;i<poly.length;i++){
      const a=poly[i],b=poly[(i+1)%poly.length];
      if(cross([b[0]-a[0],b[1]-a[1]],[point[0]-a[0],point[1]-a[1]]) < -1e-8)inside=false;
      if(pointSegDist(point,a,b)<=clearance)near=true;
    }
    if(inside||near)return true;
  }
  return false;
}
function clearLine(a,b,obstacles,clearance,z){
  const length=distance(a,b);const steps=Math.max(1,Math.ceil(length/0.05));
  for(let i=1;i<=steps;i++){
    const t=i/steps;
    if(blocked([a[0]+(b[0]-a[0])*t,a[1]+(b[1]-a[1])*t],obstacles,clearance,z))return false;
  }
  return true;
}
function isBehindHmd(p,hmdXY,face){
  const delta=[p[0]-hmdXY[0],p[1]-hmdXY[1]];
  return distance(p,hmdXY)<0.40 && delta[0]*face[0]+delta[1]*face[1]<0;
}
function adjustWhiteTarget(desired,hmdXY,heading,rawHeading,root,obstacles,movingMode,minDist){
  const z=root[2],clear=targetSettings.avoidClearance;
  const valid=p=>!blocked(p,obstacles,clear,z)&&!isBehindHmd(p,hmdXY,rawHeading);
  if(valid(desired))return desired;
  if(!movingMode){
    const start=Math.max(minDist,distance(desired,hmdXY));
    for(let d=start;d>=minDist-1e-8;d-=0.025){const p=add(hmdXY,heading,Math.max(d,minDist));if(valid(p))return p;}
  }
  // Newton basic-target fallback: HMD-centered radius 0.5m, favor heading.
  let chosen=null,score=Infinity;
  for(let d=minDist;d<=0.501;d+=0.05){
    for(let angle=-180;angle<=180;angle+=10){
      const dir=rotate2(heading,angle*Math.PI/180),p=add(hmdXY,dir,d);
      if(!valid(p))continue;
      const metric=Math.abs(angle)*0.01+distance(p,desired)*0.1;
      if(metric<score){score=metric;chosen=p;}
    }
  }
  return chosen||desired;
}
function findSafeTarget(root,white,obstacles,hmdXY,rawHeading){
  const closestDisk=withinAgentRadius(root,white);
  if(!obstacles.length)return {point:closestDisk,status:'Clear (1m disk)'};
  const origin=root.slice(0,2),clear=targetSettings.avoidClearance,z=root[2];
  const free=p=>!blocked(p,obstacles,clear,z)&&!isBehindHmd(p,hmdXY,rawHeading);
  if(free(closestDisk) && clearLine(origin,closestDisk,obstacles,clear,z))return {point:closestDisk,status:'Direct (1m disk)'};
  let best=null,score=Infinity;
  const r=targetSettings.avoidRadius,grid=Math.max(0.05,targetSettings.avoidGrid);
  for(let x=-r;x<=r+1e-8;x+=grid){for(let y=-r;y<=r+1e-8;y+=grid){
    if(x*x+y*y>r*r+1e-8)continue;
    const p=[origin[0]+x,origin[1]+y];
    if(!free(p)||!clearLine(origin,p,obstacles,clear,z))continue;
    const d=distance(p,white)+0.03*distance(p,origin);
    if(d<score){best=p;score=d;}
  }}
  return best?{point:best,status:'Safe nearby'}:{point:origin,status:'No clear route (hold)'};
}
// Called immediately before the 30Hz ONNX observation is assembled.
// root: raw Mujoco world XYZ, HMD pose: transformed MuJoCo world coordinates.
export function stepHMDTarget(targetXY,hmd,root,obstacles,dt){
  if(targetSettings.mode!=='FOLLOW')return;
  if(!hmd || !root || !hmd.pos.every(Number.isFinite)){
    resetHistory();targetSettings.avoidStatus='Waiting for HMD';return;
  }
  const minD=Math.max(0.01,Math.min(targetSettings.minDistance,targetSettings.maxDistance-0.01));
  const maxD=Math.max(minD+0.01,targetSettings.maxDistance);
  const hmdXY=hmd.pos.slice(0,2),heading=hmdHeading(hmd);
  if(!placementFacing)placementFacing=heading.slice();
  if(!initialized){
    initialized=true;needsMaxDistance=false;prevHmd=hmdXY.slice();prevFacing=heading.slice();
    placementFacing=heading.slice();lastNominal=add(hmdXY,heading,maxD);
    speedEMA=0;moving=false;
  }else{
    // Yaw dead-zone: preserve previous placement heading until excess angle occurs.
    const error=Math.atan2(cross(placementFacing,heading),
      clamp(placementFacing[0]*heading[0]+placementFacing[1]*heading[1],-1,1));
    const deadzone=targetSettings.yawDeadzoneDeg*Math.PI/180;
    const delta=Math.sign(error)*Math.max(0,Math.abs(error)-deadzone);
    placementFacing=normalize(rotate2(placementFacing,delta));
  }
  const delta=[hmdXY[0]-prevHmd[0],hmdXY[1]-prevHmd[1]];
  const forward=normalize([prevFacing[0]+heading[0],prevFacing[1]+heading[1]]);
  const forwardStep=delta[0]*forward[0]+delta[1]*forward[1];
  const priorDistance=clamp(distance(lastNominal,prevHmd),minD,maxD);
  const currentSpeed=Math.hypot(...delta)/Math.max(dt,1e-6);
  const alpha=dt/(Math.max(0,targetSettings.speedSmoothing)+dt);
  speedEMA+=(currentSpeed-speedEMA)*alpha;
  const wasMoving=moving;
  moving=moving?speedEMA>targetSettings.movingSpeedOff:speedEMA>=targetSettings.movingSpeedOn;
  let dist=initialized&& !needsMaxDistance?clamp(priorDistance-forwardStep,minD,maxD):maxD;
  const preview=add(hmdXY,placementFacing,dist);
  const travel=distance(preview,lastNominal);
  const recovery=(maxD-minD)*Math.max(0,travel-targetSettings.recoverDeadzone)/Math.max(1e-5,targetSettings.recoverDistance);
  dist=clamp(dist+recovery,minD,maxD);
  if(wasMoving&&!moving)dist=maxD;
  const nominal=add(hmdXY,placementFacing,dist);
  let desired=nominal;
  if(moving){
    const deltaAgent=[root[0]-hmdXY[0],root[1]-hmdXY[1]];
    const agentDirection=Math.hypot(...deltaAgent)>1e-6?normalize(deltaAgent):[-heading[0],-heading[1]];
    desired=add(hmdXY,agentDirection,Math.max(0,targetSettings.movingTargetDistance));
  }
  lastNominal=desired.slice();prevHmd=hmdXY;prevFacing=heading;
  targetSettings.moving=moving;
  targetSettings.nominalDistance=+distance(nominal,hmdXY).toFixed(3);
  const shapes=targetSettings.avoidFurniture?obstacles||[]:[];
  const white=targetSettings.avoidFurniture?
    adjustWhiteTarget(desired,hmdXY,placementFacing,heading,root,shapes,moving,minD):desired;
  const safe=targetSettings.avoidFurniture?
    findSafeTarget(root,white,shapes,hmdXY,heading):{point:withinAgentRadius(root,white),status:'Avoid OFF (1m disk)'};
  // Final safety invariant: even fallback paths cannot exceed the local disk.
  const finalPoint=withinAgentRadius(root,safe.point);
  targetXY.x=finalPoint[0];targetXY.y=finalPoint[1];
  targetSettings.avoidStatus=safe.status;
}
