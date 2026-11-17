// Pre-registered Newton MJCF compound colliders for browser MuJoCo.
// UDP 9001 -> Python websocket relay 'ENV1' + original UDP payload.
// No UDP 9002 shape receiver. Shape XML is read once before mjModel compilation.
export const furnitureSettings={enabled:true,showShapes:true,loaded:0,active:0,packets:0,status:'Not loaded',receivedIds:'-',missingIds:'-'};
const C=[[0,0,1],[-1,0,0],[0,1,0]];
let assets=[], registered=new Map(), latest=new Map(), visuals=new Map(), lastMessage=0, lastDiagnostic=0, lastUnknownSignature='';
const parking=[0,0,1000];
function quatNorm(q){const n=Math.hypot(...q);return n>1e-9?q.map(x=>x/n):[0,0,0,1];}
function quaternionToMatrix(q){const [x,y,z,w]=quatNorm(q);return [
 [1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
 [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
 [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]];}
function multiply(a,b){return a.map(row=>b[0].map((_,j)=>row.reduce((s,v,k)=>s+v*b[k][j],0)));}
function transpose(a){return a[0].map((_,i)=>a.map(row=>row[i]));}
function matrixToQuaternion(m){const trace=m[0][0]+m[1][1]+m[2][2];let x,y,z,w;
 if(trace>0){const s=Math.sqrt(trace+1)*2;w=.25*s;x=(m[2][1]-m[1][2])/s;y=(m[0][2]-m[2][0])/s;z=(m[1][0]-m[0][1])/s;}
 else if(m[0][0]>m[1][1]&&m[0][0]>m[2][2]){const s=Math.sqrt(1+m[0][0]-m[1][1]-m[2][2])*2;w=(m[2][1]-m[1][2])/s;x=.25*s;y=(m[0][1]+m[1][0])/s;z=(m[0][2]+m[2][0])/s;}
 else if(m[1][1]>m[2][2]){const s=Math.sqrt(1+m[1][1]-m[0][0]-m[2][2])*2;w=(m[0][2]-m[2][0])/s;x=(m[0][1]+m[1][0])/s;y=.25*s;z=(m[1][2]+m[2][1])/s;}
 else{const s=Math.sqrt(1+m[2][2]-m[0][0]-m[1][1])*2;w=(m[1][0]-m[0][1])/s;x=(m[0][2]+m[2][0])/s;y=(m[1][2]+m[2][1])/s;z=.25*s;}
 return quatNorm([x,y,z,w]);}
function basisQuat(q){return matrixToQuaternion(multiply(multiply(C,quaternionToMatrix(q)),transpose(C)));}
function parseFloatVec(s,n,fallback){const v=(s||fallback).trim().split(/\s+/).map(Number);if(v.length!==n||v.some(x=>!Number.isFinite(x)))throw Error('Invalid MJCF vector '+s);return v;}
// Runs before existing addVRBodiesToXML. Uses XML documents for safe names and attributes.
export async function addFurnitureToXML(xmlText,baseUrl){
 const manifestUrl=`${baseUrl}vam_env_boxes/manifest.json`;
 let manifest;
 try{const r=await fetch(manifestUrl,{cache:'no-store'});if(!r.ok){if(r.status===404){console.info('[Furniture] No manifest; empty environment');furnitureSettings.status='No manifest';return xmlText;}throw Error(`HTTP ${r.status}`);} manifest=await r.json();}
 catch(e){console.warn('[Furniture] Manifest unavailable:',e);furnitureSettings.status='No manifest';return xmlText;}
 if(!Array.isArray(manifest))throw Error('[Furniture] manifest.json must be an array of XML filenames');
 const baseDoc=new DOMParser().parseFromString(xmlText,'application/xml');
 if(baseDoc.querySelector('parsererror'))throw Error('[Furniture] invalid base MJCF');
 const world=baseDoc.querySelector('worldbody');if(!world)throw Error('[Furniture] no worldbody');
 assets=[];registered.clear();latest.clear();furnitureSettings.loaded=0;furnitureSettings.receivedIds='-';furnitureSettings.missingIds='-';
 for(const file of manifest){
  if(typeof file!=='string'||!/^atom_\d+\.xml$/.test(file))throw Error('[Furniture] Invalid filename '+file);
  const id=Number(/^atom_(\d+)\.xml$/.exec(file)[1]);
  if(!Number.isSafeInteger(id)||id<0||id>0xffffff||registered.has(id))throw Error('[Furniture] Invalid/duplicate Atom ID '+id);
  const response=await fetch(`${baseUrl}vam_env_boxes/${file}`);if(!response.ok)throw Error('[Furniture] Cannot load '+file+' HTTP '+response.status);
  const d=new DOMParser().parseFromString(await response.text(),'application/xml');if(d.querySelector('parsererror'))throw Error('[Furniture] Invalid '+file);
  const original=d.querySelector('worldbody > body');if(!original)throw Error('[Furniture] No body in '+file);
  const name=`web_vam_atom_${id}`, body=baseDoc.createElement('body');body.setAttribute('name',name);body.setAttribute('mocap','true');body.setAttribute('pos','0 0 1000');
  const geoms=[];
  for(const [i,g] of [...original.querySelectorAll(':scope > geom')].entries()){
   const type=g.getAttribute('type')||'sphere';
   if(!['box','sphere','capsule','cylinder'].includes(type))throw Error(`[Furniture] Unsupported geom ${type} in ${file}`);
   const clone=baseDoc.importNode(g,true);clone.setAttribute('name',`${name}_geom_${i}`);
   // Original Newton compound-shape mass/size/quat/position are retained.
   body.appendChild(clone);
   geoms.push({type,pos:parseFloatVec(g.getAttribute('pos'),3,'0 0 0'),quat:parseFloatVec(g.getAttribute('quat'),4,'1 0 0 0'),size:parseFloatVec(g.getAttribute('size'),type==='box'?3:type==='sphere'?1:2,type==='sphere'?'0.1':'0.1 0.1')});
  }
  if(!geoms.length)throw Error('[Furniture] No geoms in '+file);
  world.appendChild(body);assets.push({id,name,geoms});registered.set(id,{name,mocap:-1});
 }
 furnitureSettings.loaded=assets.length;furnitureSettings.status=`Registered ${assets.length} XML assets`;
 console.info('[Furniture DIAG] Registered XML Atom IDs:', assets.map(a=>a.id), 'geometry counts:', assets.map(a=>`${a.id}:${a.geoms.length}`).join(', '));
 console.info(`[Furniture] preloaded ${assets.length} Newton XML assets (no UDP 9002)`);
 return new XMLSerializer().serializeToString(baseDoc);
}
export function bindFurniture(mujoco,model){
 for(const a of assets){const body=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY.value,a.name);
  if(body<0)throw Error('[Furniture] Missing compiled body '+a.name);
  const mocap=model.body_mocapid[body];if(mocap<0)throw Error('[Furniture] Not mocap '+a.name);
  registered.get(a.id).mocap=mocap;
  console.info(`[Furniture DIAG] MuJoCo body registered: Atom=${a.id} body=${body} mocap=${mocap} geoms=${a.geoms.length}`);
 }
}
export function acceptFurnitureMessage(buffer){
 if(!(buffer instanceof ArrayBuffer)||buffer.byteLength<8)return false;
 const dv=new DataView(buffer);if(dv.getUint32(0,false)!==0x454e5631)return false; // ENV1
 const bytes=buffer.byteLength-4;if(bytes<4||bytes%4!==0){console.warn('[Furniture DIAG] Invalid ENV1 byte length:',buffer.byteLength);return true;}
 const count=dv.getFloat32(4,true);if(!Number.isInteger(count)||count<0||count>1000||bytes!==4*(1+count*11)){console.warn('[Furniture DIAG] Invalid ENV1 count/length:',count,buffer.byteLength);return true;}
 const next=new Map();
 for(let i=0;i<count;i++){
  const o=8+i*44,id=dv.getFloat32(o,true),v=[];
  for(let k=1;k<11;k++)v.push(dv.getFloat32(o+4*k,true));
  if(!Number.isInteger(id)||id<0||id>0xffffff||next.has(id)||v.some(x=>!Number.isFinite(x))||v.slice(3,6).some(x=>x<=0)||Math.hypot(...v.slice(6,10))<1e-8){console.warn('[Furniture DIAG] Invalid ENV1 Atom record:',i,id);return true;}
  next.set(id,{pos:v.slice(0,3),quat:quatNorm(v.slice(6,10))});
 }
 latest=next;furnitureSettings.packets++;lastMessage=performance.now();
 const ids=[...next.keys()], missing=ids.filter(id=>!registered.has(id));
 furnitureSettings.receivedIds=ids.length?ids.join(', '):'(none)';
 furnitureSettings.missingIds=missing.length?missing.join(', '):'(none)';
 const signature=missing.slice().sort((a,b)=>a-b).join(',');
 if(signature!==lastUnknownSignature){
  if(missing.length)console.warn('[Furniture DIAG] UDP Atom IDs WITHOUT registered XML:',missing,'Registered:',[...registered.keys()]);
  else console.info('[Furniture DIAG] All received Atom IDs have XML registration:',ids);
  lastUnknownSignature=signature;
 }
 if(furnitureSettings.packets===1)console.info('[Furniture DIAG] First ENV1 packet:',{count,ids,bytes:buffer.byteLength});
 return true;
}
function setMocap(data,id,pos,q){const m=registered.get(id)?.mocap;if(m===undefined||m<0)return;
 data.mocap_pos.set(pos,m*3);data.mocap_quat.set([q[3],q[0],q[1],q[2]],m*4);
}
export function resetFurniture(data){for(const a of assets)setMocap(data,a.id,parking,[0,0,0,1]);furnitureSettings.active=0;}
export function stepFurniture(data){if(!data?.mocap_pos)return;
 let n=0;
 for(const a of assets){const pose=furnitureSettings.enabled?latest.get(a.id):null;
  if(!pose){setMocap(data,a.id,parking,[0,0,0,1]);continue;}
  const p=[pose.pos[2], -pose.pos[0], (pose.pos[1]-.15)*.8+.22];
  // Newton _envbox_rebuild_pose_cache: p=C@(pos-[0,.15,0]); p.z=p.z*0.8+0.22
  setMocap(data,a.id,p,basisQuat(pose.quat));n++;
 }
 furnitureSettings.active=n;
 const now=performance.now();
 if(now-lastDiagnostic>5000){
  lastDiagnostic=now;
  console.info('[Furniture DIAG] Status',getFurnitureReport(data));
 }
 if(furnitureSettings.enabled&&assets.length&&lastMessage&&performance.now()-lastMessage>3000)furnitureSettings.status='Pose stream stale';
 else furnitureSettings.status=`${n}/${assets.length} active`;
}
export function updateFurnitureVisuals(data,worldGroup,THREE){
 if(!assets.length||!worldGroup)return;
 for(const a of assets){if(!visuals.has(a.id)){
   const group=new THREE.Group();group.name='Furniture '+a.id;worldGroup.add(group);
   for(const geom of a.geoms){let geo;const s=geom.size;
    if(geom.type==='box')geo=new THREE.BoxGeometry(s[0]*2,s[1]*2,s[2]*2);
    else if(geom.type==='sphere')geo=new THREE.SphereGeometry(s[0],12,8);
    else if(geom.type==='cylinder')geo=new THREE.CylinderGeometry(s[0],s[0],s[1]*2,12);
    else geo=new THREE.CapsuleGeometry(s[0],s[1]*2,4,10);
    const mat=new THREE.MeshStandardMaterial({color:0x9bb581,transparent:true,opacity:.45,depthWrite:false});
    const mesh=new THREE.Mesh(geo,mat);mesh.position.set(...geom.pos);
    const q=geom.quat;mesh.quaternion.set(q[1],q[2],q[3],q[0]);
    // Three.js Y-axis cylinder/capsule vs MuJoCo Z-axis.
    if(geom.type==='cylinder'||geom.type==='capsule')mesh.quaternion.multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),Math.PI/2));
    group.add(mesh);
   }visuals.set(a.id,group);console.info(`[Furniture DIAG] Three.js mesh created for Atom ${a.id}: ${a.geoms.length} shapes`);
  }
  const group=visuals.get(a.id),m=registered.get(a.id)?.mocap;group.visible=!!furnitureSettings.showShapes && !!furnitureSettings.enabled && latest.has(a.id);
  if(m>=0){group.position.set(data.mocap_pos[m*3],data.mocap_pos[m*3+1],data.mocap_pos[m*3+2]);group.quaternion.set(data.mocap_quat[m*4+1],data.mocap_quat[m*4+2],data.mocap_quat[m*4+3],data.mocap_quat[m*4]);}
 }
}

// Browser console diagnostics: distinguish UDP, XML registration, mocap, and rendering.
export function getFurnitureReport(data=null){
 const incoming=[...latest.keys()];
 const missing=incoming.filter(id=>!registered.has(id));
 const matched=incoming.filter(id=>registered.has(id));
 const mocap=matched.map(id=>({id,mocap:registered.get(id).mocap,
   position:data && registered.get(id).mocap>=0
     ?Array.from(data.mocap_pos.slice(registered.get(id).mocap*3,registered.get(id).mocap*3+3)).map(x=>+x.toFixed(3)):null}));
 return {xmlRegistered:[...registered.keys()],incoming,missing,matched,
   xmlCount:assets.length,packets:furnitureSettings.packets,active:furnitureSettings.active,
   showShapes:furnitureSettings.showShapes,enabled:furnitureSettings.enabled,
   renderedMeshes:[...visuals.keys()],mocap,
   lastPacketAgeMs:lastMessage?Math.round(performance.now()-lastMessage):null};
}

// Simplified collision-avoidance geometry for Web HMD target control.
// Return world-space box OBBs from the same compiled MJCF + mocap pose used for physics.
// No decoding of UDP 9002 and no approximation from Atom-wide bounds.
export function getFurnitureBoxes(data){
  if(!furnitureSettings.enabled || !data?.mocap_pos)return [];
  const boxes=[];
  const quatMul=(a,b)=>{const [x,y,z,w]=a,[u,v,t,s]=b;return [w*u+x*s+y*t-z*v,w*v-x*t+y*s+z*u,w*t+x*v-y*u+z*s,w*s-x*u-y*v-z*t];};
  const rotate=(q,v)=>{const [x,y,z,w]=q;const t=[2*(y*v[2]-z*v[1]),2*(z*v[0]-x*v[2]),2*(x*v[1]-y*v[0])];return [v[0]+w*t[0]+y*t[2]-z*t[1],v[1]+w*t[1]+z*t[0]-x*t[2],v[2]+w*t[2]+x*t[1]-y*t[0]];};
  for(const a of assets){
    if(!latest.has(a.id))continue;
    const m=registered.get(a.id)?.mocap;if(m===undefined||m<0)continue;
    const p=Array.from(data.mocap_pos.slice(m*3,m*3+3));
    const q=Array.from(data.mocap_quat.slice(m*4,m*4+4));const rootQ=[q[1],q[2],q[3],q[0]];
    for(const g of a.geoms){
      if(g.type!=='box')continue;
      const off=rotate(rootQ,g.pos);
      const gq=[g.quat[1],g.quat[2],g.quat[3],g.quat[0]];
      const worldQ=quatMul(rootQ,gq);
      const center=[p[0]+off[0],p[1]+off[1],p[2]+off[2]];
      // 2D footprint derived from full rotated box corners, with conservative Z extent.
      let xmin=Infinity,xmax=-Infinity,ymin=Infinity,ymax=-Infinity,zmin=Infinity,zmax=-Infinity;
      const corners=[];
      for(const x of [-1,1])for(const y of [-1,1])for(const z of [-1,1]){
        const v=rotate(worldQ,[x*g.size[0],y*g.size[1],z*g.size[2]]);
        const w=[center[0]+v[0],center[1]+v[1],center[2]+v[2]];corners.push(w);
        xmin=Math.min(xmin,w[0]);xmax=Math.max(xmax,w[0]);
        ymin=Math.min(ymin,w[1]);ymax=Math.max(ymax,w[1]);
        zmin=Math.min(zmin,w[2]);zmax=Math.max(zmax,w[2]);
      }
      // Convex hull in XY via monotone chain.
      const pts=corners.map(v=>[v[0],v[1]]).sort((a,b)=>a[0]-b[0]||a[1]-b[1]);
      const cross=(o,a,b)=>(a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0]);
      const lower=[],upper=[];
      for(const point of pts){while(lower.length>=2 && cross(lower[lower.length-2],lower[lower.length-1],point)<=0)lower.pop();lower.push(point);}
      for(let i=pts.length-1;i>=0;i--){const point=pts[i];while(upper.length>=2&&cross(upper[upper.length-2],upper[upper.length-1],point)<=0)upper.pop();upper.push(point);}
      lower.pop();upper.pop();
      boxes.push({poly:lower.concat(upper),zmin,zmax,bounds:[xmin,ymin,xmax,ymax],atomId:a.id});
    }
  }
  return boxes;
}
