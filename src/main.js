import {furnitureSettings,addFurnitureToXML,bindFurniture,acceptFurnitureMessage,resetFurniture,stepFurniture,updateFurnitureVisuals,getFurnitureReport,getFurnitureBoxes} from './vam_furniture_web.js';
import {targetSettings,resetHMDTarget,stepHMDTarget,toggleHMDTarget} from './vam_target_web.js';
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';
import {STLLoader} from 'three/addons/loaders/STLLoader.js';
import GUI from 'lil-gui';
import * as ort from 'onnxruntime-web';
import loadMujoco from '@mujoco/mujoco';
import {makeRawPosePacket} from './vam_raw_pose_ws.js';
import {vrSettings,vrBodies,acceptVRMessage,addVRBodiesToXML,bindVRBodies,resetVR,stepVR,updateVRVisuals,getVRFacingPlayerPosition,getVRHMDPose,setLeftDoubleTriggerHandler} from './vam_vr_web.js';

let mujoco, model, data, onnxSession, humanoidData = {};
let frameCount=0, running=false, lastFpsTime=0, simulationSeconds=0;
let lastObsDiagnostics=null;
let targetXY={x:0,y:0};
let playerDirection={angleDeg:0,distance:1,localX:0,localY:0};
let debugTune={actionScale:1,actionRate:0,smoothingMs:0,policySteps:8,kp:1,kd:1,force:1,inferenceMs:0,
  policyHz:30,realPolicyHz:0,actionDelta:0,clipCount:0,targetError:0,rootZ:0,rootSpeed:0};
// Add temporally correlated exploration noise to the 29 ONNX action targets.
// Noise is applied AFTER actionScale and BEFORE action-range clipping.
const actionNoise={enabled:true,stdDev:0.025,correlationMs:180};
const actionNoiseState=new Float32Array(29);
let spareGaussian=null;
function sampleGaussian(){
  if(spareGaussian!==null){const value=spareGaussian;spareGaussian=null;return value;}
  const u=Math.max(Number.MIN_VALUE,Math.random());
  const v=Math.random();
  const magnitude=Math.sqrt(-2*Math.log(u));
  spareGaussian=magnitude*Math.sin(2*Math.PI*v);
  return magnitude*Math.cos(2*Math.PI*v);
}
function advanceActionNoise(){
  const sigma=actionNoise.enabled?actionNoise.stdDev:0;
  if(sigma<=0){actionNoiseState.fill(0);return;}
  // Exact discrete OU update: steady-state standard deviation = stdDev.
  const stepSeconds=DT*Math.max(1,Number(debugTune.policySteps)||8);
  const tau=Math.max(0,actionNoise.correlationMs)/1000;
  const rho=tau>0?Math.exp(-stepSeconds/tau):0;
  const innovation=sigma*Math.sqrt(Math.max(0,1-rho*rho));
  for(let i=0;i<29;i++)actionNoiseState[i]=rho*actionNoiseState[i]+innovation*sampleGaussian();
}
let commandedAction=null,drivenAction=null,currentAction=null;
let policyEnabled=true,drivesEnabled=true,gravityEnabled=true,timeScale=0.5;
const SIM_HZ=240, DT=1/SIM_HZ, numSubsteps=8;
let dofMap=[], bodyMap=[], pdParams=[], servoMap=[], jointGeomMeshes=[];
let renderer,scene,camera,controls,worldGroup,gui;
let visualMeshGroups=[], collisionMeshGroups=[], visualMeshStatus={loaded:0,total:0,ready:false};
let visualSettings={showSTL:false,showColliders:true};
function setStatus(s){document.getElementById('loading').textContent=s;}

// VaM streaming is optional and does not block the physics or ONNX loop.
const vamStream={enabled:false,url:'ws://127.0.0.1:8765',rateHz:30,sent:0,status:'Disconnected'};
let vamSocket=null,vamNextSend=0,vamLastClock=-1;
function disconnectVaM(){
  const sock=vamSocket;vamSocket=null;
  if(sock){sock.onclose=null;sock.onerror=null;sock.close();}
  vamStream.status='Disconnected';
}
function connectVaM(){
  disconnectVaM();
  if(!vamStream.enabled)return;
  try{
    const sock=new WebSocket(vamStream.url);sock.binaryType='arraybuffer';vamSocket=sock;
    sock.onmessage=ev=>{if(vamSocket===sock && ev.data instanceof ArrayBuffer){if(!acceptFurnitureMessage(ev.data))acceptVRMessage(ev.data);}};
    vamStream.status='Connecting';
    sock.onopen=()=>{if(vamSocket!==sock)return;vamStream.status='Connected';vamNextSend=simulationSeconds;};
    sock.onerror=()=>{if(vamSocket===sock)vamStream.status='WebSocket error';};
    sock.onclose=()=>{if(vamSocket===sock){vamStream.status='Disconnected';vamSocket=null;}};
  }catch(e){console.warn('[VaM WebSocket]',e);vamStream.status='Connection error';}
}
function sendVaMIfDue(){
  if(!vamStream.enabled||!vamSocket||vamSocket.readyState!==WebSocket.OPEN)return;
  const interval=1/Math.max(1,vamStream.rateHz);
  if(simulationSeconds<vamLastClock)vamNextSend=simulationSeconds;
  vamLastClock=simulationSeconds;
  if(simulationSeconds+1e-8<vamNextSend)return;
  vamNextSend=simulationSeconds+interval;
  if(vamSocket.bufferedAmount>2048)return; // Drop instead of queueing stale poses.
  try{
    const packet=makeRawPosePacket(model,data,bodyMap,dofMap);
    vamSocket.send(packet);vamStream.sent++;
  }
  catch(e){console.error('[VaM retarget]',e);vamStream.enabled=false;disconnectVaM();}
}


function _normalize(v) {
    const n = Math.sqrt(v.reduce((s, x) => s + x * x, 0));
    return n < 1e-12 ? v : v.map(x => x / n);
}

function _cross(a, b) {
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
}

function _dot(a, b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }

function _getRotationQuat(from, to) {
    const u = _normalize(from), v = _normalize(to);
    const d = _dot(u, v);
    if (d > 1 - 1e-6) return [0, 0, 0, 1]; // identity xyzw
    if (d < 1e-6 - 1) {
        let axis = _cross([1,0,0], u);
        if (_dot(axis, axis) < 1e-6) axis = _cross([0,1,0], u);
        axis = _normalize(axis);
        return [axis[0], axis[1], axis[2], 0]; // 180 deg
    }
    const c = _cross(u, v);
    const s = Math.sqrt((1 + d) * 2), inv = 1 / s;
    const q = [c[0]*inv, c[1]*inv, c[2]*inv, 0.5*s];
    const qn = Math.sqrt(q.reduce((s, x) => s + x * x, 0));
    return q.map(x => x / qn);
}

function _quatRotate(q, v) {
    const qv = [q[0],q[1],q[2]], qw = q[3];
    const t = [2*(qv[1]*v[2]-qv[2]*v[1]), 2*(qv[2]*v[0]-qv[0]*v[2]), 2*(qv[0]*v[1]-qv[1]*v[0])];
    return [v[0]+qw*t[0]+qv[1]*t[2]-qv[2]*t[1],
            v[1]+qw*t[1]+qv[2]*t[0]-qv[0]*t[2],
            v[2]+qw*t[2]+qv[0]*t[1]-qv[1]*t[0]];
}

function _mat33ToQuat(cols) {
    const [m00,m10,m20] = cols[0], [m01,m11,m21] = cols[1], [m02,m12,m22] = cols[2];
    const tr = m00 + m11 + m22;
    let x, y, z, w;
    if (tr >= 0) {
        const h = Math.sqrt(tr + 1); w = 0.5*h; const f = 0.5/h;
        x = (m21-m12)*f; y = (m02-m20)*f; z = (m10-m01)*f;
    } else {
        let i = 0;
        if (m11 > m00) i = 1;
        if (m22 > [m00,m11,m22][i]) i = 2;
        if (i === 0) {
            const h = Math.sqrt(m00-m11-m22+1); x = 0.5*h; const f = 0.5/h;
            y = (m01+m10)*f; z = (m20+m02)*f; w = (m21-m12)*f;
        } else if (i === 1) {
            const h = Math.sqrt(m11-m22-m00+1); y = 0.5*h; const f = 0.5/h;
            z = (m12+m21)*f; x = (m01+m10)*f; w = (m02-m20)*f;
        } else {
            const h = Math.sqrt(m22-m00-m11+1); z = 0.5*h; const f = 0.5/h;
            x = (m20+m02)*f; y = (m12+m21)*f; w = (m10-m01)*f;
        }
    }
    const q = [x,y,z,w], n = Math.sqrt(q.reduce((s,v)=>s+v*v,0));
    return q.map(v => v/n);
}

function _computeJointFrame(jointAxes) {
    const axisMap = [0, 1, 2];
    const n = jointAxes.length;
    if (n === 0) return { q: [0,0,0,1], axisMap };
    if (n === 1) return { q: _getRotationQuat([1,0,0], jointAxes[0]), axisMap };

    const Q = _getRotationQuat(jointAxes[0], [1,0,0]);
    const b = _normalize(_quatRotate(Q, jointAxes[1]));

    if (n === 2) {
        if (Math.abs(_dot(b,[0,1,0])) > Math.abs(_dot(b,[0,0,1]))) {
            axisMap[1] = 1;
            const c = _normalize(_cross(jointAxes[0], jointAxes[1]));
            return { q: _mat33ToQuat([_normalize(jointAxes[0]), _normalize(jointAxes[1]), c]), axisMap };
        } else {
            axisMap[1] = 2; axisMap[2] = 1;
            const c = _normalize(_cross(jointAxes[1], jointAxes[0]));
            return { q: _mat33ToQuat([_normalize(jointAxes[0]), c, _normalize(jointAxes[1])]), axisMap };
        }
    }
    // n === 3
    if (Math.abs(_dot(b,[0,1,0])) > Math.abs(_dot(b,[0,0,1]))) {
        axisMap[1] = 1; axisMap[2] = 2;
        return { q: _mat33ToQuat([_normalize(jointAxes[0]), _normalize(jointAxes[1]), _normalize(jointAxes[2])]), axisMap };
    } else {
        axisMap[1] = 2; axisMap[2] = 1;
        return { q: _mat33ToQuat([_normalize(jointAxes[0]), _normalize(jointAxes[2]), _normalize(jointAxes[1])]), axisMap };
    }
}

// ── MJCF Parser ─────────────────────────────────────────────────────────────

function _parseVec(s, n) {
    if (!s) return null;
    const parts = s.trim().split(/\s+/).map(Number);
    return n ? parts.slice(0, n) : parts;
}

function parseMJCF(xmlText, opts = {}) {
    const parser = new DOMParser();
    const doc = parser.parseFromString(xmlText, 'text/xml');
    const root = doc.documentElement;

    let fixedBodies = opts.fixedBodies;
    if (!fixedBodies) {
        fixedBodies = new Set();
        const scan = (el, isRoot) => {
            if (!isRoot) {
                const hinges = [...el.querySelectorAll(':scope > joint')].filter(
                    j => (j.getAttribute('type') || 'hinge') !== 'free');
                const freeJ = el.querySelector(':scope > freejoint');
                if (!hinges.length && !freeJ) fixedBodies.add(el.getAttribute('name'));
            }
            for (const child of el.querySelectorAll(':scope > body')) scan(child, false);
        };
        for (const b of root.querySelector('worldbody').querySelectorAll(':scope > body'))
            scan(b, true);
    }

    const actuatorMap = {};
    const actuatorOrder = [];
    const actuatorSec = root.querySelector('actuator');
    if (actuatorSec) {
        for (const mot of actuatorSec.querySelectorAll('motor')) {
            const jname = mot.getAttribute('joint');
            if (!jname) continue;
            const gear = parseFloat(mot.getAttribute('gear') || '1');
            const frc = _parseVec(mot.getAttribute('actuatorfrcrange'), 2);
            const maxForce = frc ? Math.max(Math.abs(frc[0]), Math.abs(frc[1])) : gear;
            actuatorMap[jname] = { gear, maxForce };
            actuatorOrder.push(jname);
        }
    }

    const bodies = [], joints = [], fixedJoints = [];

    function processBody(el, parentName, parentWorldPos) {
        const name = el.getAttribute('name');
        const localPos = _parseVec(el.getAttribute('pos'), 3) || [0,0,0];
        const worldPos = localPos.map((v, i) => parentWorldPos[i] + v);

        const geoms = [];
        for (const ge of el.querySelectorAll(':scope > geom')) {
            const g = { name: ge.getAttribute('name') || name, type: ge.getAttribute('type') || 'sphere' };
            g.pos = _parseVec(ge.getAttribute('pos'), 3) || [0,0,0];
            if (ge.getAttribute('size')) g.size = _parseVec(ge.getAttribute('size'));
            if (g.type === 'sphere') g.radius = g.size[0];
            else if (g.type === 'capsule') {
                g.radius = g.size[0];
                if (ge.getAttribute('fromto')) g.fromto = _parseVec(ge.getAttribute('fromto'), 6);
            } else if (g.type === 'box') {
                g.halfExtents = g.size.slice(0, 3);
            } else if (g.type === 'cylinder') {
                g.radius = g.size[0];
                if (g.size.length > 1) g.halfHeight = g.size[1];
                if (ge.getAttribute('fromto')) {
                    g.fromto = _parseVec(ge.getAttribute('fromto'), 6);
                    const ft = g.fromto;
                    g.halfHeight = Math.sqrt((ft[3]-ft[0])**2+(ft[4]-ft[1])**2+(ft[5]-ft[2])**2) / 2;
                }
                if (!g.halfHeight) g.halfHeight = 0.1;
            }
            g.density = parseFloat(ge.getAttribute('density') || '1000');
            geoms.push(g);
        }

        const PI = Math.PI;

        function geomMassAndCenter(g) {
            let volume = 0, center = g.pos.slice();
            if (g.type === 'sphere') {
                volume = (4/3) * PI * g.radius ** 3;
            } else if (g.type === 'capsule' && g.fromto) {
                const ft = g.fromto, r = g.radius;
                const halfH = Math.sqrt((ft[3]-ft[0])**2+(ft[4]-ft[1])**2+(ft[5]-ft[2])**2) / 2;
                volume = PI * r*r * (2*halfH) + (4/3) * PI * r**3;
                center = [(ft[0]+ft[3])/2, (ft[1]+ft[4])/2, (ft[2]+ft[5])/2];
            } else if (g.type === 'capsule') {
                const r = g.radius, hh = g.size.length > 1 ? g.size[1] : 0.1;
                volume = PI * r*r * (2*hh) + (4/3) * PI * r**3;
            } else if (g.type === 'box') {
                const he = g.halfExtents || g.size.slice(0,3);
                volume = 8 * he[0] * he[1] * he[2];
            } else if (g.type === 'cylinder') {
                const r = g.radius, hh = g.halfHeight || 0.1;
                volume = PI * r*r * (2*hh);
                if (g.fromto) {
                    const ft = g.fromto;
                    center = [(ft[0]+ft[3])/2, (ft[1]+ft[4])/2, (ft[2]+ft[5])/2];
                }
            }
            return { mass: volume * g.density, center, volume };
        }

        function capsuleInertia(g) {
            const ft = g.fromto, r = g.radius;
            const dx = ft[3]-ft[0], dy = ft[4]-ft[1], dz = ft[5]-ft[2];
            const L = Math.sqrt(dx*dx+dy*dy+dz*dz);
            const halfH = L / 2;
            const cylVol = PI * r*r * L;
            const cylM = cylVol * g.density;
            const sphVol = (4/3) * PI * r**3;
            const sphM = sphVol * g.density;
            const cylIaxial = cylM * r*r / 2;
            const cylIperp = cylM * (3*r*r + L*L) / 12;
            const sphIaxial = 2 * sphM * r*r / 5;
            const sphIperp = sphIaxial + sphM * (3*r/8 + halfH)**2;
            const Iaxial = cylIaxial + sphIaxial;
            const Iperp = cylIperp + sphIperp;
            if (L < 1e-10) return [Iperp, Iperp, Iperp];
            const ax = [dx/L, dy/L, dz/L];
            return [
                Iperp + (Iaxial - Iperp) * ax[0]*ax[0],
                Iperp + (Iaxial - Iperp) * ax[1]*ax[1],
                Iperp + (Iaxial - Iperp) * ax[2]*ax[2],
            ];
        }

        let totalMass = 0;
        let com = [0, 0, 0];
        const geomData = geoms.map(g => {
            const { mass, center } = geomMassAndCenter(g);
            return { g, mass, center };
        });
        for (const { mass, center } of geomData) {
            com[0] += mass * center[0];
            com[1] += mass * center[1];
            com[2] += mass * center[2];
            totalMass += mass;
        }
        if (totalMass > 0) com = com.map(c => c / totalMass);

        let inertia = [0, 0, 0];
        for (const { g, mass, center } of geomData) {
            let Ii;
            if (g.type === 'sphere') {
                const I = (2/5) * mass * g.radius**2;
                Ii = [I, I, I];
            } else if (g.type === 'capsule' && g.fromto) {
                Ii = capsuleInertia(g);
            } else if (g.type === 'box') {
                const he = g.halfExtents || g.size.slice(0,3);
                const a=2*he[0], b=2*he[1], c=2*he[2];
                Ii = [mass*(b*b+c*c)/12, mass*(a*a+c*c)/12, mass*(a*a+b*b)/12];
            } else if (g.type === 'cylinder') {
                const r = g.radius, hh = g.halfHeight || 0.015;
                const Iaxial = mass * r*r / 2;
                const Iperp = mass * (3*r*r + (2*hh)**2) / 12;
                if (g.fromto) {
                    const ft = g.fromto, L = 2*hh;
                    const ddx = ft[3]-ft[0], ddy = ft[4]-ft[1], ddz = ft[5]-ft[2];
                    if (L > 1e-10) {
                        const ax = [ddx/L, ddy/L, ddz/L];
                        Ii = [Iperp + (Iaxial-Iperp)*ax[0]*ax[0],
                              Iperp + (Iaxial-Iperp)*ax[1]*ax[1],
                              Iperp + (Iaxial-Iperp)*ax[2]*ax[2]];
                    } else { Ii = [Iperp, Iperp, Iaxial]; }
                } else { Ii = [Iperp, Iperp, Iaxial]; }
            } else {
                Ii = [0, 0, 0];
            }
            const dx = center[0]-com[0], dy = center[1]-com[1], dz = center[2]-com[2];
            inertia[0] += Ii[0] + mass*(dy*dy+dz*dz);
            inertia[1] += Ii[1] + mass*(dx*dx+dz*dz);
            inertia[2] += Ii[2] + mass*(dx*dx+dy*dy);
        }

        // Prefer the exact MJCF inertial properties over geometry-derived estimates.
        // MJCF <inertial quat> is wxyz and defines the principal-inertia frame
        // relative to the body frame. PhysX PxTransform/PxQuat uses xyzw.
        const inertialEl = el.querySelector(':scope > inertial');
        let inertialQuat = [0, 0, 0, 1]; // xyzw
        let hasExplicitInertial = false;
        if (inertialEl) {
            const specifiedMass = Number(inertialEl.getAttribute('mass'));
            const specifiedCom = _parseVec(inertialEl.getAttribute('pos'), 3) || [0, 0, 0];
            const diagInertia = _parseVec(inertialEl.getAttribute('diaginertia'), 3);
            const quatWxyz = _parseVec(inertialEl.getAttribute('quat'), 4) || [1, 0, 0, 0];
            if (!Number.isFinite(specifiedMass) || specifiedMass <= 0 ||
                !diagInertia || diagInertia.length !== 3 ||
                !diagInertia.every(v => Number.isFinite(v) && v > 0) ||
                !specifiedCom.every(Number.isFinite) || !quatWxyz.every(Number.isFinite)) {
                throw new Error('Invalid MJCF <inertial> parameters for body ' + name);
            }
            const qlen = Math.hypot(...quatWxyz);
            if (qlen < 1e-12) throw new Error('Invalid MJCF inertial quaternion for body ' + name);
            totalMass = specifiedMass;
            com = specifiedCom;
            inertia = diagInertia;
            inertialQuat = [quatWxyz[1]/qlen, quatWxyz[2]/qlen,
                            quatWxyz[3]/qlen, quatWxyz[0]/qlen];
            hasExplicitInertial = true;
        }

        const bodyQuat = _parseVec(el.getAttribute('quat'), 4);
        const bodyLocalRot = bodyQuat ? [bodyQuat[1], bodyQuat[2], bodyQuat[3], bodyQuat[0]] : [0,0,0,1];
        bodies.push({ name, parent: parentName, pos: worldPos, localPos, localRot: bodyLocalRot, geoms,
                       mass: totalMass, inertia, com, inertialQuat, hasExplicitInertial });

        const jointEls = [...el.querySelectorAll(':scope > joint')].filter(
            j => (j.getAttribute('type') || 'hinge') !== 'free' && j.tagName !== 'freejoint');

        if (fixedBodies.has(name)) {
            fixedJoints.push({ name: name+'_fixed', parent_body: parentName, child_body: name, localPos0: localPos });
        } else if (jointEls.length && parentName !== null) {
            const axesData = [], jointAxes = [];
            for (const je of jointEls) {
                const axis = _parseVec(je.getAttribute('axis') || '1 0 0', 3);
                jointAxes.push(axis);
                const rng = _parseVec(je.getAttribute('range') || '-3.14159 3.14159', 2);
                const jname = je.getAttribute('name');
                const act = actuatorMap[jname] || { gear: 100, maxForce: 100 };
                axesData.push({
                    name: jname, mjcf_axis: axis,
                    stiffness: parseFloat(je.getAttribute('stiffness') || '0'),
                    damping: parseFloat(je.getAttribute('damping') || '0'),
                    maxForce: act.maxForce,
                    range: rng,
                    armature: parseFloat(je.getAttribute('armature') || '0'),
                });
            }

            const { q, axisMap } = _computeJointFrame(jointAxes);
            const localRot = [q[3], q[0], q[1], q[2]];

            joints.push({
                name: jointEls.length > 1 ? jointEls[0].getAttribute('name').replace(/_[^_]+$/, '') : jointEls[0].getAttribute('name'),
                parent_body: parentName, child_body: name,
                axes: axesData,
                axisMap: axisMap.slice(0, jointEls.length),
                localPos0: localPos,
                localRot,
                jointType: jointEls.length > 1 ? 'spherical' : 'revolute',
            });
        }

        for (const child of el.querySelectorAll(':scope > body'))
            processBody(child, name, worldPos);
    }

    const worldbody = root.querySelector('worldbody');
    const pelvis = worldbody.querySelector(':scope > body');
    const pelvisPos = _parseVec(pelvis.getAttribute('pos'), 3) || [0,0,0];
    processBody(pelvis, null, [0,0,0]);

    const dofInfo = [];
    for (const actName of actuatorOrder) {
        for (const jdata of joints) {
            for (let ai = 0; ai < jdata.axes.length; ai++) {
                if (jdata.axes[ai].name === actName) {
                    dofInfo.push({
                        joint_name: jdata.name, axis_name: actName,
                        physx_axis: jdata.axisMap[ai], child_body: jdata.child_body,
                    });
                    break;
                }
            }
        }
    }

    const bodyNames = bodies.map(b => b.name);
    const fk_parent_indices = bodies.map(b => b.parent === null ? -1 : bodyNames.indexOf(b.parent));
    const fk_local_translations = bodies.map(b => b.localPos);
    // MJCF body quaternions are wxyz; kinematic solver expects xyzw.
    const fk_local_rotations = bodies.map(b => b.localRot || [0, 0, 0, 1]);

    const kinematicJoints = [];
    let dofIdx = 0;
    for (let bi = 1; bi < bodies.length; bi++) {
        const bname = bodies[bi].name;
        const jdata = joints.find(j => j.child_body === bname);
        const fjdata = fixedJoints.find(j => j.child_body === bname);
        if (jdata) {
            const nDofs = jdata.axes.length;
            kinematicJoints.push({
                name: jdata.name, child_body: bname,
                type: nDofs > 1 ? 'SPHERICAL' : 'HINGE',
                dof_idx: dofIdx, dof_dim: nDofs,
                axis: nDofs === 1 ? jdata.axes[0].mjcf_axis : undefined,
            });
            dofIdx += nDofs;
        } else if (fjdata) {
            kinematicJoints.push({ name: fjdata.name, child_body: bname, type: 'FIXED', dof_idx: dofIdx, dof_dim: 0 });
        } else {
            kinematicJoints.push({ name: bname, child_body: bname, type: 'FIXED', dof_idx: dofIdx, dof_dim: 0 });
        }
    }

    return {
        bodies, joints, fixedJoints, actuatorOrder, dofInfo,
        fk_parent_indices, fk_local_translations, fk_local_rotations,
        kinematicJoints,
        act_dim: actuatorOrder.length,
    };
}

function extractOnnxMetadata(buffer) {
    const bytes = new Uint8Array(buffer);
    // Search for the sentinel key as UTF-8 bytes
    const sentinel = 'mimickit_config';
    const sentinelBytes = new TextEncoder().encode(sentinel);

    for (let i = 0; i < bytes.length - sentinelBytes.length - 10; i++) {
        let match = true;
        for (let j = 0; j < sentinelBytes.length; j++) {
            if (bytes[i + j] !== sentinelBytes[j]) { match = false; break; }
        }
        if (!match) continue;

        // Found the key. In protobuf, StringStringEntryProto has:
        //   field 1 (key): tag=0x0a, then varint length, then string
        //   field 2 (value): tag=0x12, then varint length, then string
        // Search forward from the key for the value field tag (0x12)
        const searchStart = i + sentinelBytes.length;
        for (let k = searchStart; k < Math.min(searchStart + 20, bytes.length); k++) {
            if (bytes[k] === 0x12) {
                // Read varint length
                let len = 0, shift = 0, pos = k + 1;
                while (pos < bytes.length) {
                    const b = bytes[pos++];
                    len |= (b & 0x7f) << shift;
                    shift += 7;
                    if ((b & 0x80) === 0) break;
                }
                // Extract the JSON string
                if (len > 0 && pos + len <= bytes.length) {
                    const jsonStr = new TextDecoder().decode(bytes.slice(pos, pos + len));
                    try {
                        const config = JSON.parse(jsonStr);
                        console.log('Extracted ONNX metadata (' + Object.keys(config).length + ' keys, ' +
                            Math.round(len/1024) + 'KB)');
                        return config;
                    } catch(e) {
                        console.log('Found sentinel but JSON parse failed:', e.message);
                    }
                }
                break;
            }
        }
    }
    console.log('No mimickit_config found in ONNX binary');
    return null;
}


function quatRotateVec(q, v) {
    const [qx, qy, qz, qw] = q;
    const [vx, vy, vz] = v;
    // t = 2 * cross(q_xyz, v)
    const tx = 2 * (qy*vz - qz*vy);
    const ty = 2 * (qz*vx - qx*vz);
    const tz = 2 * (qx*vy - qy*vx);
    return [
        vx + qw*tx + (qy*tz - qz*ty),
        vy + qw*ty + (qz*tx - qx*tz),
        vz + qw*tz + (qx*ty - qy*tx)
    ];
}

// Quaternion multiply a * b, both [x,y,z,w]
function quatMul(a, b) {
    const [ax,ay,az,aw] = a, [bx,by,bz,bw] = b;
    return [
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz
    ];
}

// Normalize quaternion
function quatNorm(q) {
    const [x,y,z,w] = q;
    const len = Math.sqrt(x*x + y*y + z*z + w*w) || 1;
    return [x/len, y/len, z/len, w/len];
}

// Axis-angle to quaternion [x,y,z,w] (matches torch_util.axis_angle_to_quat)
function axisAngleToQuat(axis, angle) {
    const ha = angle * 0.5;
    const s = Math.sin(ha), c = Math.cos(ha);
    const len = Math.sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]) || 1;
    return quatNorm([axis[0]/len*s, axis[1]/len*s, axis[2]/len*s, c]);
}

// Exponential map (3D) to quaternion [x,y,z,w] (matches torch_util.exp_map_to_quat)
function expMapToQuat(ex, ey, ez) {
    const angle = Math.sqrt(ex*ex + ey*ey + ez*ez);
    if (angle < 1e-5) return [0, 0, 0, 1];
    return axisAngleToQuat([ex/angle, ey/angle, ez/angle], angle);
}

// Compute heading angle from quaternion (rotation of X-axis projected to XY plane in Z-up)
// Matches torch_util.calc_heading
function calcHeading(q) {
    const rotDir = quatRotateVec(q, [1, 0, 0]);
    return Math.atan2(rotDir[1], rotDir[0]);
}

// Compute inverse heading quaternion (rotation around Z axis by -heading)
// Matches torch_util.calc_heading_quat_inv
function calcHeadingQuatInv(q) {
    const heading = calcHeading(q);
    return axisAngleToQuat([0, 0, 1], -heading);
}

// Convert quaternion to 6D tan_norm representation.
// tan = quat_rotate(q, [1,0,0])  (rotated X axis)
// norm = quat_rotate(q, [0,0,1]) (rotated Z axis)
// result = [tan(3), norm(3)] = 6 values
// Matches torch_util.quat_to_tan_norm
function quatToTanNorm(q) {
    const tan = quatRotateVec(q, [1, 0, 0]);
    const norm = quatRotateVec(q, [0, 0, 1]);
    return [tan[0], tan[1], tan[2], norm[0], norm[1], norm[2]];
}



async function verifyOrtWasmAsset() {
 // Vite returns index.html for unknown paths. That HTML begins with '<!do',
 // leading to the opaque WebAssembly 'expected magic word' error.
 const url = `${import.meta.env.BASE_URL}ort/ort-wasm-simd-threaded.wasm`;
 const resp = await fetch(url, { cache: 'no-store' });
 if (!resp.ok) throw Error(`ONNX Runtime WASM not found: ${url} (HTTP ${resp.status}). Run npm run dev again.`);
 const magic = new Uint8Array(await resp.arrayBuffer()).subarray(0, 4);
 if (magic.length !== 4 || magic[0] !== 0 || magic[1] !== 0x61 || magic[2] !== 0x73 || magic[3] !== 0x6d) {
   throw Error(`ONNX Runtime WASM URL returned HTML instead of WASM: ${url}. Run npm install and npm run dev to copy assets.`);
 }
 console.log('[ORT] Local WASM verified:', url);
 // ORT 1.21 may dynamically import the JSEP ES module despite WASM-only EP.
 // Detect an absent/HTML-fallback module before InferenceSession.create().
 const jsepUrl = `${import.meta.env.BASE_URL}ort/ort-wasm-simd-threaded.jsep.mjs`;
 const jsepResp = await fetch(jsepUrl, { cache: 'no-store' });
 if (!jsepResp.ok) throw Error(`ONNX Runtime JSEP module missing: ${jsepUrl} (HTTP ${jsepResp.status}). Restart npm run dev to copy assets.`);
 const jsepHead = (await jsepResp.text()).slice(0, 256).trimStart();
 if (/^(?:<!doctype|<html)/i.test(jsepHead)) throw Error(`ONNX Runtime JSEP URL returned HTML: ${jsepUrl}`);
 console.log('[ORT] Local JSEP module verified:', jsepUrl);
}

async function loadOnnxModel() {
 setStatus('Loading existing amp_actor.onnx ...');
 // Keep the .mjs and .wasm assets from the exact same npm package build.
 ort.env.wasm.numThreads = 1;
 ort.env.wasm.wasmPaths = `${import.meta.env.BASE_URL}ort/`;
 await verifyOrtWasmAsset();
 const res=await fetch(`${import.meta.env.BASE_URL}amp_actor.onnx`);
 if(!res.ok)throw Error('Please place existing amp_actor.onnx into public/ (HTTP '+res.status+')');
 const buf=await res.arrayBuffer();
 const meta=extractOnnxMetadata(buf);
 if(!meta || !meta.mjcf_xml)throw Error('ONNX mimickit_config MJCF metadata missing');
 onnxSession=await ort.InferenceSession.create(buf,{executionProviders:['wasm']});
 if(onnxSession.inputNames.join(',')!=='obs' || onnxSession.outputNames.indexOf('action')<0)
   throw Error('Expected ONNX input=obs and output=action');
 humanoidData={...meta,...parseMJCF(meta.mjcf_xml)};
 if(humanoidData.obs_dim!==241 || humanoidData.act_dim!==29 || humanoidData.dofInfo.length!==29 || humanoidData.global_obs!==false)
   throw Error('Model must be AMP 241D / 29D global_obs=false');
 if(!Array.isArray(humanoidData.key_body_ids)||humanoidData.key_body_ids.length!==5)
   throw Error('Missing 5 key_body_ids in ONNX metadata');
 console.log('[MuJoCo] Existing ONNX unchanged',humanoidData.obs_dim,humanoidData.act_dim);
}


function mustID(kind,name){const id=mujoco.mj_name2id(model,mujoco.mjtObj[kind].value,name);if(id<0)throw Error('Missing MuJoCo '+kind+': '+name);return id;}
function getState(){
 const rootPos=Array.from(data.qpos.slice(0,3));
 const rootRot=[data.qpos[4],data.qpos[5],data.qpos[6],data.qpos[3]];
 const rootVel=Array.from(data.qvel.slice(0,3));
 // MuJoCo free-joint angular qvel is root-body-local; convert to world before heading removal.
 const rootAngVel=quatRotateVec(rootRot,Array.from(data.qvel.slice(3,6)));
 const dofPos=dofMap.map(d=>data.qpos[d.qpos]);
 const dofVel=dofMap.map(d=>data.qvel[d.qvel]);
 const jointRot=[];
 for(const j of humanoidData.kinematicJoints){
   if(j.type==='HINGE')jointRot.push(axisAngleToQuat(j.axis,dofPos[j.dof_idx]));
   else if(j.type==='SPHERICAL')jointRot.push(expMapToQuat(...dofPos.slice(j.dof_idx,j.dof_idx+3)));
   else jointRot.push([0,0,0,1]);
 }
 const keyRel=humanoidData.key_body_ids.map(i=>{
    const id=bodyMap[i];const p=data.xpos;
    return [p[3*id]-rootPos[0],p[3*id+1]-rootPos[1],p[3*id+2]-rootPos[2]];
 });
 return {rootPos,rootRot,rootVel,rootAngVel,dofPos,dofVel,jointRot,keyRel};
}

function buildObservation() {
    const cur = getState();
    const obs = [];
    const push = a => { for (const v of a) obs.push(v); };

    // MimicKit compute_char_obs(global_obs=false, root_height_obs=true).
    // Heading removal is applied to root rotation, root velocities and key body vectors.
    const headingInv = calcHeadingQuatInv(cur.rootRot);
    const localRootRot = quatMul(headingInv, cur.rootRot);
    push([cur.rootPos[2]]);
    push(quatToTanNorm(localRootRot));
    push(quatRotateVec(headingInv, cur.rootVel));
    push(quatRotateVec(headingInv, cur.rootAngVel));
    // Joint-local rotations are NOT modified by global_obs.
    for (const q of cur.jointRot) push(quatToTanNorm(q));
    push(cur.dofVel);
    for (const p of cur.keyRel) push(quatRotateVec(headingInv, p));
    if (obs.length !== 237) throw new Error('Character observation size ' + obs.length + ' != 237');

    // MimicKit TaskLocationEnv: [heading-local target XY, heading-local normalized player direction XY].
    if(targetSettings.mode==='FOLLOW')stepHMDTarget(targetXY,getVRHMDPose(),cur.rootPos,
      targetSettings.avoidFurniture?getFurnitureBoxes(data):[],1/30);
    const targetVec = [targetXY.x - cur.rootPos[0], targetXY.y - cur.rootPos[1], 0];
    const loc = quatRotateVec(headingInv, targetVec);

    // Newton VR mode: live HMD or a controller-selected fixed WORLD point.
    // When tracking is inactive, retain the existing GUI world-angle fallback.
    const playerAngle = playerDirection.angleDeg * Math.PI / 180;
    const playerPos = getVRFacingPlayerPosition() || [
        cur.rootPos[0] + playerDirection.distance * Math.cos(playerAngle),
        cur.rootPos[1] + playerDirection.distance * Math.sin(playerAngle),
        cur.rootPos[2]
    ];
    const playerVec = [playerPos[0] - cur.rootPos[0], playerPos[1] - cur.rootPos[1], 0];
    const localPlayer = quatRotateVec(headingInv, playerVec);
    const playerLength = Math.hypot(localPlayer[0], localPlayer[1]);
    // torch.nn.functional.normalize with eps=1e-6 divides by max(norm, eps).
    const playerDenom = Math.max(playerLength, 1e-6);
    const playerX = localPlayer[0] / playerDenom;
    const playerY = localPlayer[1] / playerDenom;
    push([loc[0], loc[1], playerX, playerY]);
    if (obs.length !== humanoidData.obs_dim) throw new Error(`Observation mismatch ${obs.length} != ${humanoidData.obs_dim}`);
    if (!obs.every(Number.isFinite)) throw new Error('Non-finite observation generated');
    playerDirection.localX = +playerX.toFixed(4);
    playerDirection.localY = +playerY.toFixed(4);
    lastObsDiagnostics = { rootZ: cur.rootPos[2], targetLocal: [loc[0], loc[1]], playerLocal: [playerX, playerY] };
    if (frameCount % 15 === 0) document.getElementById('obsInfo').textContent =
        `Obs ${obs.length} / Action ${humanoidData.act_dim} · target ${loc[0].toFixed(2)}, ${loc[1].toFixed(2)} · player dir ${playerX.toFixed(2)}, ${playerY.toFixed(2)}`;
    return new Float32Array(obs);
}


async function runPolicy(obs) {
    const obsTensor = new ort.Tensor('float32', obs, [1, humanoidData.obs_dim]);
    const results = await onnxSession.run({ obs: obsTensor });
    const a = results.action.data;
    if (a.length !== humanoidData.act_dim) throw new Error('Action size mismatch ' + a.length);
    if (!Array.from(a).every(Number.isFinite)) throw new Error('Non-finite ONNX action');
    return a;
}



async function initMuJoCo(){
 setStatus('Initializing MuJoCo WASM ...');
 // Vite's SPA fallback returns index.html for missing .wasm URLs. Always serve
 // MuJoCo's WASM from public/ and give Emscripten an explicit locateFile path.
 const wasmURL = `${import.meta.env.BASE_URL}mujoco/mujoco.wasm`;
 const wasmResponse = await fetch(wasmURL, {cache:'no-store'});
 if (!wasmResponse.ok) throw Error(`[MuJoCo] WASM HTTP ${wasmResponse.status} at ${wasmURL}`);
 const magic = new Uint8Array(await wasmResponse.arrayBuffer()).slice(0,4);
 if (magic.length !== 4 || magic[0] !== 0 || magic[1] !== 0x61 || magic[2] !== 0x73 || magic[3] !== 0x6d) {
   throw Error(`[MuJoCo] Invalid WASM at ${wasmURL}; received ${Array.from(magic).map(x=>x.toString(16).padStart(2,'0')).join(' ')} (HTML fallback?). Run npm run dev to copy the package WASM.`);
 }
 console.log('[MuJoCo] Local WASM verified:', wasmURL);
 mujoco=await loadMujoco({locateFile: filename => filename.endsWith('.wasm') ? wasmURL : filename});
 const xmlRes=await fetch(`${import.meta.env.BASE_URL}g1_collision.xml`);if(!xmlRes.ok)throw Error('g1_collision.xml missing');
 model=mujoco.MjModel.from_xml_string(addVRBodiesToXML(await addFurnitureToXML(await xmlRes.text(),import.meta.env.BASE_URL)));
 data=new mujoco.MjData(model);
 const gainRes=await fetch(`${import.meta.env.BASE_URL}joint_pd.json`);if(!gainRes.ok)throw Error('joint_pd.json missing');
 const gains=await gainRes.json();
 const bodyNames=humanoidData.bodies.map(b=>b.name);
 bodyMap=bodyNames.map(name=>mustID('mjOBJ_BODY',name));
 dofMap=humanoidData.dofInfo.map((d,i)=>{
  const id=mustID('mjOBJ_JOINT',d.axis_name);
  return {name:d.axis_name,joint:id,qpos:model.jnt_qposadr[id],qvel:model.jnt_dofadr[id],body:d.child_body};
 });
 if(model.nq!==57 || model.nv!==53 || dofMap.length!==29)
  throw Error(`MuJoCo dimension mismatch nq=${model.nq}, nv=${model.nv}, DOF=${dofMap.length} expected 57,53,29 with VR auxiliary bodies`);
 bindVRBodies(mujoco,model);bindFurniture(mujoco,model);
 pdParams=dofMap.map(d=>{const p=gains[d.name];if(!p)throw Error('Missing gains for '+d.name);return p;});
 // Every original torque motor is replaced with one MuJoCo general position servo.
 if(model.nu!==29)throw Error(`Expected 29 Newton-style servos, got ${model.nu}`);
 servoMap=dofMap.map((d,i)=>{
   const id=mustID('mjOBJ_ACTUATOR',d.name+'_servo');
   return {id,name:d.name};
 });
 // Detect the expected MuJoCo parameter layout rather than silently writing to wrong offsets.
 for(const [key,expectedStride] of [['actuator_gainprm',10],['actuator_biasprm',10],['actuator_forcerange',2]]){
   if(!model[key] || model[key].length !== model.nu*expectedStride)
     throw Error(`MuJoCo JS binding mismatch: ${key}, expected ${model.nu*expectedStride}, got ${model[key]?.length}`);
 }
 console.log('[MuJoCo] Newton-style general position servos:',servoMap.length,'; no JS torque PD');
 console.log('[MuJoCo] loaded model',model.nbody,'bodies',model.nq,'nq',model.nv,'nv',model.nu,'nu');
 resetHumanoid();
}
function resetHumanoid(){
 if(!model)return;
 mujoco.mj_resetData(model,data);
 const p=humanoidData.init_root_pos || [0,0,.8], r=humanoidData.init_root_rot_quat||[0,0,0,1];
 data.qpos.set(p,0);data.qpos.set([r[3],r[0],r[1],r[2]],3);
 const init=humanoidData.init_dof_pos || new Array(29).fill(0);
 dofMap.forEach((d,i)=>{data.qpos[d.qpos]=init[i];data.qvel[d.qvel]=0;});
 data.qvel.fill(0);data.qfrc_applied.fill(0);
 // Set the position references before forward dynamics (never reset controls to zero).
 for(let i=0;i<29;i++) data.ctrl[servoMap[i].id]=init[i];
 resetVR(data);resetFurniture(data);resetHMDTarget();
 mujoco.mj_forward(model,data);
 commandedAction=Float32Array.from(init);drivenAction=Float32Array.from(init);
 currentAction=null;actionNoiseState.fill(0);spareGaussian=null;simulationSeconds=0;accum=0;physTick=0;resetGrace=120;
 console.log('[MuJoCo] reset initial root',p,'initial dof',init);
}
function applyActions(action, withNoise=true){
 if(withNoise)advanceActionNoise();else actionNoiseState.fill(0);
 const prev=commandedAction, init=humanoidData.init_dof_pos||new Array(29).fill(0);
 let maxDelta=0,clipped=0;const next=new Float32Array(29);
 for(let i=0;i<29;i++){
   const raw=init[i]+debugTune.actionScale*(action[i]-init[i])+(withNoise?actionNoiseState[i]:0);
   const lo=humanoidData.action_low?.[i] ?? -Infinity;
   const hi=humanoidData.action_high?.[i] ?? Infinity;
   const a=Math.min(hi,Math.max(lo,raw));if(Math.abs(raw-a)>1e-6)clipped++;
   maxDelta=Math.max(maxDelta,Math.abs(a-(prev ? prev[i] : a)));next[i]=a;
 }
 commandedAction=next;debugTune.actionDelta=maxDelta;debugTune.clipCount=clipped;
}
// MuJoCo built-in general actuator: gainprm=[Kp], biasprm=[0,-Kp,-Kd].
// Torque = Kp*(ctrl-qpos) - Kd*qvel, clamped internally by forcerange.
// This replaces qfrc_applied PD; qfrc_applied remains zero.
function applyServoTargets(){
 let maxErr=0;
 const gainStride=model.actuator_gainprm.length/model.nu;
 const biasStride=model.actuator_biasprm.length/model.nu;
 const alpha=debugTune.smoothingMs>0?1-Math.exp(-DT/(debugTune.smoothingMs/1000)):1;
 const maxMove=debugTune.actionRate>0?debugTune.actionRate*DT:Infinity;
 data.qfrc_applied.fill(0);
 for(let i=0;i<29;i++){
   const d=dofMap[i],p=pdParams[i],act=servoMap[i].id;
   drivenAction[i]+=Math.min(maxMove,Math.max(-maxMove,(commandedAction[i]-drivenAction[i])*alpha));
   const err=drivenAction[i]-data.qpos[d.qpos];maxErr=Math.max(maxErr,Math.abs(err));
   const kp=drivesEnabled?p.kp*debugTune.kp:0;
   const kd=drivesEnabled?p.kd*debugTune.kd:0;
   model.actuator_gainprm[act*gainStride]=kp;
   model.actuator_biasprm[act*biasStride+1]=-kp;
   model.actuator_biasprm[act*biasStride+2]=-kd;
   const limit=p.limit*debugTune.force;
   model.actuator_forcerange[act*2]=-limit;
   model.actuator_forcerange[act*2+1]=limit;
   data.ctrl[act]=drivenAction[i];
 }
 debugTune.targetError=maxErr;
}

// The original Newton training MJCF is used only as a VISUAL manifest.
// Physics continues to load g1_collision.xml unchanged. STL vertices are
// interpreted in MJCF's Z-up body-local coordinates (the group already follows
// the corresponding MuJoCo body's xpos/xquat).
async function loadG1VisualSTLs(){
 const url=`${import.meta.env.BASE_URL}g1.xml`;
 const xmlResponse=await fetch(url);
 if(!xmlResponse.ok)throw Error(`[Visual] Cannot fetch ${url}: ${xmlResponse.status}`);
 const doc=new DOMParser().parseFromString(await xmlResponse.text(),'application/xml');
 if(doc.querySelector('parsererror')) throw Error('[Visual] g1.xml parser error');
 const assets=new Map();
 for(const m of doc.querySelectorAll('asset > mesh')){
   assets.set(m.getAttribute('name'),m.getAttribute('file'));
 }
 const groupByBody=new Map();
 for(const entry of jointGeomMeshes) {
   const name=humanoidData.bodies.find((b,i)=>bodyMap[i]===entry.id)?.name;
   if(name)groupByBody.set(name,entry.group);
 }
 const loader=new STLLoader();
 const materialCache=new Map();
 const entries=[];
 for(const body of doc.querySelectorAll('worldbody body')){
   const bodyName=body.getAttribute('name');
   const parent=groupByBody.get(bodyName);
   if(!parent)continue;
   for(const geom of body.querySelectorAll(':scope > geom[type="mesh"]')){
     const meshName=geom.getAttribute('mesh');
     if(!assets.has(meshName))throw Error(`[Visual] Missing asset declaration: ${meshName}`);
     entries.push({bodyName, parent, geom, meshName, file:assets.get(meshName)});
   }
 }
 visualMeshStatus.total=entries.length;
 const geoCache=new Map();
 const getGeometry=async(file)=>{
   if(!geoCache.has(file)){
     geoCache.set(file,(async()=>{
       const src=`${import.meta.env.BASE_URL}${file}`;
       const response=await fetch(src);
       if(!response.ok)throw Error(`[Visual] ${src} HTTP ${response.status}`);
       const geom=loader.parse(await response.arrayBuffer());
       if(!geom.attributes.normal)geom.computeVertexNormals();
       return geom;
     })());
   }
   return geoCache.get(file);
 };
 // Visual attachments are kept separate from collision meshes and use the
 // body-frame transform from MuJoCo without creating any collision geometry.
 const results=await Promise.allSettled(entries.map(async(e)=>{
   const geometry=await getGeometry(e.file);
   const rgba=(e.geom.getAttribute('rgba')||'0.7 0.7 0.7 1').trim().split(/\s+/).map(Number);
   const mk=rgba.map(v=>v.toFixed(5)).join(',');
   if(!materialCache.has(mk)){
     materialCache.set(mk,new THREE.MeshStandardMaterial({
       color:new THREE.Color(rgba[0],rgba[1],rgba[2]),
       opacity:rgba[3],transparent:rgba[3]<0.999,
       roughness:.72,metalness:.16,side:THREE.DoubleSide
     }));
   }
   const visual=new THREE.Mesh(geometry,materialCache.get(mk));
   const pos=(e.geom.getAttribute('pos')||'0 0 0').trim().split(/\s+/).map(Number);
   const q=(e.geom.getAttribute('quat')||'1 0 0 0').trim().split(/\s+/).map(Number);
   visual.position.set(pos[0],pos[1],pos[2]);
   visual.quaternion.set(q[1],q[2],q[3],q[0]);
   visual.castShadow=true;visual.receiveShadow=true;
   visual.name=`${e.bodyName}/${e.meshName}`;
   e.parent.add(visual);visualMeshGroups.push(visual);
   visualMeshStatus.loaded++;
 }));
 const failed=results.filter(r=>r.status==='rejected');
 if(failed.length)console.warn(`[Visual] ${failed.length} STL files failed; fallback colliders remain available`,failed.map(r=>r.reason));
 visualMeshStatus.ready=visualMeshStatus.loaded>0;
 refreshVisualVisibility();
 console.log(`[Visual] G1 STL loaded ${visualMeshStatus.loaded}/${visualMeshStatus.total} from training MJCF; physics unchanged`);
}
function refreshVisualVisibility(){
 for(const v of visualMeshGroups)v.visible=visualSettings.showSTL;
 for(const c of collisionMeshGroups)c.visible=visualSettings.showColliders;
}

function initThree(){
 renderer=new THREE.WebGLRenderer({antialias:true});renderer.setSize(innerWidth,innerHeight);
 renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.shadowMap.enabled=true;
 document.body.appendChild(renderer.domElement);
 scene=new THREE.Scene();scene.background=new THREE.Color(0x151927);
 camera=new THREE.PerspectiveCamera(48,innerWidth/innerHeight,.05,100);camera.position.set(2.5,1.7,2.4);
 controls=new OrbitControls(camera,renderer.domElement);controls.target.set(0,0.85,0);controls.enableDamping=true;
 scene.add(new THREE.HemisphereLight(0xffffff,0x454060,2));
 const light=new THREE.DirectionalLight(0xffffff,2);light.position.set(3,5,4);scene.add(light);
 worldGroup=new THREE.Group();worldGroup.rotation.x=-Math.PI/2;scene.add(worldGroup);
 const floor=new THREE.Mesh(new THREE.PlaneGeometry(35,35),new THREE.MeshStandardMaterial({color:0x353a4b,roughness:1}));
 floor.receiveShadow=true;worldGroup.add(floor);
 const grid=new THREE.GridHelper(35,70,0x525d77,0x2a3040);grid.rotation.x=Math.PI/2;grid.position.z=.002;worldGroup.add(grid);
 // Only collision geoms. MuJoCo itself parses the real body rotations/positions.
 // Render each geom in its own body-local coordinate frame.
 const geomIds=new Map();
 for(let bi=0;bi<humanoidData.bodies.length;bi++){
   const b=humanoidData.bodies[bi];const mb=bodyMap[bi];
   const group=new THREE.Group();worldGroup.add(group);
   for(const g of b.geoms){
     let mesh=null;const material=new THREE.MeshStandardMaterial({color: b.name.includes('left')?0x72a4dc:b.name.includes('right')?0xa68bd6:0xa2adbf,metalness:.15,roughness:.7});
     if(g.type==='sphere'){
        mesh=new THREE.Mesh(new THREE.SphereGeometry(g.radius,14,10),material);
        mesh.position.set(...g.pos);
     }else if(g.type==='capsule' && g.fromto){
        const f=g.fromto,a=new THREE.Vector3(f[0],f[1],f[2]),z=new THREE.Vector3(f[3],f[4],f[5]),v=z.clone().sub(a);
        mesh=new THREE.Mesh(new THREE.CapsuleGeometry(g.radius,v.length(),5,10),material);
        mesh.position.copy(a.add(z).multiplyScalar(.5));mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),v.normalize());
     }else if(g.type==='box'){
        mesh=new THREE.Mesh(new THREE.BoxGeometry(...g.halfExtents.map(x=>2*x)),material);mesh.position.set(...g.pos);
     }else if(g.type==='cylinder'){
        const hh=g.halfHeight||0.02;mesh=new THREE.Mesh(new THREE.CylinderGeometry(g.radius,g.radius,2*hh,12),material);mesh.rotation.x=Math.PI/2;
        if(g.pos)mesh.position.set(...g.pos);
     }
     if(mesh){mesh.castShadow=true;group.add(mesh);collisionMeshGroups.push(mesh);}
   }
   jointGeomMeshes.push({id:mb,group});
 }
 const marker=new THREE.Mesh(new THREE.SphereGeometry(.07,16,10),new THREE.MeshBasicMaterial({color:0xffcc60}));marker.position.z=.07;worldGroup.add(marker);
 const arrow=new THREE.ArrowHelper(new THREE.Vector3(1,0,0),new THREE.Vector3(0,0,.08),.5,0x70dc9e);worldGroup.add(arrow);
 function syncMarkers(){marker.position.set(targetXY.x,targetXY.y,.07);const player=getVRFacingPlayerPosition();const a=playerDirection.angleDeg*Math.PI/180;const dx=player?player[0]-data.qpos[0]:Math.cos(a),dy=player?player[1]-data.qpos[1]:Math.sin(a);const len=Math.hypot(dx,dy);arrow.position.set(data.qpos[0],data.qpos[1],.08);arrow.setDirection(new THREE.Vector3(len>1e-6?dx/len:1,len>1e-6?dy/len:0,0));}
 window.__syncMarkers=syncMarkers;
 addEventListener('resize',()=>{camera.aspect=innerWidth/innerHeight;camera.updateProjectionMatrix();renderer.setSize(innerWidth,innerHeight);});
}
function updateScene(){
 for(const {id,group} of jointGeomMeshes){
  const pos=data.xpos,quat=data.xquat;
  group.position.set(pos[id*3],pos[id*3+1],pos[id*3+2]);
  group.quaternion.set(quat[id*4+1],quat[id*4+2],quat[id*4+3],quat[id*4]);
 }
 updateVRVisuals(model,data,worldGroup,THREE);
 updateFurnitureVisuals(data,worldGroup,THREE);
 window.__syncMarkers?.();controls.update();renderer.render(scene,camera);
}
function initGUI(){
 gui=new GUI({title:'VaM / MimicKit / MuJoCo'});
 gui.add({reset:resetHumanoid},'reset').name('Reset');
 const v=gui.addFolder('VaM WebSocket / UDP');
 v.add(vamStream,'enabled').name('Stream enabled').onChange(on=>on?connectVaM():disconnectVaM());
 v.add(vamStream,'url').name('WebSocket URL').onFinishChange(()=>{if(vamStream.enabled)connectVaM();});
 v.add(vamStream,'rateHz',1,60,1).name('Send Hz');
 v.add(vamStream,'status').name('Status').listen().disable();
 v.add(vamStream,'sent').name('Frames sent').listen().disable();
 const vr=gui.addFolder('VRHand / HMD sync');
 vr.add(vrSettings,'enabled').name('Tracking & physics');
 vr.add(vrSettings,'showBodies').name('Show VR colliders');
 vr.add(vrSettings,'status').name('Status').listen().disable();
 vr.add(vrSettings,'frames').name('Frames received').listen().disable();
 vr.add(vrSettings,'hmdX').name('HMD X').listen().disable();
 vr.add(vrSettings,'hmdY').name('HMD Y').listen().disable();
 vr.add(vrSettings,'hmdZ').name('HMD Z').listen().disable();
 vr.add(vrSettings,'leftTrigger').name('L trigger').listen().disable();
 vr.add(vrSettings,'rightTrigger').name('R trigger').listen().disable();
 vr.add(vrSettings,'grabEnabled').name('Bone Grab');
 vr.add(vrSettings,'leftGrab').name('L grab body').listen().disable();
 vr.add(vrSettings,'rightGrab').name('R grab body').listen().disable();
 vr.add(vrSettings,'rightDoubleEnabled').name('Right double trigger');
 vr.add(vrSettings,'leftDoubleEnabled').name('Left double trigger');
 vr.add(targetSettings,'mode').name('Target mode').listen().disable();
 vr.add({toggle:()=>toggleHMDTarget(targetXY,getVRHMDPose())},'toggle').name('Toggle STAY / FOLLOW');
 const ht=gui.addFolder('HMD target follow');
 ht.add(targetSettings,'minDistance',0.05,0.7,.01).name('Min distance');
 ht.add(targetSettings,'maxDistance',0.25,1.0,.01).name('Max distance');
 ht.add(targetSettings,'yawDeadzoneDeg',0,40,1).name('Yaw deadzone');
 ht.add(targetSettings,'movingSpeedOn',0.05,1.5,.05).name('Move ON m/s');
 ht.add(targetSettings,'movingSpeedOff',0.01,1,.05).name('Move OFF m/s');
 ht.add(targetSettings,'avoidFurniture').name('Furniture avoid');
 ht.add(targetSettings,'avoidClearance',0,.5,.01).name('Avoid clearance');
 ht.add(targetSettings,'nominalDistance').name('HMD target dist').listen().disable();
 ht.add(targetSettings,'moving').name('Moving mode').listen().disable();
 ht.add(targetSettings,'avoidStatus').name('Avoid status').listen().disable();
 vr.add(vrSettings,'facingMode').name('Facing mode').listen().disable();
 vr.add(vrSettings,'kp',0,2000,25).name('Hand Kp');
 vr.add(vrSettings,'kd',0,100,5).name('Hand Kd');
 vr.add(vrSettings,'maxForce',0,600,25).name('Hand max N');
 vr.add(vrSettings,'gravityComp').name('Hand gravity compensation');
 const fur=gui.addFolder('Furniture / XML diagnostics');
 fur.add(furnitureSettings,'enabled').name('Collision enabled');
 fur.add(furnitureSettings,'showShapes').name('Show furniture');
 fur.add(furnitureSettings,'loaded').name('XML registered').listen().disable();
 fur.add(furnitureSettings,'active').name('Matched / active').listen().disable();
 fur.add(furnitureSettings,'packets').name('ENV1 packets').listen().disable();
 fur.add(furnitureSettings,'receivedIds').name('Received IDs').listen().disable();
 fur.add(furnitureSettings,'missingIds').name('Unregistered IDs').listen().disable();
 fur.add(furnitureSettings,'status').name('Status').listen().disable();
 fur.add({printReport:()=>console.info('[Furniture DIAG] Manual report:',getFurnitureReport())},'printReport').name('Print report to console');
 const visual=gui.addFolder('G1 Appearance (STL)');
 visual.add(visualSettings,'showSTL').name('STL meshes').onChange(refreshVisualVisibility);
 visual.add(visualSettings,'showColliders').name('Collision shapes').onChange(refreshVisualVisibility);
 const t=gui.addFolder('Location target');t.add(targetXY,'x',-5,5,.05).name('Target X');t.add(targetXY,'y',-5,5,.05).name('Target Y');
 const p=gui.addFolder('Player direction');p.add(playerDirection,'angleDeg',-180,180,1).name('Fallback angle');p.add(playerDirection,'distance',0,5,.05).name('Distance');p.add(playerDirection,'localX').listen().disable();p.add(playerDirection,'localY').listen().disable();
 const s=gui.addFolder('Simulation');
 const sp={policy:true,drives:true,gravity:true,timeScale:0.5,physicsHz:240,controlHz:30};
 s.add(sp,'policy').name('NN Policy').onChange(v=>policyEnabled=v);
 s.add(sp,'drives').name('Active Drives').onChange(v=>drivesEnabled=v);
 s.add(sp,'gravity').name('Gravity').onChange(v=>{gravityEnabled=v;model.opt.gravity[2]=v?-9.81:0;});
 s.add(sp,'timeScale',.1,2,.1).name('Time scale').onChange(v=>timeScale=v);
 s.add(sp,'physicsHz').disable();s.add(sp,'controlHz').disable();
 const tune=gui.addFolder('Newton-style Servo / Recovery');
 tune.add(debugTune,'kp',0,4,.05).name('Kp x');tune.add(debugTune,'kd',0,4,.05).name('Kd x');tune.add(debugTune,'force',.1,5,.1).name('Force x');
 tune.add(debugTune,'actionScale',0,2,.05).name('Action amplitude');tune.add(debugTune,'actionRate',0,30,.25).name('Action max rad/s');
 tune.add(debugTune,'smoothingMs',0,150,5).name('Smoothing ms');
 const noise=gui.addFolder('ONNX Action Noise');
 noise.add(actionNoise,'enabled').name('Enable noise').onChange(v=>{if(!v)actionNoiseState.fill(0);});
 noise.add(actionNoise,'stdDev',0,0.15,0.005).name('Std dev (rad)');
 noise.add(actionNoise,'correlationMs',0,1000,20).name('Correlation (ms)');
 tune.add(debugTune,'policySteps',{'30 Hz':8,'60 Hz test':4,'120 Hz test':2});
 const d=gui.addFolder('Diagnostics');for(const key of ['rootZ','rootSpeed','targetError','inferenceMs','realPolicyHz','actionDelta','clipCount'])d.add(debugTune,key).listen().disable();
}


let accum=0, physTick=0, resetGrace=120,lastStamp=0,lastInferStamp=0;
async function mainLoop(stamp){
 if(!running)return;
 if(lastStamp===0)lastStamp=stamp;
 const realDT=Math.min(.1,Math.max(0,(stamp-lastStamp)/1000));lastStamp=stamp;
 accum+=realDT*timeScale;frameCount++;
 if(stamp-lastFpsTime>1000){
    document.getElementById('fpsDisplay').textContent=`FPS: ${frameCount}`;frameCount=0;lastFpsTime=stamp;
    const s=getState();debugTune.rootZ=s.rootPos[2];debugTune.rootSpeed=Math.hypot(...s.rootVel);
    document.getElementById('statInfo').textContent=`Physics 240Hz / Sim ${simulationSeconds.toFixed(2)}s / root z=${s.rootPos[2].toFixed(3)}m`;
 }
 // Policy+physics share a single deterministic simulated-time clock. Inference awaits, but sim time does not advance during await.
 let count=0;
 while(accum>=DT && count<24){
   if(physTick % debugTune.policySteps===0){
      if(policyEnabled){
       try{
          const obs=buildObservation();const start=performance.now();
          if(lastInferStamp>0)debugTune.realPolicyHz=1000/(start-lastInferStamp);
          lastInferStamp=start;currentAction=await runPolicy(obs);
          debugTune.inferenceMs=performance.now()-start;
          applyActions(currentAction);
       }catch(e){console.error('ONNX inference:',e);policyEnabled=false;}
      }else applyActions(new Float32Array(humanoidData.init_dof_pos),false);
   }
   applyServoTargets();stepVR(data,DT,bodyMap);stepFurniture(data);mujoco.mj_step(model,data);
   simulationSeconds+=DT;sendVaMIfDue();accum-=DT;physTick++;count++;
   if(resetGrace>0)resetGrace--;
 }
 if(accum>DT*48)accum=DT*48; // avoid runaway backlog on slow devices
 updateScene();
 //const s=getState();if(resetGrace===0 && (s.rootPos[2]<.22 || s.rootPos[2]>2.5 || !Number.isFinite(s.rootPos[2]))){resetHumanoid();}
 requestAnimationFrame(mainLoop);
}
async function init(){
 try{
    setLeftDoubleTriggerHandler(()=>toggleHMDTarget(targetXY,getVRHMDPose()));
    await loadOnnxModel();await initMuJoCo();initThree();
    setStatus('Loading G1 visual STL meshes ...');
    await loadG1VisualSTLs();
    initGUI();updateScene();
    document.getElementById('loading').style.display='none';running=true;
    requestAnimationFrame(mainLoop);
 }catch(e){setStatus('Startup failed: '+e.message+'\nCheck browser Console');console.error(e);}
}
init();
