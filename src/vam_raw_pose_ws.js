// Raw MuJoCo poses; no VaM retargeting is performed in the browser.
// Header: ASCII MVR1, uint32 LE frame length (984).
// Payload: 31 xyz body positions, 31 wxyz world body rotations, 29 joint positions.
// Body order is humanoidData.bodies / bodyMap, NOT MuJoCo body-ID order.
const BODY_COUNT = 31, DOF_COUNT = 29;
const FLOAT_COUNT = BODY_COUNT * 3 + BODY_COUNT * 4 + DOF_COUNT;
const BYTES = 8 + FLOAT_COUNT * 4;
export function makeRawPosePacket(model, data, bodyMap, dofMap) {
  if (bodyMap.length !== BODY_COUNT || dofMap.length !== DOF_COUNT)
    throw new Error(`Raw VaM pose requires ${BODY_COUNT} bodies / ${DOF_COUNT} DOFs; got ${bodyMap.length}/${dofMap.length}`);
  const buffer = new ArrayBuffer(BYTES);
  const dv = new DataView(buffer);
  for (let i = 0; i < 4; ++i) dv.setUint8(i, 'MVR1'.charCodeAt(i));
  dv.setUint32(4, FLOAT_COUNT * 4, true);
  let p = 8;
  function put(v) { if (!Number.isFinite(v)) throw Error('Non-finite raw pose'); dv.setFloat32(p, v, true); p += 4; }
  for (const id of bodyMap) for (let j = 0; j < 3; ++j) put(data.xpos[id * 3 + j]);
  for (const id of bodyMap) for (let j = 0; j < 4; ++j) put(data.xquat[id * 4 + j]); // wxyz
  for (const d of dofMap) put(data.qpos[d.qpos]);
  if (p !== BYTES) throw Error('Unexpected pose payload length');
  return buffer;
}
