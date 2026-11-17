import {mkdir, copyFile, readFile} from 'node:fs/promises';
import {createRequire} from 'node:module';
import {dirname, join, resolve} from 'node:path';
const require = createRequire(import.meta.url);
let source;
try {
  source = require.resolve('@mujoco/mujoco/mujoco.wasm');
} catch (err) {
  try {
    source = join(dirname(require.resolve('@mujoco/mujoco')), 'mujoco.wasm');
  } catch {
    throw new Error('Cannot locate @mujoco/mujoco/mujoco.wasm. Run npm install first.', {cause:err});
  }
}
const bytes = await readFile(source);
if (bytes.length < 8 || bytes[0] !== 0 || bytes[1] !== 0x61 || bytes[2] !== 0x73 || bytes[3] !== 0x6d) {
  throw new Error(`Invalid MuJoCo WASM in npm package: ${source}`);
}
const targetDir = resolve('public', 'mujoco');
await mkdir(targetDir, {recursive:true});
await copyFile(source, join(targetDir, 'mujoco.wasm'));
console.log(`[MuJoCo] Local WASM ready: public/mujoco/mujoco.wasm (${(bytes.length/1048576).toFixed(1)} MiB)`);
