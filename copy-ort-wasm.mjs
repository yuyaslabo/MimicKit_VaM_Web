import { copyFileSync, existsSync, mkdirSync, readFileSync, readdirSync } from 'node:fs';
import { resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

// ONNX Runtime 1.21 dynamically imports a JSEP module even when the chosen
// execution provider is WASM. Copy every runtime-specific ESM and WASM asset
// from the *same installed package version* rather than guessing which pair
// the ORT dispatcher will request.
const root = dirname(fileURLToPath(import.meta.url));
const source = resolve(root, 'node_modules/onnxruntime-web/dist');
const dest = resolve(root, 'public/ort');
if (!existsSync(source)) throw new Error(`ONNX Runtime package missing: ${source}. Run npm install first.`);
mkdirSync(dest, { recursive: true });

const names = readdirSync(source).filter(name =>
  /^ort-wasm.*\.(?:mjs|wasm)$/.test(name)
);
const required = ['ort-wasm-simd-threaded.wasm', 'ort-wasm-simd-threaded.jsep.mjs'];
for (const name of required) {
  if (!names.includes(name)) throw new Error(`Expected ONNX Runtime 1.21 asset missing: ${source}/${name}`);
}
for (const name of names) {
  const src = resolve(source, name);
  const output = resolve(dest, name);
  copyFileSync(src, output);
  if (name.endsWith('.wasm')) {
    const magic = readFileSync(output).subarray(0, 4);
    if (!magic.equals(Buffer.from([0x00, 0x61, 0x73, 0x6d]))) {
      throw new Error(`Invalid WebAssembly magic in ${name}`);
    }
  }
  console.log(`[ORT] Local asset ready: public/ort/${name}`);
}
console.log(`[ORT] Copied ${names.length} ONNX Runtime WASM/JSEP runtime files`);
