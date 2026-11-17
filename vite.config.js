import { defineConfig } from 'vite';
import { readFile } from 'node:fs/promises';
import { resolve } from 'node:path';

// onnxruntime-web 1.21 dynamically imports its JSEP runtime under wasmPaths.
// Vite appends ?import to that URL, then rejects imports from public/.
// Serve ONLY these copied runtime modules as real ESM, without Vite's
// source-module transformer. All other URLs use the normal Vite pipeline.
function serveOrtRuntimeModules() {
  return {
    name: 'serve-ort-runtime-modules',
    configureServer(server) {
      server.middlewares.use(async (req, res, next) => {
        const pathname = new URL(req.url || '/', 'http://localhost').pathname;
        const match = /^\/ort\/(ort-wasm[a-zA-Z0-9._-]*\.mjs)$/.exec(pathname);
        if (!match) return next();
        try {
          const filePath = resolve(process.cwd(), 'public', 'ort', match[1]);
          const bytes = await readFile(filePath);
          res.statusCode = 200;
          res.setHeader('Content-Type', 'text/javascript; charset=utf-8');
          res.setHeader('Cache-Control', 'no-cache');
          res.end(bytes);
        } catch (err) {
          if (err?.code === 'ENOENT') {
            res.statusCode = 404;
            res.setHeader('Content-Type', 'text/plain; charset=utf-8');
            res.end(`ONNX runtime JS module missing: ${match[1]}. Run npm run dev to copy it.`);
          } else next(err);
        }
      });
    },
  };
}


export default defineConfig({
  plugins: [serveOrtRuntimeModules()],
  base: '/MimicKit_VaM_Web/',
});
