// OpenRoad Dynamics — loader for the many-vehicle wasi-sdk build (web/build_gsp_wasi.sh).
// SPDX-License-Identifier: MIT
// Same interface as the Emscripten glue: `const M = await createGspModule()`, then M._ord_*()
// and M.HEAPF64. The core does no I/O, so the WASI imports are minimal stubs.
export default async function createGspModule() {
  const url = new URL('./gsp.wasm', import.meta.url);
  let bytes;
  if (typeof process !== 'undefined' && process.versions?.node && url.protocol === 'file:') {
    const { readFile } = await import('node:fs/promises');
    bytes = await readFile(url);
  } else {
    bytes = await (await fetch(url)).arrayBuffer();
  }
  let memory = null;
  const ENOSYS = 52;
  const wasi = new Proxy({
    proc_exit: (c) => { throw new Error('gsp.wasm exited with code ' + c); },
    fd_write: (fd, iovs, n, nwritten) => { // forward stdout/stderr to the console
      const v = new DataView(memory.buffer); let s = '', total = 0;
      for (let i = 0; i < n; ++i) { const p = v.getUint32(iovs + i * 8, true), l = v.getUint32(iovs + i * 8 + 4, true); s += new TextDecoder().decode(new Uint8Array(memory.buffer, p, l)); total += l; }
      v.setUint32(nwritten, total, true); if (s.trim()) (fd === 2 ? console.warn : console.log)(s.trimEnd()); return 0;
    },
    environ_sizes_get: (c, s) => { const v = new DataView(memory.buffer); v.setUint32(c, 0, true); v.setUint32(s, 0, true); return 0; },
    environ_get: () => 0,
    clock_time_get: (id, prec, out) => { new DataView(memory.buffer).setBigUint64(out, BigInt(Math.round(performance.now() * 1e6)), true); return 0; },
    random_get: (p, n) => { crypto.getRandomValues(new Uint8Array(memory.buffer, p, n)); return 0; },
  }, { get: (t, k) => t[k] ?? (() => ENOSYS) });
  const { instance } = await WebAssembly.instantiate(bytes, { wasi_snapshot_preview1: wasi });
  const ex = instance.exports;
  memory = ex.memory;
  ex._initialize?.();
  const M = {};
  for (const [k, f] of Object.entries(ex)) if (k.startsWith('gsp_')) M['_' + k] = f;
  let heap = new Float64Array(memory.buffer);
  Object.defineProperty(M, 'HEAPF64', { get: () => (heap.buffer === memory.buffer ? heap : (heap = new Float64Array(memory.buffer))) });
  const view = (C) => { let v = new C(memory.buffer); return () => (v.buffer === memory.buffer ? v : (v = new C(memory.buffer))); };
  const h16 = view(Int16Array), hu8 = view(Uint8Array), hf32 = view(Float32Array);
  Object.defineProperty(M, 'HEAP16', { get: h16 });
  Object.defineProperty(M, 'HEAPU8', { get: hu8 });
  Object.defineProperty(M, 'HEAPF32', { get: hf32 });
  // Emscripten returns byte addresses for pointers; keep that convention.
  return M;
}
