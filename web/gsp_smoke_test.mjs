// OpenRoad Dynamics — smoke test for the many-vehicle host module (gsp.wasm).
// SPDX-License-Identifier: MIT
// Runs cars in ChassisMode::External with a minimal host rigid-body integrator standing in for
// the page's Rapier world: 0-100 km/h, a 100-0 stop, a steady circle, rain, a deck and two cars.
// Usage: node web/gsp_smoke_test.mjs [dir-with-gsp.mjs]   (default: web/dist)
import { pathToFileURL } from 'node:url';
import path from 'node:path';

const dir = path.resolve(process.argv[2] ?? path.join(path.dirname(new URL(import.meta.url).pathname), 'dist'));
const { default: createGspModule } = await import(pathToFileURL(path.join(dir, 'gsp.mjs')).href);
const M = await createGspModule();
const F = () => M.HEAPF64;

// A fictional V8 RWD sedan (the GSP interceptor's numbers).
const P = {
  0: 1950, 1: 650, 2: 3300, 3: 3600, 4: 3.05, 5: 0.52, 6: 1.62, 7: 1.62, 8: 0.54, 9: 0.36, 10: 0.36,
  11: 1.08, 12: 1.04, 13: 1.10, 14: 1.06, 15: 15, 16: 42000, 17: 46000, 18: 0.2, 19: 3400, 20: 5000, 21: 3400, 22: 5000,
  23: 24000, 24: 14000, 25: 0.58, 26: 3200, 27: 1900, 28: 2600, 29: 0, 30: 0.4, 31: 2, 32: 3.07, 33: 0, 34: 0.18, 35: 8,
  36: 4.71, 37: 3.14, 38: 2.11, 39: 1.67, 40: 1.29, 41: 1.0, 42: 0.84, 43: 0.67, 46: 3.53, 47: 700, 48: 6200, 49: 0.25,
  50: 7, 51: 1000, 52: 410, 53: 2000, 54: 490, 55: 3000, 56: 530, 57: 4200, 58: 540, 59: 5200, 60: 520, 61: 5800, 62: 480, 63: 6200, 64: 440,
  67: 0.85, 68: 67, 69: 1.0, 70: 1.15, 71: 0.95, 72: 1.25, 73: 0.5, 74: 45, 75: 0.5, 76: 900, 77: 0.85, 78: 0.8, 79: 80, 80: 0.45, 81: 0.25,
};
function writeParams(over = {}) {
  const p = M._gsp_params() / 8, n = M._gsp_param_count();
  F().fill(0, p, p + n);
  for (const [k, v] of Object.entries({ ...P, ...over })) F()[p + Number(k)] = v;
}

// Ground: 400 m x 400 m flat asphalt at 2 m cells, a 40 m deck at 6.5 m.
const N = 201, cell = 2, x0 = -200, y0 = -200;
const hp = M._gsp_ground_alloc(N, N, x0, y0, cell) / 4;
M.HEAPF32.fill(0, hp, hp + N * N);
M._gsp_set_surface(0, 1, 1, 0, 0, 0, 0.15, 0.3, 1);
M._gsp_set_surface(7, 1, 1, 0, 0, 0, 0.15, 0.3, 1);
M._gsp_add_deck(100, 100, 6.5, 20, 6, 0, 0, 7);

// The host: semi-implicit Euler, like a game rigid-body engine. Gravity is the host's.
const quatRotate = (q, v) => {
  const [w, x, y, z] = q, [vx, vy, vz] = v;
  const ix = w * vx + y * vz - z * vy, iy = w * vy + z * vx - x * vz, iz = w * vz + x * vy - y * vx, iw = -x * vx - y * vy - z * vz;
  return [ix * w + iw * -x + iy * -z - iz * -y, iy * w + iw * -y + iz * -x - ix * -z, iz * w + iw * -z + ix * -y - iy * -x];
};
const quatConj = (q) => [q[0], -q[1], -q[2], -q[3]];
class HostBody {
  constructor(h) {
    const s = M._gsp_state(h) / 8, b = M._gsp_body_info(h) / 8;
    this.h = h; this.m = F()[b]; this.I = [F()[b + 4], F()[b + 5], F()[b + 6]];
    this.p = [F()[s], F()[s + 1], F()[s + 2]]; this.q = [F()[s + 3], F()[s + 4], F()[s + 5], F()[s + 6]];
    this.v = [F()[s + 44], F()[s + 45], F()[s + 46]]; this.w = [F()[s + 47], F()[s + 48], F()[s + 49]];
  }
  step(dt, sub) {
    M._gsp_set_body(this.h, ...this.p, ...this.q, ...this.v, ...this.w);
    if (!M._gsp_step(this.h, dt, sub)) throw new Error('faulted');
    const f = M._gsp_forces(this.h) / 8, H = F();
    const fx = H[f], fy = H[f + 1], fz = H[f + 2] - 9.80665 * this.m;
    this.v = [this.v[0] + fx / this.m * dt, this.v[1] + fy / this.m * dt, this.v[2] + fz / this.m * dt];
    // angular: torque in body frame / principal inertia
    const tb = quatRotate(quatConj(this.q), [H[f + 3], H[f + 4], H[f + 5]]);
    const wb = quatRotate(quatConj(this.q), this.w);
    const Iw = [this.I[0] * wb[0], this.I[1] * wb[1], this.I[2] * wb[2]];
    const gyro = [wb[1] * Iw[2] - wb[2] * Iw[1], wb[2] * Iw[0] - wb[0] * Iw[2], wb[0] * Iw[1] - wb[1] * Iw[0]];
    const wb2 = [wb[0] + (tb[0] - gyro[0]) / this.I[0] * dt, wb[1] + (tb[1] - gyro[1]) / this.I[1] * dt, wb[2] + (tb[2] - gyro[2]) / this.I[2] * dt];
    this.w = quatRotate(this.q, wb2);
    this.p = [this.p[0] + this.v[0] * dt, this.p[1] + this.v[1] * dt, this.p[2] + this.v[2] * dt];
    const [w, x, y, z] = this.q, [ax, ay, az] = this.w, hdt = 0.5 * dt;
    const q = [w + hdt * (-ax * x - ay * y - az * z), x + hdt * (ax * w + ay * z - az * y), y + hdt * (ay * w + az * x - ax * z), z + hdt * (az * w + ax * y - ay * x)];
    const l = Math.hypot(...q); this.q = q.map((c) => c / l);
  }
  state(k) { return F()[M._gsp_state(this.h) / 8 + k]; }
}

let failures = 0;
const check = (ok, msg) => { console.log((ok ? 'ok   ' : 'FAIL ') + msg); if (!ok) ++failures; };
const DT = 1 / 120, SUB = 3;

// 1. 0-100 km/h, SPORT, then 100-0.
writeParams();
check(M._gsp_validate() === 1, 'interceptor parameters are valid');
let h = M._gsp_create(0, 0, 0.5, 0, 0, 0);
check(h >= 0, 'create');
let body = new HostBody(h);
const rest = body.p[2];
check(Math.abs(rest - 0.54) < 0.04, `rests with its CoM at ${rest.toFixed(3)} m (asked 0.54)`);
for (let k = 0; k < 120; ++k) body.step(DT, SUB);
check(Math.abs(body.p[2] - rest) < 0.02, `settled height holds under the host (${body.p[2].toFixed(3)} m)`);
let t = 0;
M._gsp_set_input(h, 0, 1, 0, 0, 0, 0);
while (body.state(7) < 100 / 3.6 && t < 20) { body.step(DT, SUB); t += DT; }
check(t > 4 && t < 8.5, `0-100 km/h in ${t.toFixed(2)} s (gear ${body.state(15)}, ${body.state(14).toFixed(0)} rpm)`);
M._gsp_set_input(h, 0, 0, 1, 0, 0, 0);
let x0b = body.p[0];
t = 0;
while (body.state(7) > 0.3 && t < 10) { body.step(DT, SUB); t += DT; }
check(body.p[0] - x0b > 30 && body.p[0] - x0b < 55, `100-0 km/h in ${(body.p[0] - x0b).toFixed(1)} m`);
check(M.HEAPF64[M._gsp_state(h) / 8 + 39] < 0.02, `host sync error ${M.HEAPF64[M._gsp_state(h) / 8 + 39].toExponential(2)} m`);

// 2. Steady circle at ~60 km/h: lateral g with SPORT steering.
M._gsp_place(h, -150, -150, 0.5, 0, 16);
body = new HostBody(h);
let ay = 0;
for (let k = 0; k < 120 * 8; ++k) {
  const v = body.state(7);
  M._gsp_set_input(h, 0.35, v < 16.7 ? 0.6 : 0.25, 0, 0, 0, 0);
  body.step(DT, SUB);
  if (k > 120 * 6) ay = Math.max(ay, Math.abs(body.state(12)));
}
check(ay > 3 && ay < 13, `steady cornering: ${(ay / 9.81).toFixed(2)} g at ${(body.state(7) * 3.6).toFixed(0)} km/h, upright ${body.state(36).toFixed(3)}`);

// 3. Rain: a wet stop is longer.
M._gsp_set_weather(1, 0);
M._gsp_place(h, -150, 0, 0.5, 0, 100 / 3.6);
body = new HostBody(h);
M._gsp_set_aids(h, 0, 0, 0, 1);
M._gsp_set_input(h, 0, 0, 1, 0, 0, 0);
x0b = body.p[0]; t = 0;
while (body.state(7) > 0.3 && t < 15) { body.step(DT, SUB); t += DT; }
const wet = body.p[0] - x0b;
check(wet > 55, `wet stop, no ABS: ${wet.toFixed(1)} m`);
M._gsp_set_weather(0, 0);

// 4. A deck: the car rests on it, and under it on the ground.
M._gsp_place(h, 100, 100, 7.0, 0, 0);
check(Math.abs(M.HEAPF64[M._gsp_state(h) / 8 + 2] - 6.5 - 0.54) < 0.05, `rests on the deck at ${M.HEAPF64[M._gsp_state(h) / 8 + 2].toFixed(2)} m`);
M._gsp_place(h, 100, 100, 0.5, 0, 0);
check(Math.abs(M.HEAPF64[M._gsp_state(h) / 8 + 2] - 0.54) < 0.05, `and under it at ${M.HEAPF64[M._gsp_state(h) / 8 + 2].toFixed(2)} m`);

// 5. A heavy pickup and a traffic-tier car side by side, both driven by the same core.
writeParams({ 0: 2700, 1: 1100, 2: 5200, 3: 5600, 4: 3.7, 5: 0.56, 8: 0.78, 9: 0.4, 10: 0.4, 16: 60000, 17: 70000, 72: 1.6 });
const h2 = M._gsp_create(0, 50, 0.5, 0, 0, 1);
check(h2 >= 0 && h2 !== h, `second car (pickup, traffic tier) handle ${h2}`);
const pickup = new HostBody(h2);
M._gsp_set_input(h2, 0, 1, 0, 0, 0, 0);
for (let k = 0; k < 120 * 5; ++k) pickup.step(DT, SUB);
check(pickup.state(7) > 15, `pickup at ${(pickup.state(7) * 3.6).toFixed(0)} km/h after 5 s`);

// 6. Live rebuild keeps the motion.
writeParams({ 11: 1.4, 12: 1.4 });
const vBefore = pickup.state(7);
check(M._gsp_rebuild(h2) === 1 && Math.abs(pickup.state(7) - vBefore) < 0.5, 'rebuild keeps speed');

console.log(failures ? `${failures} failure(s)` : 'all passed');
process.exit(failures ? 1 : 0);
