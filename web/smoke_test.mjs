// OpenRoad Dynamics — Node smoke test for the WebAssembly build.
// SPDX-License-Identifier: MIT
// Usage: node web/smoke_test.mjs [dist-dir]
import path from 'node:path';
import { pathToFileURL } from 'node:url';

const dist = path.resolve(process.argv[2] ?? new URL('./dist', import.meta.url).pathname);
const { default: createOrdModule } = await import(pathToFileURL(path.join(dist, 'ord.mjs')).href);
const M = await createOrdModule();
const state = () => new Float64Array(M.HEAPF64.buffer, M._ord_state(), M._ord_state_header() + 4 * M._ord_state_per_wheel());
let failures = 0;
const check = (name, ok, detail) => { console.log(`${ok ? 'ok  ' : 'FAIL'} ${name} ${detail}`); if (!ok) ++failures; };

// 0-100 km/h per preset on the reference RWD car (fixed 1/360 s steps, full throttle).
const times = [];
for (const preset of [0, 1, 2]) {
  M._ord_set_preset(preset);
  if (preset === 2) M._ord_set_aids(0, 0, 0, 1); // SIMULATION aids stay off; automatic gearbox so the test needs no shifting
  check(`create preset ${preset}`, M._ord_create(0, 0, 0, 0) === 1, '');
  for (let i = 0; i < 360; ++i) { M._ord_set_input(0, 0, 1, 0, 0, 0); M._ord_advance(1 / 360); } // settle on the brakes
  let t = 0, s = state();
  while (s[7] < 100 / 3.6 && t < 20) { M._ord_set_input(0, 1, 0, 0, 0, 0); M._ord_advance(1 / 60); t += 1 / 60; s = state(); }
  times.push(t);
  check(`0-100 km/h preset ${preset}`, t > 3 && t < 9 && s[23] === 0, `${t.toFixed(2)} s`);
}
// Steady circle: lateral acceleration plausible, no fault, car stays on the ground.
M._ord_set_preset(1);
M._ord_create(0, 0, 0, 0);
let maxLat = 0, s = state();
for (let i = 0; i < 60 * 20; ++i) {
  const v = s[7];
  M._ord_set_input(0.35, v < 15 ? 0.6 : 0.25, 0, 0, 0, 0); M._ord_advance(1 / 60); s = state();
  if (i > 600) maxLat = Math.max(maxLat, Math.abs(s[12]) / 9.81);
}
check('circle lateral g', maxLat > 0.3 && maxLat < 1.3 && s[23] === 0, maxLat.toFixed(2));
check('ground height flat at start', M._ord_ground_height(0, 0) === 0, '');
check('speed bump present', M._ord_ground_height(60, M._ord_bump_lane_y()) > 0.07, '');
// Rosser Stella V8 (car 5): revs to its 9,000 rpm limiter and pulls through the gears.
M._ord_set_preset(1); M._ord_set_aids(1, 1, 0, 1);
check('create Stella', M._ord_create(5, 0, 0, 0) === 1, '');
let maxRpm = 0; s = state();
for (let i = 0; i < 60 * 20; ++i) { M._ord_set_input(0, 1, 0, 0, 0, 0); M._ord_advance(1 / 60); s = state(); maxRpm = Math.max(maxRpm, s[14]); }
check('Stella reaches 9,000 rpm', maxRpm > 8800 && maxRpm < 9300 && s[23] === 0, `${Math.round(maxRpm)} rpm, ${(s[7] * 3.6).toFixed(0)} km/h after 20 s`);
// Full steering lock + full throttle with TCS off holds 1st gear (burnout/donut) instead of upshifting.
M._ord_set_preset(2); M._ord_set_aids(0, 0, 0, 1); M._ord_create(5, 0, 0, 0);
let maxGear = 1;
for (let i = 0; i < 60 * 3; ++i) { M._ord_set_input(1, 1, 0, 0, 0, 0); M._ord_advance(1 / 60); s = state(); maxGear = Math.max(maxGear, s[15]); }
check('wheelspin holds the gear', maxGear === 1 && s[23] === 0, `max gear ${maxGear}`);
// DRIFT preset: handbrake entry, then steering centred and 80 % throttle; the drift assist holds ~20 deg.
{
  M._ord_set_preset(3);
  M._ord_create(0, 0, 0, 0);
  M._ord_set_aids(1, 0, 0, 1);
  const H = M._ord_state_header();
  const st = () => new Float64Array(M.HEAPF64.buffer, M._ord_state(), H + 4 * M._ord_state_per_wheel());
  // roll at ~50 km/h first
  for (let i = 0; i < 60 * 6 && st()[7] < 50 / 3.6; ++i) { M._ord_set_input(0, 1, 0, 0, 0, 0); M._ord_advance(1 / 60); }
  let entered = false, t = 0, sum = 0, n = 0, target = 0, spun = false;
  while (t < 14) {
    const s = st();
    if (!entered && s[32] > 0) entered = true;
    if (!entered) M._ord_set_input(0.8, 0.9, 0, t > 0.1 && t < 0.4 ? 1 : 0, 0, 0);
    else M._ord_set_input(0, 0.8, 0, 0, 0, 0);
    M._ord_advance(1 / 60); t += 1 / 60;
    const s2 = st();
    if (entered && t > 6) { sum += s2[10]; target += s2[33]; ++n; }
    if (Math.abs(s2[10]) > 1.4 || s2[7] < 1.5) { spun = true; break; }
  }
  const mean = n ? sum / n : 0, tgt = n ? target / n : 0;
  check('drift preset holds the requested angle', entered && !spun && n > 0 && Math.abs(mean - tgt) < 0.06 && Math.abs(tgt) > 0.3,
        `${(mean * 57.3).toFixed(1)} deg (target ${(tgt * 57.3).toFixed(1)})`);
}
console.log(failures ? `${failures} failure(s)` : 'PASSED');
process.exit(failures ? 1 : 0);
