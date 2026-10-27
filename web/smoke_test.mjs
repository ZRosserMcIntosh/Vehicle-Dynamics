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
// Suspension kinematics: the Stella's wishbones report roll centres above the ground; the
// reference sports car's sliders sit at the ground.
{
  const gs = () => new Float64Array(M.HEAPF64.buffer, M._ord_garage_state(), M._ord_garage_state_size());
  M._ord_set_preset(1); M._ord_create(5, 0, 0, 0);
  for (let i = 0; i < 60; ++i) { M._ord_set_input(0, 0, 1, 0, 0, 0); M._ord_advance(1 / 60); }
  const stella = gs().slice();  // copy: the next call refills the same buffer
  M._ord_create(0, 0, 0, 0);
  for (let i = 0; i < 60; ++i) { M._ord_set_input(0, 0, 1, 0, 0, 0); M._ord_advance(1 / 60); }
  const slider = gs().slice();
  check('kinematics roll centres', stella[12] > 0.05 && stella[12] < 0.2 && stella[13] > 0.05 && Math.abs(slider[12]) < 1e-9,
        `Stella ${(stella[12] * 1000).toFixed(0)} / ${(stella[13] * 1000).toFixed(0)} mm, slider ${(slider[12] * 1000).toFixed(1)} mm`);
}
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
// DRIFT preset: handbrake entry, then steering 3/4 into the turn and 80 % throttle; the drift assist holds ~24 deg.
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
    else M._ord_set_input(0.75, 0.8, 0, 0, 0, 0);
    M._ord_advance(1 / 60); t += 1 / 60;
    const s2 = st();
    if (entered && t > 6) { sum += s2[10]; target += s2[33]; ++n; }
    if (Math.abs(s2[10]) > 1.4 || s2[7] < 1.5) { spun = true; break; }
  }
  const mean = n ? sum / n : 0, tgt = n ? target / n : 0;
  check('drift preset holds the requested angle', entered && !spun && n > 0 && Math.abs(mean - tgt) < 0.06 && Math.abs(tgt) > 0.3,
        `${(mean * 57.3).toFixed(1)} deg (target ${(tgt * 57.3).toFixed(1)})`);
}
// Car tuning: a 0.9 rad steering lock reaches the wheels; grip used is published per wheel.
{
  M._ord_set_preset(2); M._ord_set_aids(0, 0, 0, 1); M._ord_create(0, 0, 0, 0);
  M._ord_set_tuning(0.9, 2, 150, 0.6, 0.4);
  for (let i = 0; i < 120; ++i) { M._ord_set_input(1, 0, 0, 0, 0, 0); M._ord_advance(1 / 60); }
  const s = state(), H = M._ord_state_header(), P = M._ord_state_per_wheel();
  check('tuned steering lock', Math.abs(s[16] - 0.9) < 1e-6, `${s[16].toFixed(3)} rad`);
  check('grip used published', P >= 17 && s[H + 16] >= 0 && s[H + 16] < 1.5, `${s[H + 16].toFixed(3)}`);
  M._ord_set_tuning(0, -1, 80, 0.45, 0.25);
}
// Two-wheelers: each one leans into a left turn (lean < 0, yaw increasing) and stays up.
{
  const bs = () => new Float64Array(M.HEAPF64.buffer, M._ord_bike_state(), M._ord_bike_state_size());
  M._ord_bike_set_assists(0, 1, 1, 1, 1, 0.04);
  for (const [kind, v0] of [[0, 6], [1, 6], [2, 20]]) {
    check(`bike ${kind} create`, M._ord_bike_create(kind, 0, 0, 0, v0) === 1, '');
    let minLean = 0;
    for (let i = 0; i < 60 * 3; ++i) { M._ord_bike_set_input(0.6, 0.15, 0, 0); M._ord_bike_advance(1 / 60); minLean = Math.min(minLean, bs()[4]); }
    const b = bs();
    check(`bike ${kind} leans and turns left`, b[1] === 0 && minLean < -0.2 && b[22] > 0.3, `lean ${(minLean * 57.3).toFixed(0)} deg, yaw ${b[22].toFixed(2)}, ${(b[3] * 3.6).toFixed(0)} km/h`);
  }
  // Wheelie assist holds the motorcycle's front up.
  M._ord_bike_set_assists(0, 1, 1, 0, 1, 0.4);
  M._ord_bike_create(2, 0, 0, 0, 5);
  let pmax = 0;
  for (let i = 0; i < 60 * 3; ++i) { M._ord_bike_set_input(0, 1, 0, 0); M._ord_bike_advance(1 / 60); pmax = Math.max(pmax, bs()[6]); }
  check('motorcycle wheelie assist', bs()[1] === 0 && pmax > 0.3 && pmax < 0.6, `peak pitch ${(pmax * 57.3).toFixed(0)} deg`);
  M._ord_bike_remove();
}
// Crash dummies: a 50 km/h run into the track's concrete barrier gives both front-seat dummies a
// frontal crash report with a plausible delta-V and occupant loads.
{
  M._ord_set_preset(1); M._ord_set_aids(1, 1, 1, 1);
  const y = M._ord_crash_lane_y(), xb = M._ord_crash_barrier_x();
  M._ord_create(0, xb - 60, y, 0);
  M._ord_set_occupants(1, 1, 1);
  for (let i = 0; i < 60 * 8; ++i) {
    const v = state()[7];
    M._ord_set_input(0, v < 50 / 3.6 ? 1 : 0, 0, 0, 0, 0); M._ord_advance(1 / 60);
  }
  for (const seat of [0, 1]) {
    const o = new Float64Array(M.HEAPF64.buffer, M._ord_occupant_state(seat), 24);
    const r = o.slice(7, 23);
    check(`crash dummy ${seat} report`, o[6] >= 1 && r[0] === 1 && r[1] === 1 && r[2] * 3.6 > 40 && r[2] * 3.6 < 70 && r[6] > 15 && r[6] < 60 && r[15] >= 0,
          `dV ${(r[2] * 3.6).toFixed(0)} km/h, chest ${r[6].toFixed(0)} g, HIC ${r[7].toFixed(0)}, AIS4+ ${(100 * r[13]).toFixed(1)}%, verdict ${r[15]}`);
  }
}
// Rollover: the proving ground's one-sided ramp lifts the left wheels 0.9 m and drops them. Fast
// enough, a car goes over (the tall, heavy S 600 Guard first); the low, wide F1 car never does.
{
  const y = M._ord_roll_ramp_lane_y();
  const run = (car, kmh) => {
    M._ord_set_preset(1); M._ord_set_aids(1, 1, 1, 1);
    M._ord_create(car, -415, y, 0);
    let minUp = 1, faulted = false;
    for (let i = 0; i < 60 * 25; ++i) {
      const s = state(), v = s[7], past = s[0] > -265, want = kmh / 3.6;
      M._ord_set_input(0, past ? 0 : v < want - 1 ? 1 : v < want ? 0.3 : 0, past || v > want + 1 ? 0.3 : 0, 0, 0, 0); M._ord_advance(1 / 60);
      const s2 = state(); minUp = Math.min(minUp, s2[36]); faulted = faulted || s2[23] !== 0;
    }
    return { minUp, faulted };
  };
  const slow = run(0, 50), fast = run(0, 110), guard = run(9, 75), f1 = run(10, 120);
  check('rollover ramp', !slow.faulted && !fast.faulted && !guard.faulted && !f1.faulted && slow.minUp > 0.7 && fast.minUp < 0 && guard.minUp < 0 && f1.minUp > 0.7,
        `upright min: sports 50 km/h ${slow.minUp.toFixed(2)}, 110 km/h ${fast.minUp.toFixed(2)}; Guard 75 km/h ${guard.minUp.toFixed(2)}; F1 120 km/h ${f1.minUp.toFixed(2)}`);
}
// Breakable walls on the smash lane: a storefront pane gives at a walking pace and barely slows the
// car; a hollow-block wall holds at 15 km/h and is breached at 40 for ~6 km/h; a brick wall holds at
// 30 and is breached at 60 for ~20 km/h. A section that has been broken stays a hole until reset.
{
  const xb = M._ord_crash_barrier_x();
  const smash = (lane, kmh, reset = true) => {
    M._ord_set_preset(1); M._ord_set_aids(1, 1, 1, 1);
    M._ord_create(0, xb - (kmh < 25 ? 30 : 150), M._ord_smash_lane_y(lane), 0);
    if (reset) M._ord_reset_obstacles();
    M._ord_break_events();
    let before = 0, after = null, passedAt = -1, breaks = 0, kept = 0;
    for (let i = 0; i < 60 * 40; ++i) {
      const s = state(), v = s[7], want = kmh / 3.6, approaching = s[0] < xb - 3;
      if (approaching) before = v;
      M._ord_set_input(0, approaching ? (v < want - 0.5 ? 1 : v < want ? 0.3 : 0) : 0, approaching && v > want + 0.5 ? 0.3 : 0, 0, 0, 0); M._ord_advance(1 / 60);
      const n = M._ord_break_events();
      if (n) { breaks += n; kept = new Float64Array(M.HEAPF64.buffer, M._ord_break_buffer(), 8)[7]; }
      if (after === null && state()[0] > xb + 4) { after = state()[7]; passedAt = i; }
      if (passedAt >= 0 && i > passedAt + 60) break;
      if (passedAt < 0 && i > 60 * 10 && state()[7] < 0.3) break;  // bounced off and stopped
    }
    return { before: before * 3.6, after: after === null ? null : after * 3.6, breaks, kept };
  };
  const g = smash(0, 30), cbHold = smash(1, 15), cb = smash(1, 40), brHold = smash(2, 30), br = smash(2, 60), hole = smash(2, 30, false);
  check('glass storefront shatters at 30 km/h and barely slows the car', g.breaks >= 1 && g.after !== null && g.before - g.after < 4,
        `${g.before.toFixed(1)} -> ${g.after?.toFixed(1)} km/h, ${g.breaks} pane(s), kept ${(100 * g.kept).toFixed(0)}% of the energy`);
  check('block wall holds at 15 km/h', cbHold.breaks === 0 && cbHold.after === null, `${cbHold.before.toFixed(1)} km/h, bounced`);
  check('block wall breached at 40 km/h', cb.breaks >= 1 && cb.after !== null && cb.before - cb.after > 3 && cb.before - cb.after < 10,
        `${cb.before.toFixed(1)} -> ${cb.after?.toFixed(1)} km/h, ${cb.breaks} section(s)`);
  check('brick wall holds at 30 km/h', brHold.breaks === 0 && brHold.after === null, `${brHold.before.toFixed(1)} km/h, bounced`);
  check('brick wall breached at 60 km/h', br.breaks >= 1 && br.after !== null && br.before - br.after > 12 && br.before - br.after < 28,
        `${br.before.toFixed(1)} -> ${br.after?.toFixed(1)} km/h, ${br.breaks} section(s)`);
  check('a broken section stays a hole until reset', hole.breaks === 0 && hole.after !== null, `${hole.before.toFixed(1)} -> ${hole.after?.toFixed(1)} km/h through the hole`);
  M._ord_reset_obstacles();
}
// Garage: read the stock setup as JSON, fit slicks and a rear wing, warm the tyres; the readouts
// show the wing's downforce and hot tyres. A malformed setup is refused with a reason.
{
  const text = () => new TextDecoder().decode(M.HEAPU8.slice(M._ord_text(), M._ord_text() + M._ord_text_length()));
  const send = (json) => { const b = new TextEncoder().encode(json); M.HEAPU8.set(b, M._ord_text_buffer(b.length)); return M._ord_apply_setup(); };
  M._ord_set_preset(1); M._ord_set_aids(1, 1, 1, 1);
  M._ord_create(0, 0, 0, 0);
  M._ord_setup_json(1);
  const stock = JSON.parse(text());
  check('garage stock setup JSON', stock.front && stock.rear && stock.gears.length > 0, `${stock.gears.length} gears, balance ${stock.brakeBalance.toFixed(2)}`);
  check('garage rejects bad JSON', send('{"front": ') === 0 && text().length > 0, `"${text()}"`);
  M._ord_set_tyre_warmers(90);
  const ok = send(JSON.stringify({ front: { compound: 'slick', camberDeg: -3 }, rear: { compound: 'slick' }, rearWing: true, wingAngleDeg: 14 }));
  const gs = () => new Float64Array(M.HEAPF64.buffer, M._ord_garage_state(), M._ord_garage_state_size());
  for (let i = 0; i < 60 * 12; ++i) { M._ord_set_input(0, state()[7] < 50 ? 1 : 0.3, 0, 0, 0, 0); M._ord_advance(1 / 60); }
  const g0 = gs();
  check('garage setup applies (slicks, wing, -3 deg)', ok === 1 && g0[5] > 200 && g0[14] * 57.3 < -2 && g0[15] > 60,
        `${(state()[7] * 3.6).toFixed(0)} km/h, wing ${g0[5].toFixed(0)} N, FL camber ${(g0[14] * 57.3).toFixed(1)} deg, FL ${g0[15].toFixed(0)} C (window ${g0[16].toFixed(0)} C)`);
  M._ord_setup_json(0);
  check('garage setup reads back', JSON.parse(text()).rear.compound === 'slick', '');
  M._ord_clear_setup(); M._ord_set_tyre_warmers(-1);
}
// Real-car approximations (6-10): each creates and pulls away cleanly.
for (const [idx, name] of [[6, '911 Carrera S'], [7, '911 Carrera 4S'], [8, 'AMG S 63'], [9, 'S 600 Guard'], [10, 'F1 2026']]) {
  M._ord_set_preset(1); M._ord_set_aids(1, 1, 1, 1);
  const ok = M._ord_create(idx, 0, 0, 0) === 1;
  for (let i = 0; i < 60 * 3; ++i) { M._ord_set_input(0, 1, 0, 0, 0, 0); M._ord_advance(1 / 60); }
  const st = state();
  check(`${name} drives`, ok && st[23] === 0 && st[7] * 3.6 > 50, `${(st[7] * 3.6).toFixed(0)} km/h after 3 s`);
}
// ---- Showcase layer: rewind, shadow car, launches, water, stunt park ----
{
  const A = new Float64Array(M.HEAPF64.buffer, M._ord_test_areas(), 35).slice();
  // rewind: restore a snapshot and replay the same frames: exactly the same state
  M._ord_set_preset(1); M._ord_set_aids(1, 1, 1, 1);
  M._ord_create(6, -400, 0, 0);
  const frames = [];
  let seed = 7; const rnd = () => (seed = (seed * 16807) % 2147483647) / 2147483647;
  for (let i = 0; i < 300; ++i) {
    if (i === 100) M._ord_snap_push();
    const f = { dt: 1 / 60 + (rnd() - 0.5) * 0.006, steer: Math.sin(i / 17) * 0.7, thr: 0.9, brk: i > 220 ? 0.6 : 0, up: i === 150 ? 1 : 0 };
    if (i >= 100) frames.push(f);
    M._ord_set_input(f.steer, f.thr, f.brk, 0, f.up, 0); M._ord_advance(f.dt);
  }
  const ref = state().slice();
  M._ord_snap_restore(0);
  for (const f of frames) { M._ord_set_input(f.steer, f.thr, f.brk, 0, f.up, 0); M._ord_advance(f.dt); }
  const rep = state().slice();
  let diff = 0; for (let i = 0; i < ref.length; ++i) diff = Math.max(diff, Math.abs(ref[i] - rep[i]));
  check('rewind: snapshot + the same frames reproduce the state exactly', diff === 0, `max difference ${diff}`);
  // shadow car: the S 600 Guard on the 911's inputs, heavier, so behind after a launch
  M._ord_create(6, -400, 0, 0); M._ord_launch(-400, 0, 0, 10, 0);
  const shOk = M._ord_shadow_start(9, 1) === 1;
  for (let i = 0; i < 360 * 3; ++i) { M._ord_set_input(0, 1, 0, 0, 0, 0); M._ord_step(1); }
  const sh = new Float64Array(M.HEAPF64.buffer, M._ord_shadow_state(), M._ord_state_header() + 4 * M._ord_state_per_wheel()).slice();
  check('shadow car runs on the same inputs', shOk && sh[0] < state()[0] - 5 && sh[23] === 0, `911 at x ${state()[0].toFixed(1)}, Guard at ${sh[0].toFixed(1)}`);
  M._ord_shadow_stop();
  // crash lab: a launch into the barrier reports one hit and a finished injury report
  const CY = M._ord_crash_lane_y(), BX = M._ord_crash_barrier_x();
  M._ord_launch(BX - 6, CY, 0, 56 / 3.6, 0);
  let hits = 0, hv = 0;
  for (let i = 0; i < 720; ++i) { M._ord_set_input(0, 0, 0, 0, 0, 0); M._ord_step(1); const n = M._ord_hit_events(); if (n) { hits += n; hv = Math.max(hv, new Float64Array(M.HEAPF64.buffer, M._ord_hit_buffer(), 7)[5]); } }
  const occ = new Float64Array(M.HEAPF64.buffer, M._ord_occupant_state(0), 24).slice();
  check('crash lab: 56 km/h into the barrier', hits >= 1 && Math.abs(hv * 3.6 - 56) < 6 && occ[7] === 1, `${hits} hit(s), closing ${(hv * 3.6).toFixed(0)} km/h, delta-V ${(occ[9] * 3.6).toFixed(0)} km/h, HIC15 ${occ[14].toFixed(0)}`);
  // water: steering into the 8 mm pool at 120 km/h hydroplanes (no yaw), at 50 it doesn't
  const yawIn = (kmh) => {
    M._ord_set_preset(2); M._ord_set_aids(0, 0, 0, 1);
    M._ord_create(6, A[2] - 30, A[1], 0); M._ord_launch(A[2] - 30, A[1], 0, kmh / 3.6, 0);
    let y = 0;
    for (let i = 0; i < 360 * 3; ++i) { const s = state(); M._ord_set_input(s[0] > A[2] + 5 ? 0.2 : 0, 0, 0, 0, 0, 0); M._ord_step(1); if (s[0] > A[2] + 5 && s[0] < A[3]) y = Math.max(y, Math.abs(state()[9])); if (s[0] > A[3]) break; }
    return y;
  };
  const y50 = yawIn(50), y120 = yawIn(120);
  check('water: the car steers in the pool at 50 km/h and hydroplanes at 120', y50 > 0.15 && y120 < 0.25 * y50, `peak yaw rate ${y50.toFixed(2)} vs ${y120.toFixed(2)} rad/s`);
  M._ord_set_weather(1);
  const np = M._ord_puddles(0, 0, 150);
  check('weather: rain makes puddles', np > 3, `${np} within 150 m`);
  M._ord_set_weather(0);
  check('weather: dry, none', M._ord_puddles(0, 0, 150) === 0, '');
  // stunt park: the small jump lands upright; the loop, with lane keeping, goes all the way round
  M._ord_set_preset(1); M._ord_set_aids(1, 1, 1, 1);
  M._ord_create(6, -460, A[11], 0); M._ord_launch(-460, A[11], 0, 80 / 3.6, 0);
  let maxZ = 0;
  for (let i = 0; i < 360 * 14; ++i) { const s = state(); M._ord_set_input(-0.2 * (s[1] - A[11]), Math.max(0, Math.min(1, (80 / 3.6 - s[7]) * 0.5)), 0, 0, 0, 0); M._ord_step(1); maxZ = Math.max(maxZ, state()[2]); if (state()[0] > A[19] + A[21] + 20) break; }
  const up = (s) => 1 - 2 * (s[4] * s[4] + s[5] * s[5]);
  check('stunt park: the 1.5 m kicker at 80 km/h flies and lands upright', maxZ > 2 && up(state()) > 0.9 && state()[0] > A[19] + A[21], `peak ${maxZ.toFixed(1)} m, ends at x ${state()[0].toFixed(0)}`);
  const nP = M._ord_loop_profile_count(), prof = new Float64Array(M.HEAPF64.buffer, M._ord_loop_profile(), 4 * nP).slice();
  const len = prof[4 * (nP - 1) + 3]; let top = 0; for (let i = 0; i < nP; ++i) top = Math.max(top, prof[4 * i + 1]);
  const x0 = -420, y0 = A[30] - (A[29] - x0) * Math.tan(A[34]);
  M._ord_create(6, x0, y0, A[34]); M._ord_launch(x0, y0, A[34], 105 / 3.6, 0);
  let sPrev = 0, zMax = 0, round = false;
  for (let i = 0; i < 360 * 14; ++i) {
    const s = state();
    let steer = 0;
    if (s[0] > A[29] - 2 && s[1] > A[30] - 4) {
      let best = 1e9, sa = -1;
      for (let k = 0; k < nP; ++k) { const sk = prof[4 * k + 3]; if (Math.abs(sk - sPrev) > 6) continue; const dx = A[29] + prof[4 * k] - s[0], dz = prof[4 * k + 1] - s[2], d = dx * dx + dz * dz; if (d < best) { best = d; sa = sk; } }
      if (sa >= 0) {
        sPrev = sa;
        const yc = A[30] + A[32] * sa / len;
        const r10 = 2 * (s[4] * s[5] + s[3] * s[6]), r11 = 1 - 2 * (s[4] * s[4] + s[6] * s[6]);
        const vy = r10 * s[7] + r11 * s[8];
        steer = Math.max(-0.3, Math.min(0.3, 0.25 * (yc - s[1]) - 0.25 * (vy - Math.abs(s[7]) * Math.sin(A[34]))));
      }
    }
    M._ord_set_input(steer, s[0] < A[29] - 5 ? Math.max(0, Math.min(1, (105 / 3.6 - s[7]) * 0.5)) : 1, 0, 0, 0, 0); M._ord_step(1);
    zMax = Math.max(zMax, state()[2]);
    if (zMax > top - 2 && sPrev > len - 3 && state()[2] < 1 && up(state()) > 0.9) { round = true; break; }
  }
  check('stunt park: the clothoid loop goes all the way round', round && state()[23] === 0, `${top.toFixed(1)} m loop, top reached ${zMax.toFixed(1)} m`);
}
// damage: the node/beam crash structure on the barrier, a storefront it breaks, repair
{
  const D = () => new Float64Array(M.HEAPF64.buffer, M._ord_damage_state(), 22 + 6 * 8).slice();
  const occ = () => new Float64Array(M.HEAPF64.buffer, M._ord_occupant_state(0), 24).slice();
  const CY = M._ord_crash_lane_y(), BX = M._ord_crash_barrier_x();
  M._ord_set_preset(2); M._ord_set_aids(0, 0, 0, 1);
  M._ord_create(6, BX - 30, CY, 0);
  M._ord_set_damage(1);
  check('damage: structure on', D()[0] === 1 && M._ord_cage_count() > 50 && M._ord_cage_beam_count() > 200, `${M._ord_cage_count()} nodes, ${M._ord_cage_beam_count()} beams`);
  M._ord_launch(BX - 6, CY, 0, 56 / 3.6, 0);
  let peak = 0, prev = 56 / 3.6;
  for (let i = 0; i < 720; ++i) { M._ord_set_input(0, 0, 0, 0, 0, 0); M._ord_step(1); const v = state()[7]; peak = Math.max(peak, (prev - v) * 360 / 9.81); prev = v; }
  const d = D(), o = occ();
  check('damage: 56 km/h into the barrier crushes ~0.5 m; the dummies ride the structure\'s pulse', d[3] > 0.35 && d[3] < 0.8 && peak > 15 && peak < 50 && o[7] === 1 && o[8] === 1 && state()[23] === 0,
        `crush ${d[3].toFixed(2)} m, peak ${peak.toFixed(0)} g, report: zone ${o[8]}, delta-V ${(o[9] * 3.6).toFixed(0)} km/h, chest ${o[13].toFixed(0)} g, radiator ${d[11].toFixed(2)}`);
  const nodes = new Float64Array(M.HEAPF64.buffer, M._ord_cage_nodes(), 8 * M._ord_cage_count()).slice();
  let moved = 0; for (let i = 0; i < M._ord_cage_count(); ++i) moved = Math.max(moved, nodes[8 * i] - nodes[8 * i + 3]);
  check('damage: the node buffer shows the crushed shape', Math.abs(moved - d[3]) < 0.05, `front node moved back ${moved.toFixed(2)} m`);
  M._ord_repair();
  check('damage: repair', D()[3] === 0 && D()[11] === 0, '');
  // the glass storefront: the structure's contact work breaks it, the car goes through
  M._ord_reset_obstacles();
  const SY = M._ord_smash_lane_y(0);
  M._ord_launch(BX - 8, SY, 0, 30 / 3.6, 0);
  let broke = 0;
  for (let i = 0; i < 720; ++i) { M._ord_set_input(0, 0, 0, 0, 0, 0); M._ord_step(1); broke += M._ord_break_events(); }
  check('damage: the structure breaks the glass storefront and carries on', broke >= 1 && state()[0] > BX + 1 && state()[7] > 4, `${broke} pane(s), ${(state()[7] * 3.6).toFixed(0)} km/h after`);
  // a scrape along the barrier at 4 degrees is not a crash: no report for the dummies
  M._ord_set_preset(1); M._ord_create(6, BX - 30, CY, 0); M._ord_set_damage(1);
  const reports0 = occ()[6];
  M._ord_launch(BX - 1.6, CY - 12, Math.PI / 2 - 4 * Math.PI / 180, 50 / 3.6, 0);
  for (let i = 0; i < 360 * 3; ++i) { M._ord_set_input(0, 0.3, 0, 0, 0, 0); M._ord_step(1); }
  for (let i = 0; i < 360; ++i) { M._ord_set_input(0, 0, 0, 0, 0, 0); M._ord_step(1); }
  check('damage: scraping a wall is not a crash', occ()[6] === reports0 && state()[23] === 0, `${occ()[6] - reports0} report(s)`);
  // the stunt park with damage on: the kicker lands clean, the loop goes round (no launch)
  const A2 = new Float64Array(M.HEAPF64.buffer, M._ord_test_areas(), 35).slice();
  M._ord_set_preset(1); M._ord_set_aids(1, 1, 1, 1); M._ord_create(6, -460, A2[11], 0); M._ord_set_damage(1);
  M._ord_launch(-460, A2[11], 0, 80 / 3.6, 0);
  for (let i = 0; i < 360 * 14; ++i) { const s = state(); M._ord_set_input(-0.2 * (s[1] - A2[11]), Math.max(0, Math.min(1, (80 / 3.6 - s[7]) * 0.5)), 0, 0, 0, 0); M._ord_step(1); }
  check('damage: the kicker lands without damage', D()[9] < 500 && state()[23] === 0, `${D()[9].toFixed(0)} J absorbed`);
  {
    const nP = M._ord_loop_profile_count(), prof = new Float64Array(M.HEAPF64.buffer, M._ord_loop_profile(), 4 * nP).slice();
    const len = prof[4 * (nP - 1) + 3];
    const x0 = -420, y0 = A2[30] - (A2[29] - x0) * Math.tan(A2[34]);
    M._ord_create(6, x0, y0, A2[34]); M._ord_set_damage(1); M._ord_launch(x0, y0, A2[34], 105 / 3.6, 0);
    let sPrev = 0, zMax = 0;
    for (let i = 0; i < 360 * 14; ++i) {
      const s = state(); let steer = 0;
      if (s[0] > A2[29] - 2 && s[1] > A2[30] - 4) {
        let best = 1e9, sa = -1;
        for (let k = 0; k < nP; ++k) { const sk = prof[4 * k + 3]; if (Math.abs(sk - sPrev) > 6) continue; const dx = A2[29] + prof[4 * k] - s[0], dz = prof[4 * k + 1] - s[2], dd = dx * dx + dz * dz; if (dd < best) { best = dd; sa = sk; } }
        if (sa >= 0) { sPrev = sa; const yc = A2[30] + A2[32] * sa / len; const r10 = 2 * (s[4] * s[5] + s[3] * s[6]), r11 = 1 - 2 * (s[4] * s[4] + s[6] * s[6]); const vy = r10 * s[7] + r11 * s[8]; steer = Math.max(-0.3, Math.min(0.3, 0.25 * (yc - s[1]) - 0.25 * (vy - Math.abs(s[7]) * Math.sin(A2[34])))); }
      }
      M._ord_set_input(steer, s[0] < A2[29] - 5 ? Math.max(0, Math.min(1, (105 / 3.6 - s[7]) * 0.5)) : 1, 0, 0, 0, 0); M._ord_step(1);
      zMax = Math.max(zMax, state()[2]);
    }
    check('damage: the loop with the crash structure on (no launch, no broken parts)', zMax > 30 && zMax < 40 && D()[9] < 5000 && [0, 1, 2, 3].every((w) => !D()[22 + 6 * w + 3]) && state()[23] === 0,
          `top ${zMax.toFixed(1)} m, ${D()[9].toFixed(0)} J (the overhangs scrape the entry)`);
  }
  M._ord_set_damage(0);
  check('damage: off again', D()[0] === 0 && M._ord_cage_count() === 0, '');
}
console.log(failures ? `${failures} failure(s)` : 'PASSED');
process.exit(failures ? 1 : 0);
