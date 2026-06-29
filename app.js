/*
 * Rekalibrasi Control Lab — standalone, dependency-free build.
 *
 * Ported from the original Design-Component (DC) source in design/. The
 * simulation core (step / plantStep / metrics / draw / auto-tune / export) is
 * preserved verbatim from the original DCLogic class; only the React/DC
 * rendering layer has been replaced with direct DOM binding so the app runs
 * as a plain static page on any web host.
 */
(function () {
  'use strict';

  // ---------------- state ----------------
  var state = {
    ctype: 'PID', mode: 'auto', dir: 'direct',
    Kp: 2.2, Ki: 0.32, Kd: 1.1,
    sp: 50, manual: 25, dt: 0.1, outMin: 0, outMax: 100,
    plant: 'fopdt', K: 1, tau: 12, theta: 2.5, zeta: 0.7, wn: 0.4, y0: 0,
    distOn: false, distMag: 15, noiseOn: false, noiseStd: 0.6, slewOn: false, slew: 120,
    running: false, started: false, speed: 2, dfiltN: 8, awOn: true,
    method: 'imc', learn: 'Kp', windowSec: 60
  };

  // simulation working vars
  var S, last, buf, hist, M, ref = null, _sp = null;
  var acc = 0, lastTs = null, lastUi = 0;

  function clamp(v, a, b) { return v < a ? a : (v > b ? b : v); }
  function fmt(v, d) { if (v == null || !isFinite(v)) return '—'; return v.toFixed(d); }
  function gauss() {
    if (_sp != null) { var s = _sp; _sp = null; return s; }
    var u = 0, v = 0; while (u === 0) u = Math.random(); while (v === 0) v = Math.random();
    var m = Math.sqrt(-2 * Math.log(u)); _sp = m * Math.sin(2 * Math.PI * v); return m * Math.cos(2 * Math.PI * v);
  }

  function resetMetrics(start) {
    M = {
      t0: S ? S.simT : 0, start: start, peakVal: start, peakTime: 0,
      iae: 0, ise: 0, itae: 0, effort: 0, maxPv: start, minPv: start, maxMv: 0,
      t10: null, t90: null, settle: 0
    };
  }

  function initInstance() {
    buf = { t: [], sp: [], pv: [], mv: [], err: [] };
    hist = [];
    S = { y: state.y0, y2: 0, I: 0, prevPv: state.y0, dFilt: 0, prevMv: 0, delay: [], simT: 0, distEnd: 0 };
    last = { pv: state.y0, mv: 0, err: 0, P: 0, I: 0, D: 0, sat: false };
    ref = null;
    resetMetrics(state.y0);
    acc = 0; lastTs = null; lastUi = 0;
  }

  // ---------------- simulation step ----------------
  function step(dt) {
    var st = state;
    var meas = S.y + (st.noiseOn ? gauss() * st.noiseStd : 0);
    var actSign = st.dir === 'direct' ? 1 : -1;
    var error = actSign * (st.sp - meas);
    var P = 0, I = S.I, D = 0, out, outSat, sat = false;

    if (st.mode === 'manual') {
      out = st.manual; outSat = clamp(out, st.outMin, st.outMax);
      S.I = outSat; S.prevPv = meas; S.dFilt = 0; I = outSat;
    } else {
      var Kp = st.Kp, Ki = st.ctype === 'P' ? 0 : st.Ki, Kd = st.ctype === 'PID' ? st.Kd : 0;
      P = Kp * error;
      var dMeas = (meas - S.prevPv) / dt;
      var Tf = (Kd > 0 && Kp > 0) ? Kd / (Kp * st.dfiltN) : 0;
      var a = Tf > 0 ? dt / (Tf + dt) : 1;
      S.dFilt = S.dFilt + a * ((-actSign * Kd * dMeas) - S.dFilt);
      D = S.dFilt;
      var Ic = S.I + Ki * error * dt;
      out = P + Ic + D; outSat = clamp(out, st.outMin, st.outMax);
      if (out !== outSat) { sat = true; if (st.awOn) { Ic = S.I; out = P + Ic + D; outSat = clamp(out, st.outMin, st.outMax); } }
      S.I = Ic; I = Ic; S.prevPv = meas;
    }

    var mv = outSat;
    if (st.slewOn) { var md = st.slew * dt; mv = clamp(mv, S.prevMv - md, S.prevMv + md); }
    S.prevMv = mv;

    var dist = st.distOn ? st.distMag : 0;
    if (S.distEnd > S.simT) dist += st.distMag;
    var uIn = mv + dist;

    plantStep(dt, uIn);

    S.simT += dt;
    record(S.simT, st.sp, meas, mv, st.sp - meas);
    accMetrics(S.simT, st.sp, meas, mv, dt);
    last = { pv: meas, mv: mv, err: st.sp - meas, P: P, I: I, D: D, sat: sat };
  }

  function plantStep(dt, u) {
    var st = state, K = st.K;
    function delayed(val) {
      if (st.theta <= 0) return val;
      var n = Math.max(1, Math.round(st.theta / dt));
      if (S.delay.length !== n) { S.delay = new Array(n).fill(val); }
      S.delay.push(val); return S.delay.shift();
    }
    if (st.plant === 'first') { S.y += dt * (K * u - S.y) / st.tau; }
    else if (st.plant === 'fopdt') { var du = delayed(u); S.y += dt * (K * du - S.y) / st.tau; }
    else if (st.plant === 'integrating') { S.y += dt * K * u * 0.1; }
    else if (st.plant === 'second') { var w = st.wn, z = st.zeta; S.y2 += dt * (K * w * w * u - 2 * z * w * S.y2 - w * w * S.y); S.y += dt * S.y2; }
    else if (st.plant === 'sopdt') { var du2 = delayed(u); var w2 = st.wn, z2 = st.zeta; S.y2 += dt * (K * w2 * w2 * du2 - 2 * z2 * w2 * S.y2 - w2 * w2 * S.y); S.y += dt * S.y2; }
  }

  function record(t, sp, pv, mv, err) {
    var b = buf; b.t.push(t); b.sp.push(sp); b.pv.push(pv); b.mv.push(mv); b.err.push(err);
    var CAP = 3000;
    if (b.t.length > CAP) { b.t.shift(); b.sp.shift(); b.pv.shift(); b.mv.shift(); b.err.shift(); }
    hist.push([t, sp, pv, mv, err]);
    if (hist.length > 60000) hist.shift();
  }

  function accMetrics(t, sp, pv, mv, dt) {
    var e = sp - pv, ae = Math.abs(e), rel = t - M.t0;
    M.iae += ae * dt; M.ise += e * e * dt; M.itae += rel * ae * dt; M.effort += Math.abs(mv) * dt;
    M.maxPv = Math.max(M.maxPv, pv); M.minPv = Math.min(M.minPv, pv); M.maxMv = Math.max(M.maxMv, mv);
    var span = sp - M.start;
    if (span >= 0) { if (pv > M.peakVal) { M.peakVal = pv; M.peakTime = rel; } }
    else { if (pv < M.peakVal) { M.peakVal = pv; M.peakTime = rel; } }
    if (span !== 0) { var f = (pv - M.start) / span; if (M.t10 == null && f >= 0.1) M.t10 = rel; if (M.t90 == null && f >= 0.9) M.t90 = rel; }
    var band = Math.max(1e-9, 0.02 * Math.abs(span));
    if (Math.abs(pv - sp) > band) M.settle = rel;
  }

  // ---------------- reset / arm ----------------
  // Re-initialize the running state from the current start conditions (y0 etc.)
  // without touching the started/running flags — shared by RESET, START, STEP,
  // and the live initial-condition preview while the sim is still armed.
  function doReset() {
    var y = state.y0;
    buf = { t: [], sp: [], pv: [], mv: [], err: [] };
    hist = [];
    S = { y: y, y2: 0, I: 0, prevPv: y, dFilt: 0, prevMv: 0, delay: [], simT: 0, distEnd: 0 };
    last = { pv: y, mv: 0, err: 0, P: 0, I: 0, D: 0, sat: false };
    resetMetrics(y); acc = 0; ref = null;
    if (el.clrRefBtn) el.clrRefBtn.style.display = 'none';
    draw(); syncReadouts();
  }

  // ---------------- main loop ----------------
  function tickfn() {
    var ts = performance.now();
    if (lastTs == null) lastTs = ts;
    var real = (ts - lastTs) / 1000; lastTs = ts;
    real = Math.min(real, 0.5);
    if (state.running) {
      acc += real * state.speed;
      var dt = state.dt, n = 0;
      while (acc >= dt && n < 20000) { step(dt); acc -= dt; n++; }
    }
    draw();
    if (ts - lastUi > 100) { lastUi = ts; syncReadouts(); }
  }

  // ---------------- theme ----------------
  function currentTheme() {
    return document.documentElement.getAttribute('data-theme') === 'light' ? 'light' : 'dark';
  }
  function setThemeLabel() {
    if (el.themeBtn) el.themeBtn.textContent = currentTheme() === 'light' ? '☾ DARK' : '☀ LIGHT';
  }
  function toggleTheme() {
    var next = currentTheme() === 'light' ? 'dark' : 'light';
    document.documentElement.setAttribute('data-theme', next);
    try { localStorage.setItem('rkl-theme', next); } catch (e) { /* ignore */ }
    setThemeLabel();
    draw(); // re-render the scope with the theme-matched palette
  }
  // Scope (canvas) palette per theme — canvas colors aren't reachable by CSS.
  function scopePalette() {
    if (currentTheme() === 'light') {
      return { bg: '#f7f9fb', grid: '#d4dde6', gridV: '#e3e9ef', axisL: '#5b6b7a', axisR: '#0e7490',
        sp: '#d97706', pv: '#059669', mv: '#0e7490', err: '#94a3b8', ref: 'rgba(100,116,139,.5)', glow: 0 };
    }
    return { bg: '#06090e', grid: '#16323c', gridV: '#0f2129', axisL: '#6b8494', axisR: '#1f93b0',
      sp: '#f5a623', pv: '#34d399', mv: '#22d3ee', err: '#7c8a9c', ref: 'rgba(148,163,184,.55)', glow: 1 };
  }

  // ---------------- canvas ----------------
  function draw() {
    var cv = el.scope; if (!cv) return;
    var PAL = scopePalette();
    var dpr = window.devicePixelRatio || 1;
    var w = cv.clientWidth || 700, h = cv.clientHeight || 352;
    if (cv.width !== Math.round(w * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
    var ctx = cv.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = PAL.bg; ctx.fillRect(0, 0, w, h);
    var padL = 42, padR = 44, padT = 14, padB = 24, pw = w - padL - padR, ph = h - padT - padB;
    var st = state, b = buf;
    var winS = st.windowSec;
    var tEnd = b.t.length ? b.t[b.t.length - 1] : 0;
    var tStart = Math.max(0, tEnd - winS);

    var lo = Infinity, hi = -Infinity;
    for (var i = 0; i < b.t.length; i++) { if (b.t[i] < tStart) continue; var p = b.pv[i], s = b.sp[i]; if (p < lo) lo = p; if (p > hi) hi = p; if (s < lo) lo = s; if (s > hi) hi = s; }
    if (!isFinite(lo)) { lo = 0; hi = st.sp || 100; }
    if (hi - lo < 5) { var mid = (hi + lo) / 2; lo = mid - 2.5; hi = mid + 2.5; }
    var pad = (hi - lo) * 0.12; lo -= pad; hi += pad;
    var rMin = st.outMin, rMax = st.outMax;

    var X = function (t) { return padL + ((t - tStart) / winS) * pw; };
    var YL = function (v) { return padT + (1 - (v - lo) / (hi - lo)) * ph; };
    var YR = function (v) { return padT + (1 - (v - rMin) / (rMax - rMin)) * ph; };

    ctx.strokeStyle = PAL.grid; ctx.lineWidth = 1; ctx.font = "500 9px 'IBM Plex Mono',monospace";
    ctx.textBaseline = 'middle';
    for (var g = 0; g <= 4; g++) {
      var y = padT + (g / 4) * ph; ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(padL + pw, y); ctx.stroke();
      var lv = hi - (g / 4) * (hi - lo); ctx.fillStyle = PAL.axisL; ctx.textAlign = 'right'; ctx.fillText(lv.toFixed(0), padL - 6, y);
      var rv = rMax - (g / 4) * (rMax - rMin); ctx.fillStyle = PAL.axisR; ctx.textAlign = 'left'; ctx.fillText(rv.toFixed(0), padL + pw + 6, y);
    }
    ctx.textAlign = 'center'; ctx.textBaseline = 'top';
    for (var gx = 0; gx <= 6; gx++) {
      var x = padL + (gx / 6) * pw; ctx.strokeStyle = PAL.gridV; ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + ph); ctx.stroke();
      var tv = tStart + (gx / 6) * winS; if (tEnd > 0) { ctx.fillStyle = PAL.axisL; ctx.fillText(tv.toFixed(0) + 's', x, padT + ph + 6); }
    }
    ctx.fillStyle = PAL.axisL; ctx.textAlign = 'left'; ctx.fillText('PV', padL, padT - 1);
    ctx.fillStyle = PAL.axisR; ctx.textAlign = 'right'; ctx.fillText('MV%', padL + pw, padT - 1);

    if (b.t.length < 2) { return; }

    function trace(arr, Y, color, width, glow) {
      ctx.beginPath(); var started = false;
      for (var i = 0; i < b.t.length; i++) {
        if (b.t[i] < tStart) continue; var x = X(b.t[i]); var y = Y(arr[i]);
        if (y < padT) y = padT; if (y > padT + ph) y = padT + ph;
        if (!started) { ctx.moveTo(x, y); started = true; } else ctx.lineTo(x, y);
      }
      ctx.strokeStyle = color; ctx.lineWidth = width; ctx.lineJoin = 'round';
      ctx.shadowBlur = glow; ctx.shadowColor = color; ctx.stroke(); ctx.shadowBlur = 0;
    }

    if (ref) {
      ctx.beginPath(); var st2 = false;
      for (var r = 0; r < ref.off.length; r++) {
        var off = ref.off[r]; if (off > winS) continue;
        var rx = padL + (1 - off / winS) * pw; var ry = YL(ref.pv[r]); if (ry < padT) ry = padT; if (ry > padT + ph) ry = padT + ph;
        if (!st2) { ctx.moveTo(rx, ry); st2 = true; } else ctx.lineTo(rx, ry);
      }
      ctx.strokeStyle = PAL.ref; ctx.lineWidth = 1.4; ctx.setLineDash([4, 4]); ctx.stroke(); ctx.setLineDash([]);
    }

    trace(b.mv, YR, PAL.mv, 1.4, 5 * PAL.glow);
    trace(b.err, YL, PAL.err, 1, 0);
    trace(b.sp, YL, PAL.sp, 1.8, 6 * PAL.glow);
    trace(b.pv, YL, PAL.pv, 1.8, 7 * PAL.glow);
  }

  // ---------------- auto-tune ----------------
  function identifyFOPDT() {
    var st = state, K = Math.abs(st.K) || 0.01, tau = st.tau, theta = st.theta;
    if (st.plant === 'first') { theta = Math.max(theta, 0.5); }
    else if (st.plant === 'fopdt') { theta = Math.max(theta, 0.001); }
    else if (st.plant === 'second' || st.plant === 'sopdt') {
      var z = st.zeta, w = st.wn; tau = Math.max(0.5, (2 * z) / w);
      theta = Math.max(st.plant === 'sopdt' ? st.theta : 0, 0.5 / w);
    }
    else if (st.plant === 'integrating') { return { K: K, tau: 0, theta: Math.max(theta, 1), integ: true }; }
    return { K: K, tau: tau, theta: Math.max(theta, 0.001), integ: false };
  }
  function computeTune() {
    var st = state, f = identifyFOPDT();
    if (f.integ) return { Kp: null, Ki: null, Kd: null, Ti: null, Td: null, f: f, note: 'Integrating process — open-loop reaction-curve rules do not apply. Tune manually or with an IMC integrator rule.' };
    var K = f.K, tau = f.tau, theta = f.theta, Kp, Ti, Td;
    if (st.method === 'zn') { Kp = 1.2 * tau / (K * theta); Ti = 2 * theta; Td = 0.5 * theta; }
    else if (st.method === 'cc') {
      var r = theta / tau;
      Kp = (1 / K) * (tau / theta) * (4 / 3 + r / 4);
      Ti = theta * (32 + 6 * r) / (13 + 8 * r);
      Td = theta * 4 / (11 + 2 * r);
    } else {
      var lam = Math.max(0.5 * theta, 0.25 * tau);
      Kp = (2 * tau + theta) / (K * (2 * lam + theta)); Ti = tau + theta / 2; Td = tau * theta / (2 * tau + theta);
    }
    var ct = st.ctype;
    var Ki = ct === 'P' ? 0 : Kp / Ti;
    var Kd = ct === 'PID' ? Kp * Td : 0;
    return { Kp: Kp, Ki: Ki, Kd: Kd, Ti: Ti, Td: Td, f: f, note: 'Educational approximation — validate before any field use.' };
  }

  // ---------------- metrics ----------------
  function metrics() {
    var sp = state.sp, span = sp - M.start;
    var over = span !== 0 ? Math.max(0, (M.peakVal - sp) / span * 100) : 0;
    var rise = (M.t10 != null && M.t90 != null) ? (M.t90 - M.t10) : null;
    var sse = sp - (last ? last.pv : M.start);
    return {
      overshoot: over, riseTime: rise, peakTime: M.peakTime, settlingTime: M.settle,
      steadyStateError: sse, iae: M.iae, ise: M.ise, itae: M.itae, controlEffort: M.effort,
      maxPV: M.maxPv, minPV: M.minPv, maxMV: M.maxMv
    };
  }

  // ---------------- export ----------------
  function download(name, content, type) {
    var b = new Blob([content], { type: type }); var u = URL.createObjectURL(b);
    var a = document.createElement('a'); a.href = u; a.download = name; document.body.appendChild(a); a.click();
    document.body.removeChild(a); setTimeout(function () { URL.revokeObjectURL(u); }, 3000);
  }
  function exportJSON() {
    var st = state, m = metrics();
    var payload = {
      tool: 'Rekalibrasi Control Lab', generated: new Date().toISOString(),
      disclaimer: 'Education / training only. Do not apply directly to real processes without proper engineering validation.',
      controller: { type: st.ctype, mode: st.mode, direction: st.dir, Kp: st.Kp, Ki: st.Ki, Kd: st.Kd, setpoint: st.sp, dt: st.dt, outMin: st.outMin, outMax: st.outMax, antiWindup: st.awOn },
      plant: { model: st.plant, K: st.K, tau: st.tau, theta: st.theta, zeta: st.zeta, wn: st.wn, y0: st.y0 },
      conditions: { disturbance: st.distOn ? st.distMag : 0, sensorNoiseStd: st.noiseOn ? st.noiseStd : 0, slewLimit: st.slewOn ? st.slew : null },
      metrics: m,
      samples: hist.map(function (r) { return { time: +r[0].toFixed(3), setpoint: r[1], processVariable: +r[2].toFixed(4), controllerOutput: +r[3].toFixed(4), error: +r[4].toFixed(4) }; })
    };
    download('rekalibrasi-report.json', JSON.stringify(payload, null, 2), 'application/json');
  }
  function exportCSV() {
    var s = 'time,setpoint,process_variable,controller_output,error\n';
    for (var i = 0; i < hist.length; i++) { var r = hist[i]; s += r[0].toFixed(3) + ',' + r[1] + ',' + r[2].toFixed(5) + ',' + r[3].toFixed(5) + ',' + r[4].toFixed(5) + '\n'; }
    download('rekalibrasi-data.csv', s, 'text/csv');
  }

  // ---------------- static content ----------------
  var eqs = {
    first: 'dy/dt = (K·u − y)/τ', fopdt: 'first order with transport delay θ',
    second: 'ÿ + 2ζωnẏ + ωn²y = Kωn²u', sopdt: 'second order with transport delay θ',
    integrating: 'dy/dt = K·u  (no self-regulation)'
  };
  var learnMap = {
    Kp: ['Proportional gain (Kp)', 'Acts on the present error. Raising Kp speeds up the response and shrinks steady-state error, but too much causes overshoot and oscillation. It is the backbone of almost every loop.'],
    Ki: ['Integral gain (Ki)', 'Acts on accumulated past error. It drives steady-state error to zero, but too much integral action slows the loop and causes overshoot and windup. Increase it to remove offset.'],
    Kd: ['Derivative gain (Kd)', 'Acts on the predicted future error (rate of change). It adds damping and reduces overshoot, but amplifies sensor noise — always pair it with a derivative filter. Use sparingly.'],
    overshoot: ['Overshoot', 'How far PV swings past the setpoint on the first peak, as a percent of the step. High Kp or Ki increases it; Kd and a slower setpoint reduce it. <10% is usually a good target.'],
    settling: ['Settling time', 'Time for PV to stay within a tolerance band (here ±2%) around the setpoint. A well-tuned loop settles quickly without lingering oscillation.'],
    sserror: ['Steady-state error', 'The residual gap between setpoint and PV once everything settles. Pure proportional control leaves offset; adding integral action removes it.'],
    windup: ['Integral windup', 'When the output saturates, the integrator keeps accumulating and the loop overshoots badly on recovery. Anti-windup freezes or back-calculates the integral while saturated — toggle it to see the effect.'],
    noise: ['Derivative noise', 'Derivative action differentiates the measurement, so sensor noise gets amplified into the output. Enable sensor noise with high Kd to see the valve chatter, then add filtering.'],
    deadtime: ['Dead time (θ)', 'Pure transport delay before the process responds. It is the single hardest dynamic to control — it limits how aggressive your gains can be. The θ/τ ratio drives every tuning rule.']
  };

  // ---------------- slider + metric factories ----------------
  var sliders = {}; // key -> {input, disp, dec}
  function makeSlider(cfg, container) {
    var wrap = document.createElement('div'); wrap.className = 'ps';
    var row = document.createElement('div'); row.className = 'ps-row';
    var lab = document.createElement('span'); lab.className = 'ps-label'; lab.title = cfg.hint; lab.textContent = cfg.label;
    var val = document.createElement('span'); val.className = 'ps-val';
    var valTxt = document.createTextNode('');
    var unit = document.createElement('span'); unit.className = 'ps-unit'; unit.textContent = cfg.unit || '';
    val.appendChild(valTxt); val.appendChild(unit);
    row.appendChild(lab); row.appendChild(val);
    var input = document.createElement('input');
    input.type = 'range'; input.min = cfg.min; input.max = cfg.max; input.step = cfg.step;
    input.value = state[cfg.key];
    input.addEventListener('input', function (e) {
      var v = parseFloat(e.target.value);
      state[cfg.key] = v;
      valTxt.nodeValue = fmt(v, cfg.dec);
      if (cfg.onChange) cfg.onChange(v);
    });
    wrap.appendChild(row); wrap.appendChild(input);
    container.appendChild(wrap);
    valTxt.nodeValue = fmt(state[cfg.key], cfg.dec);
    sliders[cfg.key] = { input: input, disp: valTxt, dec: cfg.dec, wrap: wrap };
  }

  var metricDefs = [
    { key: 'overshoot', label: 'Overshoot', unit: '%', dec: 1, tone: 'over' },
    { key: 'riseTime', label: 'Rise time', unit: 's', dec: 1, tone: '' },
    { key: 'peakTime', label: 'Peak time', unit: 's', dec: 1, tone: '' },
    { key: 'settlingTime', label: 'Settling 2%', unit: 's', dec: 1, tone: '' },
    { key: 'steadyStateError', label: 'SS error', unit: 'PV', dec: 2, tone: 'sse' },
    { key: 'controlEffort', label: 'Ctrl effort', unit: '', dec: 0, tone: '' },
    { key: 'iae', label: 'IAE', unit: '', dec: 1, tone: '' },
    { key: 'ise', label: 'ISE', unit: '', dec: 1, tone: '' },
    { key: 'itae', label: 'ITAE', unit: '', dec: 0, tone: '' },
    { key: 'maxPV', label: 'Max PV', unit: '', dec: 1, tone: '#1b1a17' }
  ];
  var metricRefs = {};
  function buildMetrics(grid) {
    metricDefs.forEach(function (d) {
      var box = document.createElement('div'); box.className = 'ms';
      var lab = document.createElement('span'); lab.className = 'ms-label'; lab.textContent = d.label;
      var val = document.createElement('span'); val.className = 'ms-val';
      var t = document.createTextNode('—');
      var unit = document.createElement('span'); unit.className = 'ms-unit'; unit.textContent = d.unit;
      val.appendChild(t); val.appendChild(unit);
      box.appendChild(lab); box.appendChild(val);
      grid.appendChild(box);
      metricRefs[d.key] = { txt: t, val: val, def: d };
    });
  }

  // ---------------- DOM cache ----------------
  var el = {};
  function $(id) { return document.getElementById(id); }

  // ---------------- sync (discrete UI) ----------------
  function syncControls() {
    var s = state;
    // segmented: ctype
    document.querySelectorAll('[data-ctype]').forEach(function (b) {
      b.className = 'seg ' + (b.getAttribute('data-ctype') === s.ctype ? 'seg-on' : 'seg-off');
    });
    document.querySelectorAll('[data-mode]').forEach(function (b) {
      b.className = 'seg ' + (b.getAttribute('data-mode') === s.mode ? 'seg-on' : 'seg-off');
    });
    document.querySelectorAll('[data-dir]').forEach(function (b) {
      b.className = 'seg ' + (b.getAttribute('data-dir') === s.dir ? 'seg-on' : 'seg-off');
    });
    // toggles
    setToggle('awOn', s.awOn, ['OFF', 'ON']);
    setToggle('distOn', s.distOn, ['OFF', 'ON']);
    setToggle('noiseOn', s.noiseOn, ['OFF', 'ON']);
    setToggle('slewOn', s.slewOn, ['OFF', 'ON']);

    // conditional sliders
    show(sliders.Ki.wrap, s.ctype === 'PI' || s.ctype === 'PID');
    show(sliders.Kd.wrap, s.ctype === 'PID');
    show(sliders.manual.wrap, s.mode === 'manual');
    show(sliders.tau.wrap, s.plant === 'first' || s.plant === 'fopdt');
    show(sliders.theta.wrap, s.plant === 'fopdt' || s.plant === 'sopdt');
    show(sliders.zeta.wrap, s.plant === 'second' || s.plant === 'sopdt');
    show(sliders.wn.wrap, s.plant === 'second' || s.plant === 'sopdt');

    el.plantEq.textContent = eqs[s.plant];
    el.plantSel.value = s.plant;
    el.methodSel.value = s.method;
    el.learnSel.value = s.learn;
    el.speedSel.value = String(s.speed);
    el.windowSel.value = String(s.windowSec);
    el.winSec.textContent = s.windowSec;

    // status
    var armed = !s.started;
    var statusText = s.mode === 'manual' ? 'MANUAL' : (armed ? 'READY' : (s.running ? 'RUNNING' : 'PAUSED'));
    var statusDot = s.mode === 'manual' ? '#ea8a0c' : (armed ? '#ea8a0c' : (s.running ? '#34d399' : '#94a3b8'));
    el.statusText.textContent = statusText;
    el.statusDot.style.background = statusDot;
    el.statusDot.style.boxShadow = '0 0 8px ' + statusDot + ', 0 0 2px ' + statusDot;
    el.statusDot.style.animation = (s.running && s.started && s.mode !== 'manual') ? 'recpulse 1.4s ease-in-out infinite' : '';
    el.runBtn.textContent = armed ? '▶ START' : (s.running ? '⏸ PAUSE' : '▶ RESUME');
    if (s.running) { // PAUSE — amber caution
      el.runBtn.style.background = 'linear-gradient(180deg,#caa53e,#9a7a1c)';
      el.runBtn.style.color = '#1c1605';
      el.runBtn.style.borderColor = '#5a4514';
      el.runBtn.style.boxShadow = '0 0 12px rgba(245,166,35,.35), inset 0 1px 0 rgba(255,255,255,.3)';
    } else { // START / RESUME — green go
      el.runBtn.style.background = 'linear-gradient(180deg,#2fc98a,#179a64)';
      el.runBtn.style.color = '#04231a';
      el.runBtn.style.borderColor = '#0c5238';
      el.runBtn.style.boxShadow = '0 0 12px rgba(52,211,153,.35), inset 0 1px 0 rgba(255,255,255,.3)';
    }
    if (el.armHint) el.armHint.style.display = armed ? '' : 'none';

    // learn text
    var lt = learnMap[s.learn] || learnMap.Kp;
    el.learnTitle.textContent = lt[0];
    el.learnBody.textContent = lt[1];

    // tune
    var t = computeTune();
    el.tuneKp.textContent = t.Kp == null ? '—' : fmt(t.Kp, 2);
    el.tuneKi.textContent = t.Ki == null ? '—' : fmt(t.Ki, 3);
    el.tuneKd.textContent = t.Kd == null ? '—' : fmt(t.Kd, 2);
    el.fopdtTxt.textContent = t.f.integ ? ('K=' + fmt(t.f.K, 2) + ', integrating') : ('K=' + fmt(t.f.K, 2) + ', τ=' + fmt(t.f.tau, 1) + 's, θ=' + fmt(t.f.theta, 1) + 's');
    el.tuneNote.textContent = t.note;

    // keep slider displays/values in sync (after programmatic changes)
    for (var k in sliders) {
      var sl = sliders[k];
      sl.input.value = state[k];
      sl.disp.nodeValue = fmt(state[k], sl.dec);
    }
    el.spNum.value = s.sp;
  }

  function setToggle(key, on, labels) {
    var btn = document.querySelector('[data-toggle="' + key + '"]');
    btn.className = 'tgl ' + (on ? 'tgl-on' : 'tgl-off');
    btn.textContent = on ? labels[1] : labels[0];
  }
  function show(node, vis) { node.style.display = vis ? '' : 'none'; }

  // ---------------- sync (live readouts) ----------------
  function syncReadouts() {
    var L = last, s = state;
    el.tD.textContent = S.simT.toFixed(1);
    el.lgSP.textContent = fmt(s.sp, 1);
    el.lgPV.textContent = fmt(L.pv, 2);
    el.lgMV.textContent = fmt(L.mv, 1);
    el.lgERR.textContent = fmt(L.err, 2);
    el.pTerm.textContent = fmt(L.P, 2);
    el.iTerm.textContent = fmt(L.I, 2);
    el.dTerm.textContent = fmt(L.D, 2);
    el.satBadge.style.display = L.sat ? '' : 'none';
    el.sampleCount.textContent = hist.length;

    var m = metrics();
    metricDefs.forEach(function (d) {
      var ref2 = metricRefs[d.key];
      ref2.txt.nodeValue = fmt(m[d.key], d.dec);
      var tone = d.tone;
      if (tone === 'over') tone = m.overshoot <= 10 ? 'var(--green)' : (m.overshoot <= 25 ? '' : 'var(--amber)');
      else if (tone === 'sse') tone = Math.abs(m.steadyStateError) <= 1 ? 'var(--green)' : 'var(--red)';
      ref2.val.style.color = tone;
    });
  }

  // ---------------- wiring ----------------
  function build() {
    ['scope', 'plantEq', 'plantSel', 'methodSel', 'learnSel', 'speedSel', 'windowSel', 'winSec',
      'statusText', 'statusDot', 'runBtn', 'learnTitle', 'learnBody', 'tuneKp', 'tuneKi', 'tuneKd',
      'fopdtTxt', 'tuneNote', 'spNum', 'tD', 'lgSP', 'lgPV', 'lgMV', 'lgERR', 'pTerm', 'iTerm', 'dTerm',
      'satBadge', 'sampleCount', 'armHint', 'themeBtn'].forEach(function (id) { el[id] = $(id); });

    // sliders
    makeSlider({ key: 'sp', label: 'Setpoint', unit: 'PV', hint: 'Target value the controller drives the process toward. Changing it triggers a step test.', min: 0, max: 100, step: 0.5, dec: 1, onChange: function () { resetMetrics(S.y); el.spNum.value = state.sp; } }, $('slot-sp'));
    makeSlider({ key: 'Kp', label: 'Kp · Prop. gain', unit: '', hint: 'Proportional gain. Higher = faster, stronger response but more overshoot and possible oscillation.', min: 0, max: 12, step: 0.01, dec: 2 }, $('slot-Kp'));
    makeSlider({ key: 'Ki', label: 'Ki · Integral gain', unit: '1/s', hint: 'Integral gain. Eliminates steady-state error; too high causes windup and oscillation.', min: 0, max: 4, step: 0.005, dec: 3 }, $('slot-Ki'));
    makeSlider({ key: 'Kd', label: 'Kd · Deriv. gain', unit: 's', hint: 'Derivative gain. Damps overshoot by reacting to rate of change; amplifies sensor noise.', min: 0, max: 6, step: 0.01, dec: 2 }, $('slot-Kd'));
    makeSlider({ key: 'manual', label: 'Manual output', unit: '%', hint: 'Direct controller output in manual mode. Switch to AUTO for bumpless transfer.', min: 0, max: 100, step: 0.5, dec: 1 }, $('slot-manual'));
    makeSlider({ key: 'dt', label: 'Sampling time dt', unit: 's', hint: 'Discrete controller / integration step. Smaller = more accurate, heavier.', min: 0.01, max: 0.5, step: 0.01, dec: 2 }, $('slot-dt'));
    makeSlider({ key: 'outMin', label: 'Output min', unit: '%', hint: 'Lower actuator / controller-output limit.', min: -50, max: 50, step: 1, dec: 0 }, $('slot-outMin'));
    makeSlider({ key: 'outMax', label: 'Output max', unit: '%', hint: 'Upper actuator / controller-output limit (actuator saturation).', min: 50, max: 150, step: 1, dec: 0 }, $('slot-outMax'));

    makeSlider({ key: 'K', label: 'K · Process gain', unit: '', hint: "Steady-state output change per unit input. Sets the loop's overall sensitivity.", min: 0.1, max: 4, step: 0.05, dec: 2 }, $('slot-K'));
    makeSlider({ key: 'tau', label: 'τ · Time constant', unit: 's', hint: 'Time to reach ~63% of final value. Larger = slower process.', min: 1, max: 40, step: 0.5, dec: 1 }, $('slot-tau'));
    makeSlider({ key: 'zeta', label: 'ζ · Damping ratio', unit: '', hint: '<1 underdamped (oscillates), =1 critical, >1 overdamped.', min: 0.1, max: 2, step: 0.02, dec: 2 }, $('slot-zeta'));
    makeSlider({ key: 'wn', label: 'ωn · Nat. frequency', unit: 'rad/s', hint: 'Natural frequency of the second-order process. Higher = faster.', min: 0.1, max: 2, step: 0.02, dec: 2 }, $('slot-wn'));
    makeSlider({ key: 'theta', label: 'θ · Dead time', unit: 's', hint: 'Transport delay before the process reacts. The hardest dynamic to control.', min: 0, max: 20, step: 0.5, dec: 1 }, $('slot-theta'));
    makeSlider({ key: 'y0', label: 'Initial condition', unit: 'PV', hint: 'Process value at reset. While the sim is armed (READY), changing this previews the starting point.', min: 0, max: 100, step: 1, dec: 0, onChange: function () { if (!state.started) doReset(); } }, $('slot-y0'));

    makeSlider({ key: 'distMag', label: 'Magnitude', unit: '', hint: 'Step load added to the process input — simulates an upset the controller must reject.', min: -40, max: 40, step: 1, dec: 0 }, $('slot-distMag'));
    makeSlider({ key: 'noiseStd', label: 'Std deviation σ', unit: 'PV', hint: 'Gaussian measurement noise on the transmitter. Watch how it couples through Kd.', min: 0, max: 4, step: 0.1, dec: 1 }, $('slot-noiseStd'));
    makeSlider({ key: 'slew', label: 'Max rate', unit: '%/s', hint: 'Limits how fast the valve / actuator can move — models real actuator dynamics.', min: 5, max: 300, step: 5, dec: 0 }, $('slot-slew'));

    buildMetrics($('metricsGrid'));

    // segmented buttons
    document.querySelectorAll('[data-ctype]').forEach(function (b) { b.addEventListener('click', function () { state.ctype = b.getAttribute('data-ctype'); syncControls(); }); });
    document.querySelectorAll('[data-mode]').forEach(function (b) { b.addEventListener('click', function () { state.mode = b.getAttribute('data-mode'); syncControls(); }); });
    document.querySelectorAll('[data-dir]').forEach(function (b) { b.addEventListener('click', function () { state.dir = b.getAttribute('data-dir'); syncControls(); }); });

    // toggles
    document.querySelector('[data-toggle="awOn"]').addEventListener('click', function () { state.awOn = !state.awOn; syncControls(); });
    document.querySelector('[data-toggle="distOn"]').addEventListener('click', function () { state.distOn = !state.distOn; syncControls(); });
    document.querySelector('[data-toggle="noiseOn"]').addEventListener('click', function () { state.noiseOn = !state.noiseOn; syncControls(); });
    document.querySelector('[data-toggle="slewOn"]').addEventListener('click', function () { state.slewOn = !state.slewOn; syncControls(); });

    // selects
    el.plantSel.addEventListener('change', function (e) { state.plant = e.target.value; syncControls(); });
    el.methodSel.addEventListener('change', function (e) { state.method = e.target.value; syncControls(); });
    el.learnSel.addEventListener('change', function (e) { state.learn = e.target.value; syncControls(); });
    el.speedSel.addEventListener('change', function (e) { state.speed = parseFloat(e.target.value); });
    el.windowSel.addEventListener('change', function (e) { state.windowSec = parseInt(e.target.value, 10); el.winSec.textContent = state.windowSec; });

    // go-to form
    el.spNum.value = state.sp;
    el.spNum.addEventListener('input', function (e) { var v = parseFloat(e.target.value); if (isFinite(v)) { state.sp = v; sliders.sp.input.value = v; sliders.sp.disp.nodeValue = fmt(v, 1); resetMetrics(S.y); } });
    $('goForm').addEventListener('submit', function (e) { e.preventDefault(); resetMetrics(S.y); });

    // transport
    el.runBtn.addEventListener('click', function () {
      if (!state.started) { doReset(); state.started = true; state.running = true; }
      else { state.running = !state.running; }
      syncControls();
    });
    $('stepBtn').addEventListener('click', function () {
      if (!state.started) { doReset(); state.started = true; }
      step(state.dt); draw(); syncReadouts(); syncControls();
    });
    $('resetBtn').addEventListener('click', function () {
      doReset(); state.started = false; state.running = false; syncControls();
    });
    $('snapBtn').addEventListener('click', function () {
      var tEnd = buf.t.length ? buf.t[buf.t.length - 1] : 0;
      ref = { off: buf.t.map(function (t) { return tEnd - t; }), pv: buf.pv.slice() };
      el.clrRefBtn.style.display = '';
    });
    el.clrRefBtn = $('clrRefBtn');
    el.clrRefBtn.addEventListener('click', function () { ref = null; el.clrRefBtn.style.display = 'none'; });

    $('pulseBtn').addEventListener('click', function () { S.distEnd = S.simT + 4; });

    // tune + export
    $('applyTuneBtn').addEventListener('click', function () {
      var t = computeTune(); if (t.Kp == null) return;
      state.Kp = +t.Kp.toFixed(3); state.Ki = +t.Ki.toFixed(4); state.Kd = +t.Kd.toFixed(4);
      syncControls();
    });
    $('expJSON').addEventListener('click', exportJSON);
    $('expCSV').addEventListener('click', exportCSV);

    // theme toggle
    el.themeBtn.addEventListener('click', toggleTheme);
    setThemeLabel();

    syncControls();
    syncReadouts();
    setInterval(tickfn, 33);
  }

  initInstance();
  if (document.readyState !== 'loading') build();
  else document.addEventListener('DOMContentLoaded', build);
})();
