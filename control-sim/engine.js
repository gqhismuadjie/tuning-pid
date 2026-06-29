/*
 * engine.js — modular closed-loop block engine for Rekalibrasi Control Lab.
 *
 * Signal chain (one direction around the loop):
 *
 *   Process(PV,EU) -> Sensor(lag+noise) -> Transmitter(EU->%/mA)
 *        ^                                                |
 *        |                                                v
 *   Valve(slew) <- Controller(discrete PID @ scan time) <-+
 *
 * Each block owns its internal state and exposes step(dt, inputs, params).
 * The Loop wires the blocks, carries signals in real engineering units
 * (EU <-> % of range <-> 4..20 mA), and runs the controller at its scan
 * time via sample-and-hold while the physical blocks integrate every dt.
 *
 * With default settings (range 0..100, no sensor lag, scan time == dt) the
 * loop reproduces the original lumped simulation exactly; the extra block
 * parameters are no-ops until changed.
 */
(function (global) {
  'use strict';

  function clamp(v, a, b) { return v < a ? a : (v > b ? b : v); }
  function euToPct(eu, lrv, urv) { var s = (urv - lrv) || 1; return (eu - lrv) / s * 100; }
  function pctToEu(p, lrv, urv) { return lrv + p / 100 * ((urv - lrv) || 1); }
  function pctToMA(p) { return 4 + clamp(p, 0, 100) / 100 * 16; }

  // Box-Muller Gaussian with a cached spare, per instance.
  function Gauss() { this.spare = null; }
  Gauss.prototype.next = function () {
    if (this.spare != null) { var s = this.spare; this.spare = null; return s; }
    var u = 0, v = 0; while (u === 0) u = Math.random(); while (v === 0) v = Math.random();
    var m = Math.sqrt(-2 * Math.log(u));
    this.spare = m * Math.sin(2 * Math.PI * v);
    return m * Math.cos(2 * Math.PI * v);
  };

  // ---- Process: the plant dynamics (FOPDT / SOPDT / integrating / ...) ----
  function Process() { this.y = 0; this.y2 = 0; this.delay = []; }
  Process.prototype.reset = function (y0) { this.y = y0; this.y2 = 0; this.delay = []; };
  Process.prototype.step = function (dt, u, p) {
    var self = this, K = p.K;
    function delayed(val) {
      if (p.theta <= 0) return val;
      var n = Math.max(1, Math.round(p.theta / dt));
      if (self.delay.length !== n) self.delay = new Array(n).fill(val);
      self.delay.push(val); return self.delay.shift();
    }
    if (p.plant === 'first') { this.y += dt * (K * u - this.y) / p.tau; }
    else if (p.plant === 'fopdt') { var du = delayed(u); this.y += dt * (K * du - this.y) / p.tau; }
    else if (p.plant === 'integrating') { this.y += dt * K * u * 0.1; }
    else if (p.plant === 'second') { var w = p.wn, z = p.zeta; this.y2 += dt * (K * w * w * u - 2 * z * w * this.y2 - w * w * this.y); this.y += dt * this.y2; }
    else if (p.plant === 'sopdt') { var d2 = delayed(u); var w2 = p.wn, z2 = p.zeta; this.y2 += dt * (K * w2 * w2 * d2 - 2 * z2 * w2 * this.y2 - w2 * w2 * this.y); this.y += dt * this.y2; }
    return this.y;
  };

  // ---- Sensor / measuring element: first-order lag + Gaussian noise ----
  function Sensor() { this.filt = 0; this.g = new Gauss(); }
  Sensor.prototype.reset = function (y0) { this.filt = y0; this.g.spare = null; };
  Sensor.prototype.step = function (dt, yTrue, p) {
    var m = yTrue;
    if (p.tau > 0) { var a = dt / (p.tau + dt); this.filt += a * (yTrue - this.filt); m = this.filt; }
    else this.filt = yTrue;
    if (p.noiseOn) m += this.g.next() * p.noiseStd;
    return m;
  };

  // ---- Transmitter: EU -> % of range (+ damping); also emits 4..20 mA ----
  function Transmitter() { this.pct = 0; }
  Transmitter.prototype.reset = function (eu, p) { this.pct = euToPct(eu, p.lrv, p.urv); };
  Transmitter.prototype.step = function (dt, measEu, p) {
    var raw = euToPct(measEu, p.lrv, p.urv);
    if (p.damp > 0) { var a = dt / (p.damp + dt); this.pct += a * (raw - this.pct); }
    else this.pct = raw;
    return { pct: this.pct, mA: pctToMA(this.pct) };
  };

  // ---- Controller: discrete PID with sample-and-hold at scan time ----
  function Controller() {
    this.I = 0; this.dFilt = 0; this.prevPv = 0; this.out = 0; this.acc = 0;
    this.P = 0; this.D = 0; this.sat = false;
  }
  Controller.prototype.reset = function (pvPct) {
    this.I = 0; this.dFilt = 0; this.prevPv = pvPct; this.out = 0; this.acc = 0;
    this.P = 0; this.D = 0; this.sat = false;
  };
  Controller.prototype.snapshot = function () { return { out: this.out, P: this.P, I: this.I, D: this.D, sat: this.sat }; };
  Controller.prototype.step = function (dt, spPct, pvPct, p) {
    this.acc += dt;
    var scan = p.scanTime > dt ? p.scanTime : dt;        // can't execute faster than the sim step
    if (this.acc + 1e-9 < scan) return this.snapshot();  // hold the output between scans
    var h = this.acc; this.acc = 0;                      // actual elapsed interval since last update
    var actSign = p.dir === 'direct' ? 1 : -1;
    var error = actSign * (spPct - pvPct);
    var P = 0, D = 0, out, outSat, sat = false;
    if (p.mode === 'manual') {
      out = p.manual; outSat = clamp(out, p.outMin, p.outMax);
      this.I = outSat; this.prevPv = pvPct; this.dFilt = 0;          // track for bumpless transfer
    } else {
      var Kp = p.Kp, Ki = p.ctype === 'P' ? 0 : p.Ki, Kd = p.ctype === 'PID' ? p.Kd : 0;
      P = Kp * error;
      var dMeas = (pvPct - this.prevPv) / h;
      var Tf = (Kd > 0 && Kp > 0) ? Kd / (Kp * p.dfiltN) : 0;       // derivative filter time constant
      var a = Tf > 0 ? h / (Tf + h) : 1;
      this.dFilt = this.dFilt + a * ((-actSign * Kd * dMeas) - this.dFilt);
      D = this.dFilt;
      var Ic = this.I + Ki * error * h;
      out = P + Ic + D; outSat = clamp(out, p.outMin, p.outMax);
      if (out !== outSat) { sat = true; if (p.awOn) { Ic = this.I; out = P + Ic + D; outSat = clamp(out, p.outMin, p.outMax); } }
      this.I = Ic; this.prevPv = pvPct;
    }
    this.out = outSat; this.P = P; this.D = D; this.sat = sat;
    return this.snapshot();
  };

  // ---- Valve / final control element: slew-rate limit ----
  function Valve() { this.pos = 0; }
  Valve.prototype.reset = function (v) { this.pos = v; };
  Valve.prototype.step = function (dt, outPct, p) {
    var v = outPct;
    if (p.slewOn) { var md = p.slew * dt; v = clamp(v, this.pos - md, this.pos + md); }
    this.pos = v;
    return v;
  };

  // ---- Loop: wires the chain and carries the signal bus ----
  function Loop() {
    this.process = new Process();
    this.sensor = new Sensor();
    this.tx = new Transmitter();
    this.ctrl = new Controller();
    this.valve = new Valve();
    this.sig = {};
  }
  Loop.prototype.reset = function (cfg) {
    var y0 = cfg.y0;
    this.process.reset(y0);
    this.sensor.reset(y0);
    this.tx.reset(y0, cfg.tx);
    this.ctrl.reset(euToPct(y0, cfg.tx.lrv, cfg.tx.urv));
    this.valve.reset(0);
    this.sig = {};
  };
  Loop.prototype.step = function (dt, cfg) {
    var yTrue = this.process.y;                                  // PV measured before this integration step
    var measEu = this.sensor.step(dt, yTrue, cfg.sensor);
    var tx = this.tx.step(dt, measEu, cfg.tx);
    var spPct = euToPct(cfg.spEu, cfg.tx.lrv, cfg.tx.urv);
    var c = this.ctrl.step(dt, spPct, tx.pct, cfg.ctrl);
    var valvePct = this.valve.step(dt, c.out, cfg.valve);
    var dist = cfg.distOn ? cfg.distMag : 0;
    if (cfg.distActive) dist += cfg.distMag;                     // timed pulse
    var uIn = valvePct + dist;                                   // load disturbance enters at the process input
    var y = this.process.step(dt, uIn, cfg.proc);
    this.sig = {
      yTrue: y, measEu: measEu, pvPct: tx.pct, pvMA: tx.mA,
      spEu: cfg.spEu, spPct: spPct, errEu: cfg.spEu - measEu, errPct: spPct - tx.pct,
      outPct: c.out, valvePct: valvePct, valveMA: pctToMA(valvePct), uIn: uIn,
      P: c.P, I: c.I, D: c.D, sat: c.sat
    };
    return this.sig;
  };

  global.RKL = { Loop: Loop, euToPct: euToPct, pctToEu: pctToEu, pctToMA: pctToMA };
})(typeof window !== 'undefined' ? window : this);
