# Rekalibrasi Control Lab

Two related, real-time, browser-based process-control simulators that share one
control-room HMI look:

| App | Path | What it is |
| --- | --- | --- |
| **tuning-pid** | `/` (root) | The focused **PID tuning simulator** — tune `Kp/Ki/Kd` against configurable plant models and watch the live strip chart. |
| **control-sim** | `/control-sim/` | The **closed-loop process simulator** — the same loop rebuilt from industrial instrument blocks (measuring element → transmitter → controller → valve → process) with an engineering-unit signal chain (EU ↔ % ↔ 4–20 mA). |

The two pages cross-link from their headers. Both are dependency-free static pages.

![Rekalibrasi Control Lab](design/screenshots/lab2.png)

## What it does

- **Controllers** — P, PI, PID (parallel form) with manual/auto modes, direct/reverse
  action, derivative filtering, and back-calculation anti-windup.
- **Plant models** — first order, FOPDT (with dead time), second order (ωn / ζ),
  SOPDT, and integrating processes.
- **Real-time strip chart** — SP, PV, MV, and error traces on a dual-axis canvas
  scope with a configurable time window and an interactive snapshot (before/after)
  ghost trace.
- **Disturbances** — step/pulse load disturbances, Gaussian sensor noise, and an
  actuator slew-rate limit.
- **Performance metrics** — overshoot, rise/peak/settling time, steady-state error,
  control effort, IAE / ISE / ITAE, computed live.
- **Auto-tuning lab** — Ziegler–Nichols (open loop), Cohen–Coon, and IMC (lambda)
  rules from an identified FOPDT model.
- **Export** — full run history + configuration + metrics as JSON or CSV.
- **Dark / light themes** — a control-room HMI dark theme and a light instrument
  theme, toggled from the header and remembered across visits (defaults to your
  OS preference). Readable typography: sans-serif labels with monospace digit
  readouts.

> ⚠️ **Safety** — This simulator is for education, conceptual design, research, and
> training only. Generated tuning parameters must **not** be applied directly to real
> industrial processes without proper hazard analysis, field validation, and review
> by qualified control engineers.

## Run locally

It's a static page with **no build step and no dependencies**. Open it directly:

```bash
# simplest — just open the file
open index.html        # macOS  (use xdg-open on Linux, start on Windows)

# or serve it (any static server works)
python3 -m http.server 8000
# then visit http://localhost:8000
```

## Deploy to the web

Because the app is a dependency-free static bundle (`index.html` + `app.js`), it can
be hosted anywhere that serves static files.

### GitHub Pages (automated)

A workflow at `.github/workflows/deploy.yml` publishes the site on every push to
`main`. To enable it once:

1. Push this repository to GitHub.
2. Go to **Settings → Pages → Build and deployment** and set **Source** to
   **GitHub Actions**.
3. Push to `main` — the site goes live at
   `https://<user>.github.io/<repo>/`.

### Other hosts

Drag-and-drop or point any of these at the repository root:

- **Netlify / Vercel / Cloudflare Pages** — no build command, publish directory `/`.
- **Any web server / CDN / S3 bucket** — upload `index.html` and `app.js`.

## Project layout

```
index.html                     tuning-pid — markup, styles, layout
app.js                         tuning-pid — simulation + UI binding (no framework, no CDN)
control-sim/
  index.html                   control-sim — markup, styles, layout
  app.js                       control-sim — UI/state binding, chart, metrics, export
  engine.js                    control-sim — modular closed-loop block engine
                               (Process → Sensor → Transmitter → Controller → Valve)
.github/workflows/deploy.yml   GitHub Pages deployment (serves both apps)
design/                        Original Design-Component (DC) source this was ported from
  Rekalibrasi Control Lab.dc.html
  ParamSlider.dc.html
  MetricStat.dc.html
  support.js
  screenshots/
```

## control-sim: the block engine

`control-sim/engine.js` runs the loop as discrete industrial components, each a
block with its own state and `step(dt, inputs, params)`:

```
Process → Sensor(lag+noise) → Transmitter(EU→%/mA) → Controller(discrete PID,
sample-and-hold @ scan time) → Valve(slew) → back to Process
```

Signals travel in real engineering units (PV in EU → transmitter % of range →
4–20 mA → controller output % → valve travel % → 4–20 mA), shown live in the
**Signal Chain** panel. It is the foundation for later phases (valve stiction,
sensor faults, cascade, feedforward, and a P&ID block-diagram view).

## How it was built

The original design (`design/`) was authored as a **Design Component** — a React
template rendered by a proprietary runtime (`support.js`) that loads React from a CDN
and fetches sibling components at runtime. To make it genuinely deployable as a live
web page, the rendering layer was reimplemented with plain DOM binding while the
**simulation core was preserved verbatim** (the discrete PID step, plant integrators,
metrics, canvas scope, auto-tuning rules, and export). The result is a single static
page with zero runtime dependencies.
