# Differential Drive Simulator

A clean, **from-scratch** 2D simulator for a differential-drive robot, written in Python with NumPy & Matplotlib.
Includes a simple **Pure Pursuit** controller for path following (circle/line/figure-8), a CLI, and ready-to-use
**documentation structure**, **Dockerfile**, **tests**, and **CI**.

---

## 🎯 Project Goals
- Implement differential-drive kinematics and simulation (Euler & RK4).
- Follow paths using a Pure Pursuit controller.
- Produce clear plots of path vs. executed trajectory.
- Be easy to run via CLI and easy to extend.
- Be a **portfolio-quality** repo with documentation, tests, CI, and Docker.

---

## 📦 Project Structure
```
diffdrive-simulator/
├─ src/diffdrive/
│  ├─ __init__.py
│  ├─ kinematics.py         # kinematic conversions
│  ├─ simulator.py          # state, integration, limits
│  ├─ controllers.py        # pure pursuit
│  ├─ trajectories.py       # circle, line, figure-8
│  ├─ plotting.py           # path & trajectory plots
│  └─ cli.py                # command-line interface
├─ examples/
│  └─ path_following.py     # runnable example script
├─ tests/
│  ├─ test_kinematics.py
│  └─ test_simulator.py
├─ .github/workflows/ci.yml # CI: lint + tests
├─ .pre-commit-config.yaml  # black + ruff
├─ Dockerfile               # run in a container
├─ pyproject.toml           # package metadata + deps
├─ requirements.txt         # optional, for convenience
├─ LICENSE                  # MIT
├─ .gitignore
└─ README.md
```

---

## 🧠 Equations (Kinematics)

Let the left and right **wheel linear velocities** be \(v_l, v_r\) (m/s). Let the **wheel base** (distance between wheels)
be \(L\), and the robot **pose** be \((x, y, \theta)\).

Body-frame velocities:
\[
v = \frac{v_r + v_l}{2}, \qquad \omega = \frac{v_r - v_l}{L}
\]

Dynamics (unicycle model):
\[
\dot{x} = v \cos\theta,\quad
\dot{y} = v \sin\theta,\quad
\dot{\theta} = \omega
\]

We integrate using **Euler** or **RK4**. Angle \(\theta\) is wrapped to \([-\pi,\pi)\).

---

## ✅ Quickstart

### 1) Create & activate a virtual environment
```bash
python -m venv .venv
# Windows: .venv\\Scripts\\activate
# Linux/Mac:
source .venv/bin/activate
```

### 2) Install (dev mode)
```bash
pip install -e .[dev]
pre-commit install  # optional but recommended
```

### 3) Run the example
```bash
python examples/path_following.py --path circle --duration 20 --dt 0.02 --save outputs/circle.png
```

or use the CLI:
```bash
diffdrive-sim --controller pure_pursuit --path circle --duration 20 --dt 0.02 --save outputs/circle.png
```

This will produce a plot of the reference path and the robot trajectory.

### 4) Run tests & lint
```bash
pytest -q
ruff src
black --check src
```

---

## 🏃 Using the CLI
```bash
diffdrive-sim \
  --controller pure_pursuit \
  --path circle \
  --radius 3.0 \
  --lookahead 0.6 \
  --speed 0.7 \
  --duration 20 \
  --dt 0.02 \
  --integrator rk4 \
  --save outputs/circle.png
```

**Common options:**
- `--path {circle,line,figure8}`
- `--radius` (for circle)
- `--start`, `--end` (for line, e.g. `--start -3,0 --end 3,0`)
- `--lookahead` (pure pursuit lookahead distance)
- `--speed` (nominal forward speed)
- `--integrator {euler,rk4}`
- `--save` to save a PNG

---

## 🧪 Experiments & Documentation Tips
- Keep a running log in `docs/EXPERIMENTS.md` (date, controller params, path, notes, screenshots).
- Save one plot per change (e.g. different lookahead). Name files with parameters, e.g. `circle_Ld0.6_v0.7.png`.
- In the README, add a **Results** section with 2–3 best plots and short commentary on tracking error.
- Commit often with meaningful messages. Tag stable states: `v0.1`, `v0.2`.
- For bigger write-ups, consider `mkdocs` later.

Template for `docs/EXPERIMENTS.md`:
```markdown
# Experiment Log

## 2025-08-20 – Circle path, Pure Pursuit
- Params: lookahead=0.6, speed=0.7, dt=0.02
- Notes: Converges after ~3 seconds, small steady-state lateral offset.
- Artifacts: outputs/circle_Ld0.6_v0.7.png
```

---

## 🐳 Docker
Build and run without touching your host Python:

```bash
docker build -t diffdrive-sim:latest .
docker run --rm -v $(pwd):/work -w /work diffdrive-sim:latest \
  diffdrive-sim --path circle --save outputs/circle.png
```

---

## ☁️ Publish to GitHub
```bash
git init
git add .
git commit -m "Init: differential drive simulator"
git branch -M main
git remote add origin YOUR_GITHUB_REMOTE_URL
git push -u origin main
```

Enable GitHub Actions (CI) by pushing to `main`. Lint & tests will run automatically.

---

## 📌 What to Showcase (README)
- **Clear GIF/PNG** of path vs. trajectory.
- Short paragraph on **how Pure Pursuit works** in your words.
- Short section on **limitations** (no dynamics, only kinematics, no slip, etc.).
- Bullet list: **what you learned**.

---

## 🧩 Extend Me
- Add velocity/omega **saturation** & acceleration limits (already supported).
- Add sensor noise & measurement models.
- Add another controller (Stanley, PID on cross-track).
- Animate and export to MP4/GIF.

---

## License
MIT
