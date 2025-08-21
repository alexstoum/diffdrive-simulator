# Differential Drive Simulator

A clean, **from-scratch** 2D simulator for a differential-drive robot, written in Python with NumPy & Matplotlib.
Includes a simple **Pure Pursuit** controller for path following (circle/line/figure-8), a CLI, and ready-to-use
**documentation structure**, **tests**, and **CI**.

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

The left and right wheel linear velocities are `v_l` and `v_r` (in m/s). The wheel base (the distance between the wheels) is `L`. The robot's pose is given by its x-position, y-position, and orientation angle `theta`.

The body-frame linear and angular velocities are calculated as:
- Linear velocity, `v` = (v_r + v_l) / 2
- Angular velocity, `omega` = (v_r - v_l) / L

The robot's dynamics follow the unicycle model:
- dx/dt = v * cos(theta)
- dy/dt = v * sin(theta)
- d(theta)/dt = omega

These dynamics are integrated over time using a numerical method like Euler or Runge-Kutta 4 (RK4). The robot's orientation angle `theta` is kept within the range [-pi, pi).
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

## License
MIT
