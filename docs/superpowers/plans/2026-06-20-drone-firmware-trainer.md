# Drone Firmware Trainer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A 2-hour hackathon-buildable training harness where a Python "drone firmware" module fails an obstacle-avoidance mission in a 2D pygame sim, gets patched based on telemetry, and succeeds on re-run.

**Architecture:** Three packages (`sim/`, `firmware/`, `harness/`) talking only through two frozen dataclasses (`SensorPacket`, `MotorCommand`) defined in `firmware/contract.py`. `sim` never imports `firmware`; `firmware` never imports `sim`; `harness` wires them. File ownership is hard-split between two devs so no two people edit the same file.

**Tech Stack:** Python 3.11+, pygame, numpy. Nothing else.

## Global Constraints

- Python 3.11+ on Windows. No native build deps beyond pygame's bundled wheels.
- Only third-party deps: `pygame`, `numpy`. Pinned in `requirements.txt`.
- `sim/` does NOT import from `firmware/`. `firmware/` does NOT import from `sim/`. Only `harness/` may import both.
- `firmware/contract.py` is LOCKED after Task 2. Any change requires verbal sync between both devs.
- Firmware modules expose exactly one public class `Firmware` with `__init__(self) -> None` and `step(self, sensors: SensorPacket) -> MotorCommand`. No other public symbols.
- Control loop: 50 Hz fixed. Renderer: 30 Hz (decoupled). Sim timeout: 30 s wall-clock-equivalent sim time.
- World: 20 m × 15 m (800 × 600 px at 40 px/m). Spawn (1.0, 1.0). Goal center (18.0, 13.0), radius 0.75 m.
- Drone: mass 0.5 kg, linear drag coefficient 0.4. Thrust clipped per-axis to [-1.0, 1.0] N (∞-norm).
- Yaw is derived: `atan2(vy, vx)`. Not a controllable DOF. Sensors are body-frame.
- 8 ray-casts at 45° spacing in body frame, max range 4.0 m. Sensor noise σ: rays 0.03 m, accel 0.05 m/s², pos 0.05 m, yaw 0.02 rad. All sensor values quantized to 0.01 before crossing the contract.
- Firmware tick budget: 50 ms soft cap, log only, never kill.
- Demo seed: `42`. Sweep seeds: `[42, 7, 13, 99, 256]`.
- Commit after every task. Single `main` branch. Push every ~10 minutes.

## File Ownership

| Owner | Files |
|---|---|
| **DEV-A (you)** | `firmware/contract.py` (paired w/ B then locked), `firmware/firmware_v1.py`, `firmware/firmware_v2.py`, `harness/scoring.py` |
| **DEV-B (teammate)** | `sim/world.py`, `sim/physics.py`, `sim/sensors.py`, `sim/renderer.py`, `harness/loop.py`, `harness/run.py` |
| **Either** | `requirements.txt`, `README.md`, `.gitignore`, `tests/test_smoke.py` |

## Task Dependency Graph

```
T1 (setup)
  └─> T2 (contract, paired, BLOCKING)
        ├─> [DEV-A track]  T3 scoring -> T4 firmware_v1
        └─> [DEV-B track]  T5 world -> T6 physics -> T7 sensors -> T8 loop -> T9 run -> T10 renderer
                                                                              ↑
                                  (T4 + T9 converge)  ────────────────────────┘
                                                ↓
                                            T11 smoke (either)
                                                ↓
                                          T12 firmware_v2 (DEV-A)
                                                ↓
                                          T13 sweep + demo polish (either)
```

T3-T4 (DEV-A) and T5-T9 (DEV-B) run in parallel after T2 lands. T10 renderer is parallelizable with T8/T9 once T6 exists.

---

### Task 1: Project setup

**Owner:** Either dev, whoever has GitHub access first.

**Files:**
- Create: `requirements.txt`
- Create: `.gitignore`
- Create: `README.md`
- Create: `firmware/__init__.py` (empty)
- Create: `sim/__init__.py` (empty)
- Create: `harness/__init__.py` (empty)
- Create: `tests/__init__.py` (empty)

**Interfaces:**
- Produces: a runnable Python project skeleton that `pip install -r requirements.txt` succeeds against.

- [ ] **Step 1: Create `requirements.txt`**

```
pygame==2.5.2
numpy==1.26.4
```

- [ ] **Step 2: Create `.gitignore`**

```
__pycache__/
*.pyc
.venv/
venv/
.vscode/
.idea/
*.log
telemetry_*.csv
```

- [ ] **Step 3: Create empty `__init__.py` files**

```bash
# PowerShell on Windows
New-Item -ItemType Directory -Force -Path firmware, sim, harness, tests | Out-Null
"" | Out-File -Encoding utf8 firmware/__init__.py
"" | Out-File -Encoding utf8 sim/__init__.py
"" | Out-File -Encoding utf8 harness/__init__.py
"" | Out-File -Encoding utf8 tests/__init__.py
```

- [ ] **Step 4: Write a minimal `README.md`**

```markdown
# Drone Firmware Trainer

2D obstacle-avoidance training harness for autonomous drone firmware.

## Run

    python -m venv .venv
    .venv\Scripts\activate
    pip install -r requirements.txt
    python -m harness.run --firmware v1 --seed 42 --render

## Demo

    python -m harness.run --firmware v1 --sweep    # expect 1/5
    python -m harness.run --firmware v2 --sweep    # expect 5/5
```

- [ ] **Step 5: Create venv, install, verify import works**

```bash
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
python -c "import pygame, numpy; print(pygame.__version__, numpy.__version__)"
```

Expected output: `2.5.2 1.26.4`

- [ ] **Step 6: Commit**

```bash
git init
git add requirements.txt .gitignore README.md firmware/__init__.py sim/__init__.py harness/__init__.py tests/__init__.py
git commit -m "chore: project skeleton with pygame+numpy"
```

---

### Task 2: Lock the firmware contract (PAIRED — BLOCKING)

**Owner:** Both devs pair on this. File is FROZEN after this commit.

**Files:**
- Create: `firmware/contract.py`

**Interfaces:**
- Produces: `SensorPacket`, `MotorCommand` frozen dataclasses. Every other task depends on these exact names and field shapes.

- [ ] **Step 1: Write `firmware/contract.py`**

```python
from dataclasses import dataclass

@dataclass(frozen=True)
class SensorPacket:
    rays: tuple[float, float, float, float, float, float, float, float]
    imu_accel: tuple[float, float]
    yaw: float
    pos_estimate: tuple[float, float]
    dt: float

@dataclass(frozen=True)
class MotorCommand:
    thrust: tuple[float, float]

    @staticmethod
    def zero() -> "MotorCommand":
        return MotorCommand(thrust=(0.0, 0.0))

    def clipped(self) -> "MotorCommand":
        fx, fy = self.thrust
        return MotorCommand(thrust=(max(-1.0, min(1.0, fx)), max(-1.0, min(1.0, fy))))
```

- [ ] **Step 2: Verify import + construction**

```bash
python -c "from firmware.contract import SensorPacket, MotorCommand; p = SensorPacket((0,)*8, (0,0), 0.0, (0,0), 0.02); c = MotorCommand((2.0, -0.5)).clipped(); print(p.rays, c.thrust)"
```

Expected: `(0, 0, 0, 0, 0, 0, 0, 0) (1.0, -0.5)`

- [ ] **Step 3: Commit (BOTH devs present, both verbally agree it is locked)**

```bash
git add firmware/contract.py
git commit -m "feat(contract): lock SensorPacket and MotorCommand"
git push
```

Verbal handshake: "Contract locked. Splitting work now."

---

### Task 3: Scoring module — `harness/scoring.py` [DEV-A]

**Owner:** DEV-A. Runs in parallel with DEV-B's Task 5.

**Files:**
- Create: `harness/scoring.py`

**Interfaces:**
- Consumes: nothing from firmware or sim — pure data record.
- Produces:
  - `Outcome` enum: `SUCCESS`, `CRASH`, `TIMEOUT`, `FAULT`
  - `RunResult` dataclass: `seed: int`, `outcome: Outcome`, `t_end: float`, `min_clearance: float`, `mean_tick_ms: float`, `max_tick_ms: float`, `num_overruns: int`, `fault_msg: str | None`
  - `format_result(r: RunResult) -> str` for CLI display
  - `summarize_sweep(results: list[RunResult]) -> str`

- [ ] **Step 1: Write the file**

```python
from dataclasses import dataclass
from enum import Enum

class Outcome(str, Enum):
    SUCCESS = "SUCCESS"
    CRASH = "CRASH"
    TIMEOUT = "TIMEOUT"
    FAULT = "FAULT"

@dataclass
class RunResult:
    seed: int
    outcome: Outcome
    t_end: float
    min_clearance: float
    mean_tick_ms: float
    max_tick_ms: float
    num_overruns: int
    fault_msg: str | None = None

def format_result(r: RunResult) -> str:
    tag = r.outcome.value
    base = f"[seed={r.seed}] {tag} @ t={r.t_end:5.2f}s  min_clr={r.min_clearance:.2f}m  tick(mean/max)={r.mean_tick_ms:.2f}/{r.max_tick_ms:.2f}ms  overruns={r.num_overruns}"
    if r.fault_msg:
        base += f"  fault={r.fault_msg!r}"
    return base

def summarize_sweep(results: list[RunResult]) -> str:
    n = len(results)
    wins = sum(1 for r in results if r.outcome is Outcome.SUCCESS)
    lines = [format_result(r) for r in results]
    lines.append(f"--- {wins}/{n} success ---")
    return "\n".join(lines)
```

- [ ] **Step 2: Smoke check**

```bash
python -c "from harness.scoring import RunResult, Outcome, format_result, summarize_sweep; r = RunResult(42, Outcome.SUCCESS, 12.7, 0.18, 4.2, 9.1, 0); print(format_result(r)); print(summarize_sweep([r, r]))"
```

Expected: two formatted lines + `--- 2/2 success ---`.

- [ ] **Step 3: Commit**

```bash
git add harness/scoring.py
git commit -m "feat(harness): scoring module with Outcome and RunResult"
git push
```

---

### Task 4: Firmware v1 — naive controller [DEV-A]

**Owner:** DEV-A. Runs in parallel with DEV-B's Tasks 6-9.

**Files:**
- Create: `firmware/firmware_v1.py`

**Interfaces:**
- Consumes: `SensorPacket`, `MotorCommand` from `firmware/contract.py`.
- Produces: `Firmware` class. This is the version that MUST FAIL on seed 42 so the demo iteration story works.

**Design intent — known failure mode:** v1 reads only `rays[0]` (forward) and `rays[4]` (backward) and ignores diagonal rays. It steers toward the goal vector and only brakes when the forward ray reads short. Diagonal obstacles (which seed 42 will place) are invisible to v1. This is the failure v2 patches.

- [ ] **Step 1: Write the file**

```python
import math
from firmware.contract import SensorPacket, MotorCommand

GOAL_X, GOAL_Y = 18.0, 13.0

class Firmware:
    def __init__(self) -> None:
        self._last_forward = 4.0

    def step(self, sensors: SensorPacket) -> MotorCommand:
        px, py = sensors.pos_estimate
        dx, dy = GOAL_X - px, GOAL_Y - py
        norm = math.hypot(dx, dy) or 1.0
        ux, uy = dx / norm, dy / norm

        forward = sensors.rays[0]
        self._last_forward = forward

        if forward < 1.0:
            # naive brake — does NOT steer around diagonal obstacles
            fx, fy = -ux * 0.3, -uy * 0.3
        else:
            fx, fy = ux * 0.8, uy * 0.8

        return MotorCommand(thrust=(fx, fy)).clipped()
```

- [ ] **Step 2: Smoke construct**

```bash
python -c "from firmware.firmware_v1 import Firmware; from firmware.contract import SensorPacket; f = Firmware(); print(f.step(SensorPacket((4,)*8,(0,0),0.0,(1,1),0.02)))"
```

Expected: a `MotorCommand` with non-zero thrust pointing roughly toward `(18, 13)` from `(1, 1)`.

- [ ] **Step 3: Commit**

```bash
git add firmware/firmware_v1.py
git commit -m "feat(firmware): v1 naive forward-only controller"
git push
```

---

### Task 5: World — `sim/world.py` [DEV-B]

**Owner:** DEV-B. Starts immediately after Task 2 lands.

**Files:**
- Create: `sim/world.py`

**Interfaces:**
- Consumes: nothing from firmware.
- Produces:
  - `Rect` dataclass: `x: float`, `y: float`, `w: float`, `h: float` (meters, axis-aligned, x,y is bottom-left corner)
  - `World` dataclass: `arena_w: float`, `arena_h: float`, `obstacles: list[Rect]`, `spawn: tuple[float, float]`, `goal_center: tuple[float, float]`, `goal_radius: float`
  - `build_world(seed: int) -> World` — deterministic obstacle layout from seed. MUST place at least one diagonal-facing obstacle gap for seed 42 so firmware_v1 fails and firmware_v2 (using diagonal rays) succeeds.

- [ ] **Step 1: Write the file**

```python
import random
from dataclasses import dataclass, field

ARENA_W = 20.0
ARENA_H = 15.0
SPAWN = (1.0, 1.0)
GOAL_CENTER = (18.0, 13.0)
GOAL_RADIUS = 0.75

@dataclass(frozen=True)
class Rect:
    x: float
    y: float
    w: float
    h: float

    def contains_point(self, px: float, py: float) -> bool:
        return self.x <= px <= self.x + self.w and self.y <= py <= self.y + self.h

@dataclass
class World:
    arena_w: float
    arena_h: float
    obstacles: list[Rect]
    spawn: tuple[float, float]
    goal_center: tuple[float, float]
    goal_radius: float

def _hand_crafted_seed_42() -> list[Rect]:
    # Diagonal wall with a gap that requires steering off the direct goal line.
    # firmware_v1 only reads forward ray and will smack into rect (6,6,4,0.6).
    return [
        Rect(6.0, 6.0, 4.0, 0.6),   # horizontal wall blocking direct path
        Rect(11.0, 8.5, 0.6, 4.5),  # vertical wall after the gap
        Rect(3.0, 10.0, 2.0, 0.6),
        Rect(14.0, 4.0, 0.6, 3.0),
        Rect(8.5, 2.0, 0.6, 3.0),
    ]

def _random_obstacles(seed: int) -> list[Rect]:
    rng = random.Random(seed)
    rects: list[Rect] = []
    attempts = 0
    while len(rects) < 9 and attempts < 200:
        attempts += 1
        x = rng.uniform(3.0, 16.0)
        y = rng.uniform(2.0, 12.0)
        if rng.random() < 0.5:
            w, h = rng.uniform(1.0, 3.0), 0.6
        else:
            w, h = 0.6, rng.uniform(1.0, 3.0)
        cand = Rect(x, y, w, h)
        if cand.contains_point(*SPAWN) or cand.contains_point(*GOAL_CENTER):
            continue
        rects.append(cand)
    return rects

def build_world(seed: int) -> World:
    obstacles = _hand_crafted_seed_42() if seed == 42 else _random_obstacles(seed)
    return World(
        arena_w=ARENA_W,
        arena_h=ARENA_H,
        obstacles=obstacles,
        spawn=SPAWN,
        goal_center=GOAL_CENTER,
        goal_radius=GOAL_RADIUS,
    )
```

- [ ] **Step 2: Verify**

```bash
python -c "from sim.world import build_world; w = build_world(42); print(len(w.obstacles), w.spawn, w.goal_center); w2 = build_world(7); print(len(w2.obstacles))"
```

Expected: `5 (1.0, 1.0) (18.0, 13.0)` and then a number 1-9.

- [ ] **Step 3: Commit**

```bash
git add sim/world.py
git commit -m "feat(sim): world layout with deterministic seed-42 obstacle field"
git push
```

---

### Task 6: Physics — `sim/physics.py` [DEV-B]

**Owner:** DEV-B.

**Files:**
- Create: `sim/physics.py`

**Interfaces:**
- Consumes: `Rect` from `sim/world.py`.
- Produces:
  - `DroneState` dataclass: `x, y, vx, vy: float`, `t: float`. Mutable.
  - `make_drone(spawn: tuple[float, float]) -> DroneState`
  - `step_physics(state: DroneState, thrust: tuple[float, float], dt: float) -> None` — mutates state in place. Mass 0.5, drag 0.4, ∞-norm thrust clip.
  - `is_inside_any(rects: list[Rect], px: float, py: float) -> bool`
  - `min_clearance(rects: list[Rect], px: float, py: float) -> float` — distance from point to nearest rectangle edge (signed: negative if inside).

- [ ] **Step 1: Write the file**

```python
from dataclasses import dataclass
from sim.world import Rect

MASS = 0.5
DRAG = 0.4

@dataclass
class DroneState:
    x: float
    y: float
    vx: float
    vy: float
    t: float

def make_drone(spawn: tuple[float, float]) -> DroneState:
    return DroneState(x=spawn[0], y=spawn[1], vx=0.0, vy=0.0, t=0.0)

def step_physics(state: DroneState, thrust: tuple[float, float], dt: float) -> None:
    fx = max(-1.0, min(1.0, thrust[0]))
    fy = max(-1.0, min(1.0, thrust[1]))
    ax = (fx - DRAG * state.vx) / MASS
    ay = (fy - DRAG * state.vy) / MASS
    state.vx += ax * dt
    state.vy += ay * dt
    state.x += state.vx * dt
    state.y += state.vy * dt
    state.t += dt

def is_inside_any(rects: list[Rect], px: float, py: float) -> bool:
    return any(r.contains_point(px, py) for r in rects)

def min_clearance(rects: list[Rect], px: float, py: float) -> float:
    if not rects:
        return float("inf")
    best = float("inf")
    for r in rects:
        cx = max(r.x, min(px, r.x + r.w))
        cy = max(r.y, min(py, r.y + r.h))
        d = ((px - cx) ** 2 + (py - cy) ** 2) ** 0.5
        if r.contains_point(px, py):
            d = -d
        if d < best:
            best = d
    return best
```

- [ ] **Step 2: Verify**

```bash
python -c "from sim.physics import make_drone, step_physics; s = make_drone((1,1)); step_physics(s, (1.0, 0.5), 0.02); print(round(s.x, 4), round(s.y, 4), round(s.vx, 4), round(s.vy, 4))"
```

Expected (rough): a small positive displacement, positive velocity in x and y.

- [ ] **Step 3: Commit**

```bash
git add sim/physics.py
git commit -m "feat(sim): kinematic point physics with drag and clearance helper"
git push
```

---

### Task 7: Sensors — `sim/sensors.py` [DEV-B]

**Owner:** DEV-B.

**Files:**
- Create: `sim/sensors.py`

**Interfaces:**
- Consumes: `Rect` from `sim/world.py`, `DroneState` from `sim/physics.py`, `SensorPacket` from `firmware/contract.py` (read-only).
- Produces:
  - `build_sensor_packet(state: DroneState, rects: list[Rect], dt: float, rng: numpy.random.Generator) -> SensorPacket`

- [ ] **Step 1: Write the file**

```python
import math
import numpy as np
from firmware.contract import SensorPacket
from sim.physics import DroneState
from sim.world import Rect

RAY_COUNT = 8
RAY_MAX = 4.0
RAY_STEP = 0.05  # meters per step in the dumb march

RAY_NOISE = 0.03
ACCEL_NOISE = 0.05
POS_NOISE = 0.05
YAW_NOISE = 0.02
QUANT = 0.01

def _q(v: float) -> float:
    return round(v / QUANT) * QUANT

def _ray_cast(px: float, py: float, ang: float, rects: list[Rect]) -> float:
    cx, cy = math.cos(ang), math.sin(ang)
    d = 0.0
    while d < RAY_MAX:
        d += RAY_STEP
        x = px + cx * d
        y = py + cy * d
        for r in rects:
            if r.contains_point(x, y):
                return d
    return RAY_MAX

def build_sensor_packet(
    state: DroneState,
    rects: list[Rect],
    dt: float,
    rng: np.random.Generator,
) -> SensorPacket:
    yaw_true = math.atan2(state.vy, state.vx) if (state.vx or state.vy) else 0.0
    rays_clean = []
    for i in range(RAY_COUNT):
        ang = yaw_true + i * (2 * math.pi / RAY_COUNT)
        rays_clean.append(_ray_cast(state.x, state.y, ang, rects))
    rays_noisy = tuple(
        _q(max(0.0, min(RAY_MAX, d + float(rng.normal(0.0, RAY_NOISE)))))
        for d in rays_clean
    )
    accel = (
        _q(float(rng.normal(0.0, ACCEL_NOISE))),
        _q(float(rng.normal(0.0, ACCEL_NOISE))),
    )
    yaw = _q(yaw_true + float(rng.normal(0.0, YAW_NOISE)))
    pos = (
        _q(state.x + float(rng.normal(0.0, POS_NOISE))),
        _q(state.y + float(rng.normal(0.0, POS_NOISE))),
    )
    return SensorPacket(
        rays=rays_noisy,
        imu_accel=accel,
        yaw=yaw,
        pos_estimate=pos,
        dt=dt,
    )
```

- [ ] **Step 2: Verify**

```bash
python -c "import numpy as np; from sim.world import build_world; from sim.physics import make_drone; from sim.sensors import build_sensor_packet; w = build_world(42); s = make_drone(w.spawn); p = build_sensor_packet(s, w.obstacles, 0.02, np.random.default_rng(0)); print(p.rays, p.pos_estimate)"
```

Expected: 8-tuple of floats in [0, 4], pos_estimate near (1.0, 1.0).

- [ ] **Step 3: Commit**

```bash
git add sim/sensors.py
git commit -m "feat(sim): 8-ray cast sensor with Gaussian noise and quantization"
git push
```

---

### Task 8: Harness loop — `harness/loop.py` [DEV-B]

**Owner:** DEV-B.

**Files:**
- Create: `harness/loop.py`

**Interfaces:**
- Consumes: `World`, `DroneState`, `make_drone`, `step_physics`, `min_clearance`, `is_inside_any`, `build_sensor_packet`, `Firmware` (duck-typed), `RunResult`, `Outcome`, `MotorCommand`.
- Produces:
  - `run_episode(world: World, firmware_obj, seed: int, max_t: float = 30.0, render_cb=None) -> RunResult`
  - `load_firmware_class(version: str) -> type` — dynamic import of `firmware.firmware_{version}.Firmware`. Reloads module each call so state never leaks between runs.

- [ ] **Step 1: Write the file**

```python
import importlib
import math
import time
import numpy as np

from firmware.contract import MotorCommand, SensorPacket
from harness.scoring import Outcome, RunResult
from sim.physics import make_drone, step_physics, is_inside_any, min_clearance
from sim.sensors import build_sensor_packet
from sim.world import World

DT = 1.0 / 50.0
TICK_BUDGET_MS = 50.0

def load_firmware_class(version: str):
    mod_name = f"firmware.firmware_{version}"
    if mod_name in __import__("sys").modules:
        mod = importlib.reload(__import__("sys").modules[mod_name])
    else:
        mod = importlib.import_module(mod_name)
    return mod.Firmware

def run_episode(world: World, firmware_obj, seed: int, max_t: float = 30.0, render_cb=None) -> RunResult:
    rng = np.random.default_rng(seed)
    drone = make_drone(world.spawn)
    tick_times_ms: list[float] = []
    overruns = 0
    min_clr = float("inf")
    outcome = Outcome.TIMEOUT
    fault_msg: str | None = None

    while drone.t < max_t:
        packet = build_sensor_packet(drone, world.obstacles, DT, rng)

        t0 = time.perf_counter()
        try:
            cmd = firmware_obj.step(packet)
            if not isinstance(cmd, MotorCommand):
                raise TypeError(f"firmware returned {type(cmd).__name__}, expected MotorCommand")
            cmd = cmd.clipped()
        except Exception as e:
            outcome = Outcome.FAULT
            fault_msg = f"{type(e).__name__}: {e}"
            break
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        tick_times_ms.append(elapsed_ms)
        if elapsed_ms > TICK_BUDGET_MS:
            overruns += 1

        step_physics(drone, cmd.thrust, DT)

        clr = min_clearance(world.obstacles, drone.x, drone.y)
        if clr < min_clr:
            min_clr = clr

        if render_cb is not None:
            render_cb(drone, packet, cmd)

        gx, gy = world.goal_center
        if math.hypot(drone.x - gx, drone.y - gy) <= world.goal_radius:
            outcome = Outcome.SUCCESS
            break
        if is_inside_any(world.obstacles, drone.x, drone.y):
            outcome = Outcome.CRASH
            break
        if drone.x < 0 or drone.x > world.arena_w or drone.y < 0 or drone.y > world.arena_h:
            outcome = Outcome.CRASH
            fault_msg = "out of bounds"
            break

    mean_tick = sum(tick_times_ms) / len(tick_times_ms) if tick_times_ms else 0.0
    max_tick = max(tick_times_ms) if tick_times_ms else 0.0
    return RunResult(
        seed=seed,
        outcome=outcome,
        t_end=drone.t,
        min_clearance=min_clr if min_clr != float("inf") else 0.0,
        mean_tick_ms=mean_tick,
        max_tick_ms=max_tick,
        num_overruns=overruns,
        fault_msg=fault_msg,
    )
```

- [ ] **Step 2: Smoke-run (no render, no firmware yet — just verify import works)**

```bash
python -c "from harness.loop import run_episode, load_firmware_class; print('imports ok')"
```

Expected: `imports ok`

- [ ] **Step 3: Commit**

```bash
git add harness/loop.py
git commit -m "feat(harness): 50Hz episode runner with telemetry and crash isolation"
git push
```

---

### Task 9: CLI entry — `harness/run.py` [DEV-B]

**Owner:** DEV-B.

**Files:**
- Create: `harness/run.py`

**Interfaces:**
- Consumes: `load_firmware_class`, `run_episode`, `build_world`, `summarize_sweep`, `format_result`.
- Produces: a CLI: `python -m harness.run --firmware {v1|v2} [--seed N] [--render] [--sweep]`.

- [ ] **Step 1: Write the file**

```python
import argparse

from harness.loop import load_firmware_class, run_episode
from harness.scoring import format_result, summarize_sweep
from sim.world import build_world

SWEEP_SEEDS = [42, 7, 13, 99, 256]

def _make_render_cb(world):
    from sim.renderer import Renderer
    r = Renderer(world)
    def cb(drone, packet, cmd):
        r.draw(drone, packet, cmd)
    return cb, r

def _run_one(version: str, seed: int, render: bool):
    world = build_world(seed)
    FirmwareCls = load_firmware_class(version)
    fw = FirmwareCls()
    cb = None
    r = None
    if render:
        cb, r = _make_render_cb(world)
    try:
        result = run_episode(world, fw, seed=seed, render_cb=cb)
    finally:
        if r is not None:
            r.close()
    print(format_result(result))
    return result

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--firmware", required=True, choices=["v1", "v2"])
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--render", action="store_true")
    p.add_argument("--sweep", action="store_true")
    args = p.parse_args()

    if args.sweep:
        results = [_run_one(args.firmware, s, render=False) for s in SWEEP_SEEDS]
        print(summarize_sweep(results))
    else:
        _run_one(args.firmware, args.seed, render=args.render)

if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Verify CLI parses (no firmware yet — will fail at import of v2 which is fine for now)**

```bash
python -m harness.run --firmware v1 --seed 42
```

Expected: prints a `format_result` line — outcome is whatever v1 produces. If DEV-A has already landed Task 4, this should print `CRASH` on seed 42. Otherwise `ModuleNotFoundError` is acceptable until v1 lands.

- [ ] **Step 3: Commit**

```bash
git add harness/run.py
git commit -m "feat(harness): CLI with --render and --sweep modes"
git push
```

---

### Task 10: Renderer — `sim/renderer.py` [DEV-B]

**Owner:** DEV-B. Can start in parallel with Task 8 once Task 6 (physics) lands.

**Files:**
- Create: `sim/renderer.py`

**Interfaces:**
- Consumes: `World`, `DroneState`, `SensorPacket`, `MotorCommand`.
- Produces: `Renderer` class with `__init__(world)`, `draw(drone, packet, cmd)`, `close()`. Renders at ~30 Hz (skips frames if sim ticks faster).

- [ ] **Step 1: Write the file**

```python
import math
import time
import pygame

from firmware.contract import MotorCommand, SensorPacket
from sim.physics import DroneState
from sim.world import World

PX_PER_M = 40
RENDER_HZ = 30.0
RENDER_DT = 1.0 / RENDER_HZ

BLACK = (0, 0, 0)
WHITE = (240, 240, 240)
RED = (220, 60, 60)
GREEN = (60, 200, 80)
BLUE = (80, 140, 230)
GREY = (130, 130, 130)

class Renderer:
    def __init__(self, world: World):
        pygame.init()
        self.world = world
        self.w_px = int(world.arena_w * PX_PER_M)
        self.h_px = int(world.arena_h * PX_PER_M)
        self.screen = pygame.display.set_mode((self.w_px, self.h_px))
        pygame.display.set_caption("Drone Firmware Trainer")
        self.font = pygame.font.SysFont("consolas", 14)
        self._last_render = 0.0

    def _to_px(self, x: float, y: float) -> tuple[int, int]:
        return int(x * PX_PER_M), self.h_px - int(y * PX_PER_M)

    def draw(self, drone: DroneState, packet: SensorPacket, cmd: MotorCommand) -> None:
        now = time.perf_counter()
        if now - self._last_render < RENDER_DT:
            return
        self._last_render = now

        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit()
                raise SystemExit(0)

        self.screen.fill(WHITE)

        for r in self.world.obstacles:
            x_px, y_px = self._to_px(r.x, r.y + r.h)
            pygame.draw.rect(self.screen, GREY, (x_px, y_px, int(r.w * PX_PER_M), int(r.h * PX_PER_M)))

        gx, gy = self._to_px(*self.world.goal_center)
        pygame.draw.circle(self.screen, GREEN, (gx, gy), int(self.world.goal_radius * PX_PER_M), width=3)

        dx, dy = self._to_px(drone.x, drone.y)
        yaw = math.atan2(drone.vy, drone.vx) if (drone.vx or drone.vy) else 0.0
        for i, ray in enumerate(packet.rays):
            ang = yaw + i * (2 * math.pi / len(packet.rays))
            ex_px, ey_px = self._to_px(drone.x + math.cos(ang) * ray, drone.y + math.sin(ang) * ray)
            pygame.draw.line(self.screen, BLUE, (dx, dy), (ex_px, ey_px), 1)
        pygame.draw.circle(self.screen, RED, (dx, dy), 6)

        hud = self.font.render(
            f"t={drone.t:5.2f}s  pos=({drone.x:4.1f},{drone.y:4.1f})  thr=({cmd.thrust[0]:+.2f},{cmd.thrust[1]:+.2f})",
            True, BLACK,
        )
        self.screen.blit(hud, (10, 10))

        pygame.display.flip()

    def close(self) -> None:
        pygame.quit()
```

- [ ] **Step 2: Visual smoke (only if firmware_v1 already landed)**

```bash
python -m harness.run --firmware v1 --seed 42 --render
```

Expected: pygame window opens, you see the drone, obstacles, goal, ray casts, drone moves toward goal, eventually crashes. If v1 not landed yet, skip.

- [ ] **Step 3: Commit**

```bash
git add sim/renderer.py
git commit -m "feat(sim): pygame renderer with rays and HUD"
git push
```

---

### Task 11: End-to-end smoke test — `tests/test_smoke.py` [Either]

**Owner:** Whoever finishes their track first.

**Files:**
- Create: `tests/test_smoke.py`

**Interfaces:**
- Consumes: everything via the public API.
- Produces: one test, runnable as `python -m tests.test_smoke` (no pytest required, keeps deps minimal).

- [ ] **Step 1: Write the file**

```python
"""End-to-end smoke test. Runs harness with v1, seed 42, no render.
Confirms the loop completes and returns a finished RunResult."""

from harness.loop import load_firmware_class, run_episode
from harness.scoring import Outcome, format_result
from sim.world import build_world

def main():
    world = build_world(42)
    Fw = load_firmware_class("v1")
    fw = Fw()
    result = run_episode(world, fw, seed=42, max_t=30.0)
    print(format_result(result))
    assert result.outcome in {Outcome.SUCCESS, Outcome.CRASH, Outcome.TIMEOUT, Outcome.FAULT}
    assert result.t_end > 0.0
    assert result.mean_tick_ms >= 0.0
    print("SMOKE OK")

if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run it**

```bash
python -m tests.test_smoke
```

Expected: prints a `format_result` line then `SMOKE OK`. On seed 42 with firmware_v1, outcome should be `CRASH` (proving the failure mode is real).

- [ ] **Step 3: Commit**

```bash
git add tests/test_smoke.py
git commit -m "test: end-to-end smoke confirming harness completes"
git push
```

---

### Task 12: Firmware v2 — patched controller [DEV-A]

**Owner:** DEV-A. Run AFTER Task 11 smoke confirms v1 actually crashes on seed 42. Watch the render run first to see how/where it fails.

**Files:**
- Create: `firmware/firmware_v2.py`

**Interfaces:**
- Consumes: `SensorPacket`, `MotorCommand`.
- Produces: `Firmware` class that succeeds on seed 42 and >=4/5 sweep seeds.

**Design intent:** v2 uses ALL 8 rays. It computes a repulsion vector summed across rays (closer = stronger), adds the attraction toward the goal, and steers along the sum. This is the patch the demo narrates.

- [ ] **Step 1: Write the file**

```python
import math
from firmware.contract import SensorPacket, MotorCommand

GOAL_X, GOAL_Y = 18.0, 13.0
RAY_MAX = 4.0
REPULSION_GAIN = 1.5
ATTRACTION_GAIN = 0.8

class Firmware:
    def __init__(self) -> None:
        self._last_thrust = (0.0, 0.0)

    def step(self, sensors: SensorPacket) -> MotorCommand:
        px, py = sensors.pos_estimate
        yaw = sensors.yaw

        # attraction toward goal
        dx, dy = GOAL_X - px, GOAL_Y - py
        norm = math.hypot(dx, dy) or 1.0
        ax = ATTRACTION_GAIN * dx / norm
        ay = ATTRACTION_GAIN * dy / norm

        # repulsion from all 8 rays
        rx = ry = 0.0
        n = len(sensors.rays)
        for i, ray in enumerate(sensors.rays):
            if ray >= RAY_MAX * 0.95:
                continue
            ang = yaw + i * (2 * math.pi / n)
            strength = REPULSION_GAIN * (1.0 - ray / RAY_MAX) ** 2
            rx -= math.cos(ang) * strength
            ry -= math.sin(ang) * strength

        # low-pass smoothing
        fx_raw = ax + rx
        fy_raw = ay + ry
        fx = 0.7 * fx_raw + 0.3 * self._last_thrust[0]
        fy = 0.7 * fy_raw + 0.3 * self._last_thrust[1]
        self._last_thrust = (fx, fy)

        return MotorCommand(thrust=(fx, fy)).clipped()
```

- [ ] **Step 2: Verify on seed 42 with render**

```bash
python -m harness.run --firmware v2 --seed 42 --render
```

Expected: drone navigates around the diagonal wall, reaches goal. Output line: `SUCCESS @ t=<something under 25s>`.

- [ ] **Step 3: Run the sweep**

```bash
python -m harness.run --firmware v2 --sweep
```

Expected: at least `4/5 success`. If only 3/5, tune `REPULSION_GAIN` (raise) or `ATTRACTION_GAIN` (lower) and re-run. If only 0/5, look at the render output and debug — likely sign error or wrong angle convention.

- [ ] **Step 4: Confirm v1 still fails the sweep**

```bash
python -m harness.run --firmware v1 --sweep
```

Expected: `1/5 success` or `0/5`. If v1 accidentally succeeds on seed 42 → world layout in Task 5 needs a tighter diagonal block.

- [ ] **Step 5: Commit**

```bash
git add firmware/firmware_v2.py
git commit -m "feat(firmware): v2 with potential-field obstacle avoidance"
git push
```

---

### Task 13: Demo polish + runbook [Either]

**Owner:** Whoever has cycles. Last 10 minutes.

**Files:**
- Modify: `README.md`

**Interfaces:**
- Produces: a README good enough to read live at the demo.

- [ ] **Step 1: Replace README contents with demo-ready version**

```markdown
# Drone Firmware Trainer

A 2D obstacle-avoidance training harness for autonomous drone firmware.

The firmware module is a Python class with a fixed contract (`SensorPacket` in, `MotorCommand` out). The harness enforces microcontroller-like constraints at the wrapper: 50 Hz fixed tick, noisy quantized sensors, crash isolation, no global state, tick-time budget.

## Install

    python -m venv .venv
    .venv\Scripts\activate
    pip install -r requirements.txt

## Demo

Run v1 (naive controller — fails):

    python -m harness.run --firmware v1 --seed 42 --render

Watch it crash into the diagonal wall. Telemetry tells us v1 only uses the forward ray.

Run v2 (uses all 8 rays, adds repulsion field):

    python -m harness.run --firmware v2 --seed 42 --render

Now it routes around obstacles to the goal.

Sweep across 5 seeds:

    python -m harness.run --firmware v1 --sweep
    python -m harness.run --firmware v2 --sweep

Expected: v1 = 1/5, v2 = 5/5.

## Architecture

- `firmware/contract.py` — frozen `SensorPacket` and `MotorCommand`. Locked first; never edited after.
- `firmware/firmware_v1.py`, `firmware/firmware_v2.py` — controllers under test.
- `sim/` — world, physics, sensors, renderer. Never imports `firmware/`.
- `harness/` — fixed-rate loop, scoring, CLI. Wires `sim` to `firmware`.
```

- [ ] **Step 2: Final smoke**

```bash
python -m tests.test_smoke
python -m harness.run --firmware v1 --sweep
python -m harness.run --firmware v2 --sweep
```

Expected: smoke ok, v1 sweep ≤ 1/5, v2 sweep ≥ 4/5.

- [ ] **Step 3: Commit + push**

```bash
git add README.md
git commit -m "docs: demo-ready README"
git push
```

---

## Time Budget

| Phase | Tasks | Duration | Owner |
|---|---|---|---|
| 0 | T1 setup | 10 min | Either |
| 1 | T2 contract (paired, BLOCKING) | 10 min | Both |
| 2 | T3, T4 | 30 min | DEV-A |
| 2 | T5, T6, T7 | 40 min | DEV-B |
| 3 | T8, T9 | 25 min | DEV-B |
| 3 | T10 renderer | 15 min | DEV-B (parallel) |
| 4 | T11 smoke | 5 min | Either |
| 5 | T12 firmware_v2 + tuning | 15 min | DEV-A |
| 6 | T13 demo polish | 10 min | Either |
| | **Total wall-clock with parallelism** | **~110 min** | |

Buffer: 10 min. If T12 sweep falls short, spend it tuning gains rather than rewriting.

## Self-Review Notes

- Every spec section maps to at least one task:
  - Spec §1 demo flow → T12 + T13
  - Spec §3 architecture → T1 file layout + dependency rule enforced by T8 imports
  - Spec §4 contract → T2 (verbatim names)
  - Spec §5 MCU spoofing → T7 (noise + quant), T8 (50 Hz, tick budget, crash isolation, module reload)
  - Spec §6 world/scoring → T5, T6, T8 (termination), T3 (scoring record)
  - Spec §7 file ownership → encoded in per-task Owner field
  - Spec §8 tech stack → T1
  - Spec §9 runbook → T13
  - Spec §10 risk: contract churn → T2 verbal lock; renderer eating budget → T10 RENDER_HZ throttle; v2 doesn't fix v1 → T5 + T12 deliberately paired
- Type names match across tasks: `SensorPacket`, `MotorCommand`, `DroneState`, `Rect`, `World`, `RunResult`, `Outcome`, `Firmware`.
- No TBDs or placeholders; every code step has complete runnable code.
