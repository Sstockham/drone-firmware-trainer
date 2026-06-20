# Drone Firmware Trainer — Design Spec

**Date:** 2026-06-20
**Context:** 2-hour AI hackathon, 2-person team
**Goal:** Demonstrate an iteration loop where a "drone firmware" module fails an obstacle-avoidance mission, is patched based on simulation telemetry, and succeeds on re-run.

---

## 1. Demo win condition

A single screen tells the story:

1. Run `python harness/run.py --firmware v1` → drone flies, crashes into a wall, harness prints `FAIL (collision @ t=4.2s, obstacle #3)`.
2. Open `firmware/firmware_v2.py` (a patched copy of v1).
3. Run `python harness/run.py --firmware v2` → drone navigates around obstacles, reaches goal, harness prints `SUCCESS (12.7s, min clearance 0.18m)`.
4. Run a 5-seed sweep: v1 = 1/5, v2 = 5/5.

The "wow" is the iteration story itself: this is a training harness for autonomous drone firmware.

## 2. Non-goals

- 3D physics, 3D rendering, 3D contract shaping. Strictly 2D throughout.
- Real microcontroller emulation (QEMU, Renode, etc.). We enforce constraints at the Python wrapper boundary, not inside a CPU emulator.
- Opposition drones, moving targets, wind, partial observability beyond sensor noise.
- Real quadrotor dynamics (rotor thrust, body torques). Kinematic point mass only.
- Networked play, multi-process, GPU.
- Tests beyond a single smoke test confirming the harness loop runs end-to-end.

## 3. Architecture

Three top-level packages communicating only through dataclasses defined in `firmware/contract.py`:

```
sim/
    world.py        # arena, obstacles, goal, seeded layout
    physics.py      # 2D kinematic point integrator
    sensors.py      # 8 ray-casts + IMU + noisy position
    renderer.py     # pygame visualization
firmware/
    contract.py     # SensorPacket, MotorCommand dataclasses (LOCKED early)
    firmware_v1.py  # naive — fails
    firmware_v2.py  # patched — succeeds
harness/
    loop.py         # fixed-rate control loop, wires sim <-> firmware
    scoring.py      # success/fail, time-to-goal, min clearance
    run.py          # CLI entry point
```

**Dependency rule:** `sim` does not import `firmware`. `firmware` does not import `sim`. `harness` imports both and wires them. This is what makes the file-ownership split (Section 7) merge-conflict-free.

## 4. Firmware contract (locked first)

`firmware/contract.py`:

```python
from dataclasses import dataclass

@dataclass(frozen=True)
class SensorPacket:
    rays: tuple[float, ...]        # 8 range readings, meters, noisy, capped 4.0
    imu_accel: tuple[float, float] # (ax, ay) m/s^2, noisy
    yaw: float                     # radians, noisy
    pos_estimate: tuple[float, float]  # (x, y) m, noisy
    dt: float                      # seconds since last tick

@dataclass(frozen=True)
class MotorCommand:
    thrust: tuple[float, float]    # (fx, fy) body-frame force, N, clipped to [-1, 1]
```

A firmware module exports:

```python
class Firmware:
    def __init__(self) -> None: ...
    def step(self, sensors: SensorPacket) -> MotorCommand: ...
```

No other public symbols, no globals read at step time, no file/network I/O.

## 5. MCU constraint spoofing

Enforced by `harness/loop.py`, not inside firmware:

| Real MCU constraint | Wrapper enforcement |
|---|---|
| Fixed control loop rate | `step()` called at exactly 50 Hz; overruns logged |
| Limited sensor inputs | Only `SensorPacket` crosses the boundary |
| Bounded compute per tick | `perf_counter` around each `step()`, soft cap 50 ms, log only |
| Noisy sensors | Gaussian noise applied in `sim/sensors.py` before packet build |
| Quantized sensor values | Ray, accel, yaw, pos values rounded to 0.01 |
| No global state | Firmware instantiated once per run; module reloaded between runs |
| Crash isolation | `try/except` around `step()`; on exception, log `FAULT` and zero thrust |

## 6. World, mission, scoring

- **Arena:** 800×600 px, 1 px = 0.025 m → 20 m × 15 m logical world.
- **Obstacles:** ~10 axis-aligned rectangles placed from a fixed seed. Demo uses seed `42`; harness sweep uses seeds `[42, 7, 13, 99, 256]`.
- **Spawn:** bottom-left, drone at (1.0, 1.0).
- **Goal:** top-right green circle, center (18.0, 13.0), radius 0.75 m.
- **Drone:** kinematic point, mass 0.5 kg, linear drag coefficient 0.4, thrust clipped per-axis to [-1, 1] N (infinity-norm bound). Yaw is derived from velocity direction (`atan2(vy, vx)`); not a controllable DOF and not a torque input. Sensors are body-frame so the firmware still sees rotated readings.
- **Sensors:** 8 ray-casts at 45° spacing in body frame, max range 4 m, Gaussian noise σ = 0.03 m. IMU noise σ = 0.05 m/s². Position estimate noise σ = 0.05 m. Yaw noise σ = 0.02 rad.
- **Termination conditions (checked in order):**
  - Firmware exception → `FAULT`.
  - Drone center inside goal circle → `SUCCESS`.
  - Drone center inside any obstacle rectangle → `CRASH`.
  - Sim time > 30 s → `TIMEOUT`.
- **Score record per run:** `{seed, outcome, t_end, min_clearance, mean_tick_ms, max_tick_ms, num_overruns}`.

## 7. Repo topology and collaboration

- **One GitHub repo, single `main` branch, push every ~10 minutes.** No PRs, no feature branches.
- **Hard file ownership** so two devs never edit the same file:

  | Owner | Files |
  |---|---|
  | You | `firmware/contract.py` (first, then locked), `firmware/firmware_v1.py`, `firmware/firmware_v2.py`, `harness/scoring.py` |
  | Teammate | `sim/world.py`, `sim/physics.py`, `sim/sensors.py`, `sim/renderer.py`, `harness/loop.py`, `harness/run.py` |

- **First 10 minutes:** both pair on `firmware/contract.py`, commit, push. After that the file is treated as frozen — any change requires a verbal sync.
- **Shared state crosses only via the contract dataclasses.** `sim` cannot import `firmware`; `firmware` cannot import `sim`; `harness` wires them.
- **`README.md` and `pyproject.toml` (or `requirements.txt`)** are written by whoever lands them first and rarely touched after.

## 8. Tech stack

- Python 3.11+, pygame, numpy. Nothing else.
- Windows-friendly (matches global CLAUDE.md preferences).
- No native build dependencies, no compiled libraries beyond pygame's bundled wheels.

## 9. Demo runbook

1. `python harness/run.py --firmware v1 --seed 42 --render` → visible crash.
2. `python harness/run.py --firmware v1 --sweep` → prints `1/5 success`.
3. Show `firmware/firmware_v2.py` diff vs v1 (~20-40 lines).
4. `python harness/run.py --firmware v2 --seed 42 --render` → visible success.
5. `python harness/run.py --firmware v2 --sweep` → prints `5/5 success`.

## 10. Risk register

| Risk | Mitigation |
|---|---|
| Pygame install issues on teammate's machine | Pin version, verify install in first 10 min before splitting work |
| Contract churn after lock | Hard rule: contract change requires verbal sync, both devs pause |
| v2 doesn't actually solve v1's failure | Pre-decide the failure mode (e.g., v1 ignores diagonal rays) so v2's fix is a known one-liner-class change |
| Ray-cast vs axis-aligned rectangle math bug eats 30 min | Use a known-good slab method; write a 3-line smoke check |
| Renderer eats the loop budget on slower hardware | Render at 30 Hz while sim runs at 50 Hz; renderer reads latest state only |
| Time overrun | Strict 90-min code freeze, last 30 min is demo prep |

## 11. Out-of-scope follow-ups (future work)

- 3D upgrade (contract reshape to include z-axis and elevation, 3D renderer, quadrotor dynamics).
- Opposition drones.
- True MCU emulation via QEMU or Renode.
- LLM-authored firmware patches between runs.
- Real microcontroller deployment (RAK11300 or similar) using the same contract.
