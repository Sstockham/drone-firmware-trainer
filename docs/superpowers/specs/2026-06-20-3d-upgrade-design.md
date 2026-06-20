# 3D Upgrade — Design Spec

**Date:** 2026-06-20
**Context:** Follow-up to the 2D Drone Firmware Trainer (shipped same day). The 2D demo proved the iteration story (firmware v1 fails → patch → v2 succeeds). This spec adds a 3D simulation, a 3D renderer with a fighter-cockpit-style HUD, and a new 3D firmware iteration pair that demonstrates altitude-aware obstacle handling.
**Goal:** A 3D obstacle course where `firmware_v1_3d` fails three set-pieces (tall wall, short bar to fly over, hanging overhang to duck under) and `firmware_v2_3d` succeeds all three, rendered with chase + cockpit cameras and a noisy/true-toggleable cockpit HUD.

---

## 1. Demo win condition

1. `python -m harness.run3d --firmware v1_3d --seed 42 --render` → drone visibly attempts the route, crashes (most likely against the tall wall first). HUD active. Window holds on a `CRASH` banner until closed.
2. `python -m harness.run3d --firmware v2_3d --seed 42 --render` → drone routes around the tall wall, climbs over the short bar, descends through the overhang tunnel, lands inside the goal sphere. HUD active. Window holds on a `SUCCESS` banner.
3. During the v2_3d run: press **C** to toggle chase ↔ cockpit camera; press **T** to toggle TRUE ↔ NOISY HUD source. Both toggles work mid-flight without affecting firmware behavior.
4. Sweep: `--sweep` shows `v1_3d` ≤ 1/5, `v2_3d` ≥ 4/5 over `[42, 7, 13, 99, 256]` (5/5 preferred but 4/5 acceptable).

The 2D demo (`harness.run --firmware v1/v2`) must continue to work unchanged.

## 2. Non-goals

- Realistic quadrotor dynamics (rotor thrust, body torques, attitude as a controllable DOF). Drone remains a 3D kinematic point.
- Roll / pitch / yaw as inputs the firmware controls. Pitch is **derived for display only** (`atan2(vz, hypot(vx, vy))`). Yaw is derived from horizontal velocity (`atan2(vy, vx)`).
- Wind, gusts, sensor drift over time, IMU bias.
- Multi-drone, opposition, networked play.
- Custom drone mesh from `.obj`. Drone is a 0.3 m cube with a yellow forward wedge.
- Real microcontroller emulation. MCU constraints still spoofed at the harness wrapper exactly as in 2D.
- Modifying any existing 2D code (`sim/`, `firmware/contract.py`, `firmware/firmware_v1.py`, `firmware/firmware_v2.py`, `harness/loop.py`, `harness/run.py`, `harness/scoring.py`). Additions only.
- Tests beyond a single 3D smoke test mirroring the 2D one.

## 3. Architecture

Two new packages and one new harness entry point. The 2D world stays intact and runnable.

```
sim3d/
    __init__.py
    world.py        # Rect3D, World3D, build_world3d(seed)
    physics.py      # DroneState3D, step_physics3d, is_inside_any3d, min_clearance3d
    sensors.py      # build_sensor_packet3d (10 rays: 8 horizontal + up + down)
    renderer.py     # raylib-py: scene, chase + cockpit cameras, HUD overlay, hold()
firmware/
    contract.py             # untouched
    contract3d.py           # SensorPacket3D, MotorCommand3D  -- LOCKED first
    firmware_v1.py, v2.py   # untouched
    firmware_v1_3d.py       # naive: forward ray only + P-altitude-hold; ignores up/down rays
    firmware_v2_3d.py       # potential field + escape + altitude-aware fly-over and duck-under
harness/
    loop.py, run.py         # untouched
    scoring.py              # reused as-is (RunResult is dimension-agnostic)
    loop3d.py               # 50 Hz 3D-aware episode loop; imports only sim3d + contract3d
    run3d.py                # CLI: --firmware {v1_3d,v2_3d} --seed N --render --sweep --cam {chase,cockpit}
tests/
    test_smoke_3d.py        # mirrors tests/test_smoke.py for v1_3d/seed=42
```

**Dependency rule (carried forward from 2D, same shape):**
- `sim3d/` may import `firmware.contract3d` for type-only use (same boundary exception as 2D).
- `sim3d/` does NOT import from `firmware/` modules other than the contract.
- `firmware/` does NOT import from `sim/` or `sim3d/`.
- Only `harness/loop3d.py` and `harness/run3d.py` wire the two together.

## 4. Locked firmware contract (`firmware/contract3d.py`)

```python
from dataclasses import dataclass

@dataclass(frozen=True)
class SensorPacket3D:
    rays_h: tuple[float, float, float, float, float, float, float, float]  # 8 horizontal at 45°, body frame
    ray_up: float
    ray_down: float
    imu_accel: tuple[float, float, float]
    yaw: float                                  # derived from horizontal velocity
    pos_estimate: tuple[float, float, float]
    dt: float

@dataclass(frozen=True)
class MotorCommand3D:
    thrust: tuple[float, float, float]          # fx, fy, fz body-frame force, N

    @staticmethod
    def zero() -> "MotorCommand3D":
        return MotorCommand3D(thrust=(0.0, 0.0, 0.0))

    def clipped(self) -> "MotorCommand3D":
        fx, fy, fz = self.thrust
        return MotorCommand3D(thrust=(
            max(-1.0, min(1.0, fx)),
            max(-1.0, min(1.0, fy)),
            max(-1.0, min(1.0, fz)),
        ))
```

Firmware exports exactly:

```python
class Firmware:
    def __init__(self) -> None: ...
    def step(self, sensors: SensorPacket3D) -> MotorCommand3D: ...
```

No other public symbols. No globals. No I/O. Same wrapper-enforced MCU constraints as 2D (50 Hz fixed tick, sensors noisy + quantized, crash isolation, module reload between runs, 50 ms soft tick budget).

## 5. World (`sim3d/world.py`)

- Arena: 20 m × 15 m floor, **4.0 m ceiling**. Drone clamped to `0 ≤ z ≤ 4`.
- Spawn: `(1.0, 1.0, 1.5)`.
- Goal: sphere centered at `(18.0, 13.0, 1.5)`, radius **0.75 m**. SUCCESS = `dist(drone, goal_center) ≤ 0.75`.
- `Rect3D(x, y, z, w, d, h)` — axis-aligned prism, `(x, y, z)` is the min corner, `(w, d, h)` are extents. `contains_point3d(px, py, pz)` is the standard 3-axis range check.
- **Seed-42 hand-crafted set-pieces (must demo the v1→v2 patch story):**
  - **Tall wall:** `Rect3D(6.0, 6.0, 0.0, 4.0, 0.6, 4.0)` — direct path, full-height. Forces horizontal routing.
  - **Short bar:** `Rect3D(11.0, 9.0, 0.0, 3.0, 0.6, 1.0)` — sits on floor, 1.0 m tall. v2_3d climbs over.
  - **Hanging overhang:** `Rect3D(14.0, 11.5, 1.0, 0.6, 3.0, 3.0)` — suspended; bottom at z=1.0, top touches the 4.0 m ceiling. The base flight altitude (1.5 m) intersects it, so the drone MUST descend below 1.0 m to pass under. v1_3d (hold-1.5-only) clips the overhang; v2_3d's ceiling-rep pushes it down through the tunnel.
- **Random seeds (`_random_obstacles3d`):** 6–9 prisms with random heights (0.8–4.0 m), z-floor either 0.0 (floor-standing) or `4 - h` (ceiling-hung), reject if they contain spawn or goal center.

## 6. Physics (`sim3d/physics.py`)

- `DroneState3D`: mutable dataclass `x, y, z, vx, vy, vz, t`.
- `make_drone3d(spawn)` → fresh state, velocities = 0.
- Constants: `MASS = 0.5`, `DRAG = 0.4` (same as 2D — proven values).
- `step_physics3d(state, thrust, dt)`:
  - Per-axis ∞-norm clip on `(fx, fy, fz)` to `[-1, 1]`.
  - Per-axis acceleration `(f - DRAG * v) / MASS`.
  - Euler integrate velocity then position.
  - Clamp `z ≥ 0` and zero `vz` on floor contact. Clamp `z ≤ 4.0` and zero `vz` on ceiling contact (without crashing — just blocks).
  - Advance `t` by `dt`.
- `is_inside_any3d(rects, px, py, pz) -> bool`.
- `min_clearance3d(rects, px, py, pz) -> float` — signed nearest distance to any rect, negative if inside.

## 7. Sensors (`sim3d/sensors.py`)

10 rays per packet: 8 horizontal + up + down.

- `rays_h[i]` for `i in 0..7`: cast from drone in direction `(cos(yaw_true + i*π/4), sin(yaw_true + i*π/4), 0)` in the floor plane. March in 0.05 m steps, return first-hit distance to any `Rect3D` or `RAY_MAX = 4.0`.
- `ray_up`: cast straight up, range `[0, ceiling - z]` clipped to `RAY_MAX`.
- `ray_down`: cast straight down, range `[0, z]` clipped to `RAY_MAX`.
- All rays: add Gaussian noise σ = 0.03 m, clamp to `[0, RAY_MAX]`, quantize to 0.01.
- IMU accel: 3-tuple of zero-mean noise σ = 0.05 m/s², quantized.
- `yaw_true = atan2(vy, vx)` if either non-zero else 0.0; add σ = 0.02 rad noise, quantize.
- Position estimate: true (x, y, z) + σ = 0.05 m noise per axis, quantized.

## 8. Harness loop (`harness/loop3d.py`)

Same shape as `harness/loop.py`. Differences:

- `DT = 1/50`, `TICK_BUDGET_MS = 50.0`, `DEFAULT_MAX_T = 45.0` (was 30 in 2D — 3D paths longer with climbs/descents).
- `load_firmware_class_3d(version)` → imports `firmware.firmware_{version}_3d` with `importlib.reload` between runs.
- `run_episode_3d(world, fw, seed, max_t=45.0, render_cb=None)`:
  - Each tick: build `SensorPacket3D` → time `fw.step()` (≤ 50 ms soft) → verify `MotorCommand3D`, clip → step physics → update min clearance → optional `render_cb(drone, packet, cmd, fw)` → real-time pacing if `render_cb`.
  - The 4-arg `render_cb` signature (firmware passed last) lets the renderer probe optional firmware attributes like `stuck_active` via `getattr(fw, "stuck_active", False)` without coupling firmware ↔ renderer at the type level.
  - Termination order: SUCCESS (drone inside goal sphere), CRASH (drone inside any `Rect3D` OR z < 0 OR z > ceiling OR x/y out of arena), TIMEOUT (max_t).
  - Exception path: outcome=FAULT, fault_msg set.
- Returns the shared `RunResult` from `harness/scoring.py` unchanged.

## 9. Renderer (`sim3d/renderer.py`)

**Library:** `raylib-py` (pinned in `requirements.txt`). Pure pip install on Windows.

**Window:** 1280×720, default. Pattern: `init_window → BeginDrawing → BeginMode3D(active_cam) → 3D scene → EndMode3D → 2D HUD overlay → EndDrawing`. Render at ~60 Hz; sim loop calls `render_cb` every 50 Hz tick, renderer throttles draws.

**Scene primitives:**
- Ground: `draw_grid(slices=30, spacing=1.0)`.
- Each `Rect3D`: `draw_cube` + `draw_cube_wires` at center with extents `(w, d, h)`. Floor-standing prisms colored `(130, 130, 130, 200)`; ceiling-hung prisms colored `(100, 100, 130, 200)` so the player can tell them apart at a glance.
- Goal: pulsating `draw_sphere_wires(center=(18, 13, 1.5), radius=0.75)`, green, alpha modulated by `0.5 + 0.5*sin(2*pi*t)`.
- Drone: `draw_cube(drone_pos, 0.3, 0.3, 0.3, RED)`. Yellow wedge (small triangle drawn as three cubes for simplicity, or a single line from drone to `drone + 0.5*forward_unit`) indicates yaw.
- Rays: `draw_line_3d` from drone to ray end for each of the 10 rays, color blue, alpha 180.

**Cameras (toggle key `C`):**
- **Chase (default):** position = `drone + R_yaw * (-3.0, 0, +1.5)`; target = drone; up = `(0, 0, 1)`; FOV = 55°. Low-pass on position (α = 0.85) to smooth jitter.
- **Cockpit:** position = `drone + R_yaw * (0.2, 0, 0.05)`; target = `drone + R_yaw * (5.0, 0, 0.05)`; up = `(0, 0, 1)`; FOV = 70°.
- Both track the **true** drone state regardless of the T toggle.

**HUD (toggle key `T`: source NOISY ↔ TRUE; default = NOISY for honesty):**

Drawn after `EndMode3D`. Phosphor green `(80, 240, 120, 230)`, monospace. All numeric values pulled from `SensorPacket3D` when NOISY, from `DroneState3D` when TRUE.

- **Boresight cross** at screen center (10 px each arm).
- **Pitch ladder:** lines at ±10°, ±20°, ±30° pitch; scroll vertically by `pitch_deg * 4 px/deg`.
- **Heading tape (top):** 80 px tall, full-width, scrolling compass; tick every 10°, label N/E/S/W; current heading boxed at center.
- **Altitude tape (right):** scale 0–4 m, current z circled.
- **Airspeed tape (left):** scale 0–5 m/s, current speed `hypot(vx, vy, vz)` circled. (Speed source uses derivative of `pos_estimate` when NOISY, true velocity when TRUE.)
- **Vertical speed (right of altitude):** numeric `vz` in m/s with sign.
- **Velocity vector marker (FPM):** small circle at the projected position the drone is moving toward (drone + velocity * 5 m, projected to screen).
- **Target marker:** project goal center to screen → yellow diamond outline + `RNG nn.n m` text below. Off-screen → arrow at nearest screen edge.
- **Status row (bottom 30 px):** `t=12.34s  ·  tick μ/max 0.02/0.05 ms  ·  ovr 0` plus a `[STUCK]` flag when v2_3d's escape mechanism is active (firmware exposes this via the renderer's source — see Section 10).
- **Corner badges:** top-left `CHASE` or `COCKPIT`; top-right `NOISY` or `TRUE`.

**Hold-after-run:** `Renderer.hold(banner_text)` displays a centered banner (e.g., `SUCCESS  t=23.50s  seed=42  fw=v2_3d`) over the final frame and blocks until QUIT, any KEY, or mouse click. Same pattern as the 2D renderer.

## 10. Firmware

### `firmware_v1_3d.py` (intentional failure)

- Horizontal logic: identical to v1 — reads only `sensors.rays_h[0]`, naive thrust toward goal, brake if forward ray < 1.0 m.
- Altitude logic: P-controller `fz = K_z * (1.5 - z)` with `K_z = 0.5`. Ignores `ray_up` and `ray_down` entirely.
- Failure modes: smashes into the tall wall (same as 2D v1), drives into the side of the short bar (didn't fly over), and clips the overhang when forced upward.

### `firmware_v2_3d.py` (the patched controller)

- Horizontal: full v2 (potential field over `rays_h` + low-pass + stuck-detection + perpendicular-to-goal escape). Same constants as v2.
- Altitude:
  - Base hold: `fz_hold = K_z * (1.5 - z)`, `K_z = 0.5`.
  - Floor-rep: if `ray_down < 1.0` → `fz_up = REP_Z * (1 - ray_down/1.0)^2`.
  - Ceiling-rep: if `ray_up < 1.0` → `fz_down = -REP_Z * (1 - ray_up/1.0)^2`.
  - Total: `fz = clip(fz_hold + fz_up + fz_down, -1, 1)`.
  - `REP_Z = 1.2`.
- The stuck-flag from v2's escape mechanism is exposed read-only via `self.stuck_active: bool`. The renderer's HUD reads `firmware_obj.stuck_active` when the firmware exposes that attribute (renderer probes with `getattr(fw, "stuck_active", False)`).
- Same `MotorCommand3D(...).clipped()` return.

## 11. Mission, scoring, sweep

- SUCCESS: `dist((drone.x, drone.y, drone.z), goal_center) ≤ 0.75`.
- CRASH: drone center inside any `Rect3D`, OR `z < 0`, OR `z > 4.0`, OR `x < 0`, OR `x > 20.0`, OR `y < 0`, OR `y > 15.0`.
- TIMEOUT: `t > 45.0`.
- FAULT: any firmware exception.
- Sweep over `[42, 7, 13, 99, 256]`. Required: v1_3d ≤ 1/5 success, v2_3d ≥ 4/5 (target 5/5).
- `summarize_sweep` reused from `harness/scoring.py`.

## 12. Tech stack additions

- `raylib-py` (latest stable on PyPI, pinned exact version once installed). Add to `requirements.txt`. No other new dependencies.
- Python 3.11+, Windows, pure pip. Verify install on first task before any sim3d work.

## 13. Demo runbook

```
.venv\Scripts\activate

# 2D still works (regression check)
python -m harness.run --firmware v2 --seed 42 --render

# 3D fail → fix story
python -m harness.run3d --firmware v1_3d --seed 42 --render
# expect CRASH banner; HUD visible; close window to dismiss
python -m harness.run3d --firmware v2_3d --seed 42 --render
# expect SUCCESS banner; press C to toggle cockpit; press T to toggle NOISY/TRUE HUD

# Aggregate
python -m harness.run3d --firmware v1_3d --sweep
python -m harness.run3d --firmware v2_3d --sweep
```

## 14. Risk register

| Risk | Mitigation |
|---|---|
| `raylib-py` install or runtime issues on Windows (DLL hell, version mismatch) | First task is install + hello-world window. If broken, fall back to `pyglet` (similar API) or `ursina` (heavier but reliable). Escalate before sinking work into a broken renderer. |
| Seed-42 layout doesn't actually defeat v1_3d (drone gets lucky) | Verify v1_3d sweep ≤ 1/5 and CRASH on seed 42 as a gating step before declaring the failure half of the demo done. |
| v2_3d gets stuck in a 3D potential field local minimum | Mirror the 2D escape-mechanism approach; tune `REP_Z` and stuck-thresholds the same way. If a 3D local minimum persists, add a perpendicular-in-3D escape bias rather than reshaping the world. |
| HUD eats render budget; chase cam jitter or low FPS | Renderer is hardware-accelerated; HUD is simple text + lines; budget headroom is huge. If FPS drops, cull off-screen HUD elements and skip pitch-ladder lines outside ±30°. |
| Cockpit camera clips into walls | Camera offsets are inside the drone hitbox (0.3 m cube + 0.2 m forward offset = inside). If clipping is visible, push forward offset to 0.4 m and accept seeing the drone's nose. |
| Real-time pacing breaks at high obstacle counts | Sweep mode runs without render, no pacing penalty. Rendered runs pace to 50 Hz wall-clock; raylib + Windows clock resolution should hold. |

## 15. Out-of-scope follow-ups

- Rotor-level dynamics + body-frame attitude as a controllable DOF (true quadrotor sim).
- Wind, gusts, sensor drift, IMU bias over time.
- LLM-authored firmware patches between runs.
- Networked multi-drone, opposition agent.
- Real RAK11300 deployment using a shared contract abstraction.
- Custom drone mesh (`.obj`), prettier visuals (skybox, post-processing).
