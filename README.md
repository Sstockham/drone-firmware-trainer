# Drone Firmware Trainer

2D and 3D obstacle-avoidance training harness for autonomous drone firmware.

The firmware module is a Python class with a fixed contract (`SensorPacket` in, `MotorCommand` out for 2D; `SensorPacket3D`/`MotorCommand3D` for 3D). The harness enforces microcontroller-like constraints at the wrapper: 50 Hz fixed tick, noisy quantized sensors, crash isolation, no global state, tick-time budget.

## Install

    python -m venv .venv
    .venv\Scriptsctivate
    pip install -r requirements.txt

## 2D Demo

Run v1 (naive controller, fails):

    python -m harness.run --firmware v1 --seed 42 --render

Run v2 (8-ray potential field, succeeds):

    python -m harness.run --firmware v2 --seed 42 --render

Sweep:

    python -m harness.run --firmware v1 --sweep    # 1/5
    python -m harness.run --firmware v2 --sweep    # 5/5

## 3D Demo

Run v1_3d (naive 2D logic + dumb altitude hold, ignores up/down rays):

    python -m harness.run3d --firmware v1_3d --seed 42 --render

Run v2_3d (3D potential field + ceiling/floor repulsion + escape):

    python -m harness.run3d --firmware v2_3d --seed 42 --render

While rendering:

- Press **C** to toggle CHASE camera (default) and COCKPIT camera.
- Press **T** to toggle NOISY (firmware-view) and TRUE (ground-truth) HUD.
- Close the window or press any key on the final banner to exit.

Sweep:

    python -m harness.run3d --firmware v1_3d --sweep    # <= 1/5
    python -m harness.run3d --firmware v2_3d --sweep    # >= 4/5

## Architecture

- `firmware/contract.py`, `firmware/contract3d.py` — frozen contract dataclasses. Locked first; never edited after.
- `firmware/firmware_v1.py`, `firmware_v2.py` — 2D controllers under test.
- `firmware/firmware_v1_3d.py`, `firmware_v2_3d.py` — 3D controllers under test.
- `sim/` (pygame) — 2D world, physics, sensors, renderer.
- `sim3d/` (raylib-py) — 3D world, physics, sensors, renderer with fighter-cockpit HUD.
- `harness/loop.py` + `run.py` wire the 2D sim to firmware; `harness/loop3d.py` + `run3d.py` do the 3D version. The 2D and 3D harnesses are completely separate.
