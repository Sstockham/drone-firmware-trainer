# Drone Firmware Trainer

A 2D obstacle-avoidance training harness for autonomous drone firmware.

The firmware module is a Python class with a fixed contract (`SensorPacket` in, `MotorCommand` out). The harness enforces microcontroller-like constraints at the wrapper: 50 Hz fixed tick, noisy quantized sensors, crash isolation, no global state, tick-time budget.

## Install

    python -m venv .venv
    .venv\Scriptsctivate
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
