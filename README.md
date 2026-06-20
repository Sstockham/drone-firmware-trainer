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