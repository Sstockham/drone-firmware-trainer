import importlib
import math
import time
import numpy as np

from firmware.contract3d import MotorCommand3D, SensorPacket3D
from harness.scoring import Outcome, RunResult
from sim3d.physics import make_drone3d, step_physics3d, is_inside_any3d, min_clearance3d
from sim3d.sensors import build_sensor_packet3d
from sim3d.world import World3D

DT = 1.0 / 50.0
TICK_BUDGET_MS = 50.0

def load_firmware_class_3d(version: str):
    mod_name = f"firmware.firmware_{version}_3d"
    if mod_name in __import__("sys").modules:
        mod = importlib.reload(__import__("sys").modules[mod_name])
    else:
        mod = importlib.import_module(mod_name)
    return mod.Firmware

def run_episode_3d(world: World3D, firmware_obj, seed: int, max_t: float = 45.0, render_cb=None) -> RunResult:
    rng = np.random.default_rng(seed)
    drone = make_drone3d(world.spawn)
    tick_times_ms: list[float] = []
    overruns = 0
    min_clr = float("inf")
    outcome = Outcome.TIMEOUT
    fault_msg: str | None = None
    next_wall = time.perf_counter() if render_cb is not None else None

    while drone.t < max_t:
        packet = build_sensor_packet3d(drone, world.obstacles, DT, rng)

        t0 = time.perf_counter()
        try:
            cmd = firmware_obj.step(packet)
            if not isinstance(cmd, MotorCommand3D):
                raise TypeError(f"firmware returned {type(cmd).__name__}, expected MotorCommand3D")
            cmd = cmd.clipped()
        except Exception as e:
            outcome = Outcome.FAULT
            fault_msg = f"{type(e).__name__}: {e}"
            break
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        tick_times_ms.append(elapsed_ms)
        if elapsed_ms > TICK_BUDGET_MS:
            overruns += 1

        step_physics3d(drone, cmd.thrust, DT)

        clr = min_clearance3d(world.obstacles, drone.x, drone.y, drone.z)
        if clr < min_clr:
            min_clr = clr

        if render_cb is not None:
            render_cb(drone, packet, cmd, firmware_obj)
            next_wall += DT
            slack = next_wall - time.perf_counter()
            if slack > 0:
                time.sleep(slack)
            else:
                next_wall = time.perf_counter()

        gx, gy, gz = world.goal_center
        if math.sqrt((drone.x - gx) ** 2 + (drone.y - gy) ** 2 + (drone.z - gz) ** 2) <= world.goal_radius:
            outcome = Outcome.SUCCESS
            break
        if is_inside_any3d(world.obstacles, drone.x, drone.y, drone.z):
            outcome = Outcome.CRASH
            break
        if drone.x < 0 or drone.x > world.arena_w or drone.y < 0 or drone.y > world.arena_d:
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
