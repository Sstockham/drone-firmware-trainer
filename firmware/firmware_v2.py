import math
from firmware.contract import SensorPacket, MotorCommand

GOAL_X, GOAL_Y = 18.0, 13.0
RAY_MAX = 4.0
REPULSION_GAIN = 1.5
ATTRACTION_GAIN = 0.8

# Stuck-detection / escape (Option A: smoothed checkpoint + perpendicular bias)
SMOOTH_ALPHA = 0.02        # slow IIR filter — tracks 50-tick average position
STUCK_CHECK_INTERVAL = 50  # ticks between progress checks (~1s at 50Hz)
STUCK_GUARD_TICKS = 250    # don't check before this tick (~5s) — avoid startup fp
STUCK_DISP_THR = 0.25      # smoothed-position net displacement threshold
ESCAPE_TICKS = 200         # ticks of escape maneuver (~4s)
ESCAPE_PERP_GAIN = 1.0     # perpendicular bias strength

class Firmware:
    def __init__(self) -> None:
        self._last_thrust = (0.0, 0.0)
        self._smooth_px: float | None = None
        self._smooth_py: float | None = None
        self._checkpoint_px = 0.0
        self._checkpoint_py = 0.0
        self._tick = 0
        self._escape_remaining = 0
        self._escape_sign = -1.0    # first toggle → +1 (left-up, escapes the min)

    def step(self, sensors: SensorPacket) -> MotorCommand:
        px, py = sensors.pos_estimate
        yaw = sensors.yaw
        self._tick += 1

        # slow IIR filter to get stable position estimate immune to noise
        if self._smooth_px is None:
            self._smooth_px, self._smooth_py = px, py
            self._checkpoint_px, self._checkpoint_py = px, py
        else:
            self._smooth_px += SMOOTH_ALPHA * (px - self._smooth_px)
            self._smooth_py += SMOOTH_ALPHA * (py - self._smooth_py)

        # progress check every STUCK_CHECK_INTERVAL ticks (after guard period)
        if self._tick % STUCK_CHECK_INTERVAL == 0 and self._tick >= STUCK_GUARD_TICKS:
            disp = math.hypot(
                self._smooth_px - self._checkpoint_px,
                self._smooth_py - self._checkpoint_py,
            )
            if disp < STUCK_DISP_THR and self._escape_remaining == 0:
                self._escape_remaining = ESCAPE_TICKS
                self._escape_sign *= -1.0
            self._checkpoint_px = self._smooth_px
            self._checkpoint_py = self._smooth_py

        # attraction toward goal
        dx, dy = GOAL_X - px, GOAL_Y - py
        norm = math.hypot(dx, dy) or 1.0
        ux, uy = dx / norm, dy / norm
        ax = ATTRACTION_GAIN * ux
        ay = ATTRACTION_GAIN * uy

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

        fx_raw = ax + rx
        fy_raw = ay + ry

        # escape: perpendicular-to-goal bias to break symmetric local minimum
        if self._escape_remaining > 0:
            perp_x = -uy * self._escape_sign
            perp_y =  ux * self._escape_sign
            fx_raw += ESCAPE_PERP_GAIN * perp_x
            fy_raw += ESCAPE_PERP_GAIN * perp_y
            self._escape_remaining -= 1

        # low-pass smoothing
        fx = 0.7 * fx_raw + 0.3 * self._last_thrust[0]
        fy = 0.7 * fy_raw + 0.3 * self._last_thrust[1]
        self._last_thrust = (fx, fy)

        return MotorCommand(thrust=(fx, fy)).clipped()
