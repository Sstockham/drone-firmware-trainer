import math
from firmware.contract3d import SensorPacket3D, MotorCommand3D

GOAL_X, GOAL_Y, GOAL_Z = 18.0, 13.0, 1.5
ALT_HOLD = 1.5
K_Z = 0.5
REP_Z = 1.2
REP_Z_RANGE = 1.0  # meters; activate when ray_up or ray_down < REP_Z_RANGE
RAY_MAX = 4.0
REPULSION_GAIN = 1.5
ATTRACTION_GAIN = 0.8
SMOOTH_ALPHA = 0.02   # IIR pole ~exp(-0.02); effective ~50-tick (1 s @ 50 Hz) window
STUCK_DISP_THR = 0.25
STUCK_TICKS = 250
ESCAPE_TICKS = 60
ESCAPE_PERP_GAIN = 0.6


class Firmware:
    def __init__(self) -> None:
        self._last_thrust = (0.0, 0.0, 0.0)
        self._smoothed_pos = None
        self._stuck_count = 0
        self._escape_remaining = 0
        self.stuck_active = False

    def step(self, sensors: SensorPacket3D) -> MotorCommand3D:
        px, py, pz = sensors.pos_estimate
        yaw = sensors.yaw

        # Smoothed position for stuck detection.
        if self._smoothed_pos is None:
            self._smoothed_pos = (px, py)
            disp = 1.0
        else:
            sx, sy = self._smoothed_pos
            sx = (1 - SMOOTH_ALPHA) * sx + SMOOTH_ALPHA * px
            sy = (1 - SMOOTH_ALPHA) * sy + SMOOTH_ALPHA * py
            disp = math.hypot(px - sx, py - sy)
            self._smoothed_pos = (sx, sy)

        # Horizontal attraction.
        dx, dy = GOAL_X - px, GOAL_Y - py
        norm = math.hypot(dx, dy) or 1.0
        ax = ATTRACTION_GAIN * dx / norm
        ay = ATTRACTION_GAIN * dy / norm

        # Horizontal repulsion from 8 rays.
        rx = ry = 0.0
        n = len(sensors.rays_h)
        for i, ray in enumerate(sensors.rays_h):
            if ray >= RAY_MAX * 0.95:
                continue
            ang = yaw + i * (2 * math.pi / n)
            strength = REPULSION_GAIN * (1.0 - ray / RAY_MAX) ** 2
            rx -= math.cos(ang) * strength
            ry -= math.sin(ang) * strength

        # Stuck detection + perpendicular escape.
        if self._escape_remaining > 0:
            self._escape_remaining -= 1
            self.stuck_active = True
            perp_x, perp_y = -dy / norm, dx / norm
            rx += ESCAPE_PERP_GAIN * perp_x
            ry += ESCAPE_PERP_GAIN * perp_y
        else:
            if disp < STUCK_DISP_THR:
                self._stuck_count += 1
                if self._stuck_count >= STUCK_TICKS:
                    self._escape_remaining = ESCAPE_TICKS
                    self._stuck_count = 0
            else:
                self._stuck_count = 0
                self.stuck_active = False

        fx_raw = ax + rx
        fy_raw = ay + ry
        fx = 0.7 * fx_raw + 0.3 * self._last_thrust[0]
        fy = 0.7 * fy_raw + 0.3 * self._last_thrust[1]

        # Altitude: base hold + floor-rep (up) + ceiling-rep (down).
        fz_hold = K_Z * (ALT_HOLD - pz)
        fz_up = 0.0
        fz_down = 0.0
        if sensors.ray_down < REP_Z_RANGE:
            fz_up = REP_Z * (1.0 - sensors.ray_down / REP_Z_RANGE) ** 2
        if sensors.ray_up < REP_Z_RANGE:
            fz_down = -REP_Z * (1.0 - sensors.ray_up / REP_Z_RANGE) ** 2
        fz_raw = fz_hold + fz_up + fz_down
        fz = 0.7 * fz_raw + 0.3 * self._last_thrust[2]

        self._last_thrust = (fx, fy, fz)
        return MotorCommand3D(thrust=(fx, fy, fz)).clipped()
