import math
import time

import raylibpy as rl

from firmware.contract3d import MotorCommand3D, SensorPacket3D
from sim3d.physics import DroneState3D
from sim3d.world import CEILING, World3D

WINDOW_W = 1280
WINDOW_H = 720
RENDER_HZ = 60.0
RENDER_DT = 1.0 / RENDER_HZ

CAM_SMOOTH = 0.85  # low-pass alpha for chase cam position
CHASE_BACK = 3.0
CHASE_UP = 1.5

COCKPIT_FORWARD = 0.2
COCKPIT_TARGET_AHEAD = 5.0
COCKPIT_FOV = 70.0
CHASE_FOV = 55.0

HUD_COLOR = rl.Color(80, 240, 120, 230)


class Renderer3D:
    def __init__(self, world: World3D):
        rl.init_window(WINDOW_W, WINDOW_H, b"Drone Firmware Trainer 3D")
        rl.set_target_fps(int(RENDER_HZ))
        self.world = world
        self.camera = rl.Camera3D(
            rl.Vector3(0.0, 0.0, 3.0),
            rl.Vector3(world.spawn[0], world.spawn[1], world.spawn[2]),
            rl.Vector3(0.0, 0.0, 1.0),
            55.0,
            rl.CameraProjection.CAMERA_PERSPECTIVE,
        )
        self._cam_pos = (0.0, 0.0, 3.0)
        self._last_render = 0.0
        self._t_start = time.perf_counter()
        self._cam_mode = "chase"
        self._hud_source = "noisy"

    def _update_chase_cam(self, drone: DroneState3D) -> None:
        yaw = math.atan2(drone.vy, drone.vx) if (drone.vx or drone.vy) else 0.0
        bx = drone.x + math.cos(yaw) * (-CHASE_BACK)
        by = drone.y + math.sin(yaw) * (-CHASE_BACK)
        bz = drone.z + CHASE_UP
        cx, cy, cz = self._cam_pos
        cx = CAM_SMOOTH * cx + (1 - CAM_SMOOTH) * bx
        cy = CAM_SMOOTH * cy + (1 - CAM_SMOOTH) * by
        cz = CAM_SMOOTH * cz + (1 - CAM_SMOOTH) * bz
        self._cam_pos = (cx, cy, cz)
        self.camera.position = rl.Vector3(cx, cy, cz)
        self.camera.target = rl.Vector3(drone.x, drone.y, drone.z)

    def _update_cockpit_cam(self, drone: DroneState3D) -> None:
        yaw = math.atan2(drone.vy, drone.vx) if (drone.vx or drone.vy) else 0.0
        px = drone.x + math.cos(yaw) * COCKPIT_FORWARD
        py = drone.y + math.sin(yaw) * COCKPIT_FORWARD
        pz = drone.z + 0.05
        tx = drone.x + math.cos(yaw) * COCKPIT_TARGET_AHEAD
        ty = drone.y + math.sin(yaw) * COCKPIT_TARGET_AHEAD
        tz = drone.z + 0.05
        self.camera.position = rl.Vector3(px, py, pz)
        self.camera.target = rl.Vector3(tx, ty, tz)
        self.camera.fovy = COCKPIT_FOV

    def _draw_world(self, drone: DroneState3D, packet: SensorPacket3D, cam_mode: str) -> None:
        rl.draw_grid(30, 1.0)
        for r in self.world.obstacles:
            cx = r.x + r.w / 2
            cy = r.y + r.d / 2
            cz = r.z + r.h / 2
            center = rl.Vector3(cx, cy, cz)
            # Distinguish floor-standing (grey) from ceiling-hung (blue-grey)
            color = rl.Color(130, 130, 130, 200) if r.z == 0.0 else rl.Color(100, 100, 130, 200)
            rl.draw_cube(center, r.w, r.d, r.h, color)
            rl.draw_cube_wires(center, r.w, r.d, r.h, rl.BLACK)
        gx, gy, gz = self.world.goal_center
        t = time.perf_counter() - self._t_start
        pulse = int(180 + 60 * math.sin(2 * math.pi * t))
        rl.draw_sphere_wires(rl.Vector3(gx, gy, gz), self.world.goal_radius, 12, 12, rl.Color(60, 220, 80, pulse))
        yaw = math.atan2(drone.vy, drone.vx) if (drone.vx or drone.vy) else 0.0
        drone_pos = rl.Vector3(drone.x, drone.y, drone.z)
        if cam_mode != "cockpit":
            rl.draw_cube(drone_pos, 0.3, 0.3, 0.3, rl.RED)
            rl.draw_cube_wires(drone_pos, 0.3, 0.3, 0.3, rl.BLACK)
            nose = rl.Vector3(drone.x + 0.5 * math.cos(yaw), drone.y + 0.5 * math.sin(yaw), drone.z)
            rl.draw_line3d(drone_pos, nose, rl.YELLOW)
        for i, ray in enumerate(packet.rays_h):
            ang = yaw + i * (2 * math.pi / len(packet.rays_h))
            end = rl.Vector3(drone.x + math.cos(ang) * ray, drone.y + math.sin(ang) * ray, drone.z)
            rl.draw_line3d(drone_pos, end, rl.Color(80, 140, 230, 180))
        rl.draw_line3d(drone_pos, rl.Vector3(drone.x, drone.y, drone.z + packet.ray_up), rl.Color(80, 140, 230, 180))
        rl.draw_line3d(drone_pos, rl.Vector3(drone.x, drone.y, drone.z - packet.ray_down), rl.Color(80, 140, 230, 180))

    def _hud_values(self, drone: DroneState3D, packet: SensorPacket3D) -> dict:
        # Velocity is always derived from ground-truth drone state — the firmware
        # contract delivers position estimates and IMU accel only, never velocity.
        # The NOISY/TRUE toggle affects position, altitude, and yaw fields.
        speed = math.sqrt(drone.vx ** 2 + drone.vy ** 2 + drone.vz ** 2)
        if self._hud_source == "noisy":
            return {
                "x": packet.pos_estimate[0],
                "y": packet.pos_estimate[1],
                "z": packet.pos_estimate[2],
                "speed": speed,
                "vx": drone.vx,
                "vy": drone.vy,
                "vz": drone.vz,
                "yaw": packet.yaw,
            }
        return {
            "x": drone.x,
            "y": drone.y,
            "z": drone.z,
            "speed": speed,
            "vx": drone.vx,
            "vy": drone.vy,
            "vz": drone.vz,
            "yaw": math.atan2(drone.vy, drone.vx) if (drone.vx or drone.vy) else 0.0,
        }

    def _project(self, world_pos: rl.Vector3) -> tuple:
        sp = rl.get_world_to_screen(world_pos, self.camera)
        on_screen = 0 <= sp.x < WINDOW_W and 0 <= sp.y < WINDOW_H
        return int(sp.x), int(sp.y), on_screen

    def _draw_hud(self, drone: DroneState3D, packet: SensorPacket3D, fw) -> None:
        hv = self._hud_values(drone, packet)
        # Boresight cross
        cx, cy = WINDOW_W // 2, WINDOW_H // 2
        rl.draw_line(cx - 10, cy, cx + 10, cy, HUD_COLOR)
        rl.draw_line(cx, cy - 10, cx, cy + 10, HUD_COLOR)
        # Heading tape (top)
        yaw_deg = (math.degrees(hv["yaw"]) + 360.0) % 360.0
        tape_y = 30
        rl.draw_line(0, tape_y + 20, WINDOW_W, tape_y + 20, HUD_COLOR)
        for deg_offset in range(-60, 61, 10):
            ang_deg = (yaw_deg + deg_offset) % 360.0
            x = cx + deg_offset * 6
            rl.draw_line(x, tape_y + 10, x, tape_y + 20, HUD_COLOR)
            if deg_offset % 30 == 0:
                label = {0: "N", 90: "E", 180: "S", 270: "W"}.get(int(ang_deg), f"{int(ang_deg):03d}")
                rl.draw_text(label.encode(), x - 12, tape_y - 8, 14, HUD_COLOR)
        rl.draw_text(f"{int(yaw_deg):03d}".encode(), cx - 18, tape_y + 24, 16, HUD_COLOR)
        # Altitude tape (right)
        alt_x = WINDOW_W - 80
        rl.draw_line(alt_x, 80, alt_x, WINDOW_H - 80, HUD_COLOR)
        for z_m in range(0, 5):
            y = WINDOW_H - 80 - int((z_m / 4.0) * (WINDOW_H - 160))
            rl.draw_line(alt_x - 6, y, alt_x + 6, y, HUD_COLOR)
            rl.draw_text(f"{z_m}".encode(), alt_x + 10, y - 8, 14, HUD_COLOR)
        z_clamped = max(0.0, min(CEILING, hv["z"]))
        y_circle = WINDOW_H - 80 - int((z_clamped / 4.0) * (WINDOW_H - 160))
        rl.draw_circle_lines(alt_x, y_circle, 8, HUD_COLOR)
        rl.draw_text(f"ALT {hv['z']:4.2f}m".encode(), alt_x - 14, WINDOW_H - 70, 14, HUD_COLOR)
        rl.draw_text(f"VS {hv['vz']:+.2f}".encode(), alt_x - 14, WINDOW_H - 50, 14, HUD_COLOR)
        # Airspeed tape (left)
        spd_x = 80
        rl.draw_line(spd_x, 80, spd_x, WINDOW_H - 80, HUD_COLOR)
        for s_m in range(0, 6):
            y = WINDOW_H - 80 - int((s_m / 5.0) * (WINDOW_H - 160))
            rl.draw_line(spd_x - 6, y, spd_x + 6, y, HUD_COLOR)
            rl.draw_text(f"{s_m}".encode(), spd_x - 22, y - 8, 14, HUD_COLOR)
        s_clamped = max(0.0, min(5.0, hv["speed"]))
        y_circle = WINDOW_H - 80 - int((s_clamped / 5.0) * (WINDOW_H - 160))
        rl.draw_circle_lines(spd_x, y_circle, 8, HUD_COLOR)
        rl.draw_text(f"SPD {hv['speed']:4.2f}".encode(), spd_x - 14, WINDOW_H - 70, 14, HUD_COLOR)
        # Velocity vector marker (FPM)
        if abs(hv["vx"]) + abs(hv["vy"]) + abs(hv["vz"]) > 0.05:
            fpm_pos = rl.Vector3(drone.x + hv["vx"] * 5.0, drone.y + hv["vy"] * 5.0, drone.z + hv["vz"] * 5.0)
            fx, fy, on = self._project(fpm_pos)
            if on:
                rl.draw_circle_lines(fx, fy, 6, HUD_COLOR)
                rl.draw_line(fx - 12, fy, fx - 6, fy, HUD_COLOR)
                rl.draw_line(fx + 6, fy, fx + 12, fy, HUD_COLOR)
        # Target marker
        gx, gy, gz = self.world.goal_center
        tx, ty, on = self._project(rl.Vector3(gx, gy, gz))
        rng = math.sqrt((drone.x - gx) ** 2 + (drone.y - gy) ** 2 + (drone.z - gz) ** 2)
        if on:
            rl.draw_line(tx - 12, ty, tx, ty - 12, rl.YELLOW)
            rl.draw_line(tx, ty - 12, tx + 12, ty, rl.YELLOW)
            rl.draw_line(tx + 12, ty, tx, ty + 12, rl.YELLOW)
            rl.draw_line(tx, ty + 12, tx - 12, ty, rl.YELLOW)
            rl.draw_text(f"RNG {rng:4.1f}m".encode(), tx - 30, ty + 16, 14, rl.YELLOW)
        else:
            # off-screen arrow at edge
            ax = max(20, min(WINDOW_W - 20, tx))
            ay = max(20, min(WINDOW_H - 20, ty))
            rl.draw_circle_lines(ax, ay, 10, rl.YELLOW)
            rl.draw_text(f"RNG {rng:4.1f}m".encode(), ax - 30, ay + 14, 14, rl.YELLOW)
        # Status row (bottom)
        stuck = getattr(fw, "stuck_active", False)
        status = f"t={drone.t:5.2f}s  pos=({hv['x']:4.1f},{hv['y']:4.1f},{hv['z']:3.1f})"
        rl.draw_text(status.encode(), 20, WINDOW_H - 28, 16, HUD_COLOR)
        if stuck:
            rl.draw_text(b"[STUCK]", WINDOW_W - 100, WINDOW_H - 28, 16, rl.ORANGE)
        # Corner badges
        cam_label = b"CHASE" if self._cam_mode == "chase" else b"COCKPIT"
        src_label = b"NOISY" if self._hud_source == "noisy" else b"TRUE"
        rl.draw_text(cam_label, 20, 20, 16, HUD_COLOR)
        rl.draw_text(src_label, WINDOW_W - 80, 20, 16, HUD_COLOR)

    def draw(self, drone: DroneState3D, packet: SensorPacket3D, cmd: MotorCommand3D, fw) -> None:
        now = time.perf_counter()
        if now - self._last_render < RENDER_DT:
            return
        self._last_render = now
        if rl.window_should_close():
            raise SystemExit(0)
        if rl.is_key_pressed(rl.KeyboardKey.KEY_C):
            self._cam_mode = "cockpit" if self._cam_mode == "chase" else "chase"
        if rl.is_key_pressed(rl.KeyboardKey.KEY_T):
            self._hud_source = "true" if self._hud_source == "noisy" else "noisy"
        if self._cam_mode == "cockpit":
            self._update_cockpit_cam(drone)
        else:
            self.camera.fovy = CHASE_FOV
            self._update_chase_cam(drone)
        rl.begin_drawing()
        rl.clear_background(rl.RAYWHITE)
        rl.begin_mode3d(self.camera)
        self._draw_world(drone, packet, self._cam_mode)
        rl.end_mode3d()
        self._draw_hud(drone, packet, fw)
        rl.end_drawing()

    def hold(self, banner: str) -> None:
        while not rl.window_should_close():
            rl.begin_drawing()
            rl.clear_background(rl.RAYWHITE)
            rl.begin_mode3d(self.camera)
            rl.draw_grid(30, 1.0)
            rl.end_mode3d()
            text_bytes = banner.encode()
            tw = rl.measure_text(text_bytes, 30)
            box_w = tw + 40
            box_h = 80
            box_x = (WINDOW_W - box_w) // 2
            box_y = (WINDOW_H - box_h) // 2
            rl.draw_rectangle(box_x, box_y, box_w, box_h, rl.RAYWHITE)
            rl.draw_rectangle_lines(box_x, box_y, box_w, box_h, rl.BLACK)
            rl.draw_text(text_bytes, box_x + 20, box_y + 15, 30, rl.BLACK)
            hint = b"press any key or close window to exit"
            rl.draw_text(hint, box_x + (box_w - rl.measure_text(hint, 16)) // 2, box_y + 55, 16, rl.DARKGRAY)
            rl.end_drawing()
            if rl.get_key_pressed() != 0 or rl.is_mouse_button_pressed(0):
                return

    def close(self) -> None:
        rl.close_window()
