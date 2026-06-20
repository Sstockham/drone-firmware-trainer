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

    def _draw_world(self, drone: DroneState3D, packet: SensorPacket3D) -> None:
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
        # drone cube
        drone_pos = rl.Vector3(drone.x, drone.y, drone.z)
        rl.draw_cube(drone_pos, 0.3, 0.3, 0.3, rl.RED)
        rl.draw_cube_wires(drone_pos, 0.3, 0.3, 0.3, rl.BLACK)
        # forward wedge (yaw indicator)
        yaw = math.atan2(drone.vy, drone.vx) if (drone.vx or drone.vy) else 0.0
        nose = rl.Vector3(drone.x + 0.5 * math.cos(yaw), drone.y + 0.5 * math.sin(yaw), drone.z)
        rl.draw_line3d(drone_pos, nose, rl.YELLOW)
        # rays
        for i, ray in enumerate(packet.rays_h):
            ang = yaw + i * (2 * math.pi / len(packet.rays_h))
            end = rl.Vector3(drone.x + math.cos(ang) * ray, drone.y + math.sin(ang) * ray, drone.z)
            rl.draw_line3d(drone_pos, end, rl.Color(80, 140, 230, 180))
        rl.draw_line3d(drone_pos, rl.Vector3(drone.x, drone.y, drone.z + packet.ray_up), rl.Color(80, 140, 230, 180))
        rl.draw_line3d(drone_pos, rl.Vector3(drone.x, drone.y, drone.z - packet.ray_down), rl.Color(80, 140, 230, 180))

    def draw(self, drone: DroneState3D, packet: SensorPacket3D, cmd: MotorCommand3D, fw) -> None:
        now = time.perf_counter()
        if now - self._last_render < RENDER_DT:
            return
        self._last_render = now
        if rl.window_should_close():
            raise SystemExit(0)
        self._update_chase_cam(drone)
        rl.begin_drawing()
        rl.clear_background(rl.RAYWHITE)
        rl.begin_mode3d(self.camera)
        self._draw_world(drone, packet)
        rl.end_mode3d()
        # tiny placeholder HUD (proper HUD lands in T12)
        rl.draw_text(
            f"t={drone.t:5.2f}s  pos=({drone.x:4.1f},{drone.y:4.1f},{drone.z:3.1f})  thr=({cmd.thrust[0]:+.2f},{cmd.thrust[1]:+.2f},{cmd.thrust[2]:+.2f})".encode(),
            10, 10, 18, rl.DARKGRAY,
        )
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
