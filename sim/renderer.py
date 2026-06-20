import math
import time
import pygame

from firmware.contract import MotorCommand, SensorPacket
from sim.physics import DroneState
from sim.world import World

PX_PER_M = 40
RENDER_HZ = 30.0
RENDER_DT = 1.0 / RENDER_HZ

BLACK = (0, 0, 0)
WHITE = (240, 240, 240)
RED = (220, 60, 60)
GREEN = (60, 200, 80)
BLUE = (80, 140, 230)
GREY = (130, 130, 130)

class Renderer:
    def __init__(self, world: World):
        pygame.init()
        self.world = world
        self.w_px = int(world.arena_w * PX_PER_M)
        self.h_px = int(world.arena_h * PX_PER_M)
        self.screen = pygame.display.set_mode((self.w_px, self.h_px))
        pygame.display.set_caption("Drone Firmware Trainer")
        self.font = pygame.font.SysFont("consolas", 14)
        self._last_render = 0.0

    def _to_px(self, x: float, y: float) -> tuple[int, int]:
        return int(x * PX_PER_M), self.h_px - int(y * PX_PER_M)

    def draw(self, drone: DroneState, packet: SensorPacket, cmd: MotorCommand) -> None:
        now = time.perf_counter()
        if now - self._last_render < RENDER_DT:
            return
        self._last_render = now

        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit()
                raise SystemExit(0)

        self.screen.fill(WHITE)

        for r in self.world.obstacles:
            x_px, y_px = self._to_px(r.x, r.y + r.h)
            pygame.draw.rect(self.screen, GREY, (x_px, y_px, int(r.w * PX_PER_M), int(r.h * PX_PER_M)))

        gx, gy = self._to_px(*self.world.goal_center)
        pygame.draw.circle(self.screen, GREEN, (gx, gy), int(self.world.goal_radius * PX_PER_M), width=3)

        dx, dy = self._to_px(drone.x, drone.y)
        yaw = math.atan2(drone.vy, drone.vx) if (drone.vx or drone.vy) else 0.0
        for i, ray in enumerate(packet.rays):
            ang = yaw + i * (2 * math.pi / len(packet.rays))
            ex_px, ey_px = self._to_px(drone.x + math.cos(ang) * ray, drone.y + math.sin(ang) * ray)
            pygame.draw.line(self.screen, BLUE, (dx, dy), (ex_px, ey_px), 1)
        pygame.draw.circle(self.screen, RED, (dx, dy), 6)

        hud = self.font.render(
            f"t={drone.t:5.2f}s  pos=({drone.x:4.1f},{drone.y:4.1f})  thr=({cmd.thrust[0]:+.2f},{cmd.thrust[1]:+.2f})",
            True, BLACK,
        )
        self.screen.blit(hud, (10, 10))

        pygame.display.flip()

    def close(self) -> None:
        pygame.quit()
