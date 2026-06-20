"""Throwaway raylib-py smoke. Opens a window with a rotating cube.
Deleted in Task 13. If this won't run, the 3D upgrade cannot proceed."""

import math
import time

import raylibpy as rl


def main() -> None:
    rl.init_window(960, 540, b"raylib hello (close window to exit)")
    rl.set_target_fps(60)
    camera = rl.Camera3D(
        rl.Vector3(4.0, 4.0, 4.0),
        rl.Vector3(0.0, 0.0, 0.0),
        rl.Vector3(0.0, 0.0, 1.0),
        55.0,
        rl.CameraProjection.CAMERA_PERSPECTIVE,
    )
    t0 = time.perf_counter()
    while not rl.window_should_close():
        t = time.perf_counter() - t0
        angle = (t * 30.0) % 360.0
        rl.begin_drawing()
        rl.clear_background(rl.RAYWHITE)
        rl.begin_mode3d(camera)
        rl.draw_grid(20, 1.0)
        cube_pos = rl.Vector3(0.0, 0.0, 1.0)
        rl.draw_cube(cube_pos, 1.0, 1.0, 1.0, rl.RED)
        rl.draw_cube_wires(cube_pos, 1.0, 1.0, 1.0, rl.BLACK)
        rl.end_mode3d()
        rl.draw_text(f"angle={angle:6.1f} t={t:5.2f}s".encode(), 10, 10, 20, rl.DARKGRAY)
        rl.end_drawing()
    rl.close_window()


if __name__ == "__main__":
    main()
