"""Quick map runner — bypasses the unfinished env-generator wiring.

Picks an obstacle list from sim3d.themes.THEME_REGISTRY, builds a World3D
manually, runs the v2_3d firmware end-to-end with rendering.

Usage:
    python scripts/demo_maps.py demo
    python scripts/demo_maps.py slalom
    python scripts/demo_maps.py arena
"""

import pathlib
import sys

# Allow running as `python scripts/demo_maps.py NAME` from the project root.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from firmware.firmware_v2_3d import Firmware
from harness.loop3d import run_episode_3d
from harness.scoring import format_result
from sim3d.themes import THEME_REGISTRY
from sim3d.world import (
    ARENA_W, ARENA_D, CEILING,
    SPAWN, GOAL_CENTER, GOAL_RADIUS,
    World3D,
)


def main(name: str) -> None:
    if name not in THEME_REGISTRY:
        print(f"unknown map {name!r}; choices: {sorted(THEME_REGISTRY)}", file=sys.stderr)
        sys.exit(2)

    obstacles = THEME_REGISTRY[name](0, 5)
    world = World3D(
        arena_w=ARENA_W,
        arena_d=ARENA_D,
        ceiling=CEILING,
        obstacles=obstacles,
        spawn=SPAWN,
        goal_center=GOAL_CENTER,
        goal_radius=GOAL_RADIUS,
    )
    fw = Firmware()

    from sim3d.renderer import Renderer3D
    r = Renderer3D(world)

    def cb(drone, packet, cmd, fw_obj):
        r.draw(drone, packet, cmd, fw_obj)

    try:
        result = run_episode_3d(world, fw, seed=42, render_cb=cb)
        banner = f"{result.outcome.value}  t={result.t_end:.2f}s  map={name}  fw=v2_3d"
        r.hold(banner)
    finally:
        r.close()

    print(format_result(result))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"usage: python scripts/demo_maps.py {{{'|'.join(sorted(THEME_REGISTRY))}}}", file=sys.stderr)
        sys.exit(2)
    main(sys.argv[1])
