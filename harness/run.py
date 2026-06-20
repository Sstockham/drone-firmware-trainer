import argparse

from harness.loop import load_firmware_class, run_episode
from harness.scoring import format_result, summarize_sweep
from sim.world import build_world

SWEEP_SEEDS = [42, 7, 13, 99, 256]

def _make_render_cb(world):
    from sim.renderer import Renderer
    r = Renderer(world)
    def cb(drone, packet, cmd):
        r.draw(drone, packet, cmd)
    return cb, r

def _run_one(version: str, seed: int, render: bool):
    world = build_world(seed)
    FirmwareCls = load_firmware_class(version)
    fw = FirmwareCls()
    cb = None
    r = None
    if render:
        cb, r = _make_render_cb(world)
    try:
        result = run_episode(world, fw, seed=seed, render_cb=cb)
    finally:
        if r is not None:
            r.close()
    print(format_result(result))
    return result

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--firmware", required=True, choices=["v1", "v2"])
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--render", action="store_true")
    p.add_argument("--sweep", action="store_true")
    args = p.parse_args()

    if args.sweep:
        results = [_run_one(args.firmware, s, render=False) for s in SWEEP_SEEDS]
        print(summarize_sweep(results))
    else:
        _run_one(args.firmware, args.seed, render=args.render)

if __name__ == "__main__":
    main()
