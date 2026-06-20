import argparse

from harness.loop3d import load_firmware_class_3d, run_episode_3d
from harness.scoring import format_result, summarize_sweep
from sim3d.world import build_world3d

SWEEP_SEEDS = [42, 7, 13, 99, 256]

def _make_render_cb(world, cam: str):
    from sim3d.renderer import Renderer3D
    r = Renderer3D(world)
    def cb(drone, packet, cmd, fw):
        r.draw(drone, packet, cmd, fw)
    return cb, r

def _run_one(version: str, seed: int, render: bool, cam: str):
    world = build_world3d(seed)
    FirmwareCls = load_firmware_class_3d(version)
    fw = FirmwareCls()
    cb = None
    r = None
    if render:
        cb, r = _make_render_cb(world, cam)
    try:
        result = run_episode_3d(world, fw, seed=seed, render_cb=cb)
        if r is not None:
            banner = f"{result.outcome.value}  t={result.t_end:.2f}s  seed={seed}  fw={version}"
            r.hold(banner)
    finally:
        if r is not None:
            r.close()
    print(format_result(result))
    return result

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--firmware", required=True, choices=["v1_3d", "v2_3d"])
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--render", action="store_true")
    p.add_argument("--sweep", action="store_true")
    p.add_argument("--cam", choices=["chase", "cockpit"], default="chase")
    args = p.parse_args()

    if args.sweep:
        results = [_run_one(args.firmware, s, render=False, cam=args.cam) for s in SWEEP_SEEDS]
        print(summarize_sweep(results))
    else:
        _run_one(args.firmware, args.seed, render=args.render, cam=args.cam)

if __name__ == "__main__":
    main()
