"""3D end-to-end smoke test. Runs v1_3d on seed 42 headless.
Confirms the 3D loop completes and returns a finished RunResult."""

from harness.loop3d import load_firmware_class_3d, run_episode_3d
from harness.scoring import Outcome, format_result
from sim3d.world import build_world3d


def main():
    world = build_world3d(42)
    Fw = load_firmware_class_3d("v1_3d")
    fw = Fw()
    result = run_episode_3d(world, fw, seed=42, max_t=45.0)
    print(format_result(result))
    assert result.outcome in {Outcome.SUCCESS, Outcome.CRASH, Outcome.TIMEOUT, Outcome.FAULT}
    assert result.t_end > 0.0
    assert result.mean_tick_ms >= 0.0
    print("SMOKE_3D OK")


if __name__ == "__main__":
    main()
