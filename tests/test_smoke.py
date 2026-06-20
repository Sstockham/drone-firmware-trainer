"""End-to-end smoke test. Runs harness with v1, seed 42, no render.
Confirms the loop completes and returns a finished RunResult."""

from harness.loop import load_firmware_class, run_episode
from harness.scoring import Outcome, format_result
from sim.world import build_world

def main():
    world = build_world(42)
    Fw = load_firmware_class("v1")
    fw = Fw()
    result = run_episode(world, fw, seed=42, max_t=30.0)
    print(format_result(result))
    assert result.outcome in {Outcome.SUCCESS, Outcome.CRASH, Outcome.TIMEOUT, Outcome.FAULT}
    assert result.t_end > 0.0
    assert result.mean_tick_ms >= 0.0
    print("SMOKE OK")

if __name__ == "__main__":
    main()
