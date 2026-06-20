from dataclasses import dataclass
from enum import Enum

class Outcome(str, Enum):
    SUCCESS = "SUCCESS"
    CRASH = "CRASH"
    TIMEOUT = "TIMEOUT"
    FAULT = "FAULT"

@dataclass
class RunResult:
    seed: int
    outcome: Outcome
    t_end: float
    min_clearance: float
    mean_tick_ms: float
    max_tick_ms: float
    num_overruns: int
    fault_msg: str | None = None

def format_result(r: RunResult) -> str:
    tag = r.outcome.value
    base = f"[seed={r.seed}] {tag} @ t={r.t_end:5.2f}s  min_clr={r.min_clearance:.2f}m  tick(mean/max)={r.mean_tick_ms:.2f}/{r.max_tick_ms:.2f}ms  overruns={r.num_overruns}"
    if r.fault_msg:
        base += f"  fault={r.fault_msg!r}"
    return base

def summarize_sweep(results: list[RunResult]) -> str:
    n = len(results)
    wins = sum(1 for r in results if r.outcome is Outcome.SUCCESS)
    return f"--- {wins}/{n} success ---"
