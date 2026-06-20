# Environment Generator — Design Spec

**Date:** 2026-06-20
**Context:** Follow-up to the 2D + 3D Drone Firmware Trainer (both shipped earlier today). The 3D sim ships with a single hand-crafted seed-42 layout (three set-pieces: tall wall, short bar, hanging overhang) plus a thin `_random_obstacles3d` helper for the other sweep seeds. This spec replaces that helper with a richer theme-and-difficulty generator so every run produces a visually distinctive, reproducible, solvability-verified 3D world.
**Goal:** Add four new themed world generators (`forest`, `warehouse`, `canyon`, `city`) plus the existing hand-crafted layout as a `demo` theme, with a single `--difficulty 1..10` knob and a coarse-grid A* solvability check that filters out impossible layouts before they reach the firmware.

---

## 1. Demo win condition

Five new runnable invocations land alongside the existing `--theme demo` (default) demo:

```
python -m harness.run3d --firmware v2_3d --theme forest    --difficulty 7 --seed 42 --render
python -m harness.run3d --firmware v2_3d --theme warehouse --difficulty 5 --seed 11 --render
python -m harness.run3d --firmware v2_3d --theme canyon    --difficulty 4 --seed 99 --render
python -m harness.run3d --firmware v2_3d --theme city      --difficulty 6 --seed 33 --render
python -m harness.run3d --firmware v2_3d --theme forest    --difficulty 5 --sweep
```

Each rendered run opens the existing 3D window, shows a visually distinct world, the v2_3d firmware navigates to the goal, and the hold-after-run banner reads `SUCCESS  t=<value>s  seed=<n>  fw=v2_3d  theme=<name>  diff=<n>`. The sweep prints 5 per-seed result lines + a `--- N/5 success ---` summary.

**Observation targets (not hard pass/fail — these calibrate the difficulty dial, they do not block the spec from shipping):** v2_3d's sweep success across the four new themes at `difficulty=5` should land roughly **≥ 3/5**; at `difficulty=7`, roughly **≥ 2/5**. v1_3d sweep at any theme + difficulty should stay **≤ 2/5** (typically 0–1). If any number lands wildly off, the difficulty `_scale` curves get one tuning pass, not a v2_3d firmware rewrite.

The existing 2D demo + the existing 3D `demo` theme (the hand-crafted three-set-piece seed-42 layout) continue to work unchanged — every existing invocation in `README.md` and `tests/test_smoke_3d.py` is byte-identical.

## 2. Non-goals

- Composite obstacle primitives (doorway-walls, corridors, ramps, slalom gates). All themes use single axis-aligned `Rect3D` instances.
- LLM-driven world generation from a text prompt.
- New mission types (multi-waypoint courses, multiple goals, opposition drones, moving obstacles, hoops).
- Persistence: dump a generated world to JSON / load `--world path/to/world.json`. The `(theme, difficulty, seed)` triplet is the persistence — every world is fully reproducible from those three values.
- A 2D port. The 2D generator stays as legacy.
- Curriculum / adaptive difficulty that tracks firmware competence.
- Anything that requires changes to `firmware/contract3d.py`, `firmware/firmware_v1_3d.py`, `firmware/firmware_v2_3d.py`, `sim3d/physics.py`, `sim3d/sensors.py`, `sim3d/renderer.py`'s HUD/camera logic, or `harness/loop3d.py`. Only `sim3d/world.py`, the new `sim3d/themes/` package, the new `sim3d/solver.py`, `harness/run3d.py`, and tests change.

## 3. Architecture

```
sim3d/
    world.py            # MODIFIED: build_world3d signature gains theme + difficulty;
                        #           dispatch to themes registry; solvability loop.
    themes/
        __init__.py     # THEME_REGISTRY: dict[str, Callable[[int, int], list[Rect3D]]]
        demo.py         # Hand-crafted three-set-piece layout (current seed-42 obstacles)
        forest.py       # Tall thin vertical pillars
        warehouse.py    # Floor-standing crates in semi-regular aisles
        canyon.py       # Long horizontal slabs perpendicular to spawn->goal
        city.py         # Mixed tall buildings + low ceiling-hung bridges
    solver.py           # NEW: coarse-grid A* solvability check
harness/
    run3d.py            # MODIFIED: adds --theme and --difficulty argparse options;
                        #           pre-run "[world] theme=... diff=... seed=..." print line;
                        #           renderer banner string updated to include theme/diff.
tests/
    test_smoke_3d.py    # UNCHANGED: defaults to theme=demo, exercises the existing path.
    test_themes_3d.py   # NEW: smoke check that each theme yields a solvable world
                        #      at difficulty=5 with at least one fixed seed.
```

**Dependency rule (carried forward):** `sim3d/themes/*.py` import only `sim3d.world` (for `Rect3D`, `ARENA_W`, `ARENA_D`, `CEILING`, `SPAWN`, `GOAL_CENTER`) and `random` from stdlib. `sim3d/solver.py` imports only `sim3d.world`. No imports from `firmware/` or `harness/` in either.

## 4. Theme registry interface

`sim3d/themes/__init__.py`:

```python
from typing import Callable
from sim3d.world import Rect3D
from sim3d.themes import demo, forest, warehouse, canyon, city

THEME_REGISTRY: dict[str, Callable[[int, int], list[Rect3D]]] = {
    "demo":      demo.generate,
    "forest":    forest.generate,
    "warehouse": warehouse.generate,
    "canyon":    canyon.generate,
    "city":      city.generate,
}

THEMES = tuple(THEME_REGISTRY.keys())  # used for argparse choices in run3d.py
```

Each `theme.generate(seed: int, difficulty: int) -> list[Rect3D]` is a pure function. Themes that ignore `seed` and/or `difficulty` (e.g. `demo`) accept the parameters and discard them. No global state.

## 5. Theme catalog

Each theme has a `_scale(difficulty)` helper that linearly interpolates between its difficulty=1 and difficulty=10 endpoint values. Linear interp formula: `lo + (hi - lo) * (clamp(d, 1, 10) - 1) / 9`.

### 5.1 `demo` — hand-crafted seed-42 layout (default)

Identical to the current `_hand_crafted_seed_42()` body. Ignores `seed` and `difficulty`. Always returns:

```python
[
    Rect3D(6.0, 6.0, 0.0, 4.0, 0.6, 4.0),    # tall wall
    Rect3D(11.0, 9.0, 0.0, 3.0, 0.6, 1.0),   # short bar
    Rect3D(14.0, 11.5, 1.0, 0.6, 3.0, 3.0),  # hanging overhang
]
```

### 5.2 `forest` — vertical pillars

- Primitive: `Rect3D(x, y, 0.0, w, d, CEILING)` with `w, d ∈ [0.3, 0.5]`, floor-standing, ceiling-touching.
- Placement: uniform random `x ∈ [3.0, ARENA_W - 3.0]`, `y ∈ [2.0, ARENA_D - 2.0]`. Reject if the candidate contains `SPAWN` or `GOAL_CENTER`. Reject if the candidate's AABB (inflated by 0.2 m for clearance) overlaps an already-placed pillar's AABB (inflated by 0.2 m) — i.e., reject when `|cx_new - cx_existing| < (w_new + w_existing)/2 + 0.2` AND `|cy_new - cy_existing| < (d_new + d_existing)/2 + 0.2`. 200-attempt cap.
- `_scale(d)` returns `count = round(5 + (25 - 5) * (d-1)/9)` → 5 pillars at d=1, 25 at d=10.

### 5.3 `warehouse` — floor-standing crates in aisles

- Primitive: `Rect3D(x, y, 0.0, w, d, h)` with `w, d ∈ [1.0, 2.0]`, `h ∈ [1.0, hmax]`.
- Placement: 3-column grid at `x ∈ {5.0, 10.0, 15.0}` with row jitter. For each column, walk `y` from 2.0 to 12.0 in steps of `~2.5 m`, randomly skip ~30% of cells. Apply uniform `(-0.3, +0.3)` jitter in x and y.
- `_scale(d)` returns `(count, hmax)`: `count = round(6 + 12 * (d-1)/9)`, `hmax = 1.5 + 1.0 * (d-1)/9`. So d=1 = 6 crates of max h=1.5 (drone-flyoverable); d=10 = 18 crates of max h=2.5 (must route around).
- Reject if it contains SPAWN or GOAL_CENTER. 200-attempt cap.

### 5.4 `canyon` — long horizontal slabs

- Primitive: `Rect3D(x, y, 0.0, w, 0.5, CEILING)` — a slab that extends in the x direction (long, computed `w` per the formula below), has thin depth in y (`d = 0.5 m`), and spans full ceiling height. Slabs are perpendicular to the spawn→goal axis (which runs roughly along y from `(1, 1)` to `(18, 13)`).
- Placement: deterministic. `_scale(d)` returns `(count, gap)`: `count = round(3 + 5 * (d-1)/9)`, `gap ∈ [4.0, 0.8]` linearly interpolated. Slabs placed at `y` values evenly spaced between `y=3.0` and `y=12.0`. Each slab leaves a `gap`-wide opening at ONE end (alternating left vs. right), forcing a zigzag.
- Concretely, the slab's `w = ARENA_W - 2.0 - gap` (`= 14.0` at d=1, `= 17.2` at d=10), placed so slab `i` (0-indexed) spans:
  - `x ∈ [1.0, 1.0 + w]` if `i % 2 == 0` (gap on the right, between `1.0 + w` and `ARENA_W - 1.0`)
  - `x ∈ [1.0 + gap, ARENA_W - 1.0]` if `i % 2 == 1` (gap on the left, between `1.0` and `1.0 + gap`)
- No reject loop needed — placement is deterministic and never contains SPAWN/GOAL by construction. `seed` is unused.

### 5.5 `city` — tall buildings + low ceiling-hung bridges

- Primitives:
  - Tall building: `Rect3D(x, y, 0.0, w, d, h)` with `w, d ∈ [1.5, 3.0]`, `h ∈ [2.0, CEILING]`, floor-standing.
  - Bridge (must obstruct the drone's default 1.5 m flight altitude — same lesson as the demo overhang): `Rect3D(x, y, 1.0, w, d, 3.0)` (hangs from z=1.0 to ceiling) with `w = 0.5`, `d ∈ [2.0, 3.0]`. Drone must descend below ~0.8 m to fit under.
- Placement: 70% tall, 30% bridge, uniform random `x ∈ [3.0, ARENA_W - 3.0]`, `y ∈ [2.0, ARENA_D - 2.0]`. Reject SPAWN/GOAL containment + AABB overlap with already-placed (same threshold as forest). 200-attempt cap.
- `_scale(d)` returns `total_count = round(5 + 11 * (d-1)/9)` → 5 obstacles at d=1, 16 at d=10. The 70/30 mix applies probabilistically.

## 6. Difficulty dial semantics

- `int`, default `5`, clamped to `[1, 10]` with one-line stderr warning if out of range (not an error — the dial is meant to be forgiving).
- The same `difficulty` value means roughly the same subjective challenge across themes: `1` is sparse/easy, `5` is moderate, `10` is dense/tight.
- `demo` ignores difficulty.

## 7. Solvability check (`sim3d/solver.py`)

```python
from sim3d.world import World3D, Rect3D

GRID_RES = 0.5  # meters per cell

def has_path(world: World3D) -> bool: ...
```

- Coarse 3D grid at `GRID_RES = 0.5 m` over the 20 m × 15 m × 4 m arena = 40 × 30 × 8 = **9 600 cells**.
- A cell `(i, j, k)` represents the cube whose center is at `((i+0.5)*GRID_RES, (j+0.5)*GRID_RES, (k+0.5)*GRID_RES)`. A cell is blocked if its center is inside any `Rect3D`.
- Movement: 6-connected (axis-aligned neighbors only). Diagonal moves are not allowed.
- Start cell = the cell containing `SPAWN`. Goal = any cell whose center is within `GOAL_RADIUS` of `GOAL_CENTER`.
- A* with Manhattan heuristic. Expected runtime: <10 ms on the target machine.
- Returns `True` if a path exists, `False` otherwise.

The 0.5 m grid is intentionally coarser than the drone's body (0.3 m cube) so that "the A* path exists" does not imply "the firmware can fly the same line." It's an under-approximation of reachability designed to filter out **clearly** impossible worlds while leaving the firmware to handle genuinely tight cases.

If the 0.5 m grid produces too many false-negatives on real-world tests, downgrade `GRID_RES` to 0.25 m (40 × 30 × 8 × 8 = 76 800 cells, still ~50 ms). If still too coarse, accept the false negatives — easier than chasing them.

## 8. World builder (`sim3d/world.py` changes)

New signature:

```python
def build_world3d(seed: int, theme: str = "demo", difficulty: int = 5) -> World3D: ...
```

Body:

```python
import warnings
from sim3d.themes import THEME_REGISTRY
from sim3d.solver import has_path

def build_world3d(seed: int, theme: str = "demo", difficulty: int = 5) -> World3D:
    if theme not in THEME_REGISTRY:
        raise ValueError(f"unknown theme {theme!r}; choices: {sorted(THEME_REGISTRY)}")
    difficulty = max(1, min(10, difficulty))
    generator = THEME_REGISTRY[theme]

    last_world = None
    for attempt in range(10):
        obstacles = generator(seed + attempt, difficulty)
        candidate = World3D(
            arena_w=ARENA_W, arena_d=ARENA_D, ceiling=CEILING,
            obstacles=obstacles,
            spawn=SPAWN, goal_center=GOAL_CENTER, goal_radius=GOAL_RADIUS,
        )
        last_world = candidate
        if has_path(candidate):
            return candidate
    warnings.warn(
        f"unsolvable after 10 attempts: theme={theme} difficulty={difficulty} seed={seed} — "
        "returning the last attempt anyway",
        RuntimeWarning,
    )
    return last_world
```

**Backwards-compat:** the existing `build_world3d(42)` call (used by `tests/test_smoke_3d.py`) keeps working because the default `theme="demo"` returns the hand-crafted layout unchanged. The existing `_hand_crafted_seed_42()` helper moves into `sim3d/themes/demo.py:generate()`; the old `_random_obstacles3d` helper is **deleted** — its role is taken over by the themed generators (`demo` is the default; non-demo seeds the user wants for variety now route through a real theme).

## 9. CLI (`harness/run3d.py` changes)

Two new argparse arguments:

```python
import argparse
from sim3d.themes import THEMES

# ... existing setup ...
p.add_argument("--theme", choices=THEMES, default="demo")
p.add_argument("--difficulty", type=int, default=5,
               help="1..10, clamped silently. Ignored by --theme demo.")
```

`_run_one(version, seed, render, cam)` becomes `_run_one(version, seed, render, cam, theme, difficulty)` and calls `build_world3d(seed, theme=theme, difficulty=difficulty)`. The sweep dispatches the same theme + difficulty across all SWEEP_SEEDS.

**Pre-run announce line** (single line to stdout, before the harness starts):

```
[world] theme=forest difficulty=7 seed=42  (use these to replay)
```

**Renderer banner update:** `harness/run3d.py:_run_one` passes the theme + difficulty into the banner string sent to `r.hold(banner)`. New format:

```
SUCCESS  t=18.30s  seed=42  fw=v2_3d  theme=forest  diff=7
```

If the banner string exceeds 60 chars after this addition (e.g., long fault messages), the banner is truncated with a trailing ellipsis — `Renderer3D.hold` already handles measured text width, but `_run_one` should pre-truncate strings > 60 chars defensively.

**Sweep output line update:** each per-seed line in the sweep output still uses `format_result(r)` from `harness.scoring`, but `_run_one` prepends `[world] theme=... diff=... seed=...` so each line is self-documenting.

## 10. Tests

`tests/test_themes_3d.py` (new):

```python
"""Per-theme smoke. Confirms each theme yields a solvable world at difficulty=5,
seed=42 (or seed=0 for demo, which ignores seed). Runs headless, no firmware."""

from sim3d.themes import THEMES
from sim3d.world import build_world3d
from sim3d.solver import has_path

def main():
    for theme in THEMES:
        world = build_world3d(seed=42, theme=theme, difficulty=5)
        assert len(world.obstacles) > 0, f"{theme}: no obstacles"
        assert has_path(world), f"{theme}: solver says no path at d=5 seed=42"
        print(f"OK {theme}: {len(world.obstacles)} obstacles, solvable")
    print("THEMES_3D OK")

if __name__ == "__main__":
    main()
```

`tests/test_smoke_3d.py` unchanged. The existing v1_3d/v2_3d/seed-42 demo continues to use `theme=demo` (the default).

## 11. Reproducibility contract

`(theme, difficulty, seed)` is the full reproducibility triplet. Given the same triplet, `build_world3d` returns the same `World3D`. The solvability retry loop perturbs `seed` (not `theme` or `difficulty`), so the actual final seed used may be `seed + attempt` for `attempt ∈ [0, 9]` — this is intentional and transparent.

## 12. Tech stack

No new third-party dependencies. Pure stdlib + numpy (already pinned). raylib-py unchanged. No build/install changes.

## 13. Demo runbook

```
.venv\Scripts\activate

# 2D + 3D regression (existing demos still work)
python -m harness.run --firmware v2 --seed 42 --render
python -m harness.run3d --firmware v2_3d --seed 42 --render

# New themed demos
python -m harness.run3d --firmware v2_3d --theme forest    --difficulty 7 --seed 42 --render
python -m harness.run3d --firmware v2_3d --theme warehouse --difficulty 5 --seed 11 --render
python -m harness.run3d --firmware v2_3d --theme canyon    --difficulty 4 --seed 99 --render
python -m harness.run3d --firmware v2_3d --theme city      --difficulty 6 --seed 33 --render

# Sweeps across themes
python -m harness.run3d --firmware v2_3d --theme forest --difficulty 5 --sweep
python -m harness.run3d --firmware v2_3d --theme city   --difficulty 7 --sweep

# New per-theme smoke
python -m tests.test_themes_3d
```

## 14. Risk register

| Risk | Mitigation |
|---|---|
| Solver false-negative makes a fly-able world look unsolvable | A* on coarse 0.5 m grid is an under-approximation by design. If too many false negatives, drop to 0.25 m grid (still ~50 ms). |
| Solver too slow on this machine | 9 600 cells × 6-connected A* completes in single-digit ms. If profiling shows otherwise, drop to 1.0 m grid (1 200 cells). |
| `--difficulty 10 --theme canyon` is always unsolvable | After 10 retry attempts, warn-and-return last attempt. Sweep N/5 reflects the genuine struggle rather than hiding it. |
| Default theme drift accidentally breaks v1_3d/v2_3d/seed-42 narrative | Default = `demo`; `theme=demo` returns the hand-crafted layout byte-identical to today; existing tests and demo runbook keep working unchanged. |
| Per-seed sweep line gets too long with theme+diff prefix | Acceptable — terminal wraps gracefully; the user wants the self-documenting per-line context for replay. |
| v2_3d's empirically-tuned constants (REP_Z etc.) overfit to the demo layout and fail across new themes | This is the intended stress test of the generator — if v2_3d gets <2/5 on a theme at d=5, that's useful signal, not a generator bug. Document in the report; do not silently re-tune v2_3d to chase numbers. |
| Renderer banner overflow at 1280 px | Pre-truncate banner strings > 60 chars in `_run_one` before passing to `r.hold`. |
| Themes with overlap-rejection retries can fail to place all requested obstacles at high difficulty | Attempt cap = 200 per obstacle. If under-target, return whatever was placed; solvability check still runs. The world will simply be sparser than requested at extreme density. |

## 15. Out-of-scope follow-ups (future specs)

- Composite primitives: doorway-walls, corridors, ramps, slalom gates.
- LLM-driven `--prompt "..."` scene generation.
- World library: dump generated worlds to JSON, load `--world path/to/world.json`, curated test suites.
- New mission types: multi-waypoint courses, multiple goals, opposition drones, moving obstacles, hoop sequences.
- Curriculum / adaptive difficulty.
- 2D port of the theme generator.
- Theme-specific renderer flavor (forest = green pillars, canyon = sandstone, etc.).
