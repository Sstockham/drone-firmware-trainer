# Environment Generator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add four new themed 3D world generators (`forest`, `warehouse`, `canyon`, `city`) plus the existing hand-crafted layout as a `demo` theme, a single `--difficulty 1..10` knob, and a coarse-grid A* solvability check that filters out impossible layouts before they reach the firmware. The existing 2D + 3D demos must continue to work unchanged.

**Architecture:** New `sim3d/themes/` package (one file per theme + registry) + new `sim3d/solver.py`. `sim3d/world.py:build_world3d` gains `theme` and `difficulty` parameters with `theme="demo"` as default so all existing invocations keep working byte-identically. `harness/run3d.py` gains `--theme` and `--difficulty` argparse options.

**Tech Stack:** Python 3.11+, stdlib only for the new code (no new third-party deps). numpy and raylib-py unchanged.

## Global Constraints

- Python 3.11+ on Windows. Use the existing `.venv` (`.venv\Scripts\python.exe`).
- No new third-party dependencies. The new code uses only `random`, `heapq`, `warnings`, `dataclasses` from stdlib.
- Do NOT modify any of: `firmware/contract.py`, `firmware/contract3d.py`, `firmware/firmware_v1.py`, `firmware/firmware_v2.py`, `firmware/firmware_v1_3d.py`, `firmware/firmware_v2_3d.py`, `sim/*`, `sim3d/physics.py`, `sim3d/sensors.py`, `sim3d/renderer.py`, `harness/loop.py`, `harness/loop3d.py`, `harness/run.py`, `harness/scoring.py`, `tests/test_smoke.py`, `tests/test_smoke_3d.py`. The 2D demo + the existing 3D `demo` theme must still pass after every task.
- `build_world3d` signature MUST be `build_world3d(seed: int, theme: str = "demo", difficulty: int = 5) -> World3D`. Default `theme="demo"` preserves backwards compatibility with existing callers (`tests/test_smoke_3d.py`, `harness/run3d.py:_run_one`).
- `THEME_REGISTRY` dict in `sim3d/themes/__init__.py` is the single source of truth for theme name → generator function mapping. `THEMES = tuple(THEME_REGISTRY.keys())` exposes the choices tuple for argparse.
- Each `generate(seed: int, difficulty: int) -> list[Rect3D]` is pure: no global state, no I/O. Themes that ignore `seed` or `difficulty` accept the params and discard them.
- Difficulty: int, default 5, **silently clamped** to `[1, 10]` in `build_world3d` (one-line stderr warning if out of range — not an error).
- Solvability: `sim3d/solver.py:has_path(world) -> bool`, coarse 3D grid A* at `GRID_RES = 0.5 m` = 40×30×8 = 9600 cells. 6-connected (axis-aligned only). Manhattan heuristic.
- `build_world3d` retries with `seed + attempt` up to 10 times if the world isn't solvable; on 10 failures, emits a `RuntimeWarning` and returns the last attempt anyway.
- `sim3d/themes/*.py` import only `sim3d.world` (`Rect3D, ARENA_W, ARENA_D, CEILING, SPAWN, GOAL_CENTER, GOAL_RADIUS`) and stdlib. `sim3d/solver.py` imports only `sim3d.world` and stdlib. No imports from `firmware/` or `harness/`.
- Commit after every task. Single `main` branch (already on `main`). Push at end (we did this for the 2D and 3D shipping cycles).
- File writes: prefer the Write tool or `pathlib.Path.write_text(..., encoding="utf-8")`. Do NOT use PowerShell `Out-File -Encoding utf8` (BOM bug). Do NOT put `\a` or `\S` inside non-raw Python strings (BEL bug).

## Task Dependency Graph

```
T1 solver.py
  T2 themes/__init__.py + demo.py (extract hand-crafted layout)
    T3 forest.py
      T4 warehouse.py
        T5 canyon.py
          T6 city.py
            T7 tests/test_themes_3d.py (smoke check all 5 themes solvable at d=5)
              T8 world.py build_world3d refactor (theme + difficulty + solver retry)
                T9 run3d.py CLI (--theme, --difficulty, announce line, banner)
                  T10 full regression + difficulty calibration + README update
```

Single controller, serial execution. T1 and T2 have no real dependency on each other but I'm ordering T1 first because T8 needs both and the test-then-integrate pattern of TDD-lite means we want the solver landed before themes lean on its behavior.

---

### Task 1: sim3d/solver.py

**Files:**
- Create: `sim3d/solver.py`

**Interfaces:**
- Consumes: `World3D`, `Rect3D`, `ARENA_W`, `ARENA_D`, `CEILING`, `SPAWN`, `GOAL_CENTER`, `GOAL_RADIUS` from `sim3d.world`.
- Produces: `has_path(world: World3D) -> bool`, plus module-level `GRID_RES = 0.5`.

- [ ] **Step 1: Write the file**

```python
"""Coarse-grid A* solvability check for World3D layouts.

Used by build_world3d to filter out impossible procedurally-generated worlds
before the firmware ever runs against them. Intentionally an under-approximation
of true reachability: the 0.5 m grid is coarser than the drone (0.3 m cube), so
a True result means "there's room for a 0.5 m-resolution path" — the firmware
still has to fly it.
"""

import heapq
import math

from sim3d.world import (
    ARENA_W, ARENA_D, CEILING,
    SPAWN, GOAL_CENTER, GOAL_RADIUS,
    Rect3D, World3D,
)

GRID_RES = 0.5
NX = int(ARENA_W / GRID_RES)
NY = int(ARENA_D / GRID_RES)
NZ = int(CEILING / GRID_RES)


def _cell_center(i: int, j: int, k: int) -> tuple[float, float, float]:
    return (
        (i + 0.5) * GRID_RES,
        (j + 0.5) * GRID_RES,
        (k + 0.5) * GRID_RES,
    )


def _cell_of(px: float, py: float, pz: float) -> tuple[int, int, int]:
    return (
        max(0, min(NX - 1, int(px / GRID_RES))),
        max(0, min(NY - 1, int(py / GRID_RES))),
        max(0, min(NZ - 1, int(pz / GRID_RES))),
    )


def _blocked(i: int, j: int, k: int, rects: list[Rect3D]) -> bool:
    cx, cy, cz = _cell_center(i, j, k)
    return any(r.contains_point3d(cx, cy, cz) for r in rects)


def _is_goal_cell(i: int, j: int, k: int) -> bool:
    cx, cy, cz = _cell_center(i, j, k)
    gx, gy, gz = GOAL_CENTER
    return math.sqrt((cx - gx) ** 2 + (cy - gy) ** 2 + (cz - gz) ** 2) <= GOAL_RADIUS


def has_path(world: World3D) -> bool:
    rects = world.obstacles
    start = _cell_of(*world.spawn)
    if _blocked(*start, rects):
        return False

    open_set: list[tuple[int, int, tuple[int, int, int]]] = []
    counter = 0
    heapq.heappush(open_set, (0, counter, start))
    g_score: dict[tuple[int, int, int], int] = {start: 0}

    neighbors = (
        (1, 0, 0), (-1, 0, 0),
        (0, 1, 0), (0, -1, 0),
        (0, 0, 1), (0, 0, -1),
    )

    while open_set:
        _, _, (i, j, k) = heapq.heappop(open_set)
        if _is_goal_cell(i, j, k):
            return True
        for di, dj, dk in neighbors:
            ni, nj, nk = i + di, j + dj, k + dk
            if not (0 <= ni < NX and 0 <= nj < NY and 0 <= nk < NZ):
                continue
            if _blocked(ni, nj, nk, rects):
                continue
            tentative = g_score[(i, j, k)] + 1
            if tentative >= g_score.get((ni, nj, nk), 10 ** 9):
                continue
            g_score[(ni, nj, nk)] = tentative
            gx, gy, gz = GOAL_CENTER
            cx, cy, cz = _cell_center(ni, nj, nk)
            heur = int((abs(cx - gx) + abs(cy - gy) + abs(cz - gz)) / GRID_RES)
            counter += 1
            heapq.heappush(open_set, (tentative + heur, counter, (ni, nj, nk)))

    return False
```

- [ ] **Step 2: Verify on the existing demo world (which IS solvable)**

```powershell
.venv\Scripts\python.exe -c "from sim3d.world import build_world3d; from sim3d.solver import has_path; w = build_world3d(42); print('demo solvable:', has_path(w))"
```

Expected: `demo solvable: True`. (If False, the solver has a bug — STOP and report BLOCKED, because the demo world is the known-good ground truth: v2_3d navigates it end-to-end in the existing demo.)

- [ ] **Step 3: Verify on a synthetic impossible world (wall across the arena)**

```powershell
.venv\Scripts\python.exe -c "
from sim3d.world import build_world3d, Rect3D
from sim3d.solver import has_path
w = build_world3d(42)
w.obstacles = [Rect3D(0.0, 7.0, 0.0, 20.0, 0.5, 4.0)]  # full-arena wall between spawn and goal
print('walled-off solvable:', has_path(w))
"
```

Expected: `walled-off solvable: False`.

- [ ] **Step 4: Time the solver on the demo world**

```powershell
.venv\Scripts\python.exe -c "
import time
from sim3d.world import build_world3d
from sim3d.solver import has_path
w = build_world3d(42)
t0 = time.perf_counter()
for _ in range(10):
    has_path(w)
elapsed_ms = (time.perf_counter() - t0) * 1000 / 10
print(f'avg solver time: {elapsed_ms:.1f} ms')
"
```

Expected: under 50 ms. The spec's budget is <10 ms but a 5× headroom is acceptable. If over 50 ms, report DONE_WITH_CONCERNS and note in the report — the controller will decide whether to raise `GRID_RES`.

- [ ] **Step 5: Commit**

```powershell
git add sim3d/solver.py
git commit -m "feat(sim3d): coarse-grid A* solvability check"
```

---

### Task 2: sim3d/themes/__init__.py + demo.py

**Files:**
- Create: `sim3d/themes/__init__.py`
- Create: `sim3d/themes/demo.py`

**Interfaces:**
- Consumes: `Rect3D` from `sim3d.world`.
- Produces:
  - `sim3d.themes.demo.generate(seed: int, difficulty: int) -> list[Rect3D]` — returns the three hand-crafted set-pieces (tall wall, short bar, hanging overhang). Ignores both parameters.
  - `sim3d.themes.THEME_REGISTRY: dict[str, Callable]` containing `"demo": demo.generate` at this stage. Later tasks will add the other four themes to the registry.
  - `sim3d.themes.THEMES: tuple[str, ...]` for argparse choices.

- [ ] **Step 1: Create the themes package**

Use Python to write the empty `__init__.py` first (avoid BOM):

```powershell
.venv\Scripts\python.exe -c "import pathlib; pathlib.Path('sim3d/themes').mkdir(exist_ok=True); pathlib.Path('sim3d/themes/__init__.py').write_text('', encoding='utf-8')"
```

(We'll fill `__init__.py` in Step 3 — for now it's empty so the directory becomes an importable package.)

- [ ] **Step 2: Write `sim3d/themes/demo.py`**

```python
"""The hand-crafted seed-42 three-set-piece layout from the original 3D demo.

This theme exists so that the v1_3d-vs-v2_3d demo narrative (tall wall +
short bar + hanging overhang) keeps working byte-identically after build_world3d
gains its theme dispatch. Ignores `seed` and `difficulty`.
"""

from sim3d.world import Rect3D


def generate(seed: int, difficulty: int) -> list[Rect3D]:
    del seed, difficulty
    return [
        Rect3D(6.0, 6.0, 0.0, 4.0, 0.6, 4.0),    # tall wall, full height
        Rect3D(11.0, 9.0, 0.0, 3.0, 0.6, 1.0),   # short bar, 1.0 m
        Rect3D(14.0, 11.5, 1.0, 0.6, 3.0, 3.0),  # hanging overhang, bottom z=1.0 to ceiling
    ]
```

- [ ] **Step 3: Write `sim3d/themes/__init__.py`**

```python
"""Theme registry — maps a theme name to a generator function.

Each generator is `generate(seed: int, difficulty: int) -> list[Rect3D]`,
pure (no global state, no I/O). Themes that ignore parameters accept them
and discard them.

Other themes (forest, warehouse, canyon, city) are added to the registry as
they land in their own tasks.
"""

from typing import Callable

from sim3d.world import Rect3D
from sim3d.themes import demo

THEME_REGISTRY: dict[str, Callable[[int, int], list[Rect3D]]] = {
    "demo": demo.generate,
}

THEMES: tuple[str, ...] = tuple(THEME_REGISTRY.keys())
```

- [ ] **Step 4: Verify imports and demo theme output**

```powershell
.venv\Scripts\python.exe -c "from sim3d.themes import THEME_REGISTRY, THEMES; obs = THEME_REGISTRY['demo'](42, 5); print('themes:', THEMES); print('demo obstacle count:', len(obs)); print('first obstacle:', obs[0])"
```

Expected:
```
themes: ('demo',)
demo obstacle count: 3
first obstacle: Rect3D(x=6.0, y=6.0, z=0.0, w=4.0, d=0.6, h=4.0)
```

- [ ] **Step 5: Commit**

```powershell
git add sim3d/themes/__init__.py sim3d/themes/demo.py
git commit -m "feat(themes): registry + demo theme (extracted hand-crafted layout)"
```

---

### Task 3: sim3d/themes/forest.py

**Files:**
- Create: `sim3d/themes/forest.py`
- Modify: `sim3d/themes/__init__.py` (add forest to registry)

**Interfaces:**
- Consumes: `Rect3D`, `ARENA_W`, `ARENA_D`, `CEILING`, `SPAWN`, `GOAL_CENTER` from `sim3d.world`.
- Produces: `sim3d.themes.forest.generate(seed: int, difficulty: int) -> list[Rect3D]`. Up to N pillars, tall thin floor-to-ceiling, rejected if they contain spawn/goal or AABB-overlap an existing pillar (with 0.2 m padding). `_scale(d)` returns `count = round(5 + (25 - 5) * (d-1)/9)`.

- [ ] **Step 1: Write the theme**

```python
"""Forest — tall thin vertical pillars (like trees).

Floor-standing, ceiling-touching, w/d in [0.3, 0.5]. Difficulty scales the
count: 5 pillars at d=1, 25 at d=10. Rejection-sampled to avoid spawn/goal
and overlapping placement.
"""

import random

from sim3d.world import (
    ARENA_W, ARENA_D, CEILING,
    SPAWN, GOAL_CENTER,
    Rect3D,
)

X_MARGIN = 3.0
Y_MARGIN = 2.0
W_RANGE = (0.3, 0.5)
PAD = 0.2
ATTEMPT_CAP = 200


def _scale(difficulty: int) -> int:
    return round(5 + (25 - 5) * (difficulty - 1) / 9)


def _overlaps(cand: Rect3D, existing: list[Rect3D]) -> bool:
    cand_cx = cand.x + cand.w / 2
    cand_cy = cand.y + cand.d / 2
    for r in existing:
        rcx = r.x + r.w / 2
        rcy = r.y + r.d / 2
        dx_thresh = (cand.w + r.w) / 2 + PAD
        dy_thresh = (cand.d + r.d) / 2 + PAD
        if abs(cand_cx - rcx) < dx_thresh and abs(cand_cy - rcy) < dy_thresh:
            return True
    return False


def generate(seed: int, difficulty: int) -> list[Rect3D]:
    rng = random.Random(seed)
    target_count = _scale(difficulty)
    rects: list[Rect3D] = []
    attempts = 0
    while len(rects) < target_count and attempts < ATTEMPT_CAP:
        attempts += 1
        w = rng.uniform(*W_RANGE)
        d = rng.uniform(*W_RANGE)
        x = rng.uniform(X_MARGIN, ARENA_W - X_MARGIN) - w / 2
        y = rng.uniform(Y_MARGIN, ARENA_D - Y_MARGIN) - d / 2
        cand = Rect3D(x, y, 0.0, w, d, CEILING)
        if cand.contains_point3d(*SPAWN) or cand.contains_point3d(*GOAL_CENTER):
            continue
        if _overlaps(cand, rects):
            continue
        rects.append(cand)
    return rects
```

- [ ] **Step 2: Register the theme**

Modify `sim3d/themes/__init__.py` so the imports and registry now include forest:

```python
"""Theme registry — maps a theme name to a generator function.

Each generator is `generate(seed: int, difficulty: int) -> list[Rect3D]`,
pure (no global state, no I/O). Themes that ignore parameters accept them
and discard them.

Other themes (warehouse, canyon, city) are added to the registry as they
land in their own tasks.
"""

from typing import Callable

from sim3d.world import Rect3D
from sim3d.themes import demo, forest

THEME_REGISTRY: dict[str, Callable[[int, int], list[Rect3D]]] = {
    "demo": demo.generate,
    "forest": forest.generate,
}

THEMES: tuple[str, ...] = tuple(THEME_REGISTRY.keys())
```

- [ ] **Step 3: Verify**

```powershell
.venv\Scripts\python.exe -c "from sim3d.themes import forest; obs = forest.generate(42, 5); print('d=5 count:', len(obs)); obs = forest.generate(42, 1); print('d=1 count:', len(obs)); obs = forest.generate(42, 10); print('d=10 count:', len(obs))"
```

Expected: `d=5 count:` around 16 (`round(5 + 20*4/9) = 14`), `d=1 count: 5`, `d=10 count:` 25 or close to it (may be less if rejection cap hits). Acceptable as long as d=1 yields exactly 5 and d=10 yields ≥ 18.

- [ ] **Step 4: Verify solvability at default difficulty**

```powershell
.venv\Scripts\python.exe -c "from sim3d.world import build_world3d, World3D, ARENA_W, ARENA_D, CEILING, SPAWN, GOAL_CENTER, GOAL_RADIUS; from sim3d.themes import forest; from sim3d.solver import has_path; obs = forest.generate(42, 5); w = World3D(arena_w=ARENA_W, arena_d=ARENA_D, ceiling=CEILING, obstacles=obs, spawn=SPAWN, goal_center=GOAL_CENTER, goal_radius=GOAL_RADIUS); print('forest d=5 seed=42 solvable:', has_path(w))"
```

Expected: `True`. If `False`, try a different seed (`forest.generate(7, 5)`, `forest.generate(13, 5)`). At difficulty=5 most seeds should be solvable.

- [ ] **Step 5: Commit**

```powershell
git add sim3d/themes/forest.py sim3d/themes/__init__.py
git commit -m "feat(themes): forest — tall thin vertical pillars"
```

---

### Task 4: sim3d/themes/warehouse.py

**Files:**
- Create: `sim3d/themes/warehouse.py`
- Modify: `sim3d/themes/__init__.py` (add warehouse to registry)

**Interfaces:**
- Consumes: `Rect3D`, `ARENA_W`, `ARENA_D`, `SPAWN`, `GOAL_CENTER` from `sim3d.world`.
- Produces: `sim3d.themes.warehouse.generate(seed: int, difficulty: int) -> list[Rect3D]`. Floor-standing crates in 3-column grid at `x ∈ {5, 10, 15}` with row jitter. `_scale(d)` returns `(count, hmax)` where `count = round(6 + 12 * (d-1)/9)` and `hmax = 1.5 + 1.0 * (d-1)/9`.

- [ ] **Step 1: Write the theme**

```python
"""Warehouse — floor-standing crates in semi-regular aisles.

3-column grid at x in {5, 10, 15} with row jitter and random skipping.
Some crates short enough to fly over (h up to hmax). Difficulty scales
count (6-18) and hmax (1.5-2.5).
"""

import random

from sim3d.world import (
    ARENA_W, ARENA_D,
    SPAWN, GOAL_CENTER,
    Rect3D,
)

COLUMNS = (5.0, 10.0, 15.0)
ROW_Y_MIN = 2.0
ROW_Y_MAX = 12.0
ROW_STEP = 2.5
W_RANGE = (1.0, 2.0)
D_RANGE = (1.0, 2.0)
H_MIN = 1.0
JITTER = 0.3


def _scale(difficulty: int) -> tuple[int, float]:
    count = round(6 + 12 * (difficulty - 1) / 9)
    hmax = 1.5 + 1.0 * (difficulty - 1) / 9
    return count, hmax


def generate(seed: int, difficulty: int) -> list[Rect3D]:
    rng = random.Random(seed)
    target_count, hmax = _scale(difficulty)
    candidates: list[Rect3D] = []
    y = ROW_Y_MIN
    while y <= ROW_Y_MAX:
        for col_x in COLUMNS:
            w = rng.uniform(*W_RANGE)
            d = rng.uniform(*D_RANGE)
            h = rng.uniform(H_MIN, hmax)
            jx = rng.uniform(-JITTER, JITTER)
            jy = rng.uniform(-JITTER, JITTER)
            cand = Rect3D(col_x - w / 2 + jx, y - d / 2 + jy, 0.0, w, d, h)
            if cand.contains_point3d(*SPAWN) or cand.contains_point3d(*GOAL_CENTER):
                continue
            candidates.append(cand)
        y += ROW_STEP

    rng.shuffle(candidates)
    return candidates[:target_count]
```

- [ ] **Step 2: Register the theme**

Modify `sim3d/themes/__init__.py` again:

```python
"""Theme registry — maps a theme name to a generator function.

Each generator is `generate(seed: int, difficulty: int) -> list[Rect3D]`,
pure (no global state, no I/O). Themes that ignore parameters accept them
and discard them.

Other themes (canyon, city) are added to the registry as they land in their
own tasks.
"""

from typing import Callable

from sim3d.world import Rect3D
from sim3d.themes import demo, forest, warehouse

THEME_REGISTRY: dict[str, Callable[[int, int], list[Rect3D]]] = {
    "demo": demo.generate,
    "forest": forest.generate,
    "warehouse": warehouse.generate,
}

THEMES: tuple[str, ...] = tuple(THEME_REGISTRY.keys())
```

- [ ] **Step 3: Verify counts and hmax scaling**

```powershell
.venv\Scripts\python.exe -c "from sim3d.themes import warehouse; obs = warehouse.generate(42, 5); print('d=5 count:', len(obs), 'max h:', round(max(r.h for r in obs), 2)); obs = warehouse.generate(42, 1); print('d=1 count:', len(obs), 'max h:', round(max(r.h for r in obs), 2)); obs = warehouse.generate(42, 10); print('d=10 count:', len(obs), 'max h:', round(max(r.h for r in obs), 2))"
```

Expected: d=1 → count=6 max h≤1.5; d=5 → count≈12 max h≤2.0; d=10 → count=18 max h≤2.5.

- [ ] **Step 4: Verify solvability**

```powershell
.venv\Scripts\python.exe -c "from sim3d.world import build_world3d, World3D, ARENA_W, ARENA_D, CEILING, SPAWN, GOAL_CENTER, GOAL_RADIUS; from sim3d.themes import warehouse; from sim3d.solver import has_path; obs = warehouse.generate(42, 5); w = World3D(arena_w=ARENA_W, arena_d=ARENA_D, ceiling=CEILING, obstacles=obs, spawn=SPAWN, goal_center=GOAL_CENTER, goal_radius=GOAL_RADIUS); print('warehouse d=5 seed=42 solvable:', has_path(w))"
```

Expected: `True`.

- [ ] **Step 5: Commit**

```powershell
git add sim3d/themes/warehouse.py sim3d/themes/__init__.py
git commit -m "feat(themes): warehouse — floor-standing crates in semi-regular aisles"
```

---

### Task 5: sim3d/themes/canyon.py

**Files:**
- Create: `sim3d/themes/canyon.py`
- Modify: `sim3d/themes/__init__.py` (add canyon to registry)

**Interfaces:**
- Consumes: `Rect3D`, `ARENA_W`, `ARENA_D`, `CEILING` from `sim3d.world`.
- Produces: `sim3d.themes.canyon.generate(seed: int, difficulty: int) -> list[Rect3D]`. Deterministic placement (ignores `seed`). `_scale(d)` returns `(count, gap)` where `count = round(3 + 5 * (d-1)/9)` and `gap = 4.0 + (0.8 - 4.0) * (d-1)/9`.

- [ ] **Step 1: Write the theme**

```python
"""Canyon — long horizontal slabs perpendicular to spawn-to-goal axis.

Deterministic placement (ignores seed). Slabs span x-direction with thin y
extent and full ceiling height. Each slab leaves a gap-wide opening at ONE
end, alternating left/right to force a zigzag. Difficulty scales slab count
(3 -> 8) and shrinks the gap (4.0 m -> 0.8 m).
"""

from sim3d.world import (
    ARENA_W, ARENA_D, CEILING,
    Rect3D,
)

SLAB_DEPTH = 0.5
Y_MIN = 3.0
Y_MAX = 12.0
X_EDGE = 1.0  # slab/gap distance from the arena's x edges


def _scale(difficulty: int) -> tuple[int, float]:
    count = round(3 + 5 * (difficulty - 1) / 9)
    gap = 4.0 + (0.8 - 4.0) * (difficulty - 1) / 9
    return count, gap


def generate(seed: int, difficulty: int) -> list[Rect3D]:
    del seed
    count, gap = _scale(difficulty)
    slab_w = ARENA_W - 2 * X_EDGE - gap
    if count == 1:
        ys = [(Y_MIN + Y_MAX) / 2]
    else:
        step = (Y_MAX - Y_MIN) / (count - 1)
        ys = [Y_MIN + i * step for i in range(count)]

    rects: list[Rect3D] = []
    for i, y in enumerate(ys):
        if i % 2 == 0:
            x = X_EDGE
        else:
            x = X_EDGE + gap
        rects.append(Rect3D(x, y - SLAB_DEPTH / 2, 0.0, slab_w, SLAB_DEPTH, CEILING))
    return rects
```

- [ ] **Step 2: Register the theme**

Modify `sim3d/themes/__init__.py`:

```python
"""Theme registry — maps a theme name to a generator function.

Each generator is `generate(seed: int, difficulty: int) -> list[Rect3D]`,
pure (no global state, no I/O). Themes that ignore parameters accept them
and discard them.

The city theme is added in its own task.
"""

from typing import Callable

from sim3d.world import Rect3D
from sim3d.themes import demo, forest, warehouse, canyon

THEME_REGISTRY: dict[str, Callable[[int, int], list[Rect3D]]] = {
    "demo": demo.generate,
    "forest": forest.generate,
    "warehouse": warehouse.generate,
    "canyon": canyon.generate,
}

THEMES: tuple[str, ...] = tuple(THEME_REGISTRY.keys())
```

- [ ] **Step 3: Verify counts and gap scaling**

```powershell
.venv\Scripts\python.exe -c "from sim3d.themes import canyon; obs = canyon.generate(0, 1); print('d=1 count:', len(obs), 'w:', round(obs[0].w, 2)); obs = canyon.generate(0, 5); print('d=5 count:', len(obs), 'w:', round(obs[0].w, 2)); obs = canyon.generate(0, 10); print('d=10 count:', len(obs), 'w:', round(obs[0].w, 2))"
```

Expected: d=1 → count=3, w≈14.0; d=5 → count≈5, w≈12.2; d=10 → count=8, w≈17.2.

- [ ] **Step 4: Verify solvability at default difficulty**

```powershell
.venv\Scripts\python.exe -c "from sim3d.world import build_world3d, World3D, ARENA_W, ARENA_D, CEILING, SPAWN, GOAL_CENTER, GOAL_RADIUS; from sim3d.themes import canyon; from sim3d.solver import has_path; obs = canyon.generate(0, 5); w = World3D(arena_w=ARENA_W, arena_d=ARENA_D, ceiling=CEILING, obstacles=obs, spawn=SPAWN, goal_center=GOAL_CENTER, goal_radius=GOAL_RADIUS); print('canyon d=5 solvable:', has_path(w))"
```

Expected: `True`.

- [ ] **Step 5: Commit**

```powershell
git add sim3d/themes/canyon.py sim3d/themes/__init__.py
git commit -m "feat(themes): canyon — long horizontal slabs with alternating gaps"
```

---

### Task 6: sim3d/themes/city.py

**Files:**
- Create: `sim3d/themes/city.py`
- Modify: `sim3d/themes/__init__.py` (add city to registry)

**Interfaces:**
- Consumes: `Rect3D`, `ARENA_W`, `ARENA_D`, `CEILING`, `SPAWN`, `GOAL_CENTER` from `sim3d.world`.
- Produces: `sim3d.themes.city.generate(seed: int, difficulty: int) -> list[Rect3D]`. 70/30 mix of tall floor-standing buildings vs. ceiling-hung low bridges. `_scale(d)` returns `total_count = round(5 + 11 * (d-1)/9)`.

- [ ] **Step 1: Write the theme**

```python
"""City — tall floor-standing buildings + low-hanging bridges.

70% buildings (1.5-3.0 w/d, h 2.0-CEILING, floor-standing),
30% bridges (0.5 w, 2.0-3.0 d, z=1.0 h=3.0 — must obstruct default 1.5 m
altitude; drone must descend below ~0.8 m to fit under).
Difficulty scales total count: 5 -> 16.
"""

import random

from sim3d.world import (
    ARENA_W, ARENA_D, CEILING,
    SPAWN, GOAL_CENTER,
    Rect3D,
)

X_MARGIN = 3.0
Y_MARGIN = 2.0
BUILDING_W_RANGE = (1.5, 3.0)
BUILDING_H_RANGE = (2.0, CEILING)
BRIDGE_W = 0.5
BRIDGE_D_RANGE = (2.0, 3.0)
BRIDGE_Z = 1.0
BRIDGE_H = 3.0
BRIDGE_FRACTION = 0.30
PAD = 0.2
ATTEMPT_CAP = 200


def _scale(difficulty: int) -> int:
    return round(5 + 11 * (difficulty - 1) / 9)


def _overlaps(cand: Rect3D, existing: list[Rect3D]) -> bool:
    cand_cx = cand.x + cand.w / 2
    cand_cy = cand.y + cand.d / 2
    for r in existing:
        rcx = r.x + r.w / 2
        rcy = r.y + r.d / 2
        dx_thresh = (cand.w + r.w) / 2 + PAD
        dy_thresh = (cand.d + r.d) / 2 + PAD
        if abs(cand_cx - rcx) < dx_thresh and abs(cand_cy - rcy) < dy_thresh:
            return True
    return False


def _make_building(rng: random.Random) -> Rect3D:
    w = rng.uniform(*BUILDING_W_RANGE)
    d = rng.uniform(*BUILDING_W_RANGE)
    h = rng.uniform(*BUILDING_H_RANGE)
    x = rng.uniform(X_MARGIN, ARENA_W - X_MARGIN) - w / 2
    y = rng.uniform(Y_MARGIN, ARENA_D - Y_MARGIN) - d / 2
    return Rect3D(x, y, 0.0, w, d, h)


def _make_bridge(rng: random.Random) -> Rect3D:
    w = BRIDGE_W
    d = rng.uniform(*BRIDGE_D_RANGE)
    x = rng.uniform(X_MARGIN, ARENA_W - X_MARGIN) - w / 2
    y = rng.uniform(Y_MARGIN, ARENA_D - Y_MARGIN) - d / 2
    return Rect3D(x, y, BRIDGE_Z, w, d, BRIDGE_H)


def generate(seed: int, difficulty: int) -> list[Rect3D]:
    rng = random.Random(seed)
    target_count = _scale(difficulty)
    rects: list[Rect3D] = []
    attempts = 0
    while len(rects) < target_count and attempts < ATTEMPT_CAP:
        attempts += 1
        cand = _make_bridge(rng) if rng.random() < BRIDGE_FRACTION else _make_building(rng)
        if cand.contains_point3d(*SPAWN) or cand.contains_point3d(*GOAL_CENTER):
            continue
        if _overlaps(cand, rects):
            continue
        rects.append(cand)
    return rects
```

- [ ] **Step 2: Register the theme**

Modify `sim3d/themes/__init__.py`:

```python
"""Theme registry — maps a theme name to a generator function.

Each generator is `generate(seed: int, difficulty: int) -> list[Rect3D]`,
pure (no global state, no I/O). Themes that ignore parameters accept them
and discard them.
"""

from typing import Callable

from sim3d.world import Rect3D
from sim3d.themes import demo, forest, warehouse, canyon, city

THEME_REGISTRY: dict[str, Callable[[int, int], list[Rect3D]]] = {
    "demo": demo.generate,
    "forest": forest.generate,
    "warehouse": warehouse.generate,
    "canyon": canyon.generate,
    "city": city.generate,
}

THEMES: tuple[str, ...] = tuple(THEME_REGISTRY.keys())
```

- [ ] **Step 3: Verify counts and bridge mix**

```powershell
.venv\Scripts\python.exe -c "from sim3d.themes import city; obs = city.generate(42, 5); bridges = sum(1 for r in obs if r.z > 0); print('d=5 total:', len(obs), 'bridges:', bridges); obs = city.generate(42, 10); bridges = sum(1 for r in obs if r.z > 0); print('d=10 total:', len(obs), 'bridges:', bridges)"
```

Expected: d=5 → total around 11 (`round(5 + 11*4/9) = 10` or 11), bridges roughly 1-4; d=10 → total around 16, bridges 3-6.

- [ ] **Step 4: Verify solvability**

```powershell
.venv\Scripts\python.exe -c "from sim3d.world import build_world3d, World3D, ARENA_W, ARENA_D, CEILING, SPAWN, GOAL_CENTER, GOAL_RADIUS; from sim3d.themes import city; from sim3d.solver import has_path; obs = city.generate(42, 5); w = World3D(arena_w=ARENA_W, arena_d=ARENA_D, ceiling=CEILING, obstacles=obs, spawn=SPAWN, goal_center=GOAL_CENTER, goal_radius=GOAL_RADIUS); print('city d=5 seed=42 solvable:', has_path(w))"
```

Expected: `True`. If False, try seed 7 — at default difficulty the city should mostly be solvable.

- [ ] **Step 5: Commit**

```powershell
git add sim3d/themes/city.py sim3d/themes/__init__.py
git commit -m "feat(themes): city — tall buildings + low ceiling-hung bridges"
```

---

### Task 7: tests/test_themes_3d.py

**Files:**
- Create: `tests/test_themes_3d.py`

**Interfaces:**
- Consumes: `THEMES` from `sim3d.themes`, `World3D`, `ARENA_W`, `ARENA_D`, `CEILING`, `SPAWN`, `GOAL_CENTER`, `GOAL_RADIUS` from `sim3d.world`, `has_path` from `sim3d.solver`.
- Produces: a runnable smoke check `python -m tests.test_themes_3d`. Confirms each theme returns at least one obstacle at difficulty=5 and that the world is solvable.

- [ ] **Step 1: Write the file**

```python
"""Per-theme smoke. Confirms each theme yields a solvable world at difficulty=5,
seed=42. Runs headless, no firmware, no rendering."""

from sim3d.themes import THEMES, THEME_REGISTRY
from sim3d.world import (
    ARENA_W, ARENA_D, CEILING,
    SPAWN, GOAL_CENTER, GOAL_RADIUS,
    World3D,
)
from sim3d.solver import has_path


def main() -> None:
    for theme in THEMES:
        obstacles = THEME_REGISTRY[theme](42, 5)
        assert len(obstacles) > 0, f"{theme}: no obstacles generated"
        world = World3D(
            arena_w=ARENA_W, arena_d=ARENA_D, ceiling=CEILING,
            obstacles=obstacles,
            spawn=SPAWN, goal_center=GOAL_CENTER, goal_radius=GOAL_RADIUS,
        )
        assert has_path(world), f"{theme}: solver says no path at d=5 seed=42"
        print(f"OK {theme}: {len(obstacles)} obstacles, solvable")
    print("THEMES_3D OK")


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run it**

```powershell
.venv\Scripts\python.exe -m tests.test_themes_3d
```

Expected: 5 `OK <theme>:` lines (one per theme) followed by `THEMES_3D OK`. The `demo` theme is solvable by construction (it's the existing demo). If any non-demo theme fails the assertion, STOP and report — the theme's parameters need adjustment (most likely the canyon at high difficulty or city with too many bridges blocking the path).

- [ ] **Step 3: Commit**

```powershell
git add tests/test_themes_3d.py
git commit -m "test: per-theme smoke (5 themes solvable at d=5)"
```

---

### Task 8: build_world3d refactor (sim3d/world.py)

**Files:**
- Modify: `sim3d/world.py` — change `build_world3d` signature, delete `_hand_crafted_seed_42` and `_random_obstacles3d`, add theme dispatch + solvability retry.

**Interfaces:**
- Consumes: `THEME_REGISTRY` from `sim3d.themes`, `has_path` from `sim3d.solver`.
- Produces: new `build_world3d(seed: int, theme: str = "demo", difficulty: int = 5) -> World3D`. `Rect3D`, `World3D`, and the module constants (`ARENA_W`, `ARENA_D`, `CEILING`, `SPAWN`, `GOAL_CENTER`, `GOAL_RADIUS`) keep their existing shape and importability.

- [ ] **Step 1: Read the current `sim3d/world.py` to understand the constants block + Rect3D + World3D**

Just read the file with the Read tool — DON'T edit it yet. Confirm the imports of `random` and `dataclass` exist; you will be removing `random` (no longer needed) and the two private helpers, but keeping everything else.

- [ ] **Step 2: Rewrite `sim3d/world.py` with theme dispatch**

Replace the existing file contents entirely with:

```python
import sys
from dataclasses import dataclass
import warnings

ARENA_W = 20.0
ARENA_D = 15.0
CEILING = 4.0
SPAWN = (1.0, 1.0, 1.5)
GOAL_CENTER = (18.0, 13.0, 1.5)
GOAL_RADIUS = 0.75


@dataclass(frozen=True)
class Rect3D:
    x: float
    y: float
    z: float
    w: float
    d: float
    h: float

    def contains_point3d(self, px: float, py: float, pz: float) -> bool:
        return (
            self.x <= px <= self.x + self.w
            and self.y <= py <= self.y + self.d
            and self.z <= pz <= self.z + self.h
        )


@dataclass
class World3D:
    arena_w: float
    arena_d: float
    ceiling: float
    obstacles: list
    spawn: tuple
    goal_center: tuple
    goal_radius: float


_SOLVABILITY_RETRIES = 10


def build_world3d(seed: int, theme: str = "demo", difficulty: int = 5):
    # Local import to break circular dep: sim3d.themes imports sim3d.world for Rect3D
    from sim3d.themes import THEME_REGISTRY
    from sim3d.solver import has_path

    if theme not in THEME_REGISTRY:
        raise ValueError(f"unknown theme {theme!r}; choices: {sorted(THEME_REGISTRY)}")

    if difficulty < 1 or difficulty > 10:
        print(
            f"[warn] difficulty {difficulty} out of range [1, 10]; clamping",
            file=sys.stderr,
        )
        difficulty = max(1, min(10, difficulty))

    generator = THEME_REGISTRY[theme]
    last_world = None
    for attempt in range(_SOLVABILITY_RETRIES):
        obstacles = generator(seed + attempt, difficulty)
        candidate = World3D(
            arena_w=ARENA_W,
            arena_d=ARENA_D,
            ceiling=CEILING,
            obstacles=obstacles,
            spawn=SPAWN,
            goal_center=GOAL_CENTER,
            goal_radius=GOAL_RADIUS,
        )
        last_world = candidate
        if has_path(candidate):
            return candidate

    warnings.warn(
        f"unsolvable after {_SOLVABILITY_RETRIES} attempts: "
        f"theme={theme} difficulty={difficulty} seed={seed} — returning the last attempt anyway",
        RuntimeWarning,
        stacklevel=2,
    )
    return last_world
```

Note the local imports inside `build_world3d`: `sim3d.themes` and `sim3d.solver` both import from `sim3d.world` (for `Rect3D`, `World3D`, constants), so the top-level imports would form a cycle. Local imports inside the function body break the cycle.

- [ ] **Step 3: Verify existing demo invocation still works**

```powershell
.venv\Scripts\python.exe -c "from sim3d.world import build_world3d; w = build_world3d(42); print('demo obstacle count:', len(w.obstacles)); print('first:', w.obstacles[0])"
```

Expected: `demo obstacle count: 3` and first = `Rect3D(x=6.0, y=6.0, z=0.0, w=4.0, d=0.6, h=4.0)`.

- [ ] **Step 4: Verify the 2D and 3D smoke tests still pass**

```powershell
.venv\Scripts\python.exe -m tests.test_smoke
.venv\Scripts\python.exe -m tests.test_smoke_3d
.venv\Scripts\python.exe -m tests.test_themes_3d
```

Expected: `SMOKE OK`, `SMOKE_3D OK`, `THEMES_3D OK`.

- [ ] **Step 5: Verify non-demo theme via build_world3d**

```powershell
.venv\Scripts\python.exe -c "from sim3d.world import build_world3d; w = build_world3d(42, theme='forest', difficulty=5); print('forest d=5 obstacle count:', len(w.obstacles))"
```

Expected: an integer > 0 (around 14-16 depending on rejection cap).

- [ ] **Step 6: Verify the unknown-theme error**

```powershell
.venv\Scripts\python.exe -c "from sim3d.world import build_world3d; build_world3d(42, theme='swamp')"
```

Expected: `ValueError: unknown theme 'swamp'; choices: ['canyon', 'city', 'demo', 'forest', 'warehouse']`.

- [ ] **Step 7: Verify the out-of-range difficulty warning + clamp**

```powershell
.venv\Scripts\python.exe -c "from sim3d.world import build_world3d; w = build_world3d(42, theme='forest', difficulty=99); print('obstacles:', len(w.obstacles))"
```

Expected: a warning line on stderr starting with `[warn] difficulty 99 out of range [1, 10]; clamping`, followed by `obstacles:` and a number consistent with d=10.

- [ ] **Step 8: Commit**

```powershell
git add sim3d/world.py
git commit -m "feat(sim3d): build_world3d dispatches via theme registry with solvability retry"
```

---

### Task 9: harness/run3d.py CLI updates

**Files:**
- Modify: `harness/run3d.py` — add `--theme` and `--difficulty` argparse options; thread them through `_run_one`; add the announce line and the banner update.

**Interfaces:**
- Consumes: `THEMES` from `sim3d.themes`.
- Produces: CLI accepts `--theme {demo,forest,warehouse,canyon,city}` (default `demo`) and `--difficulty INT` (default 5). Each run prints `[world] theme=<t> difficulty=<d> seed=<s>  (use these to replay)` before the harness starts. Renderer banner now reads `<OUTCOME>  t=<t>s  seed=<s>  fw=<v>  theme=<t>  diff=<d>` (truncated to 60 chars if longer).

- [ ] **Step 1: Read the current `harness/run3d.py` to confirm the shape**

Use the Read tool. Confirm `SWEEP_SEEDS = [42, 7, 13, 99, 256]`, the existing argparse setup, and the existing `_run_one(version, seed, render, cam)` signature.

- [ ] **Step 2: Rewrite `harness/run3d.py`**

Replace its contents entirely with:

```python
import argparse

from harness.loop3d import load_firmware_class_3d, run_episode_3d
from harness.scoring import format_result, summarize_sweep
from sim3d.themes import THEMES
from sim3d.world import build_world3d

SWEEP_SEEDS = [42, 7, 13, 99, 256]
BANNER_MAX = 60


def _truncate(s: str, cap: int = BANNER_MAX) -> str:
    return s if len(s) <= cap else s[: cap - 1] + "…"


def _announce(theme: str, difficulty: int, seed: int) -> None:
    print(f"[world] theme={theme} difficulty={difficulty} seed={seed}  (use these to replay)")


def _make_render_cb(world, cam: str):
    from sim3d.renderer import Renderer3D
    r = Renderer3D(world)
    r._cam_mode = cam

    def cb(drone, packet, cmd, fw):
        r.draw(drone, packet, cmd, fw)

    return cb, r


def _run_one(version: str, seed: int, render: bool, cam: str, theme: str, difficulty: int):
    _announce(theme, difficulty, seed)
    world = build_world3d(seed, theme=theme, difficulty=difficulty)
    FirmwareCls = load_firmware_class_3d(version)
    fw = FirmwareCls()
    cb = None
    r = None
    if render:
        cb, r = _make_render_cb(world, cam)
    try:
        result = run_episode_3d(world, fw, seed=seed, render_cb=cb)
        if r is not None:
            banner = _truncate(
                f"{result.outcome.value}  t={result.t_end:.2f}s  seed={seed}  "
                f"fw={version}  theme={theme}  diff={difficulty}"
            )
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
    p.add_argument("--theme", choices=THEMES, default="demo")
    p.add_argument(
        "--difficulty",
        type=int,
        default=5,
        help="1..10, clamped silently. Ignored by --theme demo.",
    )
    args = p.parse_args()

    if args.sweep:
        results = [
            _run_one(
                args.firmware,
                s,
                render=False,
                cam=args.cam,
                theme=args.theme,
                difficulty=args.difficulty,
            )
            for s in SWEEP_SEEDS
        ]
        print(summarize_sweep(results))
    else:
        _run_one(
            args.firmware,
            args.seed,
            render=args.render,
            cam=args.cam,
            theme=args.theme,
            difficulty=args.difficulty,
        )


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: Verify the existing demo invocation still produces the expected v1_3d crash on seed 42**

```powershell
.venv\Scripts\python.exe -m harness.run3d --firmware v1_3d --seed 42
```

Expected: a `[world] theme=demo difficulty=5 seed=42` line, then a `format_result` line showing `CRASH` (this should match the previous t≈5.80s behaviour because `theme=demo` returns the byte-identical hand-crafted layout).

- [ ] **Step 4: Verify the existing v2_3d demo still succeeds on seed 42**

```powershell
.venv\Scripts\python.exe -m harness.run3d --firmware v2_3d --seed 42
```

Expected: announce line, then `SUCCESS @ t≈13.88s` (matches the prior result).

- [ ] **Step 5: Verify the new --theme + --difficulty options end-to-end**

```powershell
.venv\Scripts\python.exe -m harness.run3d --firmware v2_3d --theme forest --difficulty 5 --seed 42
```

Expected: `[world] theme=forest difficulty=5 seed=42  (use these to replay)`, then a `format_result` line. The outcome may be SUCCESS, CRASH, or TIMEOUT — we don't pin it here. Step 6's sweep is the calibration.

- [ ] **Step 6: Sweep v2_3d on each new theme at d=5 (calibration; not pinned to a number)**

```powershell
.venv\Scripts\python.exe -m harness.run3d --firmware v2_3d --theme forest    --difficulty 5 --sweep
.venv\Scripts\python.exe -m harness.run3d --firmware v2_3d --theme warehouse --difficulty 5 --sweep
.venv\Scripts\python.exe -m harness.run3d --firmware v2_3d --theme canyon    --difficulty 5 --sweep
.venv\Scripts\python.exe -m harness.run3d --firmware v2_3d --theme city      --difficulty 5 --sweep
```

Expected: each sweep prints 5 announce + result lines + `--- N/5 success ---`. Record each `N/5` in the task report. The spec's observation target is ≥ 3/5 at d=5; if any theme lands at 0/5 or 1/5, note it in the report — T10 may need a difficulty-curve adjustment, NOT a v2_3d firmware change.

- [ ] **Step 7: Commit**

```powershell
git add harness/run3d.py
git commit -m "feat(harness): run3d gains --theme and --difficulty; replay-friendly announce line"
```

---

### Task 10: full regression, calibration tuning if needed, README update

**Files:**
- Possibly modify: one or two of `sim3d/themes/forest.py`, `sim3d/themes/warehouse.py`, `sim3d/themes/canyon.py`, `sim3d/themes/city.py` — ONLY if T9 Step 6 sweeps show v2_3d struggling badly (0/5 or 1/5 at d=5). Adjust the `_scale` curves to make d=5 land at roughly v2_3d=3-5/5. DO NOT modify any firmware file.
- Modify: `README.md` to document the new flags and themes.

**Interfaces:**
- Consumes: T9's sweep results from the task report.
- Produces: 6 final regression results (2D smoke OK, 3D smoke OK, themes smoke OK, 2D sweeps unchanged, demo-theme 3D sweeps unchanged, new-theme 3D sweeps documented). Updated README.

- [ ] **Step 1: Read T9's task report**

Look at `C:\Users\AiPC\AIcompetition\docs\superpowers\reports\task-9-env-report.md` for the four `--sweep` results recorded in Step 6. Triage:
- If all four new themes have v2_3d sweep ≥ 3/5: skip Steps 2-3 (no calibration tuning needed).
- If any theme is at 0/5 or 1/5: identify which theme and apply ONE tuning adjustment (Step 2 below) for that theme. Do not chase the targets — one pass.

- [ ] **Step 2: Tuning policy if needed**

For the theme that's struggling:
- **forest**: reduce the d=10 max pillar count from 25 to 20 (`count = round(5 + (20 - 5) * (d-1)/9)`). Re-run its sweep at d=5.
- **warehouse**: reduce d=10 max count from 18 to 14 (`count = round(6 + 8 * (d-1)/9)`). Re-run its sweep at d=5.
- **canyon**: increase d=10 min gap from 0.8 to 1.2 (`gap = 4.0 + (1.2 - 4.0) * (d-1)/9`). Re-run its sweep at d=5.
- **city**: reduce bridge fraction from 0.30 to 0.20 (cleaner sightlines + fewer hard-to-spot floor-rep traps). Re-run its sweep at d=5.

Apply ONE of these (whichever theme is struggling). If multiple themes are struggling badly, apply each respective fix but limit to one round of adjustments. Document the change in the task report. Do not retry indefinitely — one fix per theme, then accept whatever number comes back.

- [ ] **Step 3: If you adjusted, commit the calibration tweak**

```powershell
git add sim3d/themes/<theme>.py
git commit -m "tune(themes): calibrate <theme> _scale curve for v2_3d ~3-5/5 at d=5"
```

- [ ] **Step 4: Update `README.md`**

Replace the README via Python (no BOM, no BEL):

```python
import pathlib
content = '''# Drone Firmware Trainer

2D and 3D obstacle-avoidance training harness for autonomous drone firmware,
with five 3D world themes (`demo`, `forest`, `warehouse`, `canyon`, `city`)
and a single `--difficulty 1..10` knob.

The firmware module is a Python class with a fixed contract
(`SensorPacket` in, `MotorCommand` out for 2D; `SensorPacket3D` /
`MotorCommand3D` for 3D). The harness enforces microcontroller-like
constraints at the wrapper: 50 Hz fixed tick, noisy quantized sensors,
crash isolation, no global state, tick-time budget.

## Install

    python -m venv .venv
    .venv\\\\Scripts\\\\activate
    pip install -r requirements.txt

## 2D Demo

    python -m harness.run --firmware v1 --seed 42 --render   # naive controller, fails
    python -m harness.run --firmware v2 --seed 42 --render   # patched controller, succeeds

    python -m harness.run --firmware v1 --sweep   # 1/5
    python -m harness.run --firmware v2 --sweep   # 5/5

## 3D Demo (default theme = demo)

    python -m harness.run3d --firmware v1_3d --seed 42 --render
    python -m harness.run3d --firmware v2_3d --seed 42 --render

While rendering:

- Press **C** to toggle CHASE camera (default) and COCKPIT camera.
- Press **T** to toggle NOISY (firmware-view) and TRUE (ground-truth) HUD.
- Close the window or press any key on the final banner to exit.

    python -m harness.run3d --firmware v1_3d --sweep   # <= 1/5
    python -m harness.run3d --firmware v2_3d --sweep   # 5/5

## 3D Themed Worlds

Themes: `demo` (hand-crafted three-set-piece), `forest` (tall thin pillars),
`warehouse` (floor-standing crates in aisles), `canyon` (long horizontal
slabs with alternating gaps), `city` (tall buildings + low ceiling-hung
bridges). Difficulty is 1..10 (silently clamped).

    python -m harness.run3d --firmware v2_3d --theme forest    --difficulty 7 --seed 42 --render
    python -m harness.run3d --firmware v2_3d --theme warehouse --difficulty 5 --seed 11 --render
    python -m harness.run3d --firmware v2_3d --theme canyon    --difficulty 4 --seed 99 --render
    python -m harness.run3d --firmware v2_3d --theme city      --difficulty 6 --seed 33 --render

Every run prints `[world] theme=<t> difficulty=<d> seed=<s>  (use these to replay)`
before the harness starts. Pass the same triplet to reproduce any interesting
world. Worlds that fail a coarse-grid solvability check are auto-regenerated
with `seed+1` up to 10 times; if still unsolvable, the harness emits a
RuntimeWarning and runs the last attempt anyway.

    python -m harness.run3d --firmware v2_3d --theme forest --difficulty 5 --sweep
    python -m harness.run3d --firmware v2_3d --theme city   --difficulty 7 --sweep

## Architecture

- `firmware/contract.py`, `firmware/contract3d.py` — frozen contract
  dataclasses. Locked first; never edited after.
- `firmware/firmware_v1.py`, `firmware_v2.py` — 2D controllers under test.
- `firmware/firmware_v1_3d.py`, `firmware_v2_3d.py` — 3D controllers under test.
- `sim/` (pygame) — 2D world, physics, sensors, renderer.
- `sim3d/` (raylib-py) — 3D world, physics, sensors, renderer with HUD.
- `sim3d/themes/` — one generator per theme. Add a theme by writing
  `generate(seed, difficulty) -> list[Rect3D]` and registering it.
- `sim3d/solver.py` — coarse 3D A* solvability check (0.5 m grid).
- `harness/loop.py` + `run.py` wire the 2D sim to firmware;
  `harness/loop3d.py` + `run3d.py` do the 3D version.
'''
pathlib.Path('README.md').write_text(content, encoding='utf-8')
print('README written, first 5 bytes:', pathlib.Path('README.md').read_bytes()[:5])
```

Notice the **quadruple-backslash** `\\\\Scripts\\\\activate` in the Python source: inside the triple-quoted string, `\\\\` collapses to `\\`, and that `\\` on disk reads as a single `\`. The result is `.venv\Scripts\activate` on disk. This avoids both BOM (we're using Python's `write_text`) and BEL (the literal `\a` never appears in the Python source). Verify after writing:

```powershell
.venv\Scripts\python.exe -c "import pathlib; data = pathlib.Path('README.md').read_bytes(); assert b'\x07' not in data, 'BEL byte present'; assert b'Scripts\\activate' in data, 'Scripts\\activate not literal'; print('README byte check OK; first 5 bytes:', data[:5])"
```

Expected: `README byte check OK; first 5 bytes: b'# Dro'`.

- [ ] **Step 5: Full regression smoke**

```powershell
.venv\Scripts\python.exe -m tests.test_smoke
.venv\Scripts\python.exe -m tests.test_smoke_3d
.venv\Scripts\python.exe -m tests.test_themes_3d
.venv\Scripts\python.exe -m harness.run --firmware v1 --sweep
.venv\Scripts\python.exe -m harness.run --firmware v2 --sweep
.venv\Scripts\python.exe -m harness.run3d --firmware v1_3d --sweep
.venv\Scripts\python.exe -m harness.run3d --firmware v2_3d --sweep
```

Expected: `SMOKE OK`, `SMOKE_3D OK`, `THEMES_3D OK`, 2D v1 ≤ 1/5, 2D v2 5/5, 3D v1_3d ≤ 1/5, 3D v2_3d 5/5 (the demo-theme 3D sweeps are byte-identical to before).

- [ ] **Step 6: Commit**

```powershell
git add README.md
git commit -m "docs: README covers env generator (themes + difficulty + solvability retry)"
```

---

## Time / risk shape

- **T1 (solver)** is the only piece with non-trivial algorithmic content. A* on a coarse grid is well-trodden ground; if the demo-world solvability check returns False, the bug is in the cell-blocked predicate or the neighbor generation, both of which are <10 lines.
- **T3-T6 (themes)** are each ~80-100 lines of pure-function generation. The risk is parameter calibration: at d=5 the v2_3d sweep should land near 3-5/5 on each theme. T10's one-pass tuning policy handles outliers.
- **T8 (world.py refactor)** is small but touchy because it changes a public function signature. The `theme="demo"` default and the regression checks in Steps 4-7 catch any compatibility break.
- **T9 (CLI)** is mostly arg plumbing. The announce line + banner update are cosmetic and verified by inspection during T9 + the final regression in T10.
- **T10 (regression + README)** brings the BOM/BEL discipline forward from the 2D + 3D ship cycles. The Python `write_text` with quadruple-backslashes for the Windows path is the same trick used in the 3D project's README fix.

## Self-Review Notes

- **Spec coverage:**
  - §1 demo win condition → T9 + T10 (announce line, banner, sweep verification).
  - §2 non-goals are out of scope by omission.
  - §3 architecture → T1, T2-T6, T8, T9 (no changes to the listed untouched files).
  - §4 theme registry interface → T2 (skeleton with demo), T3-T6 (registrations).
  - §5 theme catalog → T2 (demo verbatim), T3 (forest), T4 (warehouse), T5 (canyon), T6 (city). All `_scale` formulas land verbatim.
  - §6 difficulty dial semantics (default=5, clamped to [1,10]) → T8 (clamping with warning).
  - §7 solver (GRID_RES=0.5, 6-connected, Manhattan heuristic) → T1.
  - §8 world builder (retry up to 10, RuntimeWarning) → T8.
  - §9 CLI + announce + banner → T9.
  - §10 tests → T7 + T10 regression.
  - §11 reproducibility contract → T9 announce line.
  - §12 tech stack — no new deps → confirmed in T1 imports (heapq, math), T2-T6 imports (random + stdlib only).
  - §13 demo runbook → T10 README update.
  - §14 risk register: solver false-negative, slow solver, unsolvable extremes, default theme drift, banner overflow — covered by T1 timing check, T8 retry-with-warning, T9 banner truncation, T10 regression check.
- **Placeholder scan:** no TBDs; every code block is complete; every command has expected output.
- **Type consistency:** `Rect3D`, `World3D`, `THEME_REGISTRY`, `THEMES`, `has_path`, `build_world3d` signature, theme `generate(seed, difficulty)` signature all match across T1-T10. `_scale` returns documented per theme (int for forest/city, tuple for warehouse/canyon).
