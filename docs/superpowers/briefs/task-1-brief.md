# Task 1: Project Setup

This is the first task of the Drone Firmware Trainer plan. Create a minimal Python project skeleton, install dependencies into a venv, and make the initial git commit.

## Files

Create exactly these files at `C:\Users\AiPC\AIcompetition\`:

- `requirements.txt`
- `.gitignore`
- `README.md`
- `firmware/__init__.py` (empty)
- `sim/__init__.py` (empty)
- `harness/__init__.py` (empty)
- `tests/__init__.py` (empty)

## Exact file contents

### requirements.txt
```
pygame==2.5.2
numpy==1.26.4
```

### .gitignore
```
__pycache__/
*.pyc
.venv/
venv/
.vscode/
.idea/
*.log
telemetry_*.csv
.superpowers/
```

(Note: I added `.superpowers/` to .gitignore — that directory is used by the subagent-driven-development skill for short-lived briefs and review packages and must not be committed.)

### README.md
```markdown
# Drone Firmware Trainer

2D obstacle-avoidance training harness for autonomous drone firmware.

## Run

    python -m venv .venv
    .venv\Scripts\activate
    pip install -r requirements.txt
    python -m harness.run --firmware v1 --seed 42 --render

## Demo

    python -m harness.run --firmware v1 --sweep    # expect 1/5
    python -m harness.run --firmware v2 --sweep    # expect 5/5
```

### Four `__init__.py` files
All four are empty files (0 bytes).

## Steps (do in order)

1. Create the four package directories (`firmware/`, `sim/`, `harness/`, `tests/`) and the empty `__init__.py` inside each.
2. Write `requirements.txt`, `.gitignore`, `README.md` at the project root with the contents above.
3. Create a venv and install dependencies:
   ```powershell
   python -m venv .venv
   .venv\Scripts\activate
   pip install -r requirements.txt
   ```
4. Verify the install:
   ```powershell
   python -c "import pygame, numpy; print(pygame.__version__, numpy.__version__)"
   ```
   Expected output: `2.5.2 1.26.4`. If a different patch version of pygame ships for Windows wheels and 2.5.2 is unavailable, escalate as BLOCKED — do not silently change the pin.
5. Initialize git and make the first commit. The repo does NOT exist yet; run `git init` from the project root.
   ```powershell
   git init
   git add requirements.txt .gitignore README.md firmware/__init__.py sim/__init__.py harness/__init__.py tests/__init__.py docs/
   git commit -m "chore: project skeleton with pygame+numpy"
   ```
   Include the existing `docs/` tree (it contains the design spec and plan that were authored before this task began) in the first commit.

## Constraints

- Work directory: `C:\Users\AiPC\AIcompetition\`
- Windows / PowerShell environment.
- Python 3.11+.
- Use UTF-8 encoding without BOM for all text files (`Out-File -Encoding utf8` is OK on Windows PowerShell 5.1, but verify the file does not contain a leading BOM byte by re-reading the file).
- Do NOT create any files not listed above. No `pyproject.toml`, no `setup.py`, no extra scaffolding.
- Do NOT use `git add .` or `git add -A`. List files explicitly as shown.

## Acceptance

- All seven new files exist with the exact contents specified.
- `python -c "import pygame, numpy; print(pygame.__version__, numpy.__version__)"` prints `2.5.2 1.26.4`.
- `git log --oneline` shows exactly one commit with the message `chore: project skeleton with pygame+numpy`.
- `git status` is clean.

## TDD

Not applicable to this setup task. No tests to write here. The smoke test arrives in Task 11.
