# Offline Robot Trajectory Simulator

This tool operates entirely offline on `quadrant_output/all_robots_merged.json`. It can (1) render a scale-accurate animation and (2) run a collision scan that treats each toio as a rotated rectangle derived from the real dimensions in `docs/common`.

## Requirements

- Python 3.10+
- [`uv`](https://github.com/astral-sh/uv) for reproducible environments
- `ffmpeg` runtime (Matplotlib bundles a build on macOS)

## Setup (once per machine)

```bash
cd /Users/ksk432/Movie
uv venv
source .venv/bin/activate
uv sync            # runtime deps (matplotlib, etc.)
uv sync --group dev  # optional: install test tooling
```

You can prefix commands with `uv run` to avoid manually activating the virtualenv.

## Rendering (mode=render)

```bash
uv run python -m simulator.run \
  --mode render \
  --input quadrant_output/all_robots_merged.json \
  --output out/robot_paths.mp4 \
  --fps 30 \
  --duration 150 \
  --robot-length-mm 72 \
  --robot-width-mm 32 \
  --safety-margin-mm 20
```

Key options:

- `--robot-length-mm` / `--robot-width-mm`: physical footprint (defaults 72×32 mm).
- `--safety-margin-mm`: extra buffer applied to both length/width (default 20 mm).
- `--field-bounds`: override the drawing extents `(min_x max_x min_y max_y)` if needed.
- `--trail`: seconds of trailing path to draw (default 5s).

Robots are rendered as rotated rectangles plus heading indicators. Trails and grid remain in millimetre space so what you see matches the doc specs.

## Collision Scan (mode=collision)

```bash
uv run python -m simulator.run \
  --mode collision \
  --input quadrant_output/all_robots_merged.json \
  --collision-step 0.05 \
  --robot-length-mm 72 \
  --robot-width-mm 32 \
  --safety-margin-mm 20
```

This samples every robot at the requested interval (default 50 ms), builds oriented rectangles, and checks every pair using the Separating Axis Theorem. The CLI prints a summary and reports the earliest timestamp for each colliding pair. Use `--mode both` to render and scan in one pass.

## Testing

Interpolation and SAT logic are covered by `tests/test_trajectory.py` and `tests/test_collision.py`. Run the suite with:

```bash
uv run python -m unittest discover -s tests
```

