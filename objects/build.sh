#!/usr/bin/env bash
# Regenerate hospital-sim-env/objects (meshes, MJCF, USD, catalog) and check it. No ROS / Isaac needed.
# Runs from the hospital-sim root (two levels up), whose .venv holds mujoco for check_objects.py.
set -euo pipefail
cd "$(dirname "$0")/../.."
uv run --no-project --python 3.11 --with usd-core --with trimesh --with coacd --with numpy --with scipy \
  --with pyyaml --with pillow --with fast-simplification --with networkx python hospital-sim-env/objects/build_objects.py "$@"
.venv/bin/python hospital-sim-env/objects/check_objects.py
