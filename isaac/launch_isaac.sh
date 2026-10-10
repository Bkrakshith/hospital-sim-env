#!/usr/bin/env bash
# Launch Isaac Sim 6.0 for Hospital_G1 with the local asset root and the MCP extension (enabled after
# startup). Pass --open to load the hospital scene:  ./launch_isaac.sh [--open]
HERE="$(cd "$(dirname "$0")" && pwd)"
ISAAC_DIR="${ISAAC_DIR:-$HOME/isaacsim}"
MCP_EXT_DIR="${MCP_EXT_DIR:-$HOME/Downloads/isaac-sim-mcp}"
ARGS=(--ext-folder "$MCP_EXT_DIR" --exec "$HERE/scene/enable_mcp_late.py"
      --/persistent/isaac/asset_root/default="$HOME/isaacsim_assets/Assets/Isaac/6.0")
[ "$1" = "--open" ] && export HG1_OPEN_STAGE="$HERE/usd/hospital_g1.usd"
cd "$ISAAC_DIR" && exec env -u LD_LIBRARY_PATH -u CONDA_PREFIX -u CONDA_DEFAULT_ENV ./isaac-sim.sh "${ARGS[@]}"
