# Source this file:  source i4h_env.sh
# Points Isaac for Healthcare (i4h) tooling at the local asset copy next to the Isaac Sim 6.0 asset pack.

export ISAAC_ASSET_ROOT="$HOME/isaacsim_assets/Assets/Isaac/6.0"
export I4H_REPO="$HOME/isaacsim_assets/i4h-digital-twin"

# i4h_asset_helper resolves assets as $I4H_ASSET_DOWNLOAD_DIR/<catalog hash>/...
export I4H_ASSET_VERSION="0.7.0"
export I4H_ASSET_DOWNLOAD_DIR="$HOME/isaacsim_assets/Assets/Isaac/Healthcare/$I4H_ASSET_VERSION"
export I4H_ASSET_ROOT="$I4H_ASSET_DOWNLOAD_DIR/724f82e"
