#!/usr/bin/env bash
# Fetch Poly Haven's CC0 "Korean Fire Extinguisher 01"
# (https://polyhaven.com/a/korean_fire_extinguisher_01, by UM JOORIN) into
# ./fire_extinguisher/, then build what fire_extinguisher_inspect.usda references
# from it (make_fire_extinguisher.py: the model split into parts, a corroded copy
# of its paint, the spilled-powder decal). ~110 MB, gitignored. Re-running skips
# downloads already there and rebuilds the generated files.
#
#   ./isaacsim/assets/fetch_fire_extinguisher.sh
#
# Don't re-run while Isaac has the stage open: it rewrites parts.usdc, which
# Isaac holds memory-mapped, and Isaac crashes.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="$HERE/fire_extinguisher"
ISAAC_DIR=${ISAAC_DIR:-$HOME/isaacsim}
mkdir -p "$DEST/textures"
# The file list comes from Poly Haven's API, so it follows their asset as published.
curl -fsSL https://api.polyhaven.com/files/korean_fire_extinguisher_01 | python3 -c '
import json, sys
usd = json.load(sys.stdin)["usd"]["2k"]["usd"]
print(usd["url"], "korean_fire_extinguisher_01_2k.usdc")
for name, f in usd["include"].items():
    print(f["url"], name)
' | while read -r url name; do
    if [ -s "$DEST/$name" ]; then echo "  have  $name"; continue; fi
    echo "  fetch $name"
    curl -fsSL --retry 3 -o "$DEST/$name" "$url"
done

# Isaac Sim's bundled Python has pxr, numpy and PIL; its library paths carry a
# build hash, so find them.
USD_LIBS=$(ls -d "$ISAAC_DIR"/extscache/omni.usd.libs-*/ 2>/dev/null | head -1)
PIP_LIBS=$(ls -d "$ISAAC_DIR"/extscache/omni.kit.pip_archive-*/pip_prebundle 2>/dev/null | head -1)
[ -n "$USD_LIBS" ] && [ -n "$PIP_LIBS" ] || { echo "Isaac Sim's USD libraries not found under $ISAAC_DIR" >&2; exit 1; }
PYTHONPATH="$USD_LIBS:$PIP_LIBS" LD_LIBRARY_PATH="$USD_LIBS/bin:${LD_LIBRARY_PATH:-}" \
    "$ISAAC_DIR/kit/python/bin/python3" "$HERE/make_fire_extinguisher.py"
echo "done: $DEST"
