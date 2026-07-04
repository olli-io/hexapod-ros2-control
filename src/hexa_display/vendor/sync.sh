#!/usr/bin/env bash
# Re-copy the vendored sources from a full checkout of the firmware repo.
# Run from anywhere; paths are resolved relative to this script.
#
# Requires the u8g2 submodule to be initialized:
#   git submodule update --init components/u8g2/u8g2
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/../../.." && pwd)"        # ros2/face/vendor -> repo root
comp="$repo/components"

if [[ ! -d "$comp/u8g2/u8g2/csrc" ]]; then
  echo "error: $comp/u8g2/u8g2/csrc not found — init the u8g2 submodule first" >&2
  exit 1
fi

# --- core/ : platform-free firmware pieces --------------------------------
mkdir -p "$here/core"
cp "$comp/renderer/include/EyeAnim.h"              "$here/core/"
cp "$comp/renderer/include/EyeRaster.h"            "$here/core/"
cp "$comp/renderer/EyeAnim.cpp"                    "$here/core/"
cp "$comp/renderer/EyeRaster.cpp"                  "$here/core/"
cp "$comp/expression/include/Expression.h"         "$here/core/"
cp "$comp/expression/include/ExpressionController.h" "$here/core/"
cp "$comp/expression/include/IRenderer.h"          "$here/core/"
cp "$comp/expression/ExpressionController.cpp"     "$here/core/"
cp "$comp/config/include/Config.h"                 "$here/core/"

# --- u8g2/ : C core minus the pure font-data blobs ------------------------
mkdir -p "$here/u8g2"
rm -f "$here"/u8g2/*.c "$here"/u8g2/*.h
for f in "$comp"/u8g2/u8g2/csrc/*.c "$comp"/u8g2/u8g2/csrc/*.h; do
  case "$(basename "$f")" in
    u8g2_fonts.c|u8x8_fonts.c) continue ;;   # font data — face draws no text
  esac
  cp "$f" "$here/u8g2/"
done

echo "synced."
echo "  repo commit: $(git -C "$repo" rev-parse --short HEAD)"
echo "  u8g2 commit: $(git -C "$comp/u8g2/u8g2" rev-parse --short HEAD 2>/dev/null || echo '?')"
echo "Update the provenance commits in vendor/README.md if they changed."
