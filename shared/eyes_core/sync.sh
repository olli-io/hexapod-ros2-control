#!/usr/bin/env bash
# Re-copy the vendored eye-animation-cpp sources and print the commit they came
# from, so README.md's provenance line can be updated.
#   shared/eyes_core/sync.sh [path to an eye-animation-cpp checkout]
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
src="${1:-$here/../../../eye-animation-cpp}"
src="$(cd "$src" && pwd)"
for f in image.hpp image.cpp eyes.hpp eyes.cpp sh1122.hpp sh1122.cpp; do
    cp "$src/src/$f" "$here/$f"
done
echo "copied from $src at $(git -C "$src" rev-parse --short HEAD)"
git -C "$src" status --short | grep -q . && echo "warning: that checkout has uncommitted changes"
exit 0
