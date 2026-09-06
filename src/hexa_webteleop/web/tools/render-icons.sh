#!/usr/bin/env bash
# Rasterize icon.svg into the three PNGs the webapp ships.
#
# A script rather than a README line because the sizes are load-bearing: 192 and
# 512 are what a manifest needs, and 180 is what iOS reads for the home-screen
# icon. Run it after editing icon.svg, then `npm run build` and commit both
# public/ and the rebuilt dist/.
set -euo pipefail
cd "$(dirname "$0")/.."
command -v rsvg-convert >/dev/null || {
    echo "rsvg-convert not found — install librsvg" >&2
    exit 1
}
render() { rsvg-convert -w "$1" -h "$1" icon.svg -o "public/$2"; echo "  public/$2 (${1}px)"; }
echo ">> Rendering icon.svg"
render 192 icon-192.png
render 512 icon-512.png
# iOS reads this one and applies its own squircle mask, so it ships square and
# unrounded. 180 is the largest size any current iPhone or iPad asks for.
render 180 apple-touch-icon.png
