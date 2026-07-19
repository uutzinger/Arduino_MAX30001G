#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if ! command -v doxygen >/dev/null 2>&1; then
  echo "doxygen is required to generate documentation." >&2
  echo "Install it with: sudo apt install doxygen" >&2
  exit 1
fi

HAVE_DOT="$(awk -F= '$1 ~ /^[[:space:]]*HAVE_DOT[[:space:]]*$/ {gsub(/[[:space:]]/, "", $2); print $2}' Doxyfile)"
if [[ "$HAVE_DOT" == "YES" ]] && ! command -v dot >/dev/null 2>&1; then
  echo "Graphviz dot is required because Doxyfile has HAVE_DOT = YES." >&2
  echo "Install it with: sudo apt install graphviz" >&2
  exit 1
fi

VERSION="$(awk -F= '$1=="version" {print $2}' library.properties)"
if [[ -z "${VERSION}" ]]; then
  echo "Could not read version from library.properties" >&2
  exit 1
fi

TMP_DOXYFILE="$(mktemp)"
awk -v version="$VERSION" '
  /^PROJECT_NUMBER[[:space:]]*=/ {
    print "PROJECT_NUMBER         = \"" version "\""
    next
  }
  { print }
' Doxyfile > "$TMP_DOXYFILE"
mv "$TMP_DOXYFILE" Doxyfile

# Remove stale generated documentation before generation. Doxygen does not clean
# files that are no longer emitted after configuration changes.
if [[ -d docs ]]; then
  find docs -mindepth 1 -maxdepth 1 ! -name '.gitkeep' -exec rm -rf {} +
fi

doxygen Doxyfile

rm -rf docs/assets
mkdir -p docs/assets
cp -f assets/Blockdiagram.png assets/ECG_InputMUX.png assets/BIOZ_InputMUX.png docs/assets/

python3 - <<'PY'
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

bad = []
for path in sorted(Path("docs").rglob("*.svg")):
    try:
        ET.parse(path)
    except Exception as exc:
        bad.append((path, exc))

if bad:
    for path, exc in bad:
        print(f"Malformed SVG: {path}: {exc}", file=sys.stderr)
    sys.exit(1)
PY

echo "Docs generated in docs/ for version $VERSION (with assets mirrored to docs/assets)."
