#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

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

doxygen Doxyfile

rm -rf docs/assets
mkdir -p docs/assets
cp -f assets/Blockdiagram.png assets/ECG_InputMUX.png assets/BIOZ_InputMUX.png docs/assets/

echo "Docs generated in docs/ for version $VERSION (with assets mirrored to docs/assets)."
