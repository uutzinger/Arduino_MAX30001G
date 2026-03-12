#!/usr/bin/env bash
set -euo pipefail

# Generate Doxygen HTML docs.
doxygen Doxyfile

# Keep README image paths (assets/...) working in generated HTML.
rm -rf docs/assets
mkdir -p docs/assets
cp -f assets/Blockdiagram.png assets/ECG_InputMUX.png assets/BIOZ_InputMUX.png docs/assets/

echo "Docs generated in docs/ (with assets mirrored to docs/assets)."
