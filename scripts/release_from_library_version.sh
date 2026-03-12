#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

TAG_PREFIX="v"
DO_SYNC_DOCS=1
DO_TAG=1
DO_PUSH=0
DO_RELEASE=0
ALLOW_DIRTY=0

usage() {
  cat <<'EOF'
Usage:
  scripts/release_from_library_version.sh [options]

Reads version from library.properties and uses it as the single source of truth.

Actions:
  1. Mirrors the version into Doxyfile PROJECT_NUMBER
  2. Regenerates docs via scripts/generate_docs.sh
  3. Creates git tag if it does not already exist
  4. Optionally pushes the tag
  5. Optionally creates a GitHub release with gh

Options:
  --prefix <value>   Tag prefix, default: v
  --no-docs          Skip Doxyfile sync and docs generation
  --no-tag           Skip tag creation/check
  --push             Push the tag to origin
  --release          Create GitHub release with gh (implies --push)
  --allow-dirty      Allow tagging from a dirty worktree
  --help             Show this help

Examples:
  scripts/release_from_library_version.sh
  scripts/release_from_library_version.sh --push
  scripts/release_from_library_version.sh --release
  scripts/release_from_library_version.sh --prefix ""
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --prefix)
      TAG_PREFIX="$2"
      shift 2
      ;;
    --no-docs)
      DO_SYNC_DOCS=0
      shift
      ;;
    --no-tag)
      DO_TAG=0
      shift
      ;;
    --push)
      DO_PUSH=1
      shift
      ;;
    --release)
      DO_RELEASE=1
      DO_PUSH=1
      shift
      ;;
    --allow-dirty)
      ALLOW_DIRTY=1
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

VERSION="$(awk -F= '$1=="version" {print $2}' library.properties)"
if [[ -z "${VERSION}" ]]; then
  echo "Could not read version from library.properties" >&2
  exit 1
fi

TAG="${TAG_PREFIX}${VERSION}"

if [[ "$DO_SYNC_DOCS" -eq 1 ]]; then
  bash scripts/generate_docs.sh
fi

if [[ "$DO_TAG" -eq 0 ]]; then
  echo "Version: $VERSION"
  exit 0
fi

if [[ "$ALLOW_DIRTY" -ne 1 ]] && [[ -n "$(git status --short)" ]]; then
  echo "Worktree is dirty. Commit version/docs changes before tagging, or rerun with --allow-dirty." >&2
  exit 1
fi

if git rev-parse -q --verify "refs/tags/$TAG" >/dev/null; then
  echo "Tag $TAG already exists."
else
  git tag -a "$TAG" -m "Release $TAG"
  echo "Created tag $TAG"
fi

if [[ "$DO_PUSH" -eq 1 ]]; then
  git push origin "$TAG"
fi

if [[ "$DO_RELEASE" -eq 1 ]]; then
  if ! command -v gh >/dev/null 2>&1; then
    echo "gh CLI is required for --release" >&2
    exit 1
  fi

  if gh release view "$TAG" >/dev/null 2>&1; then
    echo "GitHub release $TAG already exists."
  else
    gh release create "$TAG" --title "$TAG" --generate-notes
    echo "Created GitHub release $TAG"
  fi
fi

echo "Version: $VERSION"
echo "Tag: $TAG"
