#!/bin/bash

# GitBook helper script for the documentation workspace

set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
DOCS_DIR="${SCRIPT_DIR}/docs"
OUTPUT_DIR="${DOCS_DIR}/site"

show_help() {
    cat <<'EOF'
Experimental Robotics ─ GitBook helper

Usage:
  ./docs.sh help           Show this message
  ./docs.sh preview        Start a local GitBook preview (requires Node.js + npx)
  ./docs.sh build          Export a static build to docs/site (requires Node.js + npx)
  ./docs.sh clean          Remove docs/site

Notes:
  • The content is edited directly in Markdown under docs/.
  • Navigation is controlled through docs/SUMMARY.md.
  • If you manage the space in app.gitbook.com, push changes here first to keep it in sync.
EOF
}

require_npx() {
    if ! command -v npx >/dev/null 2>&1; then
        echo "Error: npx (Node.js) is required for this command." >&2
        echo "Install Node.js LTS from https://nodejs.org/ and try again." >&2
        exit 1
    fi
}

preview_docs() {
    require_npx
    echo "Starting GitBook local preview on http://localhost:4000 ..."
    echo "Press Ctrl+C to stop."
    (cd "${DOCS_DIR}" && exec npx @gitbook/cli@latest dev)
}

build_docs() {
    require_npx
    mkdir -p "${OUTPUT_DIR}"
    echo "Building static documentation into ${OUTPUT_DIR} ..."
    (cd "${DOCS_DIR}" && npx @gitbook/cli@latest build --output "${OUTPUT_DIR}")
    echo "Build complete."
}

clean_docs() {
    if [ -d "${OUTPUT_DIR}" ]; then
        rm -rf "${OUTPUT_DIR}"
        echo "Removed ${OUTPUT_DIR}"
    else
        echo "Nothing to clean."
    fi
}

command="${1:-help}"

case "${command}" in
    help|--help|-h)
        show_help
        ;;
    preview)
        preview_docs
        ;;
    build)
        build_docs
        ;;
    clean)
        clean_docs
        ;;
    *)
        echo "Unknown command: ${command}" >&2
        echo "Run ./docs.sh help for usage." >&2
        exit 1
        ;;
esac
