#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

if [ $# -lt 1 ]; then
    echo "Usage: $0 <part-directory> [<part-directory> ...]"
    echo "Example: $0 sht40 ht7833"
    exit 1
fi

CHECKER_REPO="https://github.com/fritzing/fritzing-parts.git"
CHECKER_DIR="$SCRIPT_DIR/.fritzing-parts"

# --- Ensure checker is available ---
if [ ! -d "$CHECKER_DIR" ]; then
    echo "Cloning fritzing-parts for fzp_checker.py..."
    git clone --depth 1 "$CHECKER_REPO" "$CHECKER_DIR"
else
    echo "Updating fritzing-parts..."
    git -C "$CHECKER_DIR" pull --ff-only 2>/dev/null || true
fi
CHECKER="$CHECKER_DIR/fzp_checker.py"

for PART_DIR in "$@"; do
    PART_PATH="$SCRIPT_DIR/$PART_DIR"

    if [ ! -d "$PART_PATH" ]; then
        echo "Error: directory '$PART_DIR' not found"
        exit 1
    fi

    cd "$PART_PATH"

    # Find the FZP file
    FZP_FILE=$(ls part.*.fzp 2>/dev/null | head -1)
    if [ -z "$FZP_FILE" ]; then
        echo "Error: no part.*.fzp file found in $PART_DIR"
        exit 1
    fi

    # Derive part name and output path
    PART_NAME=$(echo "$FZP_FILE" | sed 's/^part\.\(.*\)\.fzp$/\1/')
    OUTPUT="$SCRIPT_DIR/$PART_NAME.fzpz"

    echo "=== Building $PART_NAME from $PART_DIR ==="

    # --- Version bump ---
    CURRENT_VERSION=$(grep -oP '<version>\K[0-9]+' "$FZP_FILE")
    NEW_VERSION=$((CURRENT_VERSION + 1))
    sed -i "s|<version>${CURRENT_VERSION}</version>|<version>${NEW_VERSION}</version>|" "$FZP_FILE"

    # Bump moduleId (matches any trailing _N number)
    CURRENT_MODULE_ID=$(grep -oP 'moduleId="\K[^"]+' "$FZP_FILE")
    MODULE_PREFIX=$(echo "$CURRENT_MODULE_ID" | sed 's/_[0-9]*$//')
    sed -i "s|moduleId=\"${MODULE_PREFIX}_[0-9]*\"|moduleId=\"${MODULE_PREFIX}_${NEW_VERSION}\"|" "$FZP_FILE"
    echo "Version: $CURRENT_VERSION -> $NEW_VERSION (moduleId: ${MODULE_PREFIX}_${NEW_VERSION})"

    # --- Lint ---
    if [ -f "$CHECKER" ]; then
        echo "--- Linting ---"
        python3 "$CHECKER" "$PART_PATH"
    else
        echo "Warning: fzp_checker.py not found, skipping lint"
    fi

    # --- Build .fzpz ---
    echo "--- Building $OUTPUT ---"
    rm -f "$OUTPUT"
    zip -j "$OUTPUT" "$FZP_FILE" svg.*.svg

    echo "--- Contents ---"
    unzip -l "$OUTPUT"

    echo ""
    echo "Built $OUTPUT (version $NEW_VERSION)"
    echo ""
done
