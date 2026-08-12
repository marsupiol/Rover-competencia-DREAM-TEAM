#!/usr/bin/env bash
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." >/dev/null 2>&1 && pwd)"
source "$ROOT_DIR/.venv/bin/activate"
cd "$ROOT_DIR"

export PYTHONPATH="$ROOT_DIR/src/sdk_server:$PYTHONPATH"
exec python3 "$ROOT_DIR/run_sdk.py"
