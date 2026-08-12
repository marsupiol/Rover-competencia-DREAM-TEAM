#!/usr/bin/env bash
set -euo pipefail

# Usage: ./scripts/save_image.sh yourusername/mini_plus_ros2:tag output_file.tar.gz
IMAGE=${1:-}
OUT=${2:-}
if [ -z "$IMAGE" ] || [ -z "$OUT" ]; then
  echo "Usage: $0 <image:tag> <output.tar.gz>"
  exit 2
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." >/dev/null 2>&1 && pwd)"
cd "$ROOT_DIR"

# Ensure image exists locally
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "Image $IMAGE not found locally; building first"
  docker build -t "$IMAGE" -f Dockerfile .
fi

# Save and compress
docker save "$IMAGE" | gzip > "$OUT"

echo "Saved $IMAGE -> $OUT"
