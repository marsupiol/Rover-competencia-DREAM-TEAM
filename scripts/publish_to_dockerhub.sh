#!/usr/bin/env bash
set -euo pipefail

# Usage: ./scripts/publish_to_dockerhub.sh yourusername/mini_plus_ros2:tag
IMAGE=${1:-}
if [ -z "$IMAGE" ]; then
  echo "Usage: $0 <image:tag>"
  exit 2
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." >/dev/null 2>&1 && pwd)"
cd "$ROOT_DIR"

# Build
docker build -t "$IMAGE" -f Dockerfile .

# Push
docker push "$IMAGE"

echo "Published $IMAGE"
