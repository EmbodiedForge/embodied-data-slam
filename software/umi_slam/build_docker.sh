#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.."
image_tag="${IMAGE_TAG:-umi-slam:0.1}"
docker build -f software/umi_slam/Dockerfile -t "${image_tag}" \
  --build-arg BUILD_JOBS="${BUILD_JOBS:-2}" .
