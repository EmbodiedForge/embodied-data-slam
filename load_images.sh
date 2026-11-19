#!/usr/bin/env bash
set -euo pipefail
bundle_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd -- "$bundle_dir"
command -v docker >/dev/null || { echo 'Docker Engine must be installed first.' >&2; exit 1; }
docker info >/dev/null
echo 'Verifying bundle checksums…'
sha256sum --check --ignore-missing SHA256SUMS
for archive in images/*.tar.gz; do
  [ -f "$archive" ] || { echo "No image archives found under images/" >&2; exit 1; }
  echo "Loading $archive"
  docker image load --input "$archive"
done
echo 'Images loaded. Run: bash launch_station.sh'
