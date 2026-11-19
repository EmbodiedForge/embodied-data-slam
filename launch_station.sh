#!/usr/bin/env bash
set -euo pipefail
bundle_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
for candidate in \
    "$bundle_dir/dist-linux/SLAMStation/slam_station" \
    "$bundle_dir/dist/SLAMStation/slam_station" \
    "$bundle_dir/build-linux/slam_station" \
    "${HOME}/.cache/eds-cpp-build-linux/slam_station"; do
    if [ -x "$candidate" ]; then
        exec "$candidate" "$@"
    fi
done
echo "ERROR: slam_station executable not found. Run: bash build_linux.sh" >&2
exit 1
