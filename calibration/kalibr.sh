#!/usr/bin/env bash
set -euo pipefail
bundle_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [ "$#" -lt 2 ]; then
  echo 'Usage: bash calibration/kalibr.sh WORK_DIRECTORY {bag|stereo|imu|convert|shell} [arguments]'
  echo 'WORK_DIRECTORY is mounted read/write at /work. This toolkit is mounted read-only at /toolkit.'
  exit 2
fi
work_dir="$(realpath -- "$1")"
test -d "$work_dir"
shift
docker image inspect kalibr:portable-20260921 >/dev/null
exec docker run --rm --init --network none --user "$(id -u):$(id -g)" \
  -e HOME=/tmp -e MPLBACKEND=Agg -e QT_QPA_PLATFORM=offscreen \
  -v "$work_dir:/work" -v "$bundle_dir:/toolkit:ro" --workdir /work \
  --entrypoint bash kalibr:portable-20260921 -c '
    set -e
    source /opt/ros/noetic/setup.bash
    source /kalibr_workspace/devel/setup.bash
    command_name="$1"; shift
    case "$command_name" in
      bag) exec python3 /toolkit/calibration/tools/kalibr_to_bag.py "$@" ;;
      stereo) exec rosrun kalibr kalibr_calibrate_cameras --dont-show-report "$@" ;;
      imu) exec rosrun kalibr kalibr_calibrate_imu_camera --dont-show-report "$@" ;;
      convert) exec python3 /toolkit/calibration/tools/kalibr_to_orb.py "$@" ;;
      shell) exec bash -c "$@" ;;
      *) echo "Unknown command: $command_name" >&2; exit 2 ;;
    esac
  ' bash "$@"
