#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_ROOT"

CLEAN=0
CLEAN_ONLY=0
for arg in "$@"; do
  case "$arg" in
    --clean) CLEAN=1 ;;
    --clean-only) CLEAN=1; CLEAN_ONLY=1 ;;
  esac
done

JOBS="${BUILD_JOBS:-$(nproc 2>/dev/null || echo 4)}"
BUILD_STATION="${EDS_BUILD_STATION:-ON}"
SKIP_BACKEND="${SKIP_BACKEND:-0}"

# Under WSL2 (/mnt/*), build on native Linux ext4 (~/.cache) to avoid DrvFs I/O slowdown
# and avoid colliding with Windows MSVC build/Release artifacts.
if [[ -n "${EDS_BUILD_DIR:-}" ]]; then
  WORK_ROOT="$EDS_BUILD_DIR"
elif [[ "$REPO_ROOT" == /mnt/* ]]; then
  WORK_ROOT="${HOME}/.cache/eds-cpp-linux-build"
else
  WORK_ROOT="$REPO_ROOT"
fi

BUILD_DIR="$WORK_ROOT/build/Release"
TOOLCHAIN="$BUILD_DIR/generators/conan_toolchain.cmake"
BACKEND_BUILD_DIR="${EDS_BACKEND_BUILD_DIR:-$WORK_ROOT/backend-build-linux}"
DIST_DIR="$REPO_ROOT/dist-linux/SLAMStation"

if [[ "$CLEAN" == "1" ]]; then
  echo "== Cleaning previous Linux build & dist artifacts =="
  if [[ "$WORK_ROOT" != "$REPO_ROOT" ]]; then
    rm -rf "$WORK_ROOT"
  else
    rm -rf "$REPO_ROOT/build" "$BACKEND_BUILD_DIR"
  fi
  rm -rf "$REPO_ROOT/dist-linux" "$REPO_ROOT/software/umi_slam/.build-linux" "$REPO_ROOT/CMakeUserPresets.json"
  echo "Clean complete (Conan cache in ~/.conan2 is preserved)."
  if [[ "$CLEAN_ONLY" == "1" ]]; then
    exit 0
  fi
fi

echo "== Conan 2 install (Linux Release, station=${BUILD_STATION}, output=${WORK_ROOT}) =="
conan profile detect --exist-ok
conan install "$REPO_ROOT" \
  -of "$WORK_ROOT" \
  --build=missing \
  -s build_type=Release \
  -s compiler.cppstd=17 \
  -o build_station="$( [[ "$BUILD_STATION" == "ON" ]] && echo True || echo False )" \
  -c tools.cmake.cmaketoolchain:generator=Ninja

echo "== Configuring & Building SLAM Station (Linux Native + Ninja) =="
cmake -S "$REPO_ROOT" -B "$BUILD_DIR" -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
  -DCMAKE_BUILD_TYPE=Release \
  -DEDS_BUILD_STATION="$BUILD_STATION"
cmake --build "$BUILD_DIR" -j "$JOBS"

echo "== Running CTest =="
ctest --test-dir "$BUILD_DIR" --output-on-failure

BACKEND_BUILD_DIR="${EDS_BACKEND_BUILD_DIR:-$WORK_ROOT/backend-build-linux}"
if [[ "$SKIP_BACKEND" != "1" ]]; then
  echo "== Building isolated ORB-SLAM3 native backend (Linux + Conan) =="
  CMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" "$BUILD_DIR/umi_slam_pipeline" --build-backend \
    --source-root "$REPO_ROOT/source/stereo_imu_trajectory" \
    --runner-dir "$REPO_ROOT/software/umi_slam/backend" \
    --build-dir "$BACKEND_BUILD_DIR" \
    --jobs "$JOBS"
fi

DIST_DIR="$REPO_ROOT/dist-linux/SLAMStation"
rm -rf "$DIST_DIR"
mkdir -p "$DIST_DIR"
cp "$BUILD_DIR/umi_slam_pipeline" "$DIST_DIR/"
if [[ -f "$BUILD_DIR/slam_station" ]]; then
  cp "$BUILD_DIR/slam_station" "$DIST_DIR/"
fi
cp -r "$REPO_ROOT/software/slam_station/devices" "$DIST_DIR/devices"
cp -r "$REPO_ROOT/software/umi_slam/profiles" "$DIST_DIR/profiles"
if [[ -f "$REPO_ROOT/software/slam_station/DEVICE_API.md" ]]; then
  cp "$REPO_ROOT/software/slam_station/DEVICE_API.md" "$DIST_DIR/"
fi

if [[ -d "$BUILD_DIR/generators/runtime_libs" ]]; then
  cp -P "$BUILD_DIR/generators/runtime_libs/"*.so* "$DIST_DIR/" 2>/dev/null || true
fi
if [[ -f "$BACKEND_BUILD_DIR/backend.json" ]]; then
  mkdir -p "$DIST_DIR/backend"
  cp "$BACKEND_BUILD_DIR/runner/umi_slam_runner" "$DIST_DIR/backend/"
  cp "$BACKEND_BUILD_DIR/ORB_SLAM3/Vocabulary/ORBvoc.txt" "$DIST_DIR/backend/"
  if [[ -d "$BUILD_DIR/generators/runtime_libs" ]]; then
    cp -P "$BUILD_DIR/generators/runtime_libs/"*.so* "$DIST_DIR/backend/" 2>/dev/null || true
  fi
  cat > "$DIST_DIR/backend/backend.json" <<'EOF'
{
  "binary": "umi_slam_runner",
  "vocabulary": "ORBvoc.txt",
  "library_paths": ["."]
}
EOF
fi

echo "Linux build complete: $DIST_DIR"
