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

JOBS="${BUILD_JOBS:-$(sysctl -n hw.ncpu 2>/dev/null || echo 4)}"
BUILD_STATION="${EDS_BUILD_STATION:-ON}"
SKIP_BACKEND="${SKIP_BACKEND:-0}"

if [[ "$CLEAN" == "1" ]]; then
  echo "== Cleaning previous macOS build & dist artifacts =="
  rm -rf "$REPO_ROOT/build" "$REPO_ROOT/dist-macos" "$REPO_ROOT/software/umi_slam/.build-macos" "$REPO_ROOT/CMakeUserPresets.json"
  echo "Clean complete (Conan cache in ~/.conan2 is preserved)."
  if [[ "$CLEAN_ONLY" == "1" ]]; then
    exit 0
  fi
fi

conan profile detect --exist-ok
conan install . \
  --build=missing \
  -s build_type=Release \
  -s compiler.cppstd=17 \
  -o build_station="$( [[ "$BUILD_STATION" == "ON" ]] && echo True || echo False )" \
  -c tools.cmake.cmaketoolchain:generator=Ninja

TOOLCHAIN="$REPO_ROOT/build/Release/generators/conan_toolchain.cmake"
BUILD_DIR="$REPO_ROOT/build/Release"

cmake -S "$REPO_ROOT" -B "$BUILD_DIR" -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
  -DCMAKE_BUILD_TYPE=Release \
  -DEDS_BUILD_STATION="$BUILD_STATION"
cmake --build "$BUILD_DIR" -j "$JOBS"
ctest --test-dir "$BUILD_DIR" --output-on-failure

if [[ "$SKIP_BACKEND" != "1" ]]; then
  CMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" "$BUILD_DIR/umi_slam_pipeline" --build-backend \
    --source-root "$REPO_ROOT/source/stereo_imu_trajectory" \
    --runner-dir "$REPO_ROOT/software/umi_slam/backend" \
    --build-dir "$REPO_ROOT/software/umi_slam/.build-macos" \
    --jobs "$JOBS"
fi

DIST_DIR="$REPO_ROOT/dist-macos/SLAMStation"
rm -rf "$DIST_DIR"
mkdir -p "$DIST_DIR"
cp "$BUILD_DIR/umi_slam_pipeline" "$DIST_DIR/"
if [[ -f "$BUILD_DIR/slam_station" ]]; then
  cp "$BUILD_DIR/slam_station" "$DIST_DIR/"
fi
if [[ -d "$BUILD_DIR/generators/runtime_libs" ]]; then
  cp -P "$BUILD_DIR/generators/runtime_libs/"*.dylib* "$DIST_DIR/" 2>/dev/null || true
fi
cp -r "$REPO_ROOT/software/slam_station/devices" "$DIST_DIR/devices"
cp -r "$REPO_ROOT/software/umi_slam/profiles" "$DIST_DIR/profiles"
if [[ -f "$REPO_ROOT/software/slam_station/DEVICE_API.md" ]]; then
  cp "$REPO_ROOT/software/slam_station/DEVICE_API.md" "$DIST_DIR/"
fi

BACKEND_BUILD_DIR="$REPO_ROOT/software/umi_slam/.build-macos"
if [[ -f "$BACKEND_BUILD_DIR/backend.json" ]]; then
  mkdir -p "$DIST_DIR/backend"
  cp "$BACKEND_BUILD_DIR/runner/umi_slam_runner" "$DIST_DIR/backend/"
  cp "$BACKEND_BUILD_DIR/ORB_SLAM3/Vocabulary/ORBvoc.txt" "$DIST_DIR/backend/"
  if [[ -d "$BUILD_DIR/generators/runtime_libs" ]]; then
    cp -P "$BUILD_DIR/generators/runtime_libs/"*.dylib* "$DIST_DIR/backend/" 2>/dev/null || true
  fi
  cat > "$DIST_DIR/backend/backend.json" <<'EOF'
{
  "binary": "umi_slam_runner",
  "vocabulary": "ORBvoc.txt",
  "library_paths": ["."]
}
EOF
fi
echo "macOS build complete: $DIST_DIR"
