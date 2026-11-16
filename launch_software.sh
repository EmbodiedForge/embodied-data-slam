#!/usr/bin/env bash
set -euo pipefail
bundle_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python3 -c 'import tkinter' 2>/dev/null || {
  echo 'Missing Python Tk. On Ubuntu 22.04: sudo apt install python3-tk' >&2
  echo 'Docker command-line processing is available without Tk.' >&2
  exit 1
}
exec python3 "$bundle_dir/software/umi_slam/app.py"
