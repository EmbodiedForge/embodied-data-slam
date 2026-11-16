#!/usr/bin/env bash
set -euo pipefail
bundle_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd -- "$bundle_dir"
command -v docker >/dev/null || { echo 'Docker Engine must be installed first.' >&2; exit 1; }
docker info >/dev/null
echo 'Verifying bundle checksums…'
sha256sum --check SHA256SUMS
for archive in images/*.tar.gz; do
  echo "Loading $archive"
  docker image load --input "$archive"
done
python3 - <<'PY'
import json, subprocess
from pathlib import Path
manifest=json.loads(Path('MANIFEST.json').read_text())
for item in manifest['images']:
    actual=subprocess.check_output(['docker','image','inspect',item['tag'],'--format','{{.Id}}'],text=True).strip()
    if actual != item['id']: raise SystemExit('Image ID mismatch: '+item['tag'])
print('Images loaded and verified. Run: bash launch_software.sh')
PY
