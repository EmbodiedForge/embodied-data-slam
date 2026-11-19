@echo off
setlocal
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -Command ^
  "$ErrorActionPreference='Stop';" ^
  "docker info | Out-Null;" ^
  "$sums=@{}; Get-Content SHA256SUMS -Encoding UTF8 | ForEach-Object { if ($_ -match '^([0-9a-fA-F]{64})\s+\*?(.+)$') { $sums[$Matches[2].Trim()]=$Matches[1].ToLowerInvariant() } };" ^
  "$archives = Get-ChildItem -Path images -Filter '*.tar.gz' -File -ErrorAction SilentlyContinue;" ^
  "if (-not $archives) { throw '未找到 images\*.tar.gz 离线镜像归档' };" ^
  "foreach ($a in $archives) {" ^
  "  $rel = 'images/' + $a.Name;" ^
  "  if ($sums.ContainsKey($rel)) {" ^
  "    $h = (Get-FileHash -Path $a.FullName -Algorithm SHA256).Hash.ToLowerInvariant();" ^
  "    if ($h -ne $sums[$rel]) { throw ('校验和不一致: ' + $rel) }" ^
  "  };" ^
  "  Write-Host ('Loading ' + $rel);" ^
  "  docker image load --input $a.FullName;" ^
  "  if ($LASTEXITCODE -ne 0) { throw ('docker image load failed: ' + $rel) }" ^
  "};" ^
  "Write-Host 'Images loaded. Run launch_windows.cmd'"
if errorlevel 1 pause
exit /b %errorlevel%
