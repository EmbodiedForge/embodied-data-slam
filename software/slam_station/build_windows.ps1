$ErrorActionPreference = 'Stop'
if ($env:OS -ne 'Windows_NT') { throw 'Build this executable on Windows.' }
Set-Location $PSScriptRoot
py -3 -m venv .venv-build
if ($LASTEXITCODE -ne 0) { throw 'Python venv creation failed' }
$buildPython = Join-Path $PSScriptRoot '.venv-build\Scripts\python.exe'
& $buildPython -m pip install 'pyinstaller>=6.20,<7'
if ($LASTEXITCODE -ne 0) { throw 'PyInstaller install failed' }
& $buildPython -m PyInstaller --noconfirm --clean --windowed --onedir --name SLAMStation --add-data 'devices;devices' --add-data 'DEVICE_API.md;.' --add-data '../umi_slam/profiles;profiles' app.py
if ($LASTEXITCODE -ne 0) { throw 'Executable build failed' }
Write-Host 'Built dist\SLAMStation\SLAMStation.exe. Distribute the entire SLAMStation directory.'
Write-Host 'Python is not needed on the target PC; Docker Desktop and the backend image are still required.'
