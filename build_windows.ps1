param(
    [switch]$CheckOnly,
    [switch]$Clean,
    [switch]$CleanOnly,
    [switch]$SkipBackend,
    [switch]$SkipTests,
    [switch]$BuildInstaller,
    [int]$Jobs = 8
)

$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $RepoRoot

function Invoke-CleanBuildArtifacts {
    Write-Host "== Cleaning previous Windows build & dist artifacts ==" -ForegroundColor Yellow
    $targets = @(
        (Join-Path $RepoRoot "build"),
        (Join-Path $RepoRoot "build-windows"),
        (Join-Path $RepoRoot "dist"),
        (Join-Path $RepoRoot "software\umi_slam\.build-windows"),
        (Join-Path $RepoRoot "software\umi_slam\.build-windows-msvc"),
        (Join-Path $RepoRoot "CMakeUserPresets.json")
    )
    foreach ($t in $targets) {
        if (Test-Path $t) {
            Write-Host "  Removing $t"
            Remove-Item -Recurse -Force $t
        }
    }
    Write-Host "Clean complete (Conan cache in ~/.conan2 is preserved)." -ForegroundColor Green
}

if ($Clean -or $CleanOnly) {
    Invoke-CleanBuildArtifacts
    if ($CleanOnly) { exit 0 }
}

function Find-VsDevCmd {
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path $vswhere) {
        $inst = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
        if ($inst) {
            $bat = Join-Path $inst "VC\Auxiliary\Build\vcvars64.bat"
            if (Test-Path $bat) { return $bat }
        }
    }
    return $null
}

function Import-VcVars([string]$BatPath) {
    $cmd = "`"$BatPath`" && set"
    cmd /c $cmd | ForEach-Object {
        if ($_ -match "^([^=]+)=(.*)$") {
            [System.Environment]::SetEnvironmentVariable($matches[1], $matches[2], "Process")
        }
    }
}

Write-Host "== Checking Windows MSVC + Conan 2 toolchain ==" -ForegroundColor Cyan
foreach ($tool in @("conan", "cmake", "ninja")) {
    if (-not (Get-Command $tool -ErrorAction SilentlyContinue)) {
        throw "Required tool not found on PATH: $tool"
    }
}

if (-not (Get-Command cl.exe -ErrorAction SilentlyContinue)) {
    $vcvars = Find-VsDevCmd
    if (-not $vcvars) {
        throw "MSVC (cl.exe) not found. Please install Visual Studio 2022 Build Tools (Desktop development with C++)."
    }
    Write-Host "Loading MSVC environment from $vcvars"
    Import-VcVars $vcvars
}

conan --version
cmake --version
ninja --version
if ($CheckOnly) {
    Write-Host "Environment check passed." -ForegroundColor Green
    exit 0
}

Write-Host "== Installing C++ dependencies via Conan 2 (MSVC) ==" -ForegroundColor Cyan
conan profile detect --exist-ok
conan install . `
    --build=missing `
    -s build_type=Release `
    -s compiler=msvc `
    -s compiler.cppstd=17 `
    -c tools.cmake.cmaketoolchain:generator=Ninja

$Toolchain = Join-Path $RepoRoot "build\Release\generators\conan_toolchain.cmake"
$BuildDir = Join-Path $RepoRoot "build\Release"
if (-not (Test-Path $Toolchain)) {
    throw "Conan toolchain not found at $Toolchain"
}

Write-Host "== Configuring & Building SLAM Station (Dear ImGui + MSVC + Ninja) ==" -ForegroundColor Cyan
cmake -S $RepoRoot -B $BuildDir -G Ninja `
    -DCMAKE_TOOLCHAIN_FILE="$Toolchain" `
    -DCMAKE_BUILD_TYPE=Release `
    -DEDS_BUILD_STATION=ON
cmake --build $BuildDir -j $Jobs
if ($LASTEXITCODE -ne 0) { throw "cmake --build failed" }

$RuntimeLibsDir = Join-Path $BuildDir "generators\runtime_libs"
if (Test-Path $RuntimeLibsDir) {
    Get-ChildItem -Path $RuntimeLibsDir -Filter "*.dll" -ErrorAction SilentlyContinue | Copy-Item -Destination $BuildDir -Force
}

if (-not $SkipTests) {
    Write-Host "== Updating SHA256SUMS & Running CTest ==" -ForegroundColor Cyan
    & (Join-Path $BuildDir "umi_slam_pipeline.exe") --update-checksums
    if ($LASTEXITCODE -ne 0) { throw "update-checksums failed" }
    ctest --test-dir $BuildDir --output-on-failure
    if ($LASTEXITCODE -ne 0) { throw "ctest failed" }
}

$PipelineExe = Join-Path $BuildDir "umi_slam_pipeline.exe"
$StationExe = Join-Path $BuildDir "slam_station.exe"
$BackendDir = Join-Path $RepoRoot "software\umi_slam\.build-windows"

if (-not $SkipBackend) {
    Write-Host "== Building isolated ORB-SLAM3 native backend (MSVC + Conan) ==" -ForegroundColor Cyan
    $env:CMAKE_TOOLCHAIN_FILE = $Toolchain
    & $PipelineExe --build-backend `
        --source-root (Join-Path $RepoRoot "source\stereo_imu_trajectory") `
        --runner-dir (Join-Path $RepoRoot "software\umi_slam\backend") `
        --build-dir $BackendDir `
        --jobs $Jobs
}

Write-Host "== Staging standalone dist\SLAMStation ==" -ForegroundColor Cyan
$DistDir = Join-Path $RepoRoot "dist\SLAMStation"
if (Test-Path $DistDir) { Remove-Item -Recurse -Force $DistDir }
New-Item -ItemType Directory -Force -Path $DistDir | Out-Null

Copy-Item $StationExe $DistDir
Copy-Item $PipelineExe $DistDir
Get-ChildItem -Path $BuildDir -Filter "*.dll" -ErrorAction SilentlyContinue | Copy-Item -Destination $DistDir
Copy-Item -Recurse (Join-Path $RepoRoot "software\slam_station\devices") (Join-Path $DistDir "devices")
Copy-Item -Recurse (Join-Path $RepoRoot "software\umi_slam\profiles") (Join-Path $DistDir "profiles")
if (Test-Path (Join-Path $RepoRoot "software\slam_station\DEVICE_API.md")) {
    Copy-Item (Join-Path $RepoRoot "software\slam_station\DEVICE_API.md") $DistDir
}

if (Test-Path (Join-Path $BackendDir "backend.json")) {
    $DistBackend = Join-Path $DistDir "backend"
    New-Item -ItemType Directory -Force -Path $DistBackend | Out-Null
    Copy-Item (Join-Path $BackendDir "runner\umi_slam_runner.exe") $DistBackend
    Copy-Item (Join-Path $BackendDir "ORB_SLAM3\Vocabulary\ORBvoc.txt") $DistBackend
    Get-ChildItem -Path (Join-Path $BackendDir "runner") -Filter "*.dll" -ErrorAction SilentlyContinue | Copy-Item -Destination $DistBackend
    if (Test-Path $RuntimeLibsDir) {
        Get-ChildItem -Path $RuntimeLibsDir -Filter "*.dll" -ErrorAction SilentlyContinue | Copy-Item -Destination $DistBackend -Force
    }
    @{
        binary = "umi_slam_runner.exe"
        vocabulary = "ORBvoc.txt"
        library_paths = @(".")
    } | ConvertTo-Json | Set-Content -Encoding UTF8 (Join-Path $DistBackend "backend.json")
}

# Bundle matching Microsoft Visual C++ x64 Redistributable (matches /MD MultiThreadedDLL)
$VcRedist = $null
if ($env:VCToolsRedistDir -and (Test-Path $env:VCToolsRedistDir)) {
    $VcRedist = Get-ChildItem -Path $env:VCToolsRedistDir -Filter "vc_redist.x64.exe" -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1 -ExpandProperty FullName
}
if (-not $VcRedist) {
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path $vswhere) {
        $inst = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
        if ($inst) {
            $VcRedist = Get-ChildItem -Path (Join-Path $inst "VC\Redist") -Filter "vc_redist.x64.exe" -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1 -ExpandProperty FullName
        }
    }
}
if ($VcRedist) {
    $RedistDir = Join-Path $DistDir "redist"
    New-Item -ItemType Directory -Force -Path $RedistDir | Out-Null
    Copy-Item $VcRedist (Join-Path $RedistDir "vc_redist.x64.exe")
    Write-Host "Bundled vc_redist.x64.exe from $VcRedist"
} else {
    Write-Warning "vc_redist.x64.exe not found in Visual Studio installation."
}

$MakeNsis = Get-Command makensis -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Source
if (-not $MakeNsis) {
    foreach ($cand in @(
        "${env:ProgramFiles(x86)}\NSIS\makensis.exe",
        "$env:ProgramFiles\NSIS\makensis.exe",
        "$env:USERPROFILE\scoop\apps\nsis\current\makensis.exe",
        "C:\app\scoop\apps\nsis\current\makensis.exe"
    )) {
        if (Test-Path $cand) { $MakeNsis = $cand; break }
    }
}

$NsiPath = Join-Path $RepoRoot "installer\windows\slam_station.nsi"
if ($BuildInstaller -or $MakeNsis) {
    if (-not $MakeNsis) {
        throw "NSIS (makensis.exe) not found. Install it with: scoop install extras/nsis  OR  winget install NSIS.NSIS"
    }
    $OutInstaller = Join-Path $RepoRoot "dist\SLAMStation-Setup-2.0.0-x64.exe"
    Write-Host "== Building Commercial-Friendly Windows Installer via NSIS ($MakeNsis) ==" -ForegroundColor Cyan
    & $MakeNsis "/DDIST_DIR=$DistDir" "/DOUT_FILE=$OutInstaller" $NsiPath
    if ($LASTEXITCODE -ne 0) { throw "NSIS failed with exit code $LASTEXITCODE" }
    Write-Host "Installer created: $OutInstaller" -ForegroundColor Green
} else {
    Write-Host "Tip: Install NSIS (e.g. 'scoop install extras/nsis' or 'winget install NSIS.NSIS') to generate dist\SLAMStation-Setup-2.0.0-x64.exe automatically." -ForegroundColor Yellow
}

Write-Host "Build & packaging complete: $DistDir" -ForegroundColor Green
