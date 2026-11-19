$ErrorActionPreference = 'Stop'
$repoScript = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..\build_windows.ps1'))
& $repoScript @args
exit $LASTEXITCODE
