@echo off
setlocal
cd /d "%~dp0"
if exist "dist\SLAMStation\slam_station.exe" (
    start "" "%~dp0dist\SLAMStation\slam_station.exe" %*
    exit /b 0
)
if exist "build\Release\slam_station.exe" (
    start "" "%~dp0build\Release\slam_station.exe" %*
    exit /b 0
)
echo [ERROR] 未找到 slam_station.exe，请先运行 build_windows.ps1 完成编译打包。
pause
exit /b 1
