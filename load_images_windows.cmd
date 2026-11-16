@echo off
setlocal
cd /d "%~dp0"
where py >nul 2>nul
if errorlevel 1 goto use_python
py -3 software\slam_station\cli.py load-image
if errorlevel 1 pause
exit /b
:use_python
python software\slam_station\cli.py load-image
if errorlevel 1 pause
