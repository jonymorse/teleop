@echo off
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\g1_backend\start_wsl_backend.ps1"
echo.
echo The G1 backend stopped. Review any error shown above.
pause
