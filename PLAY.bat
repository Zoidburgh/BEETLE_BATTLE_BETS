@echo off
echo Starting Beetle Battle...
echo.
echo Make sure Steam is running!
echo.

cd /d "%~dp0"

REM Try py launcher first
py -3.12 --version >nul 2>&1
if not errorlevel 1 (
    py -3.12 beetle_physics.py
    goto :end
)

REM Try python command
python --version >nul 2>&1
if not errorlevel 1 (
    python beetle_physics.py
    goto :end
)

echo.
echo ERROR: Python not found!
echo.
echo Please install Python 3.12 from https://python.org
echo IMPORTANT: Check "Add Python to PATH" during installation!
echo.
pause
exit /b 1

:end
if errorlevel 1 (
    echo.
    echo Game crashed or closed with error.
    pause
)
