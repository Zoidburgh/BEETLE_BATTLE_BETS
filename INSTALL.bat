@echo off
echo ========================================
echo   BEETLE BATTLE - First Time Setup
echo ========================================
echo.

REM Try to find Python
set PYTHON_CMD=

py -3.12 --version >nul 2>&1
if not errorlevel 1 (
    set PYTHON_CMD=py -3.12
    goto :found
)

python --version >nul 2>&1
if not errorlevel 1 (
    set PYTHON_CMD=python
    goto :found
)

echo ERROR: Python not found!
echo.
echo Please install Python 3.12 from:
echo   https://www.python.org/downloads/release/python-3120/
echo.
echo IMPORTANT: Check "Add Python to PATH" during installation!
echo.
pause
exit /b 1

:found
echo Python found: %PYTHON_CMD%
echo.
echo Installing Taichi...
%PYTHON_CMD% -m pip install taichi

echo.
echo Installing py_steam_net from wheel...
%PYTHON_CMD% -m pip install "%~dp0py_steam_net-0.1.0-cp312-cp312-win_amd64.whl"

if errorlevel 1 (
    echo.
    echo ERROR: Failed to install dependencies.
    echo Try running as Administrator.
    pause
    exit /b 1
)

REM Copy steam_api64.dll to Python folder
echo.
echo Copying Steam DLL to Python folder...
for /f "tokens=*" %%i in ('%PYTHON_CMD% -c "import sys; print(sys.prefix)"') do set PYTHON_PATH=%%i
copy /Y "%~dp0steam_api64.dll" "%PYTHON_PATH%\" >nul 2>&1

if errorlevel 1 (
    echo WARNING: Could not copy steam_api64.dll automatically.
    echo Copy it manually to: %PYTHON_PATH%
) else (
    echo Steam DLL copied to %PYTHON_PATH%
)

echo.
echo ========================================
echo   Installation Complete!
echo ========================================
echo.
echo Run PLAY.bat to start (make sure Steam is running)
echo.
pause
