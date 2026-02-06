@echo off
echo ========================================
echo BEETLE BATTLE - UPLOAD TO DEMO
echo ========================================
echo.

cd /d "%~dp0"

if not exist "dist\BeetleBattle\BeetleBattle.exe" (
    echo ERROR: No build found! Run BUILD_AND_UPLOAD.bat first.
    pause
    exit /b 1
)

if not exist "steamcmd\steamcmd.exe" (
    echo ERROR: steamcmd.exe not found in steamcmd folder!
    pause
    exit /b 1
)

echo Uploading to Demo app (4397150)...
echo.

set /p STEAM_USER="Enter Steam username: "
echo.

cd steamcmd
steamcmd.exe +login %STEAM_USER% +run_app_build ..\steamcmd\app_build_demo.vdf +quit

echo.
echo ========================================
echo DEMO UPLOAD COMPLETE!
echo ========================================
echo.
echo Go to Steamworks to set the build live on default branch.
echo.
pause
