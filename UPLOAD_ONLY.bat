@echo off
echo ========================================
echo BEETLE BATTLE - UPLOAD TO STEAM (Skip Build)
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
    echo Download from: https://developer.valvesoftware.com/wiki/SteamCMD
    pause
    exit /b 1
)

echo Uploading existing build to Steam...
echo.

set /p STEAM_USER="Enter Steam username: "
echo.

cd steamcmd
steamcmd.exe +login %STEAM_USER% +run_app_build ..\steamcmd\app_build_3998620.vdf +quit

echo.
echo UPLOAD COMPLETE!
pause
