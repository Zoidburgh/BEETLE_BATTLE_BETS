@echo off
echo ========================================
echo BEETLE BATTLE - BUILD AND UPLOAD TO STEAM
echo ========================================
echo.

cd /d "%~dp0"

echo [1/3] Building with PyInstaller (RELEASE - no console)...
pyinstaller --clean -y beetle_battle_release.spec
if errorlevel 1 (
    echo BUILD FAILED!
    pause
    exit /b 1
)

echo.
echo [2/3] Build complete! Files in dist\BeetleBattle
echo.

echo [3/3] Uploading to Steam...
echo.

REM Check if steamcmd exists
if not exist "steamcmd\steamcmd.exe" (
    echo ERROR: steamcmd.exe not found in steamcmd folder!
    echo.
    echo Download SteamCMD from: https://developer.valvesoftware.com/wiki/SteamCMD#Downloading_SteamCMD
    echo Extract steamcmd.exe to the steamcmd folder
    echo.
    pause
    exit /b 1
)

REM Prompt for Steam username
set /p STEAM_USER="Enter Steam username: "

echo.
echo Logging in as: %STEAM_USER%
echo (You'll be prompted for password and possibly Steam Guard code)
echo.

cd steamcmd
steamcmd.exe +login %STEAM_USER% +run_app_build ..\steamcmd\app_build_3998620.vdf +quit

echo.
echo ========================================
echo UPLOAD COMPLETE!
echo ========================================
echo.
echo Your build is now on Steam (in a build branch).
echo Go to Steamworks to set it live on default branch.
echo.
pause
