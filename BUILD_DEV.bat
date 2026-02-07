@echo off
echo ========================================
echo BEETLE BATTLE - DEV BUILD (with console)
echo ========================================
echo.

cd /d "%~dp0"

echo Building with PyInstaller (DEV - with console for debugging)...
pyinstaller --clean -y beetle_battle.spec
if errorlevel 1 (
    echo BUILD FAILED!
    pause
    exit /b 1
)

echo.
echo ========================================
echo DEV BUILD COMPLETE!
echo ========================================
echo.
echo Files in dist\BeetleBattle
echo Run dist\BeetleBattle\BeetleBattle.exe to test
echo.
pause
