@echo off
rem Double-click this file to start the AR4 robot controller app (same as "npm start").
cd /d "%~dp0"

if not exist "node_modules\electron\dist\electron.exe" (
    echo Installing Electron for the first time, please wait...
    node node_modules\electron\install.js
    if errorlevel 1 (
        echo.
        echo Electron could not be installed. Check that Node.js is installed and the PC is online.
        pause
        exit /b 1
    )
)

start "" "node_modules\electron\dist\electron.exe" .
