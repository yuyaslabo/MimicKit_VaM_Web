
@echo off
setlocal
cd /d "%~dp0"

set "ISCC=C:\Program Files\Inno Setup 7\ISCC.exe"

if not exist "%ISCC%" (
    echo Inno Setup 7 not found.
    pause
    exit /b 1
)

echo Using: %ISCC%

"%ISCC%" "VaM_Web_Bridge_Installer.iss"

if errorlevel 1 (
    echo Installer build failed.
    pause
    exit /b 1
)

echo Installer build completed successfully.
pause
