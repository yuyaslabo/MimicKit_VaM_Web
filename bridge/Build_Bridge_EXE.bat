@echo off
setlocal
cd /d "%~dp0"
echo [Bridge] Windows standalone EXE build
where py >nul 2>nul
if not errorlevel 1 (
  set "PY_CMD=py -3"
) else (
  set "PY_CMD=python"
)
if not exist ".venv\Scripts\python.exe" (
  echo [Bridge] Creating Python virtual environment...
  %PY_CMD% -m venv .venv
  if errorlevel 1 goto :error
)
call .venv\Scripts\activate.bat
if errorlevel 1 goto :error
python -m pip install --upgrade pip
if errorlevel 1 goto :error
python -m pip install -r requirements-build.txt
if errorlevel 1 goto :error
python -m PyInstaller --noconfirm --clean --onedir --console --name VaM_Web_Bridge --collect-submodules scipy --collect-submodules numpy vam_python_bridge.py
if errorlevel 1 goto :error
if not exist "dist\VaM_Web_Bridge\VaM_Web_Bridge.exe" goto :error
copy /Y "bridge_config.json" "dist\VaM_Web_Bridge\bridge_config.json" >nul
if errorlevel 1 goto :error
echo.
echo [Bridge] Build successful: dist\VaM_Web_Bridge\VaM_Web_Bridge.exe
echo [Bridge] Run this EXE first for a local smoke test.
echo [Bridge] For a clickable setup EXE, install Inno Setup and run Build_Installer.bat
pause
exit /b 0
:error
echo.
echo [Bridge] ERROR: build failed. Review the console output.
pause
exit /b 1
