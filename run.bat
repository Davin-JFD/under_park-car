@echo off
chcp 65001 >nul
cd /d %~dp0
if not exist .venv\Scripts\python.exe (
  echo 尚未安装，请先双击 install.bat
  pause
  exit /b 1
)
.venv\Scripts\python.exe app.py
if errorlevel 1 pause
