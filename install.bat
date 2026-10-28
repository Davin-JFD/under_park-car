@echo off
chcp 65001 >nul
cd /d %~dp0
where py >nul 2>nul && (set PY=py -3) || (set PY=python)
%PY% -m venv .venv
call .venv\Scripts\activate.bat
python -m pip install --upgrade pip
pip install -r requirements.txt
if errorlevel 1 pause & exit /b 1
echo 安装完成。运行 run.bat 启动。
pause
