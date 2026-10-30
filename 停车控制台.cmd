@echo off
chcp 65001 >nul
rem 文件职责：唯一启动入口，打开我们自己的Processing停车仿真。
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\run.ps1" %*
if errorlevel 1 pause
