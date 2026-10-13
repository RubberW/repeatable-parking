@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\control_panel.ps1"
if errorlevel 1 pause
