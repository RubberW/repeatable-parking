@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\boundary_heading_demo.ps1"
if errorlevel 1 pause
