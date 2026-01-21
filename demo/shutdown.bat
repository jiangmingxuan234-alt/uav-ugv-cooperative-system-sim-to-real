@echo off
REM 关闭WSL发行版（例如RflySim-20.04）
wsl --shutdown

REM 关闭VcXsrv进程
taskkill /f /im vcxsrv.exe >nul 2>&1

echo WSL和VcXsrv已关闭

REM kill all applications when press a key
tasklist|find /i "CopterSim.exe" && taskkill /im "CopterSim.exe"
tasklist|find /i "QGroundControl.exe" && taskkill /f /im "QGroundControl.exe"
tasklist|find /i "RflySim3D.exe" && taskkill /f /im "RflySim3D.exe"

pause