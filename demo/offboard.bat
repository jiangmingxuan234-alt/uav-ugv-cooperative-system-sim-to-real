@echo off
REM Set the path of the RflySim tools
if not defined PSP_PATH (
    SET PSP_PATH=D:\PX4PSP
    SET PSP_PATH_LINUX=/mnt/d/PX4PSP
)
cd /d %PSP_PATH%\VcXsrv
tasklist|find /i "vcxsrv.exe" >nul || Xlaunch.exe -run config1.xlaunch

cd /d %~dp0

echo python3 EnvSet.py
start wsl -d RflySim-20.04 -e bash -lic "python3 EnvSet.py"
choice /t 5 /d y /n >nul
echo python3 main.py
start wsl -d RflySim-20.04 -e bash -lic "python3 main.py"
choice /t 20 /d y /n >nul
REM echo ./run_demo.sh
REM start wsl -d RflySim-20.04 -e bash -lic "./run_demo.sh"
REM choice /t 10 /d y /n >nul
echo  roslaunch faster_lio rflysim.launch
start wsl -d RflySim-20.04 -e bash -lic "roslaunch faster_lio rflysim.launch"
choice /t 10 /d y /n >nul
echo python3 offboard.py 
start wsl -d RflySim-20.04 -e bash -lic "python3 offboard.py" 
choice /t 31 /d y /n >nul
echo python3 bz.py 
start wsl -d RflySim-20.04 -e bash -lic "python3 bz.py" 
choice /t 20 /d y /n >nul
echo python3 nm.py 
start wsl -d RflySim-20.04 -e bash -lic "python3 nm.py" 
echo python3 balloon_detect.py 
start wsl -d RflySim-20.04 -e bash -lic "python3 balloon_detect.py" 