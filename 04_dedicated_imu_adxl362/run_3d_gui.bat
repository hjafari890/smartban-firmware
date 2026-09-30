@echo off
title SmartBAN ADXL362 3D IMU Indicator
cd /d "%~dp0"
python gui_3d_imu.py
if errorlevel 1 pause
