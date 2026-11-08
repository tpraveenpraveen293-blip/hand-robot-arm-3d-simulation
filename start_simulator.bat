@echo off
title 5-DOF Robotic Arm 3D Simulator & Controller
cd /d "%~dp0"
echo ========================================================
echo   Launching 5-DOF Robotic Arm 3D Simulator & Hub...
echo ========================================================
echo Starting local web server on port 3000...
start http://localhost:3000
node server.js
pause

