@echo off
title Deploy Robotic Arm to Vercel
cd /d "%~dp0"
echo ========================================================
echo   Deploying 5-DOF Robotic Arm to Vercel...
echo ========================================================
echo.
echo Running Vercel CLI...
echo (If prompted, log in with your GitHub or Email account)
echo.
call npx vercel --prod
echo.
echo ========================================================
echo   Deployment complete! Open your live Vercel URL above.
echo ========================================================
pause

