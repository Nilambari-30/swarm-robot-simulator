@echo off
REM ===========================================================
REM build.bat  —  Compile swarm_math.c into swarm_math.dll
REM Requires MinGW gcc. Download: https://www.mingw-w64.org/
REM ===========================================================
echo ===========================================================
echo   Building swarm_math.c  --^>  swarm_math.dll
echo ===========================================================
gcc -O2 -shared -o swarm_math.dll swarm_math.c -lm
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] Build failed.
    echo Make sure MinGW gcc is installed and added to PATH.
    echo Download from: https://www.mingw-w64.org/
    pause
    exit /b 1
)
echo.
echo Build successful!  swarm_math.dll is ready.
echo.
echo Next steps:
echo   pip install pygame
echo   python swarm_simulator.py
echo ===========================================================
pause
