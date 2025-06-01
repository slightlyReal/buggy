@echo off

REM Kill PlatformIO monitor (if running)
taskkill /IM python.exe /FI "WINDOWTITLE eq *PlatformIO Device Monitor*" /F >nul 2>&1

REM Wait a moment
timeout /t 1 >nul

REM Upload firmware
pio run -t upload -e buggy

REM Wait a moment
timeout /t 1 >nul

REM Reopen serial monitor
pio device monitor -e buggy