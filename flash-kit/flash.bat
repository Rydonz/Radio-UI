@echo off
setlocal
title DelSol Head Unit - OTA Flasher

rem Find Python (py launcher preferred, then python on PATH)
set PY=
where py >nul 2>nul && set PY=py
if "%PY%"=="" ( where python >nul 2>nul && set PY=python )
if "%PY%"=="" (
  echo.
  echo   Python was not found. Install it from https://www.python.org/downloads/
  echo   and tick "Add python.exe to PATH" during setup, then run this again.
  echo.
  pause
  exit /b 1
)

echo.
echo   DelSol Head Unit - OTA flash
echo   ---------------------------------------------
echo   Before continuing:
echo     1. Phone hotspot "Caleb's S25 Ultra" is ON
echo     2. This laptop is connected to that hotspot
echo     3. The unit is in OTA mode (hold NEXT while powering on)
echo     4. Read the IP address shown on the unit's screen
echo.

set /p IP=  Enter the IP shown on the unit (e.g. 192.168.x.x):
if "%IP%"=="" ( echo   No IP entered. Aborting. & pause & exit /b 1 )

echo.
echo   Flashing firmware.bin to %IP% ...
echo.
%PY% "%~dp0espota.py" -i %IP% -p 3232 -f "%~dp0firmware.bin" -r -d
set RC=%errorlevel%

echo.
if "%RC%"=="0" (
  echo   DONE. The unit will reboot into the new firmware.
) else (
  echo   FAILED (exit %RC%). Check the IP, that the laptop is on the hotspot,
  echo   and that the unit still shows the OTA screen, then try again.
)
echo.
pause
