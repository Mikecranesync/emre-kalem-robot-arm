@echo off
REM ============================================================
REM  START ARM GUI.bat
REM  Double-click this file to run the FactoryLM arm console.
REM
REM  What it does, in order:
REM    1. finds a Python on this computer that has "pyserial"
REM    2. starts the local bridge  (Software\arm-console\arm-bridge.py)
REM    3. opens http://127.0.0.1:8770/ in your browser
REM
REM  LEAVE THIS WINDOW OPEN while you use the arm console.
REM  Closing it stops the bridge and the page stops working.
REM
REM  NOTE on finding Python: this laptop has four of them, and only
REM  Python 3.11 has pyserial installed. "py -3" points at 3.14,
REM  which does NOT. So this script tries each Python in turn and
REM  picks the first one that can actually import pyserial, rather
REM  than picking the first one that merely exists.
REM
REM  This is a supervised hobby bench tool. The Arduino, this
REM  script, and the on-screen stop button are NOT safety devices.
REM  The rocker switch and the inline fuse are the real stop.
REM ============================================================

title FactoryLM Arm GUI
setlocal

set "ROOT=%~dp0"
set "BRIDGE=%ROOT%Software\arm-console\arm-bridge.py"
set "URL=http://127.0.0.1:8770/"
set "PY311=C:\Program Files\Python311\python.exe"
set "PY="
set "PIPHINT="

echo.
echo  ============================================================
echo   FACTORYLM ARM GUI
echo  ============================================================
echo.

if not exist "%BRIDGE%" goto NOBRIDGE

echo  Looking for a Python that can talk to the USB port...

REM ---- pass 1: find a Python that ALREADY has pyserial ----

py -3 -c "import serial" >nul 2>&1
if not errorlevel 1 set "PY=py -3"
if defined PY goto GOTPY

python -c "import serial" >nul 2>&1
if not errorlevel 1 set "PY=python"
if defined PY goto GOTPY

if not exist "%PY311%" goto PASS2
"%PY311%" -c "import serial" >nul 2>&1
if not errorlevel 1 set PY="%PY311%"
if defined PY goto GOTPY


:PASS2
REM ---- pass 2: no Python has pyserial. Find any Python, so the
REM ---- fix line we print names the right one.

py -3 -c "import sys" >nul 2>&1
if not errorlevel 1 set "PIPHINT=py -3 -m pip install --user pyserial"
if defined PIPHINT goto NOPYSERIAL

python -c "import sys" >nul 2>&1
if not errorlevel 1 set "PIPHINT=python -m pip install --user pyserial"
if defined PIPHINT goto NOPYSERIAL

if not exist "%PY311%" goto NOPYTHON
set PIPHINT="%PY311%" -m pip install --user pyserial
goto NOPYSERIAL


:GOTPY
echo  Found it. Using this Python:
%PY% -c "import sys;print('    '+sys.executable)"
echo.
echo  Starting the bridge. Your browser will open in a few seconds.
echo  If it does not, type this address in yourself:
echo.
echo     %URL%
echo.
echo  ------------------------------------------------------------
echo   LEAVE THIS WINDOW OPEN. Press Ctrl-C here to stop.
echo  ------------------------------------------------------------
echo.

REM Wait ~3 seconds for the bridge to come up, then open the browser.
REM Full paths on purpose: if Git's Unix tools are on the PATH, a bare
REM "timeout" runs the wrong program. ping is used for the delay because
REM timeout.exe refuses to run whenever input is redirected.
start "Arm GUI browser" /min cmd /c "%SystemRoot%\System32\ping.exe -n 4 127.0.0.1 >nul & %SystemRoot%\explorer.exe %URL%"

%PY% "%BRIDGE%"

echo.
echo  The bridge has stopped. The arm console page will not work now.
echo  Double-click START ARM GUI.bat again to restart it.
echo.
pause
exit /b 0


:NOBRIDGE
echo  PROBLEM: the bridge program is missing.
echo.
echo  It should be here:
echo     %BRIDGE%
echo.
echo  Nothing was started. Check that file is really in that folder.
echo.
pause
exit /b 1


:NOPYSERIAL
echo.
echo  PROBLEM: Python is installed, but the "pyserial" package is missing.
echo.
echo  pyserial is the small add-on that lets Python talk to the Arduino
echo  over the USB cable. Without it the bridge cannot open the port.
echo.
echo  Fix it: type this one line into this window and press Enter.
echo.
echo     %PIPHINT%
echo.
echo  Then close this window and double-click START ARM GUI.bat again.
echo.
echo  If that does not work, you can skip Python completely:
echo  double-click  arm-console.html  in this folder --
echo     %ROOT%Software\arm-console
echo  and use it from Google Chrome or Microsoft Edge instead.
echo.
pause
exit /b 1


:NOPYTHON
echo.
echo  PROBLEM: Python is not installed on this computer.
echo.
echo  You do NOT need Python to use the arm console. There is a second
echo  way in that installs nothing at all:
echo.
echo     1. Open this folder:
echo          %ROOT%Software\arm-console
echo     2. Double-click  arm-console.html
echo     3. Press CHOOSE PORT and pick the entry that says
echo        Arduino Uno, then press CONNECT.
echo.
echo  That second way works in Google Chrome and Microsoft Edge only.
echo  It will not work in Firefox.
echo.
pause
exit /b 1
