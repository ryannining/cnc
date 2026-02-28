@echo off
setlocal enabledelayedexpansion

REM ESP8266 Flash Script (Windows)
REM Usage: espflash.bat <ip_address|usb_port> <bin_file>
REM Example OTA:  espflash.bat 192.168.1.73 firmware.bin
REM Example USB:  espflash.bat COM3 firmware.bin

REM === CONFIG ===
if not defined ARDUINO15 set "ARDUINO15=%USERPROFILE%\Arduino15"
set "PYTHON=%ARDUINO15%\packages\esp8266\tools\python3\3.7.2-post1\python3.exe"
if not exist "%PYTHON%" set "PYTHON=python"
set "ESPOTA=%ARDUINO15%\packages\esp8266\hardware\esp8266\3.1.2\tools\espota.py"
set "UPLOAD=%ARDUINO15%\packages\esp8266\hardware\esp8266\3.1.2\tools\upload.py"
REM ==============

if "%~2"=="" (
    echo Usage: %~nx0 ^<ip_address^|usb_port^> ^<bin_file^>
    echo.
    echo OTA mode:  %~nx0 192.168.1.73 firmware.bin
    echo USB mode:  %~nx0 COM3 firmware.bin
    exit /b 1
)

set "TARGET=%~1"
set "BIN_FILE=%~f2"

if not exist "%BIN_FILE%" (
    echo Error: Bin file not found: %BIN_FILE%
    exit /b 1
)

echo %TARGET% | findstr /b "COM" >nul
if not errorlevel 1 goto :usb

echo === OTA Flash Mode ===
echo   IP:   %TARGET%
echo   File: %BIN_FILE%
echo.
"%PYTHON%" -I "%ESPOTA%" -i %TARGET% -p 8266 --auth= -f "%BIN_FILE%"
exit /b %errorlevel%

:usb
echo === USB Flash Mode ===
echo   Port: %TARGET%
echo   File: %BIN_FILE%
echo.
"%PYTHON%" -I "%UPLOAD%" --chip esp8266 --port %TARGET% --baud 921600 --before default_reset --after hard_reset write_flash 0x0 "%BIN_FILE%"
exit /b %errorlevel%
