@echo off
setlocal enabledelayedexpansion

REM ESP32 Flash Script (Windows)
REM Usage: esp32flash.bat <ip_address|usb_port> <bin_file|build_path>
REM Example OTA:  esp32flash.bat 192.168.1.73 C:\tmp\arduino_build_xxx\cnc.ino.bin
REM Example USB:  esp32flash.bat COM3 C:\tmp\arduino_build_xxx\cnc.ino.bin

REM === CONFIG ===
if not defined ARDUINO15 set "ARDUINO15=%USERPROFILE%\Arduino15"
set "PYTHON=%ARDUINO15%\packages\esp8266\tools\python3\3.7.2-post1\python3.exe"
if not exist "%PYTHON%" set "PYTHON=python"
set "ESPOTA=%ARDUINO15%\packages\esp8266\hardware\esp8266\3.1.2\tools\espota.py"
set "ESPTOOL=%ARDUINO15%\packages\esp32\tools\esptool_py\5.1.0\esptool.exe"
if not exist "%ESPTOOL%" set "ESPTOOL=%ARDUINO15%\packages\esp32\tools\esptool_py\5.1.0\esptool.py"
set "BOOT_APP0=%ARDUINO15%\packages\esp32\hardware\esp32\3.3.6\tools\partitions\boot_app0.bin"
REM ==============

if "%~2"=="" (
    echo Usage: %~nx0 ^<ip_address^|usb_port^> ^<bin_file^|build_path^>
    echo.
    echo OTA mode:  %~nx0 192.168.1.73 cnc.ino.bin
    echo USB mode:  %~nx0 COM3 cnc.ino.bin
    exit /b 1
)

set "TARGET=%~1"
set "BIN_PATH=%~f2"

if not exist "%BIN_PATH%" (
    echo Error: Path not found: %BIN_PATH%
    exit /b 1
)

echo %TARGET% | findstr /b "COM" >nul
if not errorlevel 1 goto :usb

REM --- OTA mode ---
set "BIN_FILE=%BIN_PATH%"
if exist "%BIN_PATH%\" (
    set "BIN_FILE="
    for %%B in ("%BIN_PATH%\*.ino.bin") do set "BIN_FILE=%%B"
)
if not defined BIN_FILE (
    echo Error: No .ino.bin file found in %BIN_PATH%
    exit /b 1
)
if not exist "%BIN_FILE%" (
    echo Error: Bin file not found: %BIN_FILE%
    exit /b 1
)

echo === ESP32 OTA Flash Mode ===
echo   IP:   %TARGET%
echo   File: %BIN_FILE%
echo.
"%PYTHON%" -I "%ESPOTA%" -i %TARGET% -p 8266 --auth= -f "%BIN_FILE%"
exit /b %errorlevel%

:usb
REM --- USB mode ---
set "BUILD_PATH=%BIN_PATH%"
if not exist "%BIN_PATH%\" set "BUILD_PATH=%~dp2"

set "BIN_FILE="
for %%B in ("%BUILD_PATH%\*.ino.bin") do set "BIN_FILE=%%B"
if not defined BIN_FILE (
    echo Error: No .ino.bin file found in %BUILD_PATH%
    exit /b 1
)

for %%B in ("%BIN_FILE%") do set "PROJECT_NAME=%%~nB"
set "BOOTLOADER=%BUILD_PATH%\%PROJECT_NAME%.bootloader.bin"
set "PARTITIONS=%BUILD_PATH%\%PROJECT_NAME%.partitions.bin"

if not exist "%BOOTLOADER%" (
    echo Error: Bootloader not found: %BOOTLOADER%
    exit /b 1
)
if not exist "%PARTITIONS%" (
    echo Error: Partitions not found: %PARTITIONS%
    exit /b 1
)

REM Deteksi chip dari build path
set "CHIP=esp32"
echo %BUILD_PATH% | findstr /i "c3" >nul && set "CHIP=esp32c3"

echo === ESP32 USB Flash Mode ===
echo   Port: %TARGET%
echo   Chip: %CHIP%
echo   File: %BIN_FILE%
echo.
"%ESPTOOL%" --chip %CHIP% --port %TARGET% --baud 921600 --before default-reset --after hard-reset write-flash -z --flash-mode keep --flash-freq keep --flash-size keep 0x0 "%BOOTLOADER%" 0x8000 "%PARTITIONS%" 0xe000 "%BOOT_APP0%" 0x10000 "%BIN_FILE%"
exit /b %errorlevel%
