@echo off
setlocal enabledelayedexpansion

REM ESP8266 Compile Script (Windows)
REM Usage: espcompile.bat [target] <project.ino>
REM Example compile only:  espcompile.bat cnc.ino
REM Example compile + OTA: espcompile.bat 192.168.1.73 cnc.ino
REM Example compile + USB: espcompile.bat COM3 cnc.ino

REM === CONFIG: sesuaikan path install Arduino di mesin Anda ===
if not defined ARDUINO15 set "ARDUINO15=%USERPROFILE%\Arduino15"
if not defined ARDUINO_LIBS set "ARDUINO_LIBS=%USERPROFILE%\Documents\Arduino\libraries"
if not defined ARDUINO_BUILDER set "ARDUINO_BUILDER=%ARDUINO15%\packages\builtin\tools\arduino-builder\1.7.0\arduino-builder.exe"
if not defined ESPFLASH_SCRIPT set "ESPFLASH_SCRIPT=%~dp0espflash.bat"
REM =============================================================

set "FQBN=esp8266:esp8266:d1_mini:xtal=160,vt=flash,exception=disabled,stacksmash=disabled,ssl=basic,mmu=4816,non32xfer=fast,eesz=4M3M,ip=lm2n,dbg=Disabled,lvl=None____,wipe=none,baud=921600"

if "%~1"=="" (
    echo Usage: %~nx0 [target] ^<project.ino^>
    echo.
    echo Compile only:   %~nx0 cnc.ino
    echo Compile + OTA:  %~nx0 192.168.1.73 cnc.ino
    echo Compile + USB:  %~nx0 COM3 cnc.ino
    exit /b 1
)

set "TARGET="
set "INO_FILE="
set "ARG1=%~1"

REM Deteksi: arg pertama .ino = compile saja, selain itu = target
if /i "%ARG1:~-4%"==".ino" (
    set "INO_FILE=%~f1"
) else (
    if "%~2"=="" (
        echo Error: Missing .ino file parameter
        exit /b 1
    )
    set "TARGET=%~1"
    set "INO_FILE=%~f2"
)

if not exist "%INO_FILE%" (
    echo Error: INO file not found: %INO_FILE%
    exit /b 1
)

REM Build path konsisten dari path project (mirip md5 di versi bash)
set "BUILD_PATH=%TEMP%\arduino_build_%INO_FILE:\=_%"
set "CACHE_PATH=%TEMP%\arduino_cache_%INO_FILE:\=_%"
if "%BUILD_PATH:~180,1%" NEQ "" set "BUILD_PATH=%TEMP%\arduino_build_cnc"
if "%CACHE_PATH:~180,1%" NEQ "" set "CACHE_PATH=%TEMP%\arduino_cache_cnc"
mkdir "%BUILD_PATH%" 2>nul
mkdir "%CACHE_PATH%" 2>nul

for %%F in ("%INO_FILE%") do set "PROJECT_NAME=%%~nF"

echo === ESP8266 Compile ===
echo   Project: %PROJECT_NAME%
echo   File:    %INO_FILE%
if defined TARGET echo   Target:  %TARGET%
echo.

set "TOOL_PREFS=-prefs=runtime.tools.python3.path=%ARDUINO15%\packages\esp8266\tools\python3\3.7.2-post1"
set "TOOL_PREFS=!TOOL_PREFS! -prefs=runtime.tools.python3-3.7.2-post1.path=%ARDUINO15%\packages\esp8266\tools\python3\3.7.2-post1"
set "TOOL_PREFS=!TOOL_PREFS! -prefs=runtime.tools.mklittlefs.path=%ARDUINO15%\packages\esp8266\tools\mklittlefs\3.1.0-gcc10.3-e5f9fec"
set "TOOL_PREFS=!TOOL_PREFS! -prefs=runtime.tools.mklittlefs-3.1.0-gcc10.3-e5f9fec.path=%ARDUINO15%\packages\esp8266\tools\mklittlefs\3.1.0-gcc10.3-e5f9fec"
set "TOOL_PREFS=!TOOL_PREFS! -prefs=runtime.tools.mkspiffs.path=%ARDUINO15%\packages\esp8266\tools\mkspiffs\3.1.0-gcc10.3-e5f9fec"
set "TOOL_PREFS=!TOOL_PREFS! -prefs=runtime.tools.mkspiffs-3.1.0-gcc10.3-e5f9fec.path=%ARDUINO15%\packages\esp8266\tools\mkspiffs\3.1.0-gcc10.3-e5f9fec"
set "TOOL_PREFS=!TOOL_PREFS! -prefs=runtime.tools.xtensa-lx106-elf-gcc.path=%ARDUINO15%\packages\esp8266\tools\xtensa-lx106-elf-gcc\3.1.0-gcc10.3-e5f9fec"
set "TOOL_PREFS=!TOOL_PREFS! -prefs=runtime.tools.xtensa-lx106-elf-gcc-3.1.0-gcc10.3-e5f9fec.path=%ARDUINO15%\packages\esp8266\tools\xtensa-lx106-elf-gcc\3.1.0-gcc10.3-e5f9fec"

set "HARDWARE=-hardware "%ARDUINO15%\packages" -hardware "%ARDUINO15%\packages\esp8266\hardware""
set "TOOLS=-tools "%ARDUINO15%\packages\tools-builder" -tools "%ARDUINO15%\packages""

echo --- Compiling ---
"%ARDUINO_BUILDER%" -compile -logger=machines -hardware "%ARDUINO15%\packages" -tools "%ARDUINO15%\packages\tools-builder" -tools "%ARDUINO15%\packages" -libraries "%ARDUINO_LIBS%" -fqbn=%FQBN% -ide-version=10819 -build-path "%BUILD_PATH%" -warnings=none -build-cache "%CACHE_PATH%" -prefs=build.warn_data_percentage=75 %TOOL_PREFS% "%INO_FILE%"

if errorlevel 1 (
    echo.
    echo === Compile Failed ===
    exit /b 1
)

set "BIN_FILE="
for %%B in ("%BUILD_PATH%\*.ino.bin") do set "BIN_FILE=%%B"

if not defined BIN_FILE (
    echo.
    echo === Compile Failed: BIN not found ===
    exit /b 1
)

for %%B in ("%BIN_FILE%") do set "BIN_SIZE=%%~zB"
echo.
echo === Compile Success ===
echo   BIN:  %BIN_FILE%
echo   Size: !BIN_SIZE! bytes

if defined TARGET (
    echo.
    echo === Flashing to %TARGET ===
    call "%ESPFLASH_SCRIPT%" "%TARGET%" "%BIN_FILE%"
)
