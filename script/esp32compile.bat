@echo off
setlocal enabledelayedexpansion

REM ESP32 Compile Script (Windows)
REM Usage: esp32compile.bat [board] [target] <project.ino>
REM Example compile only:  esp32compile.bat cnc.ino
REM Example with board:    esp32compile.bat lolin_c3_mini cnc.ino
REM Example compile + OTA: esp32compile.bat lolin_c3_mini 192.168.1.73 cnc.ino
REM Example compile + USB: esp32compile.bat lolin_c3_mini COM3 cnc.ino

REM === CONFIG: sesuaikan path install Arduino di mesin Anda ===
if not defined ARDUINO15 set "ARDUINO15=%USERPROFILE%\Arduino15"
if not defined ARDUINO_LIBS set "ARDUINO_LIBS=%USERPROFILE%\Documents\Arduino\libraries"
if not defined ARDUINO_BUILDER set "ARDUINO_BUILDER=%ARDUINO15%\packages\builtin\tools\arduino-builder\1.7.0\arduino-builder.exe"
if not defined ESPFLASH_SCRIPT set "ESPFLASH_SCRIPT=%~dp0esp32flash.bat"
REM =============================================================

set "DEFAULT_BOARD=lolin_c3_mini"
set "ESP32_VERSION=3.3.6"

if "%~1"=="" (
    echo Usage: %~nx0 [board] [target] ^<project.ino^>
    echo.
    echo Compile only:              %~nx0 cnc.ino
    echo With board:                %~nx0 lolin_c3_mini cnc.ino
    echo Compile + OTA:             %~nx0 lolin_c3_mini 192.168.1.73 cnc.ino
    echo Compile + USB:             %~nx0 lolin_c3_mini COM3 cnc.ino
    echo.
    echo Available boards: esp32, lolin_c3_mini, esp32c3, esp32s2, esp32s3, lolin_s2_mini, lolin_s3_mini
    echo Or use full FQBN: esp32:esp32:esp32c3:...
    exit /b 1
)

set "TARGET="
set "INO_FILE="
set "BOARD=%DEFAULT_BOARD%"

for %%A in (%*) do (
    set "ARG=%%~A"
    if /i "!ARG:~-4!"==".ino" (
        set "INO_FILE=%%~fA"
    ) else if "!ARG:~0,5!"=="/dev/" (
        set "TARGET=%%~A"
    ) else if "!ARG:~0,3!"=="COM" (
        set "TARGET=%%~A"
    ) else if "!ARG:~0,8!"=="esp32:esp" (
        set "BOARD=%%~A"
    ) else if "!ARG!"=="%%~nA" (
        REM IP address atau nama board
        echo !ARG!| findstr /r "^[0-9]*\.[0-9]*\.[0-9]*\.[0-9]*$" >nul && (set "TARGET=%%~A") || (set "BOARD=%%~A")
    ) else (
        set "BOARD=%%~A"
    )
)

if not defined INO_FILE (
    echo Error: Missing .ino file parameter
    exit /b 1
)
if not exist "%INO_FILE%" (
    echo Error: INO file not found: %INO_FILE%
    exit /b 1
)

REM Resolve FQBN
set "FQBN="
if /i "%BOARD%"=="esp32" set "FQBN=esp32:esp32:esp32:CDCOnBoot=default,PartitionScheme=default,CPUFreq=240,FlashFreq=80,UploadSpeed=921600,DebugLevel=none,EraseFlash=none"
if /i "%BOARD%"=="esp32_devkit" set "FQBN=esp32:esp32:esp32:CDCOnBoot=default,PartitionScheme=default,CPUFreq=240,FlashFreq=80,UploadSpeed=921600,DebugLevel=none,EraseFlash=none"
if /i "%BOARD%"=="lolin_c3_mini" set "FQBN=esp32:esp32:lolin_c3_mini:CDCOnBoot=default,PartitionScheme=default,CPUFreq=160,FlashFreq=80,UploadSpeed=921600,DebugLevel=none,EraseFlash=none"
if /i "%BOARD%"=="c3" set "FQBN=esp32:esp32:lolin_c3_mini:CDCOnBoot=default,PartitionScheme=default,CPUFreq=160,FlashFreq=80,UploadSpeed=921600,DebugLevel=none,EraseFlash=none"
if /i "%BOARD%"=="esp32c3" set "FQBN=esp32:esp32:esp32c3:CDCOnBoot=default,PartitionScheme=default,CPUFreq=160,FlashFreq=80,UploadSpeed=921600,DebugLevel=none,EraseFlash=none"
if /i "%BOARD%"=="c3_devkit" set "FQBN=esp32:esp32:esp32c3:CDCOnBoot=default,PartitionScheme=default,CPUFreq=160,FlashFreq=80,UploadSpeed=921600,DebugLevel=none,EraseFlash=none"
if /i "%BOARD%"=="esp32s2" set "FQBN=esp32:esp32:esp32s2:CDCOnBoot=default,PartitionScheme=default,CPUFreq=240,FlashFreq=80,UploadSpeed=921600,DebugLevel=none,EraseFlash=none"
if /i "%BOARD%"=="s2" set "FQBN=esp32:esp32:esp32s2:CDCOnBoot=default,PartitionScheme=default,CPUFreq=240,FlashFreq=80,UploadSpeed=921600,DebugLevel=none,EraseFlash=none"
if /i "%BOARD%"=="esp32s3" set "FQBN=esp32:esp32:esp32s3:CDCOnBoot=default,PartitionScheme=default,CPUFreq=240,FlashFreq=80,UploadSpeed=921600,DebugLevel=none,EraseFlash=none"
if /i "%BOARD%"=="s3" set "FQBN=esp32:esp32:esp32s3:CDCOnBoot=default,PartitionScheme=default,CPUFreq=240,FlashFreq=80,UploadSpeed=921600,DebugLevel=none,EraseFlash=none"
if /i "%BOARD%"=="lolin_s2_mini" set "FQBN=esp32:esp32:lolin_s2_mini:CDCOnBoot=default,PartitionScheme=default,CPUFreq=240,FlashFreq=80,UploadSpeed=921600,DebugLevel=none,EraseFlash=none"
if /i "%BOARD%"=="s2_mini" set "FQBN=esp32:esp32:lolin_s2_mini:CDCOnBoot=default,PartitionScheme=default,CPUFreq=240,FlashFreq=80,UploadSpeed=921600,DebugLevel=none,EraseFlash=none"
if /i "%BOARD%"=="lolin_s3_mini" set "FQBN=esp32:esp32:lolin_s3_mini:CDCOnBoot=default,PartitionScheme=default,CPUFreq=240,FlashFreq=80,UploadSpeed=921600,DebugLevel=none,EraseFlash=none"
if /i "%BOARD%"=="s3_mini" set "FQBN=esp32:esp32:lolin_s3_mini:CDCOnBoot=default,PartitionScheme=default,CPUFreq=240,FlashFreq=80,UploadSpeed=921600,DebugLevel=none,EraseFlash=none"
if not defined FQBN set "FQBN=%BOARD%"
if "%FQBN:~0,10%" NEQ "esp32:esp32" set "FQBN=esp32:esp32:%BOARD%:CDCOnBoot=default,PartitionScheme=default,CPUFreq=160,FlashFreq=80,UploadSpeed=921600,DebugLevel=none,EraseFlash=none"

REM Build path konsisten dari path project + board
set "KEY=%INO_FILE%:%BOARD%"
set "BUILD_PATH=%TEMP%\arduino_build_%KEY::=_&_" & rem fallback simple
set "BUILD_PATH=%TEMP%\arduino_build_cnc"
set "CACHE_PATH=%TEMP%\arduino_cache_cnc"
mkdir "%BUILD_PATH%" 2>nul
mkdir "%CACHE_PATH%" 2>nul

for %%F in ("%INO_FILE%") do set "PROJECT_NAME=%%~nF"

echo === ESP32 Compile ===
echo   Project: %PROJECT_NAME%
echo   Board:   %BOARD%
echo   File:    %INO_FILE%
if defined TARGET echo   Target:  %TARGET%
echo.

set "P=%ARDUINO15%\packages\esp32\tools"
set "TOOL_PREFS=-prefs=runtime.tools.esp32-libs.path=%P%\esp32-libs\%ESP32_VERSION%"
for %%T in (esp32s2-libs esp32s3-libs esp32c3-libs esp32h2-libs esp32c6-libs esp32c5-libs esp32p4-libs esp32p4_es-libs) do (
    set "TOOL_PREFS=!TOOL_PREFS! -prefs=runtime.tools.%%T.path=%P%\%%T\%ESP32_VERSION%"
)
set "TOOL_PREFS=!TOOL_PREFS! -prefs=runtime.tools.esp-x32.path=%P%\esp-x32\2511"
set "TOOL_PREFS=!TOOL_PREFS! -prefs=runtime.tools.esp-rv32.path=%P%\esp-rv32\2511"
set "TOOL_PREFS=!TOOL_PREFS! -prefs=runtime.tools.esptool_py.path=%P%\esptool_py\5.1.0"
set "TOOL_PREFS=!TOOL_PREFS! -prefs=runtime.tools.mklittlefs.path=%P%\mklittlefs\4.0.2-db0513a"
set "TOOL_PREFS=!TOOL_PREFS! -prefs=runtime.tools.mkspiffs.path=%P%\mkspiffs\0.2.3"
set "TOOL_PREFS=!TOOL_PREFS! -prefs=runtime.tools.openocd-esp32.path=%P%\openocd-esp32\v0.12.0-esp32-20251215"
set "TOOL_PREFS=!TOOL_PREFS! -prefs=runtime.tools.xtensa-esp-elf-gdb.path=%P%\xtensa-esp-elf-gdb\16.3_20250913"
set "TOOL_PREFS=!TOOL_PREFS! -prefs=runtime.tools.riscv32-esp-elf-gdb.path=%P%\riscv32-esp-elf-gdb\16.3_20250913"
set "TOOL_PREFS=!TOOL_PREFS! -prefs=runtime.tools.dfu-util.path=%ARDUINO15%\packages\arduino\tools\dfu-util\0.11.0-arduino5"

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
    call "%ESPFLASH_SCRIPT%" "%TARGET%" "%BUILD_PATH%"
)
