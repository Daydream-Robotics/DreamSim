@echo off
:: Builds native/ (robot-core's robotcore_mpc + DreamSim tools like als_dump) into build/.
:: Loads the 64-bit MSVC environment itself, so it works from any terminal.
::
:: usage: scripts\build_native.bat [clean] [extra cmake --build args]
::   clean  wipe build/ and reconfigure from scratch
setlocal
cd /d "%~dp0.."

if /i "%~1"=="clean" (
    if exist build rmdir /s /q build
    shift
)

:: load x64 MSVC unless this terminal already has it
if /i not "%VSCMD_ARG_TGT_ARCH%"=="x64" (
    set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
    if not exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" (
        echo Visual Studio Installer not found. Install VS Build Tools with "Desktop development with C++".
        exit /b 1
    )
    for /f "usebackq delims=" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSINSTALL=%%i"
)
if /i not "%VSCMD_ARG_TGT_ARCH%"=="x64" (
    if not defined VSINSTALL (
        echo No Visual Studio install with the MSVC x64 compiler found. Add "Desktop development with C++" in the VS Installer.
        exit /b 1
    )
    call "%VSINSTALL%\VC\Auxiliary\Build\vcvars64.bat" >nul || exit /b 1
)

uv run cmake -S native -B build -G Ninja -DCMAKE_BUILD_TYPE=Release || exit /b 1
uv run cmake --build build %1 %2 %3 %4 %5 || exit /b 1
echo.
echo Built: build\als_dump.exe, build\robotcore\robotcore_mpc.lib
