@echo off
REM build.bat - Wrapper for Makefile
REM
REM Usage:
REM   build.bat         - Build all targets (parallel)
REM   build.bat clean   - Remove build artifacts
REM   build.bat <target> - Build specific target
REM
REM NOTE: Requires mingw32-make (shipped with MinGW/MSYS2)

if "%1"=="" (
    mingw32-make -j8
) else (
    mingw32-make %*
)