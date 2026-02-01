@echo off
echo Cleaning build files...

if exist *.o del *.o
if exist pair_analyzer.exe del pair_analyzer.exe
if exist std_analyzer.exe del std_analyzer.exe
if exist eo_cross_analyzer.exe del eo_cross_analyzer.exe

echo Clean completed!