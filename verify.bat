@echo off
setlocal enabledelayedexpansion

REM ========================================
REM verify.bat - Automated test verification
REM Runs each analyzer and compares output
REM CSV (first 21 lines) against golden files.
REM ========================================

set PASS_COUNT=0
set FAIL_COUNT=0
set SKIP_COUNT=0
set TOTAL=5

echo.
echo ============================================
echo   Automated Verification Suite
echo ============================================
echo.

REM --- Test 1: std_analyzer ---
call :run_test "std_analyzer" "scramble_1000.txt" "std_analyzer.exe" "scramble_1000_std.csv" "golden\scramble_1000_std.txt"

REM --- Test 2: pseudo_analyzer ---
call :run_test "pseudo_analyzer" "scramble_1000.txt" "pseudo_analyzer.exe" "scramble_1000_pseudo.csv" "golden\scramble_1000_pseudo.txt"

REM --- Test 3: pair_analyzer ---
call :run_test "pair_analyzer" "scramble_1000.txt" "pair_analyzer.exe" "scramble_1000_pair.csv" "golden\scramble_1000_pair.txt"

REM --- Test 4: pseudo_pair_analyzer ---
call :run_test "pseudo_pair_analyzer" "scramble_100.txt" "pseudo_pair_analyzer.exe" "scramble_100_pseudo_pair.csv" "golden\scramble_100_pseudo_pair.txt"

REM --- Test 5: eo_cross_analyzer ---
call :run_test "eo_cross_analyzer" "scramble_20.txt" "eo_cross_analyzer.exe" "scramble_20_eo.csv" "golden\scramble_20_eo.txt"

REM --- Summary ---
echo.
echo ============================================
echo   Results: %PASS_COUNT% PASS / %FAIL_COUNT% FAIL / %SKIP_COUNT% SKIP (total %TOTAL%)
echo ============================================

if %FAIL_COUNT% gtr 0 (
    echo   [FAILED] Some tests did not pass!
    exit /b 1
) else if %SKIP_COUNT% gtr 0 (
    echo   [WARNING] Some tests were skipped.
    exit /b 0
) else (
    echo   [ALL PASS] All tests passed!
    exit /b 0
)

REM ========================================
REM Subroutine: run_test
REM Args: %1=name %2=input %3=exe %4=output_csv %5=golden_file
REM ========================================
:run_test
set "TEST_NAME=%~1"
set "INPUT_FILE=%~2"
set "EXE_FILE=%~3"
set "OUTPUT_CSV=%~4"
set "GOLDEN_FILE=%~5"

echo [TEST] %TEST_NAME%

REM Check exe exists
if not exist "%EXE_FILE%" (
    echo   SKIP - %EXE_FILE% not found
    set /a SKIP_COUNT+=1
    echo.
    goto :eof
)

REM Check input exists
if not exist "%INPUT_FILE%" (
    echo   SKIP - %INPUT_FILE% not found
    set /a SKIP_COUNT+=1
    echo.
    goto :eof
)

REM Check golden exists
if not exist "%GOLDEN_FILE%" (
    echo   SKIP - %GOLDEN_FILE% not found
    set /a SKIP_COUNT+=1
    echo.
    goto :eof
)

REM Run analyzer
echo   Running %EXE_FILE%...
cmd /c "echo %INPUT_FILE% | %EXE_FILE%" >nul 2>&1

REM Check output exists
if not exist "%OUTPUT_CSV%" (
    echo   FAIL - %OUTPUT_CSV% not generated
    set /a FAIL_COUNT+=1
    echo.
    goto :eof
)

REM Extract first 21 lines from output CSV to temp file
set "TEMP_FILE=%TEST_NAME%_verify_tmp.txt"
set LINE_NUM=0
(
    for /f "usebackq delims=" %%L in ("%OUTPUT_CSV%") do (
        if !LINE_NUM! lss 21 (
            echo %%L
            set /a LINE_NUM+=1
        )
    )
) > "%TEMP_FILE%"

REM Compare with golden file
fc /w "%TEMP_FILE%" "%GOLDEN_FILE%" >nul 2>&1
if %errorlevel% equ 0 (
    echo   PASS
    set /a PASS_COUNT+=1
) else (
    echo   FAIL - output differs from golden file
    echo   Run: fc "%TEMP_FILE%" "%GOLDEN_FILE%" to see differences
    set /a FAIL_COUNT+=1
)

REM Clean up temp file
del "%TEMP_FILE%" 2>nul

echo.
goto :eof
