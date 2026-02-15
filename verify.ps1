# verify.ps1 - 自动化测试验证
# 运行每个 analyzer 并对比输出 CSV (前 21 行) 与 golden 文件

$passCount = 0
$failCount = 0
$skipCount = 0
$total = 5

Write-Host ""
Write-Host "============================================"
Write-Host "  Automated Verification Suite"
Write-Host "============================================"
Write-Host ""

function Invoke-Test {
    param(
        [string]$TestName,
        [string]$InputFile,
        [string]$ExeFile,
        [string]$OutputCsv,
        [string]$GoldenFile
    )

    Write-Host "[TEST] $TestName"

    # NOTE: timing 文件在临时目录中 (.csv -> _timing.txt)
    $baseName = [System.IO.Path]::GetFileNameWithoutExtension($OutputCsv)
    $timingFile = Join-Path $env:TEMP "${baseName}_timing.txt"

    # 检查 exe 是否存在
    if (-not (Test-Path $ExeFile)) {
        Write-Host "  SKIP - $ExeFile not found"
        $script:skipCount++
        Write-Host ""
        return
    }

    # 检查输入文件是否存在
    if (-not (Test-Path $InputFile)) {
        Write-Host "  SKIP - $InputFile not found"
        $script:skipCount++
        Write-Host ""
        return
    }

    # 检查 golden 文件是否存在
    if (-not (Test-Path $GoldenFile)) {
        Write-Host "  SKIP - $GoldenFile not found"
        $script:skipCount++
        Write-Host ""
        return
    }

    # 运行 analyzer：通过管道传入输入文件名
    Write-Host "  Running $ExeFile..."
    $InputFile | & ".\$ExeFile" 2>$null | Out-Null

    # 检查输出文件是否生成
    if (-not (Test-Path $OutputCsv)) {
        Write-Host "  FAIL - $OutputCsv not generated"
        $script:failCount++
        Write-Host ""
        return
    }

    # 提取输出 CSV 的前 21 行
    $outputLines = Get-Content $OutputCsv -TotalCount 21

    # 读取 golden 文件
    $goldenLines = Get-Content $GoldenFile

    # 读取 timing 信息
    $timeSuffix = ""
    if (Test-Path $timingFile) {
        $timingInfo = (Get-Content $timingFile -TotalCount 1).Trim()
        $timeSuffix = " [${timingInfo}s]"
        Remove-Item $timingFile -ErrorAction SilentlyContinue
    }

    # 逐行比对（等价于 fc /w）
    $diff = Compare-Object $outputLines $goldenLines
    if ($null -eq $diff) {
        Write-Host "  PASS$timeSuffix"
        $script:passCount++
    }
    else {
        Write-Host "  FAIL - output differs from golden file$timeSuffix"
        Write-Host "  Run: Compare-Object (Get-Content `"$OutputCsv`" -TotalCount 21) (Get-Content `"$GoldenFile`") to see differences"
        $script:failCount++
    }

    Write-Host ""
}

# --- 运行测试 ---
Invoke-Test "std_analyzer"         "scramble_1000.txt" "std_analyzer.exe"         "scramble_1000_std.csv"         "golden\scramble_1000_std.txt"
Invoke-Test "pseudo_analyzer"      "scramble_1000.txt" "pseudo_analyzer.exe"      "scramble_1000_pseudo.csv"      "golden\scramble_1000_pseudo.txt"
Invoke-Test "pair_analyzer"        "scramble_1000.txt" "pair_analyzer.exe"        "scramble_1000_pair.csv"        "golden\scramble_1000_pair.txt"
Invoke-Test "pseudo_pair_analyzer" "scramble_100.txt"  "pseudo_pair_analyzer.exe" "scramble_100_pseudo_pair.csv"  "golden\scramble_100_pseudo_pair.txt"
Invoke-Test "eo_cross_analyzer"    "scramble_20.txt"   "eo_cross_analyzer.exe"    "scramble_20_eo.csv"            "golden\scramble_20_eo.txt"

# --- 汇总 ---
Write-Host "============================================"
Write-Host "  Results: $passCount PASS / $failCount FAIL / $skipCount SKIP (total $total)"
Write-Host "============================================"

if ($failCount -gt 0) {
    Write-Host "  [FAILED] Some tests did not pass!"
    exit 1
}
elseif ($skipCount -gt 0) {
    Write-Host "  [WARNING] Some tests were skipped."
    exit 0
}
else {
    Write-Host "  [ALL PASS] All tests passed!"
    exit 0
}
