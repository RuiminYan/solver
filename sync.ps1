# sync.ps1 - 同步至远程仓库
#
# NOTE: PowerShell 7+ 默认 UTF-8，无需 chcp 65001

$ErrorActionPreference = "Stop"
Push-Location $PSScriptRoot

try {
    Write-Host "=== Starting Remote Repository Sync Process ==="

    # 1. 执行上游同步：拉取远程仓库最新提交，防止推送冲突
    Write-Host "[1/4] Fetching remote changes (git pull)..."
    git pull origin main

    # 2. 更新索引区：扫描本地变更并应用 .gitignore 过滤规则
    Write-Host "[2/4] Updating local index (git add)..."
    git add .

    # 3. 事务提交：封装变更快照，使用 PowerShell 原生时间戳
    $msg = "Sync_$(Get-Date -Format 'yyyyMMdd_HHmm')"
    Write-Host "[3/4] Creating local snapshot (git commit: $msg)..."
    git commit -m $msg

    # 4. 远程推送：将本地增量提交上传至 GitHub
    Write-Host "[4/4] Pushing changes to cloud (git push)..."
    git push origin main

    Write-Host "=== Remote Sync Task Completed ==="
}
finally {
    Pop-Location
}

Read-Host "Press Enter to continue"
