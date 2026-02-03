@echo off
:: 强制控制台使用 UTF-8 编码以支持中文注释显示，并防止乱码
chcp 65001 >nul
:: 切换至当前脚本所在的绝对路径，确保工作目录正确
cd /d %~dp0

echo === Starting Remote Repository Sync Process ===

:: 1. 执行上游同步：拉取远程仓库最新提交，防止推送冲突
echo [1/4] Fetching remote changes (git pull)...
git pull origin main

:: 2. 更新索引区：扫描本地变更并应用 .gitignore 过滤规则
echo [2/4] Updating local index (git add)...
git add .

:: 3. 事务提交：封装变更快照，利用 WMIC 获取标准时间戳作为提交消息
for /f "tokens=2 delims==" %%I in ('wmic os get localdatetime /value') do set "dt=%%I"
set "msg=Sync_%dt:~0,8%_%dt:~8,4%"

echo [3/4] Creating local snapshot (git commit: %msg%)...
git commit -m "%msg%"

:: 4. 远程推送：将本地增量提交上传至 GitHub，利用 16 线程硬件加速
echo [4/4] Pushing changes to cloud (git push)...
git push origin main

echo === Remote Sync Task Completed ===
pause