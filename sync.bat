@echo off
:: 进入当前脚本所在目录
cd /d %~dp0

echo === Step 1: Adding changes... ===
git add .

echo === Step 2: Committing with timestamp... ===
:: 自动生成带日期时间的备注，省去手动输入 -m 的麻烦
set msg=Sync_Update_%date:~0,4%%date:~5,2%%date:~8,2%_%time:~0,2%%time:~3,2%
git commit -m "%msg%"

echo === Step 3: Pushing to GitHub... ===
git push origin main

echo === Done! ===
pause