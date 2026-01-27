@echo off
:: 设置当前脚本所在目录为工作路径，确保相对路径引用的正确性
cd /d %~dp0

echo === 正在启动远程仓库同步程序 ===

:: 1. 执行上游同步 (Upstream Synchronization)
:: 从远程仓库 (origin) 获取最新提交并合并至本地 main 分支，防止因版本落后导致的推送冲突 (Push Rejection)
echo [1/4] 正在拉取远程变更 (git pull)...
git pull origin main

:: 2. 更新暂存区 (Staging Area)
:: 扫描当前目录下的变更（新增、修改、物理删除），根据 .gitignore 过滤策略更新文件索引
[cite_start]:: 注意：3.1GB 的 .bin 剪枝表受忽略规则保护，不会被索引记录 [cite: 1]
echo [2/4] 正在更新本地索引区 (git add)...
git add .

:: 3. 提交事务 (Commit Transaction)
:: 将暂存区的变更封装为原子提交，并生成基于 ISO 8601 格式的时间戳快照作为物理 ID
echo [3/4] 正在生成本地快照 (git commit)...
set msg=Sync_%date:~0,4%%date:~5,2%%date:~8,2%_%time:~0,2%%time:~3,2%
git commit -m "%msg%"

:: 4. 推送至远程仓库 (Remote Push)
[cite_start]:: 将本地 main 分支的增量存档推送至 GitHub。该过程将利用多线程（当前环境：16 线程）进行数据压缩 [cite: 1]
echo [4/4] 正在将变更推送至云端 (git push)...
git push origin main

echo === 远程同步任务执行完毕 ===
pause