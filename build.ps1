# build.ps1 - Makefile 包装脚本
#
# 用法:
#   .\build.ps1          - 并行编译所有目标
#   .\build.ps1 clean    - 清理编译产物
#   .\build.ps1 <target> - 编译指定目标
#
# NOTE: 需要 mingw32-make (MinGW/MSYS2 自带)

param([Parameter(ValueFromRemainingArguments)][string[]]$MakeArgs)

if ($MakeArgs.Count -eq 0) {
    mingw32-make -j8
} else {
    mingw32-make @MakeArgs
}
