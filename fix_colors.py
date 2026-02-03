import os
import re

# 要修改的文件
files = [
    'std_analyzer.cpp',
    'pair_analyzer.cpp',
    'pseudo_analyzer.cpp',
    'pseudo_pair_analyzer.cpp',
    'eo_cross_analyzer.cpp',
    'analyzer_executor.h',
    'move_tables.cpp',
    'prune_tables.cpp'
]

for filename in files:
    if not os.path.exists(filename):
        print(f'Skipped {filename} (not found)')
        continue
    with open(filename, 'r', encoding='utf-8') as f:
        content = f.read()
    
    original = content
    
    # 1. 替换无颜色的 "[Init]" 为带颜色版本
    # 匹配: std::cout << "[Init] ...
    content = re.sub(
        r'std::cout\s*<<\s*"\[Init\]\s*',
        r'std::cout << TAG_COLOR << "[INIT]" << ANSI_RESET << " ',
        content
    )
    
    # 2. 替换 ANSI_CYAN 为 TAG_COLOR (已有 << ANSI_RESET 的情况)
    content = re.sub(
        r'ANSI_CYAN\s*<<\s*"\[([A-Z]+)\]"\s*<<\s*ANSI_RESET\s*<<\s*" "',
        r'TAG_COLOR << "[\1]" << ANSI_RESET << " "',
        content
    )
    
    # 3. 替换 ANSI_GREEN << "[SUCCESS] 为 TAG_COLOR
    content = re.sub(
        r'ANSI_GREEN\s*<<\s*"\[SUCCESS\]',
        r'TAG_COLOR << "[SUCCESS]" << ANSI_RESET << "',
        content
    )
    
    if content != original:
        with open(filename, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f'Updated {filename}')
    else:
        print(f'No changes in {filename}')

print('Done!')
