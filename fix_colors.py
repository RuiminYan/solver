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
    
    # 将所有 ANSI_CYAN << "[xxx] " 改为 TAG_COLOR << "[xxx]" << ANSI_RESET << " "
    # 注意: [INIT] 后面有空格
    content = re.sub(
        r'ANSI_CYAN\s*<<\s*"\[([A-Z]+)\]\s*"',
        r'TAG_COLOR << "[\1]" << ANSI_RESET << " "',
        content
    )
    
    # 将所有 ANSI_BLUE << "[xxx]" 改为 TAG_COLOR << "[xxx]" << ANSI_RESET
    content = re.sub(
        r'ANSI_BLUE\s*<<\s*"\[([A-Z]+)\]"',
        r'TAG_COLOR << "[\1]" << ANSI_RESET',
        content
    )
    
    # 移除可能的双重 ANSI_RESET
    content = content.replace('<< ANSI_RESET << " " << ANSI_RESET', '<< ANSI_RESET << " "')
    content = content.replace('ANSI_RESET << ANSI_RESET', 'ANSI_RESET')
    
    if content != original:
        with open(filename, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f'Updated {filename}')
    else:
        print(f'No changes in {filename}')

print('Done!')
