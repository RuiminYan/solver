/*
 * table_generator.cpp - 移动表和剪枝表生成工具
 */

#include "cube_common.h"
#include "move_tables.h"
#include "prune_tables.h"
#include <iostream>

int main() {
  // 设置控制台颜色（Windows）
  system("color 0A");

  // 初始化基础查找表（如组合数计算等，如果在 cube_common 中有的话）
  init_matrix();

  auto &mtm = MoveTableManager::getInstance();
  auto &ptm = PruneTableManager::getInstance();

  std::cout << "=== Table Generator ===" << std::endl;
  std::cout << "Checking and generating tables if necessary..." << std::endl;

  // 初始化管理器
  // 使用 Sequential 模式生成，以节省内存
  mtm.generateAllSequentially();

  ptm.generateAllSequentially();

  std::cout << "\nAll tables are ready." << std::endl;
  std::cout << "You can now run std_analyzer.exe" << std::endl;

  return 0;
}
