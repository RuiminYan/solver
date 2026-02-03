/*
 * table_generator.cpp - 移动表和剪枝表生成工具
 */

#include "cube_common.h"
#include "move_tables.h"
#include "prune_tables.h"
#include <iostream>

int main() {
  // 打印 CUBEROOT Logo
  printCuberootLogo();

  // 初始化基础查找表（如组合数计算等，如果在 cube_common 中有的话）
  init_matrix();

  auto &mtm = MoveTableManager::getInstance();
  auto &ptm = PruneTableManager::getInstance();

  std::cout << "Checking and generating tables if necessary..." << std::endl;

  // 初始化管理器
  // 使用 Sequential 模式生成，以节省内存
  mtm.generateAllSequentially();

  ptm.generateAllSequentially();

  std::cout << "\nAll tables are ready." << std::endl;

  return 0;
}
