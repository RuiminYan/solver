/*
 * move_tables.cpp - 移动表实现
 */

#include "move_tables.h"

MoveTableManager *MoveTableManager::instance = nullptr;

MoveTableManager &MoveTableManager::getInstance() {
  if (instance == nullptr) {
    instance = new MoveTableManager();
  }
  return *instance;
}

void MoveTableManager::initialize() {
  std::cout << "Initializing move tables..." << std::endl;

  generateMTEdge();
  generateMTCorn();
  generateMTCross();
  generateMTEdge2();
  generateMTEdge3();
  generateMTEdge6();
  generateMTCorn2();
  generateMTCorn3();
}

bool MoveTableManager::loadAll() {
  if (!loadTable(mt_edge, "mt_edge.bin"))
    return false;
  if (!loadTable(mt_corn, "mt_corn.bin"))
    return false;
  if (!loadTable(mt_cross, "mt_cross.bin"))
    return false;
  if (!loadTable(mt_edge2, "mt_edge2.bin"))
    return false;
  if (!loadTable(mt_edge3, "mt_edge3.bin"))
    return false;
  if (!loadTable(mt_edge6, "mt_edge6.bin"))
    return false;
  if (!loadTable(mt_corn2, "mt_corn2.bin"))
    return false;
  if (!loadTable(mt_corn3, "mt_corn3.bin"))
    return false;
  return true;
}

void MoveTableManager::generateAllSequentially() {
  // NOTE: 每张表生成+保存后立即释放，避免内存累积导致 OOM
  // mt_edge/mt_corn 是基础依赖表且极小(<100KB)，在最后释放

  // 1. Edge Table (Base for others)
  if (!fileExists("mt_edge.bin")) {
    GenerationTimer timer;
    std::cout << "Generating mt_edge.bin..." << std::endl;
    mt_edge = create_edge_mt();
    saveTable(mt_edge, "mt_edge.bin");
    timer.printElapsed("mt_edge.bin");
  }

  // 2. Corner Table (Base for others)
  if (!fileExists("mt_corn.bin")) {
    GenerationTimer timer;
    std::cout << "Generating mt_corn.bin..." << std::endl;
    mt_corn = create_corn_mt();
    saveTable(mt_corn, "mt_corn.bin");
    timer.printElapsed("mt_corn.bin");
  }

  // 3. Cross Table (depends on edge_table, 17.4MB)
  if (!fileExists("mt_cross.bin")) {
    loadMTEdge();
    GenerationTimer timer;
    std::cout << "Generating mt_cross.bin..." << std::endl;
    mt_cross = create_multi_move_table2(4, 2, 12, 24 * 22 * 20 * 18, mt_edge);
    saveTable(mt_cross, "mt_cross.bin");
    timer.printElapsed("mt_cross.bin");
    releaseMTCross();
  }

  // 4. Edges2 Table (depends on edge_table, <100KB)
  if (!fileExists("mt_edge2.bin")) {
    loadMTEdge();
    GenerationTimer timer;
    std::cout << "Generating mt_edge2.bin..." << std::endl;
    mt_edge2 = create_multi_move_table(2, 2, 12, 24 * 22, mt_edge);
    saveTable(mt_edge2, "mt_edge2.bin");
    timer.printElapsed("mt_edge2.bin");
    releaseMTEdge2();
  }

  // 5. Edge3 Table (depends on edge_table, 742KB)
  if (!fileExists("mt_edge3.bin")) {
    loadMTEdge();
    GenerationTimer timer;
    std::cout << "Generating mt_edge3.bin..." << std::endl;
    mt_edge3 = create_multi_move_table(3, 2, 12, 10560, mt_edge);
    saveTable(mt_edge3, "mt_edge3.bin");
    timer.printElapsed("mt_edge3.bin");
    releaseMTEdge3();
  }

  // 6. Edge6 Table (depends on edge_table, 2.85GB)
  if (!fileExists("mt_edge6.bin")) {
    loadMTEdge();
    GenerationTimer timer;
    std::cout << "Generating mt_edge6.bin..." << std::endl;
    mt_edge6 = create_multi_move_table(6, 2, 12, 42577920, mt_edge);
    saveTable(mt_edge6, "mt_edge6.bin");
    timer.printElapsed("mt_edge6.bin");
    releaseMTEdge6();
  }

  // 7. Corner2 Table (depends on corner_table, <100KB)
  if (!fileExists("mt_corn2.bin")) {
    loadMTCorn();
    GenerationTimer timer;
    std::cout << "Generating mt_corn2.bin..." << std::endl;
    mt_corn2 = create_multi_move_table(2, 3, 8, 504, mt_corn);
    saveTable(mt_corn2, "mt_corn2.bin");
    timer.printElapsed("mt_corn2.bin");
    releaseMTCorn2();
  }

  // 8. Corner3 Table (depends on corner_table, 637KB)
  if (!fileExists("mt_corn3.bin")) {
    loadMTCorn();
    GenerationTimer timer;
    std::cout << "Generating mt_corn3.bin..." << std::endl;
    // 3个角块 (8P3 * 3^3 = 9072)
    mt_corn3 = create_multi_move_table(3, 3, 8, 9072, mt_corn);
    saveTable(mt_corn3, "mt_corn3.bin");
    timer.printElapsed("mt_corn3.bin");
    releaseMTCorn3();
  }

  // 释放基础依赖表
  releaseMTEdge();
  releaseMTCorn();
}

bool MoveTableManager::loadTable(std::vector<int> &table,
                                 const std::string &filename) {
  // NOTE: 移动表不显示进度条（第三个参数false）
  if (load_vector_chunked(table, filename, false)) {
    // 计算并打印表大小
    size_t size_bytes = table.size() * sizeof(int);
    std::cout << TAG_COLOR << "[LOAD]" << ANSI_RESET << " ("
              << formatFileSize(size_bytes) << ") " << filename << std::endl;
    return true;
  }
  return false;
}

void MoveTableManager::saveTable(const std::vector<int> &table,
                                 const std::string &filename) {
  save_vector_chunked(table, filename);
}

void MoveTableManager::generateMTEdge() {
  if (loadTable(mt_edge, "mt_edge.bin")) {
    std::cout << "Loaded edge table from file." << std::endl;
    return;
  }

  std::cout << "Generating mt_edge.bin..." << std::endl;
  mt_edge = create_edge_mt();
  saveTable(mt_edge, "mt_edge.bin");
}

void MoveTableManager::generateMTCorn() {
  if (loadTable(mt_corn, "mt_corn.bin")) {
    std::cout << "Loaded corner table from file." << std::endl;
    return;
  }

  std::cout << "Generating mt_corn.bin..." << std::endl;
  mt_corn = create_corn_mt();
  saveTable(mt_corn, "mt_corn.bin");
}

void MoveTableManager::generateMTCross() {
  if (loadTable(mt_cross, "mt_cross.bin")) {
    std::cout << "Loaded cross table from file." << std::endl;
    return;
  }

  std::cout << "Generating mt_cross.bin..." << std::endl;
  // Cross表：4个棱块的组合 (24*22*20*18 states)
  mt_cross = create_multi_move_table2(4, 2, 12, 24 * 22 * 20 * 18, mt_edge);
  saveTable(mt_cross, "mt_cross.bin");
}

void MoveTableManager::generateMTEdge2() {
  if (loadTable(mt_edge2, "mt_edge2.bin")) {
    std::cout << "Loaded edges_2 table from file." << std::endl;
    return;
  }

  std::cout << "Generating mt_edge2.bin..." << std::endl;
  // 2个棱块的组合 (24*22 states)
  mt_edge2 = create_multi_move_table(2, 2, 12, 24 * 22, mt_edge);
  saveTable(mt_edge2, "mt_edge2.bin");
}

void MoveTableManager::generateMTEdge3() {
  if (loadTable(mt_edge3, "mt_edge3.bin")) {
    std::cout << "Loaded edge3 table from file." << std::endl;
    return;
  }

  std::cout << "Generating mt_edge3.bin..." << std::endl;
  // 3个棱块的组合 (10560 states)
  mt_edge3 = create_multi_move_table(3, 2, 12, 10560, mt_edge);
  saveTable(mt_edge3, "mt_edge3.bin");
}

void MoveTableManager::generateMTEdge6() {
  if (loadTable(mt_edge6, "mt_edge6.bin")) {
    std::cout << "Loaded edge6 table from file." << std::endl;
    return;
  }

  std::cout << "Generating mt_edge6.bin..." << std::endl;
  // 6个棱块的组合 (用于巨型剪枝表)
  mt_edge6 = create_multi_move_table(6, 2, 12, 42577920, mt_edge);
  saveTable(mt_edge6, "mt_edge6.bin");
}

void MoveTableManager::generateMTCorn2() {
  if (loadTable(mt_corn2, "mt_corn2.bin")) {
    std::cout << "Loaded corner2 table from file." << std::endl;
    return;
  }

  std::cout << "Generating mt_corn2.bin..." << std::endl;
  // 2个角块的组合 (504 states)
  mt_corn2 = create_multi_move_table(2, 3, 8, 504, mt_corn);
  saveTable(mt_corn2, "mt_corn2.bin");
}

void MoveTableManager::generateMTCorn3() {
  if (loadTable(mt_corn3, "mt_corn3.bin")) {
    std::cout << "Loaded corner3 table from file." << std::endl;
    return;
  }

  std::cout << "Generating mt_corn3.bin..." << std::endl;
  // 3个角块 (9072 states)
  mt_corn3 = create_multi_move_table(3, 3, 8, 9072, mt_corn);
  saveTable(mt_corn3, "mt_corn3.bin");
}

// 加载 EOCross 专用移动表
// NOTE: 这两个表仅被 eo_cross_analyzer 使用
bool MoveTableManager::loadMTEOCross() {
  // 1. EO Alt 移动表
  if (mt_eo_alt.empty()) {
    if (!loadTable(mt_eo_alt, "mt_eo_alt.bin")) {
      GenerationTimer timer;
      std::cout << "Generating mt_eo_alt.bin..." << std::endl;
      mt_eo_alt = create_eo_alt_mt();
      saveTable(mt_eo_alt, "mt_eo_alt.bin");
      timer.printElapsed("mt_eo_alt.bin");
    }
  }

  // 2. EP4 移动表（需要先加载 EP1 作为基础）
  if (mt_ep4.empty()) {
    if (!loadTable(mt_ep4, "mt_ep4.bin")) {
      // 使用Manager加载的EP1表
      loadMTEP1();
      GenerationTimer timer;
      std::cout << "Generating mt_ep4.bin..." << std::endl;
      // 生成 EP4 表
      mt_ep4 = create_multi_move_table(4, 1, 12, 12 * 11 * 10 * 9, mt_ep1);
      saveTable(mt_ep4, "mt_ep4.bin");
      timer.printElapsed("mt_ep4.bin");
    }
  }

  return true;
}

// 加载 EO 移动表 (mt_eo.bin)
bool MoveTableManager::loadMTEO() {
  if (!mt_eo.empty())
    return true;
  if (!loadTable(mt_eo, "mt_eo.bin")) {
    GenerationTimer timer;
    std::cout << "Generating mt_eo.bin..." << std::endl;
    mt_eo = create_eo_mt();
    saveTable(mt_eo, "mt_eo.bin");
    timer.printElapsed("mt_eo.bin");
  }
  return true;
}

// 加载 EP1 移动表 (move_table_ep_1.bin)
bool MoveTableManager::loadMTEP1() {
  if (!mt_ep1.empty())
    return true;
  if (!loadTable(mt_ep1, "mt_ep1.bin")) {
    GenerationTimer timer;
    std::cout << "Generating mt_ep1.bin..." << std::endl;
    mt_ep1 = create_ep_mt();
    saveTable(mt_ep1, "mt_ep1.bin");
    timer.printElapsed("mt_ep1.bin");
  }
  return true;
}

// --- 基础移动表生成函数 ---
std::vector<int> create_edge_mt() {
  std::vector<int> mt(24 * 18, -1);
  for (int i = 0; i < 24; ++i) {
    State s;
    s.ep = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    s.eo = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    s.ep[i / 2] = i / 2;
    s.eo[i / 2] = i % 2;
    for (int j = 0; j < 18; ++j) {
      State ns = s.apply_move_edge(moves_map[move_names[j]], i / 2);
      auto it = std::find(ns.ep.begin(), ns.ep.end(), i / 2);
      int idx = std::distance(ns.ep.begin(), it);
      mt[18 * i + j] = 2 * idx + ns.eo[idx];
    }
  }
  return mt;
}

std::vector<int> create_corn_mt() {
  std::vector<int> mt(24 * 18, -1);
  for (int i = 0; i < 24; ++i) {
    State s;
    s.cp = {0, 1, 2, 3, 4, 5, 6, 7};
    s.co = {0, 0, 0, 0, 0, 0, 0, 0};
    s.cp[i / 3] = i / 3;
    s.co[i / 3] = i % 3;
    for (int j = 0; j < 18; ++j) {
      State ns = s.apply_move_corner(moves_map[move_names[j]], i / 3);
      auto it = std::find(ns.cp.begin(), ns.cp.end(), i / 3);
      int idx = std::distance(ns.cp.begin(), it);
      mt[18 * i + j] = 3 * idx + ns.co[idx];
    }
  }
  return mt;
}

std::vector<int> create_ep_mt() {
  std::vector<int> mt(12 * 18, -1);
  for (int i = 0; i < 12; ++i) {
    State s;
    s.ep.assign(12, -1);
    s.eo.assign(12, -1);
    s.ep[i] = i;
    s.eo[i] = 0;
    for (int j = 0; j < 18; ++j) {
      State ns = s.apply_move_edge(moves_map[move_names[j]], i);
      auto it = std::find(ns.ep.begin(), ns.ep.end(), i);
      mt[18 * i + j] = std::distance(ns.ep.begin(), it);
    }
  }
  return mt;
}

std::vector<int> create_eo_mt() {
  std::vector<int> mt(2048 * 18, -1);
  for (int i = 0; i < 2048; ++i) {
    std::vector<int> eo(12, 0);
    index_to_o(eo, i, 2, 12);
    State s({0, 1, 2, 3, 4, 5, 6, 7}, {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, eo);
    for (int j = 0; j < 18; ++j) {
      State ns = s.apply_move(moves_map[move_names[j]]);
      mt[18 * i + j] = 18 * o_to_index(ns.eo, 2, 12);
    }
  }
  return mt;
}

std::vector<int> create_eo_alt_mt() {
  std::vector<int> mt(2048 * 18, -1);
  for (int i = 0; i < 2048; ++i) {
    std::vector<int> eo(12, 0);
    index_to_o(eo, i, 2, 12);
    State s({0, 1, 2, 3, 4, 5, 6, 7}, {0, 0, 0, 0, 0, 0, 0, 0},
            {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, eo);
    for (int j = 0; j < 18; ++j) {
      State ns = s.apply_move(moves_map[move_names[j]]);
      mt[18 * i + j] = o_to_index(ns.eo, 2, 12);
    }
  }
  return mt;
}