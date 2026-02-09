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
  std::cout << TAG_COLOR << "[MOVE]" << ANSI_RESET
            << " Initializing move tables..." << std::endl;

  generateEdgeMT();
  generateCornMT();
  generateCrossMT();
  generateEdge2MT();
  generateEdge3MT();
  generateEdge6MT();
  generateCorn2MT();
  generateCorn3MT();
}

bool MoveTableManager::loadAll() {
  if (!loadTable(mt_edge, "move_table_edge.bin"))
    return false;
  if (!loadTable(mt_corn, "move_table_corner.bin"))
    return false;
  if (!loadTable(mt_cross, "move_table_cross.bin"))
    return false;
  if (!loadTable(mt_edge2, "move_table_edges_2.bin"))
    return false;
  if (!loadTable(mt_edge3, "move_table_edges_3.bin"))
    return false;
  if (!loadTable(mt_edge6, "move_table_edges_6.bin"))
    return false;
  if (!loadTable(mt_corn2, "move_table_corners_2.bin"))
    return false;
  if (!loadTable(mt_corn3, "move_table_corners_3.bin"))
    return false;
  return true;
}

void MoveTableManager::generateAllSequentially() {
  // 1. Edge Table (Base for others)
  if (!fileExists("move_table_edge.bin")) {
    std::cout << TAG_COLOR << "[MOVE]" << ANSI_RESET
              << " Generating edge table..." << std::endl;
    mt_edge = create_edge_move_table();
    saveTable(mt_edge, "move_table_edge.bin");
  }

  // 2. Corner Table (Base for others)
  if (!fileExists("move_table_corner.bin")) {
    std::cout << "[MoveTable] Generating corner table..." << std::endl;
    mt_corn = create_corner_move_table();
    saveTable(mt_corn, "move_table_corner.bin");
  }

  // 3. Cross Table (depends on edge_table)
  if (!fileExists("move_table_cross.bin")) {
    loadEdgeMT(); // 确保依赖表已加载
    std::cout << "[MoveTable] Generating cross table..." << std::endl;
    mt_cross = create_multi_move_table2(4, 2, 12, 24 * 22 * 20 * 18, mt_edge);
    saveTable(mt_cross, "move_table_cross.bin");
  }

  // 4. Edges2 Table (depends on edge_table)
  if (!fileExists("move_table_edges_2.bin")) {
    loadEdgeMT(); // 确保依赖表已加载
    std::cout << "[MoveTable] Generating edges_2 table..." << std::endl;
    mt_edge2 = create_multi_move_table(2, 2, 12, 24 * 22, mt_edge);
    saveTable(mt_edge2, "move_table_edges_2.bin");
  }

  // 5. Edge3 Table (depends on edge_table)
  if (!fileExists("move_table_edges_3.bin")) {
    loadEdgeMT(); // 确保依赖表已加载
    std::cout << "[MoveTable] Generating edges_3 table..." << std::endl;
    mt_edge3 = create_multi_move_table(3, 2, 12, 10560, mt_edge);
    saveTable(mt_edge3, "move_table_edges_3.bin");
  }

  // 6. Edge6 Table (depends on edge_table)
  if (!fileExists("move_table_edges_6.bin")) {
    loadEdgeMT(); // 确保依赖表已加载
    std::cout << "[MoveTable] Generating edge6 table..." << std::endl;
    mt_edge6 = create_multi_move_table(6, 2, 12, 42577920, mt_edge);
    saveTable(mt_edge6, "move_table_edges_6.bin");
  }

  // 7. Corner2 Table (depends on corner_table)
  if (!fileExists("move_table_corners_2.bin")) {
    loadCornMT(); // 确保依赖表已加载
    std::cout << "[MoveTable] Generating corner2 table..." << std::endl;
    mt_corn2 = create_multi_move_table(2, 3, 8, 504, mt_corn);
    saveTable(mt_corn2, "move_table_corners_2.bin");
  }

  // 8. Corner3 Table (depends on corner_table)
  if (!fileExists("move_table_corners_3.bin")) {
    loadCornMT(); // 确保依赖表已加载
    std::cout << "[MoveTable] Generating corner3 table..." << std::endl;
    // 3个角块 (8P3 * 3^3 = 9072)
    mt_corn3 = create_multi_move_table(3, 3, 8, 9072, mt_corn);
    saveTable(mt_corn3, "move_table_corners_3.bin");
  }

  // NOTE: 不释放任何表，保持在内存中供后续 PruneTableManager 使用
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

void MoveTableManager::generateEdgeMT() {
  if (loadTable(mt_edge, "move_table_edge.bin")) {
    std::cout << "[MoveTable] Loaded edge table from file." << std::endl;
    return;
  }

  std::cout << "[MoveTable] Generating edge table..." << std::endl;
  mt_edge = create_edge_move_table();
  saveTable(mt_edge, "move_table_edge.bin");
}

void MoveTableManager::generateCornMT() {
  if (loadTable(mt_corn, "move_table_corner.bin")) {
    std::cout << "[MoveTable] Loaded corner table from file." << std::endl;
    return;
  }

  std::cout << "[MoveTable] Generating corner table..." << std::endl;
  mt_corn = create_corner_move_table();
  saveTable(mt_corn, "move_table_corner.bin");
}

void MoveTableManager::generateCrossMT() {
  if (loadTable(mt_cross, "move_table_cross.bin")) {
    std::cout << "[MoveTable] Loaded cross table from file." << std::endl;
    return;
  }

  std::cout << "[MoveTable] Generating cross table..." << std::endl;
  // Cross表：4个棱块的组合 (24*22*20*18 states)
  mt_cross = create_multi_move_table2(4, 2, 12, 24 * 22 * 20 * 18, mt_edge);
  saveTable(mt_cross, "move_table_cross.bin");
}

void MoveTableManager::generateEdge2MT() {
  if (loadTable(mt_edge2, "move_table_edges_2.bin")) {
    std::cout << "[MoveTable] Loaded edges_2 table from file." << std::endl;
    return;
  }

  std::cout << "[MoveTable] Generating edges_2 table..." << std::endl;
  // 2个棱块的组合 (24*22 states)
  mt_edge2 = create_multi_move_table(2, 2, 12, 24 * 22, mt_edge);
  saveTable(mt_edge2, "move_table_edges_2.bin");
}

void MoveTableManager::generateEdge3MT() {
  if (loadTable(mt_edge3, "move_table_edges_3.bin")) {
    std::cout << "[MoveTable] Loaded edge3 table from file." << std::endl;
    return;
  }

  std::cout << "[MoveTable] Generating edge3 table..." << std::endl;
  // 3个棱块的组合 (10560 states)
  mt_edge3 = create_multi_move_table(3, 2, 12, 10560, mt_edge);
  saveTable(mt_edge3, "move_table_edges_3.bin");
}

void MoveTableManager::generateEdge6MT() {
  if (loadTable(mt_edge6, "move_table_edges_6.bin")) {
    std::cout << "[MoveTable] Loaded edge6 table from file." << std::endl;
    return;
  }

  std::cout << "[MoveTable] Generating edge6 table..." << std::endl;
  // 6个棱块的组合 (用于巨型剪枝表)
  mt_edge6 = create_multi_move_table(6, 2, 12, 42577920, mt_edge);
  saveTable(mt_edge6, "move_table_edges_6.bin");
}

void MoveTableManager::generateCorn2MT() {
  if (loadTable(mt_corn2, "move_table_corners_2.bin")) {
    std::cout << "[MoveTable] Loaded corner2 table from file." << std::endl;
    return;
  }

  std::cout << "[MoveTable] Generating corner2 table..." << std::endl;
  // 2个角块的组合 (504 states)
  mt_corn2 = create_multi_move_table(2, 3, 8, 504, mt_corn);
  saveTable(mt_corn2, "move_table_corners_2.bin");
}

void MoveTableManager::generateCorn3MT() {
  if (loadTable(mt_corn3, "move_table_corners_3.bin")) {
    std::cout << "[MoveTable] Loaded corner3 table from file." << std::endl;
    return;
  }

  std::cout << "[MoveTable] Generating corner3 table..." << std::endl;
  // 3个角块 (9072 states)
  mt_corn3 = create_multi_move_table(3, 3, 8, 9072, mt_corn);
  saveTable(mt_corn3, "move_table_corners_3.bin");
}

// 加载 EOCross 专用移动表
// NOTE: 这两个表仅被 eo_cross_analyzer 使用
bool MoveTableManager::loadEOCrossMTs() {
  // 1. EO Alt 移动表
  if (mt_eo_alt.empty()) {
    if (!loadTable(mt_eo_alt, "move_table_eo_12_alt.bin")) {
      std::cout << "[MoveTable] Generating EO Alt table..." << std::endl;
      mt_eo_alt = create_eo_move_table2();
      saveTable(mt_eo_alt, "move_table_eo_12_alt.bin");
    }
  }

  // 2. EP4 移动表（需要先加载 EP1 作为基础）
  if (mt_ep4.empty()) {
    if (!loadTable(mt_ep4, "move_table_ep_4.bin")) {
      std::cout << "[MoveTable] Generating EP4 table..." << std::endl;
      // 使用Manager加载的EP1表
      loadEP1MT();
      // 生成 EP4 表
      mt_ep4 = create_multi_move_table(4, 1, 12, 12 * 11 * 10 * 9, mt_ep1);
      saveTable(mt_ep4, "move_table_ep_4.bin");
    }
  }

  return true;
}

// 加载 EO 移动表 (move_table_eo_12.bin)
bool MoveTableManager::loadEOMT() {
  if (!mt_eo.empty())
    return true;
  if (!loadTable(mt_eo, "move_table_eo_12.bin")) {
    std::cout << "[MoveTable] Generating EO table..." << std::endl;
    mt_eo = create_eo_move_table();
    saveTable(mt_eo, "move_table_eo_12.bin");
  }
  return true;
}

// 加载 EP1 移动表 (move_table_ep_1.bin)
bool MoveTableManager::loadEP1MT() {
  if (!mt_ep1.empty())
    return true;
  if (!loadTable(mt_ep1, "move_table_ep_1.bin")) {
    std::cout << "[MoveTable] Generating EP1 table..." << std::endl;
    mt_ep1 = create_ep_move_table();
    saveTable(mt_ep1, "move_table_ep_1.bin");
  }
  return true;
}

// --- 基础移动表生成函数 ---
std::vector<int> create_edge_move_table() {
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

std::vector<int> create_corner_move_table() {
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

std::vector<int> create_ep_move_table() {
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

std::vector<int> create_eo_move_table() {
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

std::vector<int> create_eo_move_table2() {
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