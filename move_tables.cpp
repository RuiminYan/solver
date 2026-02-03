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

  generateEdgeTable();
  generateCornerTable();
  generateCrossTable();
  generateEdges2Table();
  generateEdge3Table();
  generateEdge6Table();
  generateCorner2Table();
  generateCorner3Table();

  std::cout << TAG_COLOR << "[MOVE]" << ANSI_RESET
            << " All move tables initialized." << std::endl;
}

bool MoveTableManager::loadAll() {
  std::cout << TAG_COLOR << "[MOVE]" << ANSI_RESET << " Loading move tables..."
            << std::endl;
  if (!loadTable(edge_table, "move_table_edge.bin"))
    return false;
  if (!loadTable(corner_table, "move_table_corner.bin"))
    return false;
  if (!loadTable(cross_table, "move_table_cross.bin"))
    return false;
  if (!loadTable(edges_2_table, "move_table_edges_2.bin"))
    return false;
  if (!loadTable(edge3_table, "move_table_edges_3.bin"))
    return false;
  if (!loadTable(edge6_table, "move_table_edges_6.bin"))
    return false;
  if (!loadTable(corner2_table, "move_table_corners_2.bin"))
    return false;
  if (!loadTable(corner3_table, "move_table_corners_3.bin"))
    return false;
  return true;
}

void MoveTableManager::generateAllSequentially() {
  std::cout << TAG_COLOR << "[MOVE]" << ANSI_RESET
            << " Generating tables sequentially to save memory..." << std::endl;

  // 1. Edge Table (Base for others)
  if (!loadTable(edge_table, "move_table_edge.bin")) {
    std::cout << TAG_COLOR << "[MOVE]" << ANSI_RESET
              << " Generating edge table..." << std::endl;
    edge_table = create_edge_move_table();
    saveTable(edge_table, "move_table_edge.bin");
  }
  // Edge table is needed for Cross, Edges2, Edge6. Keep it in memory.

  // 2. Corner Table (Base for others)
  if (!loadTable(corner_table, "move_table_corner.bin")) {
    std::cout << "[MoveTable] Generating corner table..." << std::endl;
    corner_table = create_corner_move_table();
    saveTable(corner_table, "move_table_corner.bin");
  }
  // Corner table is needed for Corner2. Keep it in memory.

  // 3. Cross Table
  if (!loadTable(cross_table, "move_table_cross.bin")) {
    std::cout << "[MoveTable] Generating cross table..." << std::endl;
    cross_table =
        create_multi_move_table2(4, 2, 12, 24 * 22 * 20 * 18, edge_table);
    saveTable(cross_table, "move_table_cross.bin");
  }
  std::vector<int>().swap(cross_table); // Release

  // 4. Edges2 Table
  if (!loadTable(edges_2_table, "move_table_edges_2.bin")) {
    std::cout << "[MoveTable] Generating edges_2 table..." << std::endl;
    edges_2_table = create_multi_move_table(2, 2, 12, 24 * 22, edge_table);
    saveTable(edges_2_table, "move_table_edges_2.bin");
  }
  std::vector<int>().swap(edges_2_table); // Release

  // 5. Edge3 Table
  if (!loadTable(edge3_table, "move_table_edges_3.bin")) {
    std::cout << "[MoveTable] Generating edges_3 table..." << std::endl;
    edge3_table = create_multi_move_table(3, 2, 12, 10560, edge_table);
    saveTable(edge3_table, "move_table_edges_3.bin");
  }
  std::vector<int>().swap(edge3_table); // Release

  // 6. Edge6 Table
  if (!loadTable(edge6_table, "move_table_edges_6.bin")) {
    std::cout << "[MoveTable] Generating edge6 table..." << std::endl;
    edge6_table = create_multi_move_table(6, 2, 12, 42577920, edge_table);
    saveTable(edge6_table, "move_table_edges_6.bin");
  }
  std::vector<int>().swap(edge6_table); // Release

  // 7. Corner2 Table
  if (!loadTable(corner2_table, "move_table_corners_2.bin")) {
    std::cout << "[MoveTable] Generating corner2 table..." << std::endl;
    corner2_table = create_multi_move_table(2, 3, 8, 504, corner_table);
    saveTable(corner2_table, "move_table_corners_2.bin");
  }
  std::vector<int>().swap(corner2_table); // Release

  // 8. Corner3 Table
  if (!loadTable(corner3_table, "move_table_corners_3.bin")) {
    std::cout << "[MoveTable] Generating corner3 table..." << std::endl;
    // 3个角块 (8P3 * 3^3 = 9072)
    corner3_table = create_multi_move_table(3, 3, 8, 9072, corner_table);
    saveTable(corner3_table, "move_table_corners_3.bin");
  }
  std::vector<int>().swap(corner3_table); // Release

  // Finally release base tables
  std::vector<int>().swap(edge_table);
  std::vector<int>().swap(corner_table);

  std::cout << "[MoveTable] Sequential generation complete." << std::endl;
}

bool MoveTableManager::loadTable(std::vector<int> &table,
                                 const std::string &filename) {
  // NOTE: 移动表不显示进度条（第三个参数false）
  return load_vector_chunked(table, filename, false);
}

void MoveTableManager::saveTable(const std::vector<int> &table,
                                 const std::string &filename) {
  save_vector_chunked(table, filename);
}

void MoveTableManager::generateEdgeTable() {
  if (loadTable(edge_table, "move_table_edge.bin")) {
    std::cout << "[MoveTable] Loaded edge table from file." << std::endl;
    return;
  }

  std::cout << "[MoveTable] Generating edge table..." << std::endl;
  edge_table = create_edge_move_table();
  saveTable(edge_table, "move_table_edge.bin");
}

void MoveTableManager::generateCornerTable() {
  if (loadTable(corner_table, "move_table_corner.bin")) {
    std::cout << "[MoveTable] Loaded corner table from file." << std::endl;
    return;
  }

  std::cout << "[MoveTable] Generating corner table..." << std::endl;
  corner_table = create_corner_move_table();
  saveTable(corner_table, "move_table_corner.bin");
}

void MoveTableManager::generateCrossTable() {
  if (loadTable(cross_table, "move_table_cross.bin")) {
    std::cout << "[MoveTable] Loaded cross table from file." << std::endl;
    return;
  }

  std::cout << "[MoveTable] Generating cross table..." << std::endl;
  // Cross表：4个棱块的组合 (24*22*20*18 states)
  cross_table =
      create_multi_move_table2(4, 2, 12, 24 * 22 * 20 * 18, edge_table);
  saveTable(cross_table, "move_table_cross.bin");
}

void MoveTableManager::generateEdges2Table() {
  if (loadTable(edges_2_table, "move_table_edges_2.bin")) {
    std::cout << "[MoveTable] Loaded edges_2 table from file." << std::endl;
    return;
  }

  std::cout << "[MoveTable] Generating edges_2 table..." << std::endl;
  // 2个棱块的组合 (24*22 states)
  edges_2_table = create_multi_move_table(2, 2, 12, 24 * 22, edge_table);
  saveTable(edges_2_table, "move_table_edges_2.bin");
}

void MoveTableManager::generateEdge3Table() {
  if (loadTable(edge3_table, "move_table_edges_3.bin")) {
    std::cout << "[MoveTable] Loaded edge3 table from file." << std::endl;
    return;
  }

  std::cout << "[MoveTable] Generating edge3 table..." << std::endl;
  // 3个棱块的组合 (10560 states)
  edge3_table = create_multi_move_table(3, 2, 12, 10560, edge_table);
  saveTable(edge3_table, "move_table_edges_3.bin");
}

void MoveTableManager::generateEdge6Table() {
  if (loadTable(edge6_table, "move_table_edges_6.bin")) {
    std::cout << "[MoveTable] Loaded edge6 table from file." << std::endl;
    return;
  }

  std::cout << "[MoveTable] Generating edge6 table..." << std::endl;
  // 6个棱块的组合 (用于巨型剪枝表)
  edge6_table = create_multi_move_table(6, 2, 12, 42577920, edge_table);
  saveTable(edge6_table, "move_table_edges_6.bin");
}

void MoveTableManager::generateCorner2Table() {
  if (loadTable(corner2_table, "move_table_corners_2.bin")) {
    std::cout << "[MoveTable] Loaded corner2 table from file." << std::endl;
    return;
  }

  std::cout << "[MoveTable] Generating corner2 table..." << std::endl;
  // 2个角块的组合 (504 states)
  corner2_table = create_multi_move_table(2, 3, 8, 504, corner_table);
  saveTable(corner2_table, "move_table_corners_2.bin");
}

void MoveTableManager::generateCorner3Table() {
  if (loadTable(corner3_table, "move_table_corners_3.bin")) {
    std::cout << "[MoveTable] Loaded corner3 table from file." << std::endl;
    return;
  }

  std::cout << "[MoveTable] Generating corner3 table..." << std::endl;
  // 3个角块 (9072 states)
  corner3_table = create_multi_move_table(3, 3, 8, 9072, corner_table);
  saveTable(corner3_table, "move_table_corners_3.bin");
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