/*
 * prune_tables.cpp - 剪枝表实现
 */

#include "prune_tables.h"
#include "move_tables.h"

#ifdef _WIN32
#include <windows.h>
#endif

PruneTableManager *PruneTableManager::instance = nullptr;

PruneTableManager &PruneTableManager::getInstance() {
  if (instance == nullptr) {
    instance = new PruneTableManager();
  }
  return *instance;
}

void PruneTableManager::initialize() {
  std::cout << "[PruneTable] Initializing prune tables..." << std::endl;

  generateCrossPrune();
  generateCrossC4Prune();
  generatePairC4E0Prune();
  generateXCrossC4E0Prune();
  generateHugeNeighborPrune();
  generateHugeDiagonalPrune();

  std::cout << "[PruneTable] All prune tables initialized." << std::endl;
}

unsigned char *PruneTableManager::loadTableMMap(const std::string &filename) {
#ifdef _WIN32
  std::cout << "[PruneTable] MMap loading " << filename << "..." << std::endl;
  HANDLE hFile = CreateFile(filename.c_str(), GENERIC_READ, FILE_SHARE_READ,
                            NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
  if (hFile == INVALID_HANDLE_VALUE) {
    std::cout << "  Failed to open file." << std::endl;
    return nullptr;
  }

  HANDLE hMap = CreateFileMapping(hFile, NULL, PAGE_READONLY, 0, 0, NULL);
  if (hMap == NULL) {
    std::cout << "  Failed to create file mapping." << std::endl;
    CloseHandle(hFile);
    return nullptr;
  }

  unsigned char *ptr =
      (unsigned char *)MapViewOfFile(hMap, FILE_MAP_READ, 0, 0, 0);
  if (ptr == NULL) {
    std::cout << "  Failed to map view." << std::endl;
    CloseHandle(hMap);
    CloseHandle(hFile);
    return nullptr;
  }

  // Leak handles intentionally for process lifetime persistence
  return ptr;
#else
  std::cout << "MMap not supported on non-Windows" << std::endl;
  return nullptr;
#endif
}

bool PruneTableManager::loadAll() {
  std::cout << "[PruneTable] Loading prune tables..." << std::endl;
  if (!loadTable(cross_prune, "prune_table_cross.bin"))
    return false;
  if (!loadTable(cross_c4_prune, "prune_table_cross_C4.bin"))
    return false;
  if (!loadTable(pair_c4_e0_prune, "prune_table_C4_E0.bin"))
    return false;
  if (!loadTable(xcross_c4_e0_prune, "prune_table_cross_C4_E0.bin"))
    return false;
  if (!loadTable(huge_neighbor_prune, "prune_table_cross_C4_E0_C5_E1.bin"))
    return false;
  if (ENABLE_DIAGONAL_TABLE) {
    if (!loadTable(huge_diagonal_prune, "prune_table_cross_C4_E0_C6_E2.bin"))
      return false;
  }
  return true;
}

bool PruneTableManager::loadPseudoTables() {
  std::cout << "[PruneTable] Loading pseudo tables only..." << std::endl;
  if (!loadTable(pseudo_cross_prune, "prune_table_pseudo_cross.bin"))
    return false;
  for (int i = 0; i < 4; ++i) {
    std::string fn =
        "prune_table_pseudo_cross_C4_E" + std::to_string(i) + ".bin";
    if (!loadTable(pseudo_cross_base_prune[i], fn))
      return false;
  }
  if (!loadTable(pseudo_cross_E0_E2_prune,
                 "prune_table_pseudo_cross_E0_E2.bin")) {
    std::cout << "Warning: prune_table_pseudo_cross_E0_E2.bin not found."
              << std::endl;
  }
  // 新增邻棱表加载
  if (!loadTable(pseudo_cross_E0_E1_prune,
                 "prune_table_pseudo_cross_E0_E1.bin")) {
    std::cout << "Warning: prune_table_pseudo_cross_E0_E1.bin not found."
              << std::endl;
  }

  // Edge3 Tables Loading
  if (!loadTable(pseudo_cross_E0_E1_E2_prune,
                 "prune_table_pseudo_cross_E0_E1_E2.bin")) {
    std::cout << "Warning: prune_table_pseudo_cross_E0_E1_E2.bin not found."
              << std::endl;
  }

  // 对角表加载
  if (!loadTable(pseudo_cross_C4_C6_prune,
                 "prune_table_pseudo_cross_C4_C6.bin")) {
    std::cout << "Warning: prune_table_pseudo_cross_C4_C6.bin not found."
              << std::endl;
  }
  // 邻角表加载
  if (!loadTable(pseudo_cross_C4_C5_prune,
                 "prune_table_pseudo_cross_C4_C5.bin")) {
    std::cout << "Warning: prune_table_pseudo_cross_C4_C5.bin not found."
              << std::endl;
  }

  // Corner3 Tables Loading
  if (!loadTable(pseudo_cross_C4_C5_C6_prune,
                 "prune_table_pseudo_cross_C4_C5_C6.bin")) {
    std::cout << "Warning: prune_table_pseudo_cross_C4_C5_C6.bin not found."
              << std::endl;
  }

  return true;
}

// --- 前向声明: Pseudo 表生成函数 ---
void create_prune_table_pseudo_cross_corner(
    int index2, int depth, const std::vector<int> &table1,
    const std::vector<int> &table2, std::vector<unsigned char> &prune_table,
    const std::string &log_prefix);
void create_prune_table_pseudo_xcross(int index3, int index2, int depth,
                                      const std::vector<int> &table1,
                                      const std::vector<int> &table2,
                                      std::vector<unsigned char> &prune_table,
                                      const std::string &log_prefix);
void create_prune_table_pseudo_pair(int index1, int index2, int size1,
                                    int size2, int depth,
                                    const std::vector<int> &table1,
                                    const std::vector<int> &table2,
                                    std::vector<unsigned char> &prune_table,
                                    const std::string &log_prefix);

void PruneTableManager::generateAllSequentially() {
  std::cout << "[PruneTable] Generating tables sequentially to save memory..."
            << std::endl;
  auto &mtm = MoveTableManager::getInstance();

  // 1. Cross Prune (Needs Edges2)
  if (!loadTable(cross_prune, "prune_table_cross.bin")) {
    std::cout << "[PruneTable] Generating cross prune table..." << std::endl;
    mtm.loadEdges2Table();
    generateCrossPrune();
    mtm.releaseEdges2Table();
  }
  std::vector<unsigned char>().swap(cross_prune);

  // 2. Cross C4 Prune (Needs Cross, Corner)
  if (!loadTable(cross_c4_prune, "prune_table_cross_C4.bin")) {
    mtm.loadCrossTable();
    mtm.loadCornerTable();
    generateCrossC4Prune();
    mtm.releaseCrossTable();
    mtm.releaseCornerTable();
  }
  std::vector<unsigned char>().swap(cross_c4_prune);

  // 3. Pair C4 E0 Prune (Needs Edge, Corner)
  if (!loadTable(pair_c4_e0_prune, "prune_table_C4_E0.bin")) {
    mtm.loadEdgeTable();
    mtm.loadCornerTable();
    generatePairC4E0Prune();
    mtm.releaseEdgeTable();
    mtm.releaseCornerTable();
  }
  std::vector<unsigned char>().swap(pair_c4_e0_prune);

  // 4. XCross C4 E0 Prune (Needs Cross, Corner, Edge)
  if (!loadTable(xcross_c4_e0_prune, "prune_table_cross_C4_E0.bin")) {
    mtm.loadCrossTable();
    mtm.loadCornerTable();
    mtm.loadEdgeTable();
    generateXCrossC4E0Prune();
    mtm.releaseCrossTable();
    mtm.releaseCornerTable();
    mtm.releaseEdgeTable();
  }
  std::vector<unsigned char>().swap(xcross_c4_e0_prune);

  // 5. Huge Neighbor (Needs Edge6, Corner2)
  if (!loadTable(huge_neighbor_prune, "prune_table_cross_C4_E0_C5_E1.bin")) {
    mtm.loadEdge6Table();
    mtm.loadCorner2Table();
    generateHugeNeighborPrune();
    mtm.releaseEdge6Table();
    mtm.releaseCorner2Table();
  }
  std::vector<unsigned char>().swap(huge_neighbor_prune);

  // 6. Huge Diagonal (Needs Edge6, Corner2)
  if (ENABLE_DIAGONAL_TABLE) {
    if (!loadTable(huge_diagonal_prune, "prune_table_cross_C4_E0_C6_E2.bin")) {
      mtm.loadEdge6Table();
      mtm.loadCorner2Table();
      generateHugeDiagonalPrune();
      mtm.releaseEdge6Table();
      mtm.releaseCorner2Table();
    }
    std::vector<unsigned char>().swap(huge_diagonal_prune);
  }

  // 7. Pseudo Cross Prune
  if (!loadTable(pseudo_cross_prune, "prune_table_pseudo_cross.bin")) {
    mtm.loadEdges2Table();
    generatePseudoCrossPrune();
    mtm.releaseEdges2Table();
  }
  std::vector<unsigned char>().swap(pseudo_cross_prune);

  // 8. Pseudo XCross Prunes
  for (int i = 0; i < 4; ++i) {
    std::string fn =
        "prune_table_pseudo_cross_C4_E" + std::to_string(i) + ".bin";
    if (!loadTable(pseudo_cross_base_prune[i], fn)) {
      mtm.loadCrossTable();
      mtm.loadCornerTable();
      mtm.loadEdgeTable();
      generatePseudoCrossBasePrune(i);
      mtm.releaseCrossTable();
      mtm.releaseCornerTable();
      mtm.releaseEdgeTable();
    }
    std::vector<unsigned char>().swap(pseudo_cross_base_prune[i]);
  }

  // 9. Pseudo Cross + E0,E2 Prune
  if (!loadTable(pseudo_cross_E0_E2_prune,
                 "prune_table_pseudo_cross_E0_E2.bin")) {
    mtm.loadCrossTable();
    mtm.loadEdges2Table();
    generatePseudoCrossE0E2Prune();
    mtm.releaseCrossTable();
    mtm.releaseEdges2Table();
  }
  std::vector<unsigned char>().swap(pseudo_cross_E0_E2_prune);

  // 11. Pseudo Cross + E0,E1 Prune
  if (!loadTable(pseudo_cross_E0_E1_prune,
                 "prune_table_pseudo_cross_E0_E1.bin")) {
    mtm.loadCrossTable();
    mtm.loadEdges2Table();
    generatePseudoCrossE0E1Prune();
    mtm.releaseCrossTable();
    mtm.releaseEdges2Table();
  }
  std::vector<unsigned char>().swap(pseudo_cross_E0_E1_prune);

  // 14.1 Pseudo Cross + E0,E1,E2 Prune (Edge3)
  if (!loadTable(pseudo_cross_E0_E1_E2_prune,
                 "prune_table_pseudo_cross_E0_E1_E2.bin")) {
    mtm.loadCrossTable();
    mtm.loadEdge3Table();
    generatePseudoCrossE0E1E2Prune();
    mtm.releaseCrossTable();
    mtm.releaseEdge3Table();
  }
  std::vector<unsigned char>().swap(pseudo_cross_E0_E1_E2_prune);

  // 15. Pseudo Cross + C4,C6 Prune (Corner2)
  if (!loadTable(pseudo_cross_C4_C6_prune,
                 "prune_table_pseudo_cross_C4_C6.bin")) {
    mtm.loadCrossTable();
    mtm.loadCorner2Table();
    generatePseudoCrossC4C6Prune();
    mtm.releaseCrossTable();
    mtm.releaseCorner2Table();
  }
  std::vector<unsigned char>().swap(pseudo_cross_C4_C6_prune);

  // 17. Pseudo Cross + C4,C5 Prune (Corner2)
  if (!loadTable(pseudo_cross_C4_C5_prune,
                 "prune_table_pseudo_cross_C4_C5.bin")) {
    mtm.loadCrossTable();
    mtm.loadCorner2Table();
    generatePseudoCrossC4C5Prune();
    mtm.releaseCrossTable();
    mtm.releaseCorner2Table();
  }
  std::vector<unsigned char>().swap(pseudo_cross_C4_C5_prune);

  // 21. Pseudo Cross + C4,C5,C6 Prune (Corner3)
  if (!loadTable(pseudo_cross_C4_C5_C6_prune,
                 "prune_table_pseudo_cross_C4_C5_C6.bin")) {
    mtm.loadCrossTable();
    mtm.loadCorner3Table();
    generatePseudoCrossC4C5C6Prune();
    mtm.releaseCrossTable();
    mtm.releaseCorner3Table();
  }
  std::vector<unsigned char>().swap(pseudo_cross_C4_C5_C6_prune);

  // --- Pseudo Cross/XCross/Pair 变体表 (共 36 个) ---
  std::vector<int> corner_indices = {12, 15, 18, 21}; // C4, C5, C6, C7
  std::vector<int> edge_indices = {0, 2, 4, 6};       // E0, E1, E2, E3
  std::vector<unsigned char> temp_table;

  // 22. Pseudo Cross + Corner 变体表 (4 个: C4, C5, C6, C7)
  std::cout << "[PruneTable] Generating Pseudo Cross + Corner variants..."
            << std::endl;
  mtm.loadCrossTable();
  mtm.loadCornerTable();
  for (int c = 0; c < 4; ++c) {
    std::string fn =
        "prune_table_pseudo_cross_C" + std::to_string(c + 4) + ".bin";
    if (!loadTable(temp_table, fn)) {
      std::cout << "  Generating " << fn << "..." << std::endl;
      create_prune_table_pseudo_cross_corner(
          corner_indices[c], 10, mtm.getCrossTable(), mtm.getCornerTable(),
          temp_table, "[Gen Cross C" + std::to_string(c + 4) + "]");
      saveTable(temp_table, fn);
    }
    std::vector<unsigned char>().swap(temp_table);
  }

  // 23. Pseudo XCross 变体表 (16 个: C{4-7}_into_slot{0-3})
  std::cout << "[PruneTable] Generating Pseudo XCross variants..." << std::endl;
  for (int c = 0; c < 4; ++c) {
    for (int e = 0; e < 4; ++e) {
      std::string fn = "prune_table_pseudo_cross_C" + std::to_string(c + 4) +
                       "_into_slot" + std::to_string(e) + ".bin";
      if (!loadTable(temp_table, fn)) {
        std::cout << "  Generating " << fn << "..." << std::endl;
        create_prune_table_pseudo_xcross(edge_indices[e], corner_indices[c], 10,
                                         mtm.getCrossTable(),
                                         mtm.getCornerTable(), temp_table,
                                         "[Gen XC C" + std::to_string(c + 4) +
                                             " S" + std::to_string(e) + "]");
        saveTable(temp_table, fn);
      }
      std::vector<unsigned char>().swap(temp_table);
    }
  }
  mtm.releaseCrossTable();
  mtm.releaseCornerTable();

  // 24. Pseudo Pair 变体表 (16 个: C{4-7}_E{0-3})
  std::cout << "[PruneTable] Generating Pseudo Pair variants..." << std::endl;
  mtm.loadEdgeTable();
  mtm.loadCornerTable();
  for (int c = 0; c < 4; ++c) {
    for (int e = 0; e < 4; ++e) {
      std::string fn = "prune_table_pseudo_pair_C" + std::to_string(c + 4) +
                       "_E" + std::to_string(e) + ".bin";
      if (!loadTable(temp_table, fn)) {
        std::cout << "  Generating " << fn << "..." << std::endl;
        create_prune_table_pseudo_pair(edge_indices[e], corner_indices[c], 24,
                                       24, 8, mtm.getEdgeTable(),
                                       mtm.getCornerTable(), temp_table,
                                       "[Gen Pair C" + std::to_string(c + 4) +
                                           " E" + std::to_string(e) + "]");
        saveTable(temp_table, fn);
      }
      std::vector<unsigned char>().swap(temp_table);
    }
  }
  mtm.releaseEdgeTable();
  mtm.releaseCornerTable();

  std::cout << "[PruneTable] Sequential generation complete." << std::endl;
}

bool PruneTableManager::loadTable(std::vector<unsigned char> &table,
                                  const std::string &filename) {
  return load_vector_chunked(table, filename);
}

void PruneTableManager::saveTable(const std::vector<unsigned char> &table,
                                  const std::string &filename) {
  save_vector_chunked(table, filename);
}

void PruneTableManager::generateCrossPrune() {
  if (loadTable(cross_prune, "prune_table_cross.bin"))
    return;
  std::cout << "[PruneTable] Generating cross prune table..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  const auto &edges_2_table = mtm.getEdges2Table();
  long long sz = 24LL * 22 * 24 * 22;
  std::vector<unsigned char> tmp(sz, 255);
  int i1 = 416, i2 = 520;
  tmp[(long long)i1 * 528 + i2] = 0;
  for (int d = 0; d < 10; ++d) {
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < sz; ++i) {
      if (tmp[i] == d) {
        cnt++;
        int idx1 = (i / 528) * 18, idx2 = (i % 528) * 18;
        for (int j = 0; j < 18; ++j) {
          long long ni = (long long)edges_2_table[idx1 + j] * 528 +
                         edges_2_table[idx2 + j];
          if (tmp[ni] == 255)
            tmp[ni] = d + 1;
        }
      }
    }
  }
  int c_sz = (sz + 1) / 2;
  cross_prune.assign(c_sz, 0xFF);
  for (long long i = 0; i < sz; ++i)
    set_prune(cross_prune, i, tmp[i]);
  saveTable(cross_prune, "prune_table_cross.bin");
}

void PruneTableManager::generateCrossC4Prune() {
  if (loadTable(cross_c4_prune, "prune_table_cross_C4.bin"))
    return;
  std::cout << "[PruneTable] Generating cross+c4 prune table..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  cross_c4_prune.resize((long long)24 * 22 * 20 * 18 * 24, 255);
  create_prune_table_cross_c4(187520, 12, 24 * 22 * 20 * 18, 24, 10,
                              mtm.getCrossTable(), mtm.getCornerTable(),
                              cross_c4_prune);
  saveTable(cross_c4_prune, "prune_table_cross_C4.bin");
}

void PruneTableManager::generatePairC4E0Prune() {
  if (loadTable(pair_c4_e0_prune, "prune_table_C4_E0.bin"))
    return;
  std::cout << "[PruneTable] Generating pair c4+e0 prune table..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  pair_c4_e0_prune.resize(24 * 24, 255);
  create_prune_table_pair_base(0, 12, 24, 24, 8, mtm.getEdgeTable(),
                               mtm.getCornerTable(), pair_c4_e0_prune);
  saveTable(pair_c4_e0_prune, "prune_table_C4_E0.bin");
}

void PruneTableManager::generateXCrossC4E0Prune() {
  if (loadTable(xcross_c4_e0_prune, "prune_table_cross_C4_E0.bin"))
    return;
  std::cout << "[PruneTable] Generating xcross c4+e0 prune table..."
            << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  long long c_sz = ((long long)24 * 22 * 20 * 18 * 24 * 24 + 1) / 2;
  xcross_c4_e0_prune.resize(c_sz, 0xFF);
  create_prune_table_xcross_full(187520, 12, 0, 24 * 22 * 20 * 18, 24, 24, 11,
                                 mtm.getCrossTable(), mtm.getCornerTable(),
                                 mtm.getEdgeTable(), xcross_c4_e0_prune);
  saveTable(xcross_c4_e0_prune, "prune_table_cross_C4_E0.bin");
}

void PruneTableManager::generateHugeNeighborPrune() {
  if (loadTable(huge_neighbor_prune, "prune_table_cross_C4_E0_C5_E1.bin"))
    return;
  std::cout << "[PruneTable] Generating huge neighbor prune table..."
            << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  create_prune_table_huge(42577920, 504, 15, {0, 2, 16, 18, 20, 22}, {12, 15},
                          mtm.getEdge6Table(), mtm.getCorner2Table(),
                          huge_neighbor_prune);
  saveTable(huge_neighbor_prune, "prune_table_cross_C4_E0_C5_E1.bin");
}

void PruneTableManager::generateHugeDiagonalPrune() {
  if (!ENABLE_DIAGONAL_TABLE)
    return;
  if (loadTable(huge_diagonal_prune, "prune_table_cross_C4_E0_C6_E2.bin"))
    return;
  std::cout << "[PruneTable] Generating huge diagonal prune table..."
            << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  create_prune_table_huge(42577920, 504, 15, {0, 4, 16, 18, 20, 22}, {12, 18},
                          mtm.getEdge6Table(), mtm.getCorner2Table(),
                          huge_diagonal_prune);
  saveTable(huge_diagonal_prune, "prune_table_cross_C4_E0_C6_E2.bin");
}

void PruneTableManager::generatePseudoCrossPrune() {
  if (loadTable(pseudo_cross_prune, "prune_table_pseudo_cross.bin"))
    return;
  std::cout << "[PruneTable] Generating pseudo cross prune table..."
            << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  const auto &edges_2_table = mtm.getEdges2Table();
  long long sz = 24LL * 22 * 24 * 22;
  std::vector<unsigned char> tmp(sz, 255);
  int d_moves[] = {-1, 3, 4, 5};
  for (int k = 0; k < 4; ++k) {
    int i1 = 416, i2 = 520;
    if (k > 0) {
      i1 = edges_2_table[i1 * 18 + d_moves[k]];
      i2 = edges_2_table[i2 * 18 + d_moves[k]];
    }
    tmp[(long long)i1 * 528 + i2] = 0;
  }
  for (int d = 0; d < 10; ++d) {
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < sz; ++i) {
      if (tmp[i] == d) {
        cnt++;
        int idx1 = (i / 528) * 18, idx2 = (i % 528) * 18;
        for (int j = 0; j < 18; ++j) {
          long long ni = (long long)edges_2_table[idx1 + j] * 528 +
                         edges_2_table[idx2 + j];
          if (tmp[ni] == 255)
            tmp[ni] = d + 1;
        }
      }
    }
  }
  int c_sz = (sz + 1) / 2;
  pseudo_cross_prune.assign(c_sz, 0xFF);
  for (long long i = 0; i < sz; ++i)
    set_prune(pseudo_cross_prune, i, tmp[i]);
  saveTable(pseudo_cross_prune, "prune_table_pseudo_cross.bin");
}

void PruneTableManager::generatePseudoCrossBasePrune(int i) {
  std::string fn = "prune_table_pseudo_cross_C4_E" + std::to_string(i) + ".bin";
  if (loadTable(pseudo_cross_base_prune[i], fn))
    return;
  std::cout << "[PruneTable] Generating pseudo xcross prune table (offset " << i
            << ")..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  int e_diffs[] = {0, 2, 4, 6};
  pseudo_cross_base_prune[i].resize(
      ((long long)24 * 22 * 20 * 18 * 24 * 24 + 1) / 2, 0xFF);
  create_prune_table_xcross_full(187520, 12, e_diffs[i], 24 * 22 * 20 * 18, 24,
                                 24, 11, mtm.getCrossTable(),
                                 mtm.getCornerTable(), mtm.getEdgeTable(),
                                 pseudo_cross_base_prune[i], true);
  saveTable(pseudo_cross_base_prune[i], fn);
}

void PruneTableManager::generatePseudoCrossE0E2Prune() {
  if (loadTable(pseudo_cross_E0_E2_prune, "prune_table_pseudo_cross_E0_E2.bin"))
    return;
  std::cout << "[PruneTable] Generating pseudo cross + E0,E2 prune table..."
            << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {0, 4};
  int idx_e0_e2_solved = array_to_index(target, 2, 2, 12);
  pseudo_cross_E0_E2_prune.resize(((long long)190080 * 528 + 1) / 2, 0xFF);
  create_prune_table_pseudo_cross_edges2(
      187520, idx_e0_e2_solved, 190080, 528, 11, mtm.getCrossTable(),
      mtm.getEdges2Table(), pseudo_cross_E0_E2_prune);
  saveTable(pseudo_cross_E0_E2_prune, "prune_table_pseudo_cross_E0_E2.bin");
}

void PruneTableManager::generatePseudoCrossE0E1Prune() {
  if (loadTable(pseudo_cross_E0_E1_prune, "prune_table_pseudo_cross_E0_E1.bin"))
    return;
  std::cout << "[PruneTable] Generating pseudo cross + E0,E1 prune table..."
            << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {0, 2}; // E0(0), E1(1) -> 0*2, 1*2
  int idx_solved = array_to_index(target, 2, 2, 12);
  pseudo_cross_E0_E1_prune.resize(((long long)190080 * 528 + 1) / 2, 0xFF);
  create_prune_table_pseudo_cross_edges2(
      187520, idx_solved, 190080, 528, 11, mtm.getCrossTable(),
      mtm.getEdges2Table(), pseudo_cross_E0_E1_prune);
  saveTable(pseudo_cross_E0_E1_prune, "prune_table_pseudo_cross_E0_E1.bin");
}

void PruneTableManager::generatePseudoCrossE0E1E2Prune() {
  if (loadTable(pseudo_cross_E0_E1_E2_prune,
                "prune_table_pseudo_cross_E0_E1_E2.bin"))
    return;
  std::cout << "[PruneTable] Generating pseudo cross + E0,E1,E2 prune table..."
            << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {0, 2, 4}; // E0, E1, E2 (0, 2, 4)
  int idx_solved = array_to_index(target, 3, 2, 12);
  pseudo_cross_E0_E1_E2_prune.resize(((long long)190080 * 10560 + 1) / 2, 0xFF);
  create_prune_table_pseudo_cross_edges3(
      187520, idx_solved, 190080, 10560, 12, mtm.getCrossTable(),
      mtm.getEdge3Table(), pseudo_cross_E0_E1_E2_prune);
  saveTable(pseudo_cross_E0_E1_E2_prune,
            "prune_table_pseudo_cross_E0_E1_E2.bin");
}

void PruneTableManager::generatePseudoCrossC4C6Prune() {
  if (loadTable(pseudo_cross_C4_C6_prune, "prune_table_pseudo_cross_C4_C6.bin"))
    return;
  std::cout << "[PruneTable] Generating pseudo cross + C4,C6 prune table..."
            << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {12, 18}; // C4(4*3), C6(6*3)
  int idx_solved = array_to_index(target, 2, 3, 8);
  pseudo_cross_C4_C6_prune.resize(((long long)190080 * 504 + 1) / 2, 0xFF);
  create_prune_table_pseudo_cross_corners2(
      187520, idx_solved, 190080, 504, 11, mtm.getCrossTable(),
      mtm.getCorner2Table(), pseudo_cross_C4_C6_prune);
  saveTable(pseudo_cross_C4_C6_prune, "prune_table_pseudo_cross_C4_C6.bin");
}

void PruneTableManager::generatePseudoCrossC4C5Prune() {
  if (loadTable(pseudo_cross_C4_C5_prune, "prune_table_pseudo_cross_C4_C5.bin"))
    return;
  std::cout << "[PruneTable] Generating pseudo cross + C4,C5 prune table..."
            << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {12, 15};
  int idx_solved = array_to_index(target, 2, 3, 8);
  pseudo_cross_C4_C5_prune.resize(((long long)190080 * 504 + 1) / 2, 0xFF);
  create_prune_table_pseudo_cross_corners2(
      187520, idx_solved, 190080, 504, 11, mtm.getCrossTable(),
      mtm.getCorner2Table(), pseudo_cross_C4_C5_prune);
  saveTable(pseudo_cross_C4_C5_prune, "prune_table_pseudo_cross_C4_C5.bin");
}

void PruneTableManager::generatePseudoCrossC4C5C6Prune() {
  if (loadTable(pseudo_cross_C4_C5_C6_prune,
                "prune_table_pseudo_cross_C4_C5_C6.bin"))
    return;
  std::cout << "[PruneTable] Generating pseudo cross + C4,C5,C6 prune table..."
            << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {12, 15, 18}; // C4, C5, C6
  int idx_solved = array_to_index(target, 3, 3, 8);
  pseudo_cross_C4_C5_C6_prune.resize(((long long)190080 * 9072 + 1) / 2, 0xFF);
  create_prune_table_pseudo_cross_corners3(
      187520, idx_solved, 190080, 9072, 13, mtm.getCrossTable(),
      mtm.getCorner3Table(), pseudo_cross_C4_C5_C6_prune);
  saveTable(pseudo_cross_C4_C5_C6_prune,
            "prune_table_pseudo_cross_C4_C5_C6.bin");
}

// ... existing helper functions ...

void create_prune_table_pseudo_cross_corners2(int idx_cr, int idx_c2, int sz_cr,
                                              int sz_c2, int depth,
                                              const std::vector<int> &t_cr,
                                              const std::vector<int> &t_c2,
                                              std::vector<unsigned char> &pt) {
  long long total = (long long)sz_cr * sz_c2;
  std::vector<unsigned char> tmp;
  try {
    tmp.resize(total, 255);
  } catch (...) {
    std::cerr << "Alloc fail (Corner2)\n";
    exit(1);
  }

  int d_moves[] = {-1, 3, 4, 5};
  for (int m_idx = 0; m_idx < 4; ++m_idx) {
    int cur_cr = idx_cr;
    int cur_c2 = idx_c2;
    if (m_idx > 0) {
      int move = d_moves[m_idx];
      cur_cr = t_cr[idx_cr * 24 + move] / 24;
      cur_c2 = t_c2[idx_c2 * 18 + move];
    }
    long long start_idx = (long long)cur_cr * sz_c2 + cur_c2;
    if (start_idx < total)
      tmp[start_idx] = 0;
  }

  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      if (tmp[i] == d) {
        cnt++;
        int cur_c2 = i % sz_c2;
        int cur_cr = i / sz_c2;
        int base_cr = cur_cr * 24;
        int base_c2 = cur_c2 * 18;
        for (int j = 0; j < 18; ++j) {
          int n_cr = t_cr[base_cr + j] / 24;
          int n_c2 = t_c2[base_c2 + j];
          long long ni = (long long)n_cr * sz_c2 + n_c2;
          if (tmp[ni] == 255)
            tmp[ni] = nd;
        }
      }
    }
    std::cout << "  [Gen Corner2] Depth " << d << ": " << cnt << std::endl;
    if (cnt == 0)
      break;
  }
  std::fill(pt.begin(), pt.end(), 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

void create_prune_table_pseudo_cross_corners3(int idx_cr, int idx_c3, int sz_cr,
                                              int sz_c3, int depth,
                                              const std::vector<int> &t_cr,
                                              const std::vector<int> &t_c3,
                                              std::vector<unsigned char> &pt) {
  long long total = (long long)sz_cr * sz_c3;
  std::cout << "Allocating Corner3 Prune Table: " << total / 1024 / 1024
            << " MB" << std::endl;
  std::vector<unsigned char> tmp;
  try {
    tmp.resize(total, 255);
  } catch (...) {
    std::cerr << "Alloc fail (Corner3)\n";
    exit(1);
  }

  int d_moves[] = {-1, 3, 4, 5};
  for (int m_idx = 0; m_idx < 4; ++m_idx) {
    int cur_cr = idx_cr;
    int cur_c3 = idx_c3;
    if (m_idx > 0) {
      int move = d_moves[m_idx];
      cur_cr = t_cr[idx_cr * 24 + move] / 24;
      cur_c3 = t_c3[idx_c3 * 18 + move];
    }
    long long start_idx = (long long)cur_cr * sz_c3 + cur_c3;
    if (start_idx < total)
      tmp[start_idx] = 0;
  }

  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      if (tmp[i] == d) {
        cnt++;
        int cur_c3 = i % sz_c3;
        int cur_cr = i / sz_c3;
        int base_cr = cur_cr * 24;
        int base_c3 = cur_c3 * 18;
        for (int j = 0; j < 18; ++j) {
          int n_cr = t_cr[base_cr + j] / 24;
          int n_c3 = t_c3[base_c3 + j];
          long long ni = (long long)n_cr * sz_c3 + n_c3;
          if (tmp[ni] == 255)
            tmp[ni] = nd;
        }
      }
    }
    std::cout << "  [Gen Corner3] Depth " << d << ": " << cnt << std::endl;
    if (cnt == 0)
      break;
  }
  std::fill(pt.begin(), pt.end(), 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

void create_prune_table_pseudo_cross_edges3(int idx_cr, int idx_e3, int sz_cr,
                                            int sz_e3, int depth,
                                            const std::vector<int> &t_cr,
                                            const std::vector<int> &t_e3,
                                            std::vector<unsigned char> &pt) {
  long long total = (long long)sz_cr * sz_e3;
  std::cout << "Allocating Edge3 Prune Table: " << total / 1024 / 1024 << " MB"
            << std::endl;
  std::vector<unsigned char> tmp;
  try {
    tmp.resize(total, 255);
  } catch (...) {
    std::cerr << "Alloc fail (Edge3)\n";
    exit(1);
  }

  int d_moves[] = {-1, 3, 4, 5};
  for (int m_idx = 0; m_idx < 4; ++m_idx) {
    int cur_cr = idx_cr;
    int cur_e3 = idx_e3;
    if (m_idx > 0) {
      int move = d_moves[m_idx];
      cur_cr = t_cr[idx_cr * 24 + move] / 24;
      cur_e3 = t_e3[idx_e3 * 18 + move];
    }
    long long start_idx = (long long)cur_cr * sz_e3 + cur_e3;
    if (start_idx < total)
      tmp[start_idx] = 0;
  }

  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      if (tmp[i] == d) {
        cnt++;
        int cur_e3 = i % sz_e3;
        int cur_cr = i / sz_e3;
        int base_cr = cur_cr * 24;
        int base_e3 = cur_e3 * 18;
        for (int j = 0; j < 18; ++j) {
          int n_cr = t_cr[base_cr + j] / 24;
          int n_e3 = t_e3[base_e3 + j];
          long long ni = (long long)n_cr * sz_e3 + n_e3;
          if (tmp[ni] == 255)
            tmp[ni] = nd;
        }
      }
    }
    std::cout << "  [Gen Edge3] Depth " << d << ": " << cnt << std::endl;
    if (cnt == 0)
      break;
  }
  std::fill(pt.begin(), pt.end(), 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

// --- 辅助生成函数实现 (移植自 pseudo_pair_analyzer.cpp) ---

void create_prune_table_pseudo_base(int idx_cr, int idx_cn, int idx_ed,
                                    int sz_cr, int sz_cn, int sz_ed, int depth,
                                    const std::vector<int> &t1,
                                    const std::vector<int> &t2,
                                    const std::vector<int> &t3,
                                    std::vector<unsigned char> &pt) {
  long long total = (long long)sz_cr * sz_cn * sz_ed;
  std::vector<unsigned char> tmp;
  try {
    tmp.resize(total, 255);
  } catch (...) {
    std::cerr << "Alloc fail\n";
    exit(1);
  }

  int d_moves[] = {-1, 3, 4, 5};
  int num_init = 4;

  for (int m_idx = 0; m_idx < num_init; ++m_idx) {
    long long cur_cr;
    int cur_cn;
    int cur_ed = idx_ed;
    if (m_idx > 0) {
      int move = d_moves[m_idx];
      cur_cr = t1[idx_cr * 24 + move];
      cur_cn = t2[idx_cn * 18 + move] * 18;
    } else {
      cur_cr = idx_cr * 24LL;
      cur_cn = idx_cn * 18;
    }
    long long start_idx = (cur_cr + cur_cn / 18) * 24 + cur_ed;
    if (start_idx < total)
      tmp[start_idx] = 0;
  }

  std::cout << "  [Gen Pseudo Diff " << (idx_ed / 2) << "] Depth 0 initialized."
            << std::endl;
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      if (tmp[i] == d) {
        cnt++;
        long long comb = i / sz_ed;
        int cur_ed = i % sz_ed;
        int cur_cr = (comb / sz_cn) * 24;
        int cur_cn = (comb % sz_cn) * 18;
        int idx3_base = cur_ed * 18;
        for (int j = 0; j < 18; ++j) {
          long long n_cr = t1[cur_cr + j];
          int n_cn = t2[cur_cn + j];
          long long ni = (n_cr + n_cn) * 24 + t3[idx3_base + j];
          if (tmp[ni] == 255)
            tmp[ni] = nd;
        }
      }
    }
    std::cout << "  [Gen Pseudo Diff " << (idx_ed / 2) << "] Depth " << nd
              << ": " << cnt << std::endl;
    if (cnt == 0)
      break;
  }
  pt.resize((total + 1) / 2);
  std::fill(pt.begin(), pt.end(), 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

// 1. Cross + C4 (Base)
void create_prune_table_cross_c4(int idx1, int idx2, int sz1, int sz2,
                                 int depth, const std::vector<int> &t1,
                                 const std::vector<int> &t2,
                                 std::vector<unsigned char> &pt) {
  long long total = (long long)sz1 * sz2;
  std::fill(pt.begin(), pt.end(), 255);
  std::vector<std::string> am = {"L U L'", "L U' L'", "B' U B", "B' U' B"};
  pt[(long long)idx1 * sz2 + idx2] = 0;
  for (const auto &s : am) {
    int i1 = idx1 * 24, i2 = idx2;
    for (int m : string_to_alg(s)) {
      i1 = t1[i1 + m];
      i2 = t2[i2 * 18 + m];
    }
    pt[(long long)i1 / 24 * sz2 + i2] = 0;
    int base1 = i1, base2 = i2 * 18;
    pt[t1[base1] + t2[base2]] = 0;
    pt[t1[base1 + 1] + t2[base2 + 1]] = 0;
    pt[t1[base1 + 2] + t2[base2 + 2]] = 0;
  }
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      if (pt[i] == d) {
        cnt++;
        int i1 = (i / sz2) * 24;
        int i2 = (i % sz2) * 18;
        for (int j = 0; j < 18; ++j) {
          long long ni = (long long)t1[i1 + j] + t2[i2 + j];
          if (pt[ni] == 255)
            pt[ni] = nd;
        }
      }
    }
    std::cout << "  [Base Cross+C4] Depth " << d << ": " << cnt << std::endl;
    if (cnt == 0)
      break;
  }
}

// 2. Pair C4 + E0 (Base)
void create_prune_table_pair_base(int idx_e, int idx_c, int sz_e, int sz_c,
                                  int depth, const std::vector<int> &t_edge,
                                  const std::vector<int> &t_corn,
                                  std::vector<unsigned char> &pt) {
  long long total = (long long)sz_e * sz_c;
  std::fill(pt.begin(), pt.end(), 255);
  std::vector<std::string> am = {"L U L'", "L U' L'", "B' U B", "B' U' B"};
  pt[idx_e * sz_c + idx_c] = 0;
  for (const auto &s : am) {
    int c1 = idx_e, c2 = idx_c;
    for (int m : string_to_alg(s)) {
      c1 = t_edge[c1 * 18 + m];
      c2 = t_corn[c2 * 18 + m];
    }
    pt[c1 * sz_c + c2] = 0;
    for (int k = 0; k < 3; ++k) {
      int n1 = t_edge[c1 * 18 + k];
      int n2 = t_corn[c2 * 18 + k];
      pt[n1 * sz_c + n2] = 0;
    }
  }
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      if (pt[i] == d) {
        cnt++;
        int i1 = (i / sz_c) * 18;
        int i2 = (i % sz_c) * 18;
        for (int j = 0; j < 18; ++j) {
          long long ni = (long long)t_edge[i1 + j] * sz_c + t_corn[i2 + j];
          if (pt[ni] == 255)
            pt[ni] = nd;
        }
      }
    }
    std::cout << "  [Base Pair C4+E0] Depth " << d << ": " << cnt << std::endl;
    if (cnt == 0)
      break;
  }
}

// 3. XCross Base Generator
void create_prune_table_xcross_base(int idx_cr, int idx_cn, int idx_ex,
                                    int sz_cr, int sz_cn, int sz_ex, int depth,
                                    const std::vector<int> &t1,
                                    const std::vector<int> &t2,
                                    const std::vector<int> &t3,
                                    std::vector<unsigned char> &pt) {
  long long total = (long long)sz_cr * sz_cn * sz_ex;
  std::vector<unsigned char> tmp;
  try {
    tmp.resize(total, 255);
  } catch (...) {
    std::cerr << "Alloc fail XCross\n";
    exit(1);
  }
  long long cur_cr = (long long)idx_cr * 24;
  long long start_idx = (cur_cr + idx_cn) * 24 + idx_ex;
  if (start_idx < total)
    tmp[start_idx] = 0;

  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      if (tmp[i] == d) {
        cnt++;
        long long comb = i / sz_ex;
        int cur_ex = i % sz_ex;
        int cur_cr = (comb / sz_cn) * 24;
        int cur_cn = (comb % sz_cn) * 18;
        int idx3_base = cur_ex * 18;
        for (int j = 0; j < 18; ++j) {
          long long n_cr = t1[cur_cr + j];
          int n_cn = t2[cur_cn + j];
          long long ni = (n_cr + n_cn) * 24 + t3[idx3_base + j];
          if (tmp[ni] == 255)
            tmp[ni] = nd;
        }
      }
    }
    std::cout << "  [XCross Gen] Depth " << d << ": " << cnt << std::endl;
    if (cnt == 0)
      break;
  }
  pt.assign((total + 1) / 2, 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

void create_prune_table_xcross_full(int idx_cr, int idx_cn, int idx_ed,
                                    int sz_cr, int sz_cn, int sz_ed, int depth,
                                    const std::vector<int> &t1,
                                    const std::vector<int> &t2,
                                    const std::vector<int> &t3,
                                    std::vector<unsigned char> &pt,
                                    bool is_pseudo) {
  long long total = (long long)sz_cr * sz_cn * sz_ed;
  std::vector<unsigned char> tmp;
  try {
    tmp.resize(total, 255);
  } catch (...) {
    std::cerr << "Alloc fail\n";
    exit(1);
  }

  int d_moves[] = {-1, 3, 4, 5};
  int num_init = is_pseudo ? 4 : 1;

  for (int m_idx = 0; m_idx < num_init; ++m_idx) {
    long long cur_cr;
    int cur_cn;
    int cur_ed = idx_ed;
    if (m_idx > 0) {
      int move = d_moves[m_idx];
      cur_cr = t1[idx_cr * 24 + move];
      cur_cn = t2[idx_cn * 18 + move] * 18;
    } else {
      cur_cr = idx_cr * 24LL;
      cur_cn = idx_cn * 18;
    }
    long long start_idx = (cur_cr + cur_cn / 18) * 24 + cur_ed;
    if (start_idx < total)
      tmp[start_idx] = 0;
  }

  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      if (tmp[i] == d) {
        cnt++;
        long long comb = i / sz_ed;
        int cur_ed = i % sz_ed;
        int cur_cr = (comb / sz_cn) * 24;
        int cur_cn = (comb % sz_cn) * 18;
        int idx3_base = cur_ed * 18;
        for (int j = 0; j < 18; ++j) {
          long long n_cr = t1[cur_cr + j];
          int n_cn = t2[cur_cn + j];
          long long ni = (n_cr + n_cn) * 24 + t3[idx3_base + j];
          if (tmp[ni] == 255)
            tmp[ni] = nd;
        }
      }
    }
    std::cout << "  [Gen XCross] Depth " << d << ": " << cnt << std::endl;
    if (cnt == 0)
      break;
  }
  std::fill(pt.begin(), pt.end(), 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

void create_prune_table_huge(int sz_e6, int sz_c2, int depth,
                             const std::vector<int> &target_e_ids,
                             const std::vector<int> &target_c_ids,
                             const std::vector<int> &mt_e6,
                             const std::vector<int> &mt_c2,
                             std::vector<unsigned char> &pt) {
  long long total = (long long)sz_e6 * sz_c2;
  std::cout << "  Allocating " << (total / 2 / 1024 / 1024)
            << " MB for Huge Table..." << std::endl;
  std::vector<unsigned char> tmp;
  try {
    tmp.resize(total, 255);
  } catch (...) {
    std::cerr << "Alloc fail (Huge). Need ~20GB RAM.\n";
    exit(1);
  }
  int idx_e6_solved = array_to_index(target_e_ids, 6, 2, 12);
  int idx_c2_solved = array_to_index(target_c_ids, 2, 3, 8);
  long long start_idx = (long long)idx_e6_solved * sz_c2 + idx_c2_solved;
  if (start_idx < total)
    tmp[start_idx] = 0;

  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      if (tmp[i] == d) {
        cnt++;
        int cur_c2 = i % sz_c2;
        int cur_e6 = i / sz_c2;
        int base_e6 = cur_e6 * 18;
        int base_c2 = cur_c2 * 18;
        for (int j = 0; j < 18; ++j) {
          int n_e6 = mt_e6[base_e6 + j];
          int n_c2 = mt_c2[base_c2 + j];
          long long ni = (long long)n_e6 * sz_c2 + n_c2;
          if (tmp[ni] == 255)
            tmp[ni] = nd;
        }
      }
    }
    std::cout << "  [Gen Huge] Depth " << d << ": " << cnt << std::endl;
    if (cnt == 0)
      break;
  }
  long long c_sz = (total + 1) / 2;
  pt.assign(c_sz, 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
  std::vector<unsigned char>().swap(tmp);
}

void create_prune_table_pseudo_cross_edges2(int idx_cr, int idx_e2, int sz_cr,
                                            int sz_e2, int depth,
                                            const std::vector<int> &t_cr,
                                            const std::vector<int> &t_e2,
                                            std::vector<unsigned char> &pt) {
  long long total = (long long)sz_cr * sz_e2;
  std::vector<unsigned char> tmp;
  try {
    tmp.resize(total, 255);
  } catch (...) {
    std::cerr << "Alloc fail (E0E2)\n";
    exit(1);
  }

  int d_moves[] = {-1, 3, 4, 5};
  for (int m_idx = 0; m_idx < 4; ++m_idx) {
    int cur_cr = idx_cr;
    int cur_e2 = idx_e2;
    if (m_idx > 0) {
      int move = d_moves[m_idx];
      cur_cr = t_cr[idx_cr * 24 + move] / 24;
      cur_e2 = t_e2[idx_e2 * 18 + move];
    }
    long long start_idx = (long long)cur_cr * sz_e2 + cur_e2;
    if (start_idx < total)
      tmp[start_idx] = 0;
  }

  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      if (tmp[i] == d) {
        cnt++;
        int cur_e2 = i % sz_e2;
        int cur_cr = i / sz_e2;
        int base_cr = cur_cr * 24;
        int base_e2 = cur_e2 * 18;
        for (int j = 0; j < 18; ++j) {
          int n_cr = t_cr[base_cr + j] / 24;
          int n_e2 = t_e2[base_e2 + j];
          long long ni = (long long)n_cr * sz_e2 + n_e2;
          if (tmp[ni] == 255)
            tmp[ni] = nd;
        }
      }
    }
    std::cout << "  [Gen E0E2] Depth " << d << ": " << cnt << std::endl;
    if (cnt == 0)
      break;
  }
  std::fill(pt.begin(), pt.end(), 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

// --- 级联剪枝表生成函数实现 (from eo_cross_analyzer) ---

std::vector<unsigned char>
create_cascaded_prune_table(int i1, int i2, int s1, int s2, int depth,
                            const std::vector<int> &t1,
                            const std::vector<int> &t2) {
  int sz = s1 * s2;
  std::vector<unsigned char> tmp(sz, 0xF);
  tmp[i1 * s2 + i2] = 0;

  long long cnt_0 = 0;
  if (tmp[i1 * s2 + i2] == 0)
    cnt_0 = 1;
  std::cout << "    [Gen Prune 1] Depth 0: " << cnt_0 << std::endl;

  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    if (nd >= 15)
      break;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (int i = 0; i < sz; ++i) {
      if (tmp[i] == d) {
        int t1b = (i / s2) * 18, t2b = (i % s2) * 18;
        for (int j = 0; j < 18; ++j) {
          int ni = t1[t1b + j] * s2 + t2[t2b + j];
          if (tmp[ni] == 0xF) {
            tmp[ni] = nd;
            cnt++;
          }
        }
      }
    }
    std::cout << "    [Gen Prune 1] Depth " << nd << ": " << cnt << std::endl;
    if (cnt == 0)
      break;
  }
  std::vector<unsigned char> pt((sz + 1) / 2, 0xFF);
  for (int i = 0; i < sz; ++i)
    if (tmp[i] != 0xF)
      set_prune(pt, i, tmp[i]);
  return pt;
}

void create_cascaded_prune_table2(int i1, int i2, int s1, int s2, int depth,
                                  const std::vector<int> &t1,
                                  const std::vector<int> &t2,
                                  std::vector<unsigned char> &pt) {
  int sz = s1 * s2;
  std::vector<unsigned char> tmp(sz, 0xF);
  tmp[i1 * s2 + i2] = 0;

  long long cnt_1 = 0;
  int t1b = i1 * 24, t2b = i2 * 18;
  for (int j = 0; j < 18; ++j) {
    int ni = t1[t1b + j] + t2[t2b + j];
    if (tmp[ni] == 0xF) {
      tmp[ni] = 1;
      cnt_1++;
    }
  }
  std::cout << "    [Gen Prune 2] Depth 0: 1" << std::endl;
  std::cout << "    [Gen Prune 2] Depth 1: " << cnt_1 << std::endl;

  for (int d = 1; d < depth; ++d) {
    int nd = d + 1;
    if (nd >= 15)
      break;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (int i = 0; i < sz; ++i) {
      if (tmp[i] == d) {
        int tb1 = (i / s2) * 24, tb2 = (i % s2) * 18;
        for (int j = 0; j < 18; ++j) {
          int ni = t1[tb1 + j] + t2[tb2 + j];
          if (tmp[ni] == 0xF) {
            tmp[ni] = nd;
            cnt++;
          }
        }
      }
    }
    std::cout << "    [Gen Prune 2] Depth " << nd << ": " << cnt << std::endl;
    if (cnt == 0)
      break;
  }
  pt.assign((sz + 1) / 2, 0xFF);
  for (int i = 0; i < sz; ++i)
    if (tmp[i] != 0xF)
      set_prune(pt, i, tmp[i]);
}

void create_cascaded_prune_table3(int i1, int i2, int s1, int s2, int depth,
                                  const std::vector<int> &t1,
                                  const std::vector<int> &t2,
                                  std::vector<unsigned char> &pt) {
  int sz = s1 * s2;
  std::vector<unsigned char> tmp(sz, 0xF);
  tmp[i1 * s2 + i2] = 0;
  std::cout << "    [Gen Prune 3] Depth 0: 1" << std::endl;
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    if (nd >= 15)
      break;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (int i = 0; i < sz; ++i) {
      if (tmp[i] == d) {
        int tb1 = (i / s2) * 18, tb2 = (i % s2) * 18;
        for (int j = 0; j < 18; ++j) {
          int ni = t1[tb1 + j] * s2 + t2[tb2 + j];
          if (tmp[ni] == 0xF) {
            tmp[ni] = nd;
            cnt++;
          }
        }
      }
    }
    std::cout << "    [Gen Prune 3] Depth " << nd << ": " << cnt << std::endl;
    if (cnt == 0)
      break;
  }
  pt.assign((sz + 1) / 2, 0xFF);
  for (int i = 0; i < sz; ++i)
    if (tmp[i] != 0xF)
      set_prune(pt, i, tmp[i]);
}

void create_prune_table_xcross_plus(
    int idx_cr, int idx_cn, int idx_ed, int idx_extra, int sz_cr, int sz_cn,
    int sz_ed, int sz_ex, int depth, const std::vector<int> &t1,
    const std::vector<int> &t2, const std::vector<int> &t3,
    const std::vector<int> &t4, std::vector<unsigned char> &pt) {
  long long total = (long long)sz_cr * sz_cn * sz_ed * sz_ex;
  std::cout << "  Allocating " << (total / 1024 / 1024)
            << " MB for Plus Table..." << std::endl;
  std::vector<unsigned char> tmp;
  try {
    tmp.resize(total, 255);
  } catch (...) {
    std::cerr << "Alloc fail (Plus Table).\n";
    exit(1);
  }

  long long cur_cr = idx_cr * 24LL;
  int cur_cn = idx_cn * 18;
  int cur_ed = idx_ed;
  int cur_ex = idx_extra;

  long long start_idx = ((cur_cr + cur_cn / 18) * 24 + cur_ed) * 24 + cur_ex;
  if (start_idx < total)
    tmp[start_idx] = 0;

  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      if (tmp[i] == d) {
        cnt++;
        long long rem = i;
        int c_ex = rem % sz_ex;
        rem /= sz_ex;
        int c_ed = rem % sz_ed;
        rem /= sz_ed;
        int c_cn = rem % sz_cn;
        rem /= sz_cn;
        long long c_mul = rem;

        int idx1_base = c_mul * 24;
        int idx2_base = c_cn * 18;
        int idx3_base = c_ed * 18;
        int idx4_base = c_ex * 18;

        for (int j = 0; j < 18; ++j) {
          long long n_cr = t1[idx1_base + j];
          int n_cn = t2[idx2_base + j];
          int n_ed = t3[idx3_base + j];
          int n_ex = t4[idx4_base + j];

          long long ni = ((n_cr + n_cn) * 24 + n_ed) * 24 + n_ex;
          if (tmp[ni] == 255)
            tmp[ni] = nd;
        }
      }
    }
    std::cout << "  [Gen XC+Plus] Depth " << d << ": " << cnt << std::endl;
    if (cnt == 0)
      break;
  }

  long long c_sz = (total + 1) / 2;
  pt.assign(c_sz, 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

void create_prune_table_xcross_corn3(
    int idx_cr, int idx_cn, int idx_c5, int idx_c6, int sz_cr, int sz_cn,
    int sz_c5, int sz_c6, int depth, const std::vector<int> &t1,
    const std::vector<int> &t2, const std::vector<int> &t_c5,
    const std::vector<int> &t_c6, std::vector<unsigned char> &pt) {
  long long total = (long long)sz_cr * sz_cn * sz_c5 * sz_c6;
  std::cout << "  Allocating " << (total / 1024 / 1024)
            << " MB for 3-Corner Table..." << std::endl;
  std::vector<unsigned char> tmp;
  try {
    tmp.resize(total, 255);
  } catch (...) {
    std::cerr << "Alloc fail (3-Corner Table).\n";
    exit(1);
  }

  long long cur_cr = idx_cr * 24LL;
  int cur_cn = idx_cn * 18;

  long long start_idx = ((cur_cr + cur_cn / 18) * 24 + idx_c5) * 24 + idx_c6;
  if (start_idx < total)
    tmp[start_idx] = 0;

  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      if (tmp[i] == d) {
        cnt++;
        long long rem = i;
        int c_c6 = rem % sz_c6;
        rem /= sz_c6;
        int c_c5 = rem % sz_c5;
        rem /= sz_c5;
        int c_cn = rem % sz_cn;
        rem /= sz_cn;
        long long c_mul = rem;

        int idx1_base = c_mul * 24;
        int idx2_base = c_cn * 18;
        int idx5_base = c_c5 * 18;
        int idx6_base = c_c6 * 18;

        for (int j = 0; j < 18; ++j) {
          long long n_cr = t1[idx1_base + j];
          int n_cn = t2[idx2_base + j];
          int n_c5 = t_c5[idx5_base + j];
          int n_c6 = t_c6[idx6_base + j];

          long long ni = ((n_cr + n_cn) * 24 + n_c5) * 24 + n_c6;
          if (tmp[ni] == 255)
            tmp[ni] = nd;
        }
      }
    }
    std::cout << "  [Gen 3-Corn] Depth " << d << ": " << cnt << std::endl;
    if (cnt == 0)
      break;
  }

  long long c_sz = (total + 1) / 2;
  pt.assign(c_sz, 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

// --- Pseudo Cross/XCross/Pair 变体表生成函数 ---
// NOTE: 以下函数用于生成 C{4-7} 变体的剪枝表

// 生成 Pseudo Cross + Corner 表 (例如: prune_table_pseudo_cross_C4.bin)
// index2: corner 初始索引 (如 12=C4, 15=C5, 18=C6, 21=C7)
void create_prune_table_pseudo_cross_corner(
    int index2, int depth, const std::vector<int> &table1,
    const std::vector<int> &table2, std::vector<unsigned char> &prune_table,
    const std::string &log_prefix) {
  int size1 = 190080, size2 = 24, size = size1 * size2;
  std::vector<unsigned char> temp_table(size, 0xF);
  int next_i, index1_tmp, index2_tmp, next_d;
  std::vector<int> a = {16, 18, 20, 22};
  int index1 = array_to_index(a, 4, 2, 12);
  temp_table[index1 * size2 + index2] = 0;
  temp_table[table1[index1 * 24 + 3] + table2[index2 * 18 + 3]] = 0;
  temp_table[table1[index1 * 24 + 4] + table2[index2 * 18 + 4]] = 0;
  temp_table[table1[index1 * 24 + 5] + table2[index2 * 18 + 5]] = 0;
  long long count = 0;
  for (int i = 0; i < size; ++i)
    if (temp_table[i] == 0)
      count++;
  std::cout << "  " << log_prefix << " Depth 0: " << count << std::endl;
  for (int d = 0; d < depth; ++d) {
    next_d = d + 1;
    if (next_d >= 15)
      break;
    long long next_count = 0;
    for (int i = 0; i < size; ++i) {
      if (temp_table[i] == d) {
        index1_tmp = (i / size2) * 24;
        index2_tmp = (i % size2) * 18;
        for (int j = 0; j < 18; ++j) {
          next_i = table1[index1_tmp + j] + table2[index2_tmp + j];
          if (temp_table[next_i] == 0xF) {
            temp_table[next_i] = next_d;
            next_count++;
          }
        }
      }
    }
    std::cout << "  " << log_prefix << " Depth " << next_d << ": " << next_count
              << std::endl;
  }
  prune_table.resize((size + 1) / 2);
  std::fill(prune_table.begin(), prune_table.end(), 0xFF);
  for (int i = 0; i < size; ++i)
    if (temp_table[i] != 0xF)
      set_prune(prune_table, i, temp_table[i]);
}

// 生成 Pseudo XCross 表 (例如: prune_table_pseudo_cross_C4_into_slot0.bin)
// index3: edge 初始索引 (0, 2, 4, 6 for E0-E3)
// index2: corner 初始索引 (12, 15, 18, 21 for C4-C7)
void create_prune_table_pseudo_xcross(int index3, int index2, int depth,
                                      const std::vector<int> &table1,
                                      const std::vector<int> &table2,
                                      std::vector<unsigned char> &prune_table,
                                      const std::string &log_prefix) {
  int size1 = 190080, size2 = 24, size = size1 * size2;
  std::vector<unsigned char> temp_table(size, 0xF);
  int next_i, index1_tmp, index2_tmp, next_d;

  // NOTE: 统一使用 slot0 初始化序列，支持运行时 Conj 复用
  // 原来的代码根据 index3 选择不同的 appl_moves，这导致表无法互换
  // 现在所有表都使用相同的初始化，通过 Conj 变换在运行时适配不同 slot
  std::vector<std::string> appl_moves = {"L U L'", "L U' L'", "B' U B",
                                         "B' U' B"};
  std::vector<int> tmp_moves = {0, 3, 4, 5};
  (void)index3; // 参数保留但不再使用

  std::vector<int> a = {16, 18, 20, 22};
  int index1 = array_to_index(a, 4, 2, 12);
  temp_table[index1 * size2 + index2] = 0;
  temp_table[table1[index1 * 24 + 3] + table2[index2 * 18 + 3]] = 0;
  temp_table[table1[index1 * 24 + 4] + table2[index2 * 18 + 4]] = 0;
  temp_table[table1[index1 * 24 + 5] + table2[index2 * 18 + 5]] = 0;

  // 初始化所有 pseudo 等效状态
  for (int i = 0; i < 4; i++) {
    int index1_tmp_2 = index1 * 24, index2_tmp_2 = index2;
    index1_tmp_2 = table1[index1_tmp_2 + tmp_moves[index2 / 3 - 4]];
    index2_tmp_2 = table2[index2_tmp_2 * 18 + tmp_moves[index2 / 3 - 4]];
    for (int m : string_to_alg(appl_moves[i])) {
      index1_tmp_2 = table1[index1_tmp_2 + m];
      index2_tmp_2 = table2[index2_tmp_2 * 18 + m];
    }
    temp_table[index1_tmp_2 + index2_tmp_2] = 0;
    temp_table[table1[index1_tmp_2 + 3] + table2[index2_tmp_2 * 18 + 3]] = 0;
    temp_table[table1[index1_tmp_2 + 4] + table2[index2_tmp_2 * 18 + 4]] = 0;
    temp_table[table1[index1_tmp_2 + 5] + table2[index2_tmp_2 * 18 + 5]] = 0;
    temp_table[table1[table1[index1_tmp_2] + 3] +
               table2[table2[index2_tmp_2 * 18] * 18 + 3]] = 0;
    temp_table[table1[table1[index1_tmp_2] + 4] +
               table2[table2[index2_tmp_2 * 18] * 18 + 4]] = 0;
    temp_table[table1[table1[index1_tmp_2] + 5] +
               table2[table2[index2_tmp_2 * 18] * 18 + 5]] = 0;
    temp_table[table1[index1_tmp_2 + 1] + table2[index2_tmp_2 * 18 + 1]] = 0;
    temp_table[table1[table1[index1_tmp_2 + 1] + 3] +
               table2[table2[index2_tmp_2 * 18 + 1] * 18 + 3]] = 0;
    temp_table[table1[table1[index1_tmp_2 + 1] + 4] +
               table2[table2[index2_tmp_2 * 18 + 1] * 18 + 4]] = 0;
    temp_table[table1[table1[index1_tmp_2 + 1] + 5] +
               table2[table2[index2_tmp_2 * 18 + 1] * 18 + 5]] = 0;
    temp_table[table1[index1_tmp_2 + 2] + table2[index2_tmp_2 * 18 + 2]] = 0;
    temp_table[table1[table1[index1_tmp_2 + 2] + 3] +
               table2[table2[index2_tmp_2 * 18 + 2] * 18 + 3]] = 0;
    temp_table[table1[table1[index1_tmp_2 + 2] + 4] +
               table2[table2[index2_tmp_2 * 18 + 2] * 18 + 4]] = 0;
    temp_table[table1[table1[index1_tmp_2 + 2] + 5] +
               table2[table2[index2_tmp_2 * 18 + 2] * 18 + 5]] = 0;
  }

  long long count = 0;
  for (int i = 0; i < size; ++i)
    if (temp_table[i] == 0)
      count++;
  std::cout << "  " << log_prefix << " Depth 0: " << count << std::endl;

  for (int d = 0; d < depth; ++d) {
    next_d = d + 1;
    if (next_d >= 15)
      break;
    long long next_count = 0;
    for (int i = 0; i < size; ++i) {
      if (temp_table[i] == d) {
        index1_tmp = (i / size2) * 24;
        index2_tmp = (i % size2) * 18;
        for (int j = 0; j < 18; ++j) {
          next_i = table1[index1_tmp + j] + table2[index2_tmp + j];
          if (temp_table[next_i] == 0xF) {
            temp_table[next_i] = next_d;
            next_count++;
          }
        }
      }
    }
    std::cout << "  " << log_prefix << " Depth " << next_d << ": " << next_count
              << std::endl;
  }
  prune_table.resize((size + 1) / 2);
  std::fill(prune_table.begin(), prune_table.end(), 0xFF);
  for (int i = 0; i < size; ++i)
    if (temp_table[i] != 0xF)
      set_prune(prune_table, i, temp_table[i]);
}

// 生成 Pseudo Pair 表 (例如: prune_table_pseudo_pair_C4_E0.bin)
// index1: edge 初始索引 (0, 2, 4, 6)
// index2: corner 初始索引 (12, 15, 18, 21)
void create_prune_table_pseudo_pair(int index1, int index2, int size1,
                                    int size2, int depth,
                                    const std::vector<int> &table1,
                                    const std::vector<int> &table2,
                                    std::vector<unsigned char> &prune_table,
                                    const std::string &log_prefix) {
  int size = size1 * size2;
  std::vector<unsigned char> temp_table(size, 0xF);
  int start = index1 * size2 + index2, next_i, index1_tmp, index2_tmp, next_d;
  temp_table[start] = 0;
  temp_table[table1[index1 * 18 + 3] * size2 + table2[index2 * 18 + 3]] = 0;
  temp_table[table1[index1 * 18 + 4] * size2 + table2[index2 * 18 + 4]] = 0;
  temp_table[table1[index1 * 18 + 5] * size2 + table2[index2 * 18 + 5]] = 0;

  std::vector<std::string> appl_moves;
  std::vector<int> tmp_moves;
  if (index1 == 0) {
    appl_moves = {"L U L'", "L U' L'", "B' U B", "B' U' B"};
    tmp_moves = {0, 3, 4, 5};
  }
  if (index1 == 2) {
    appl_moves = {"R' U R", "R' U' R", "B U B'", "B U' B'"};
    tmp_moves = {5, 0, 3, 4};
  }
  if (index1 == 4) {
    appl_moves = {"R U R'", "R U' R'", "F' U F", "F' U' F"};
    tmp_moves = {4, 5, 0, 3};
  }
  if (index1 == 6) {
    appl_moves = {"L' U L", "L' U' L", "F U F'", "F U' F'"};
    tmp_moves = {3, 4, 5, 0};
  }

  for (int i = 0; i < 4; i++) {
    int index1_tmp_2 = index1, index2_tmp_2 = index2;
    index1_tmp_2 = table1[index1_tmp_2 * 18 + tmp_moves[index2 / 3 - 4]];
    index2_tmp_2 = table2[index2_tmp_2 * 18 + tmp_moves[index2 / 3 - 4]];
    for (int m : string_to_alg(appl_moves[i])) {
      index1_tmp_2 = table1[index1_tmp_2 * 18 + m];
      index2_tmp_2 = table2[index2_tmp_2 * 18 + m];
    }
    temp_table[index1_tmp_2 * size2 + index2_tmp_2] = 0;
    temp_table[table1[index1_tmp_2 * 18 + 3] * size2 +
               table2[index2_tmp_2 * 18 + 3]] = 0;
    temp_table[table1[index1_tmp_2 * 18 + 4] * size2 +
               table2[index2_tmp_2 * 18 + 4]] = 0;
    temp_table[table1[index1_tmp_2 * 18 + 5] * size2 +
               table2[index2_tmp_2 * 18 + 5]] = 0;
    temp_table[table1[index1_tmp_2 * 18] * size2 + table2[index2_tmp_2 * 18]] =
        0;
    temp_table[table1[table1[index1_tmp_2 * 18] * 18 + 3] * size2 +
               table2[table2[index2_tmp_2 * 18] * 18 + 3]] = 0;
    temp_table[table1[table1[index1_tmp_2 * 18] * 18 + 4] * size2 +
               table2[table2[index2_tmp_2 * 18] * 18 + 4]] = 0;
    temp_table[table1[table1[index1_tmp_2 * 18] * 18 + 5] * size2 +
               table2[table2[index2_tmp_2 * 18] * 18 + 5]] = 0;
    temp_table[table1[index1_tmp_2 * 18 + 1] * size2 +
               table2[index2_tmp_2 * 18 + 1]] = 0;
    temp_table[table1[table1[index1_tmp_2 * 18 + 1] * 18 + 3] * size2 +
               table2[table2[index2_tmp_2 * 18 + 1] * 18 + 3]] = 0;
    temp_table[table1[table1[index1_tmp_2 * 18 + 1] * 18 + 4] * size2 +
               table2[table2[index2_tmp_2 * 18 + 1] * 18 + 4]] = 0;
    temp_table[table1[table1[index1_tmp_2 * 18 + 1] * 18 + 5] * size2 +
               table2[table2[index2_tmp_2 * 18 + 1] * 18 + 5]] = 0;
    temp_table[table1[index1_tmp_2 * 18 + 2] * size2 +
               table2[index2_tmp_2 * 18 + 2]] = 0;
    temp_table[table1[table1[index1_tmp_2 * 18 + 2] * 18 + 3] * size2 +
               table2[table2[index2_tmp_2 * 18 + 2] * 18 + 3]] = 0;
    temp_table[table1[table1[index1_tmp_2 * 18 + 2] * 18 + 4] * size2 +
               table2[table2[index2_tmp_2 * 18 + 2] * 18 + 4]] = 0;
    temp_table[table1[table1[index1_tmp_2 * 18 + 2] * 18 + 5] * size2 +
               table2[table2[index2_tmp_2 * 18 + 2] * 18 + 5]] = 0;
  }

  long long count = 0;
  for (int i = 0; i < size; ++i)
    if (temp_table[i] == 0)
      count++;
  std::cout << "  " << log_prefix << " Depth 0: " << count << std::endl;

  for (int d = 0; d < depth; ++d) {
    next_d = d + 1;
    if (next_d >= 15)
      break;
    long long next_count = 0;
    for (int i = 0; i < size; ++i) {
      if (temp_table[i] == d) {
        index1_tmp = (i / size2) * 18;
        index2_tmp = (i % size2) * 18;
        for (int j = 0; j < 18; ++j) {
          next_i = table1[index1_tmp + j] * size2 + table2[index2_tmp + j];
          if (temp_table[next_i] == 0xF) {
            temp_table[next_i] = next_d;
            next_count++;
          }
        }
      }
    }
    std::cout << "  " << log_prefix << " Depth " << next_d << ": " << next_count
              << std::endl;
  }
  prune_table.resize((size + 1) / 2);
  std::fill(prune_table.begin(), prune_table.end(), 0xFF);
  for (int i = 0; i < size; ++i)
    if (temp_table[i] != 0xF)
      set_prune(prune_table, i, temp_table[i]);
}
