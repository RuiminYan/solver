/*
 * prune_tables.cpp - 剪枝表实现
 */

#include "prune_tables.h"
#include "move_tables.h"
#include <omp.h>

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
  std::cout << "Initializing prune tables..." << std::endl;

  genPTCross();
  genPTCrossInsC4();
  genPTPairC4E0();
  genPTCrossC4E0();
  genPTCrossC4C5E0E1();
  genPTCrossC4C6E0E2();
}

unsigned char *PruneTableManager::loadTableMMap(const std::string &filename) {
  // NOTE: 自动拼接 TABLE_DIR 前缀
  std::string path = std::string(TABLE_DIR) + filename;
#ifdef _WIN32
  std::cout << "MMap loading " << filename << "..." << std::endl;
  HANDLE hFile = CreateFile(path.c_str(), GENERIC_READ, FILE_SHARE_READ, NULL,
                            OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
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
  if (!loadTable(pt_cross, "pt_cross.bin"))
    return false;
  if (!loadTable(pt_cross_ins_C4, "pt_cross_ins_C4.bin"))
    return false;
  if (!loadTable(pt_pair_C4E0, "pt_pair_C4E0.bin"))
    return false;
  if (!loadTable(pt_cross_C4E0, "pt_cross_C4E0.bin"))
    return false;
  if (!loadTable(pt_cross_C4C5E0E1, "pt_cross_C4C5E0E1.bin"))
    return false;
  // 只要任意 analyzer 需要Diagonal 表就加载
  if (ENABLE_DIAGONAL_STD || ENABLE_DIAGONAL_PAIR || ENABLE_DIAGONAL_EO_CROSS) {
    if (!loadTable(pt_cross_C4C6E0E2, "pt_cross_C4C6E0E2.bin"))
      return false;
  }
  return true;
}

bool PruneTableManager::loadPseudoTables() {
  if (!loadTable(pt_pscross, "pt_pscross.bin"))
    return false;
  for (int i = 0; i < 4; ++i) {
    std::string fn = "pt_pscross_C4E" + std::to_string(i) + ".bin";
    if (!loadTable(pt_pscross_C4E[i], fn))
      return false;
  }

  // Edge2: 仅加载规范表 (E0E1=邻接, E0E2=对角)
  if (!loadTable(pt_pscross_Edge2[pairIdx(0, 1)], "pt_pscross_E0E1.bin")) {
    std::cout << "Warning: pt_pscross_E0E1.bin not found." << std::endl;
  }
  if (!loadTable(pt_pscross_Edge2[pairIdx(0, 2)], "pt_pscross_E0E2.bin")) {
    std::cout << "Warning: pt_pscross_E0E2.bin not found." << std::endl;
  }

  // Edge3: 仅加载规范表 (E0E1E2)
  // NOTE: 其他组合通过 conj 复用 E0E1E2
  if (!loadTable(pt_pscross_Edge3[tripleIdx(0, 1, 2)],
                 "pt_pscross_E0E1E2.bin")) {
    std::cout << "Warning: pt_pscross_E0E1E2.bin not found." << std::endl;
  }

  // Corner2: 仅加载规范表 (C4C5=邻角, C4C6=对角)
  if (!loadTable(pt_pscross_Corner2[pairIdx(0, 1)], "pt_pscross_C4C5.bin")) {
    std::cout << "Warning: pt_pscross_C4C5.bin not found." << std::endl;
  }
  if (!loadTable(pt_pscross_Corner2[pairIdx(0, 2)], "pt_pscross_C4C6.bin")) {
    std::cout << "Warning: pt_pscross_C4C6.bin not found." << std::endl;
  }

  // Corner3: 仅加载规范表 (C4C5C6)
  // NOTE: 其他组合通过 conj 复用 C4C5C6
  if (!loadTable(pt_pscross_Corner3[tripleIdx(0, 1, 2)],
                 "pt_pscross_C4C5C6.bin")) {
    std::cout << "Warning: pt_pscross_C4C5C6.bin not found." << std::endl;
  }

  return true;
}

bool PruneTableManager::loadPseudoPairTables() {
  // 1. 加载 Base 表(Cross + C{4-7})
  for (int c = 0; c < 4; ++c) {
    std::string fn = "pt_pscross_C" + std::to_string(c + 4) + ".bin";
    if (!loadTable(pt_pscross_C[c], fn))
      return false;
  }

  // 2. 加载 XC 表(Cross + C{4-7} diff{0-3})
  for (int e = 0; e < 4; ++e) {
    for (int c = 0; c < 4; ++c) {
      int idx = e * 4 + c;
      std::string fn = "pt_pscross_ins_C" + std::to_string(c + 4) + "_diff" +
                       std::to_string(e) + ".bin";
      if (!loadTable(pt_pscross_ins_C_diff[idx], fn))
        return false;
    }
  }

  // 3. 加载 EC 表 (Pair C{4-7}_E{0-3})
  for (int e = 0; e < 4; ++e) {
    for (int c = 0; c < 4; ++c) {
      int idx = e * 4 + c;
      std::string fn = "pt_pspair_C" + std::to_string(c + 4) + "_E" +
                       std::to_string(e) + ".bin";
      if (!loadTable(pt_pspair_CE[idx], fn))
        return false;
    }
  }

  // 4. 加载 Pseudo XCross Base (C4+E{0-3})
  // 这组表用了 Conj 优化，从 C4 视角追踪 XCross 状态
  for (int e = 0; e < 4; ++e) {
    std::string fn = "pt_pscross_C4E" + std::to_string(e) + ".bin";
    if (!loadTable(pt_pscross_C4E[e], fn))
      return false;
  }

  // 5. 加载 Aux 表 (复用 Pseudo Analyzer 的 Aux 表)
  // Edge2/Corner2/Edge3/Corner3 表已由 loadPseudoTables 加载，此处确保已加载
  if (pt_pscross_Edge2[pairIdx(0, 1)].empty()) {
    if (!loadTable(pt_pscross_Edge2[pairIdx(0, 1)], "pt_pscross_E0E1.bin")) {
      std::cout << "Warning: pt_pscross_E0E1.bin not found." << std::endl;
    }
  }
  if (pt_pscross_Edge2[pairIdx(0, 2)].empty()) {
    if (!loadTable(pt_pscross_Edge2[pairIdx(0, 2)], "pt_pscross_E0E2.bin")) {
      std::cout << "Warning: pt_pscross_E0E2.bin not found." << std::endl;
    }
  }
  if (pt_pscross_Corner2[pairIdx(0, 1)].empty()) {
    if (!loadTable(pt_pscross_Corner2[pairIdx(0, 1)], "pt_pscross_C4C5.bin")) {
      std::cout << "Warning: pt_pscross_C4C5.bin not found." << std::endl;
    }
  }
  if (pt_pscross_Corner2[pairIdx(0, 2)].empty()) {
    if (!loadTable(pt_pscross_Corner2[pairIdx(0, 2)], "pt_pscross_C4C6.bin")) {
      std::cout << "Warning: pt_pscross_C4C6.bin not found." << std::endl;
    }
  }
  if (pt_pscross_Edge3[tripleIdx(0, 1, 2)].empty()) {
    if (!loadTable(pt_pscross_Edge3[tripleIdx(0, 1, 2)],
                   "pt_pscross_E0E1E2.bin")) {
      std::cout << "Warning: pt_pscross_E0E1E2.bin not found." << std::endl;
    }
  }
  if (pt_pscross_Corner3[tripleIdx(0, 1, 2)].empty()) {
    if (!loadTable(pt_pscross_Corner3[tripleIdx(0, 1, 2)],
                   "pt_pscross_C4C5C6.bin")) {
      std::cout << "Warning: pt_pscross_C4C5C6.bin not found." << std::endl;
    }
  }

  return true;
}

// 加载 EOCross Analyzer 所需的表
// NOTE: 仅加载表到PruneTableManager，不负责生成
// 表生成逻辑仍保留在 eo_cross_analyzer.cpp，阶段2会迁移
bool PruneTableManager::loadEOCrossTables() {
  // 1. Cross+C4 (EOCross专用版本 - 使用不同生成算法)
  if (!loadTable(pt_eoc_ins_C4, "pt_cross_ins_C4.bin"))
    return false;

  // 2. Dependency+EO 表
  if (!loadTable(pt_ep4eo12, "pt_ep4eo12.bin"))
    return false;

  // 3. Plus Edge 表 (Right/Diag/Left)
  const char *edge_files[] = {"pt_cross_C4E0E1.bin", "pt_cross_C4E0E2.bin",
                              "pt_cross_C4E0E3.bin"};
  for (int i = 0; i < 3; ++i) {
    if (!loadTable(pt_cross_CEE[i], edge_files[i]))
      return false;
  }

  // 4. Plus Corner 表 (Right/Diag/Left)
  const char *corn_files[] = {"pt_cross_C4C5E0.bin", "pt_cross_C4C6E0.bin",
                              "pt_cross_C4C7E0.bin"};
  for (int i = 0; i < 3; ++i) {
    if (!loadTable(pt_cross_CCE[i], corn_files[i]))
      return false;
  }

  // 5. 3-Corner 表
  if (!loadTable(pt_cross_C4C5C6, "pt_cross_C4C5C6.bin"))
    return false;

  return true;
}

// --- 前向声明: Pseudo 表生成函数 ---
void createPTPsCrossCorn(int index2, int depth, const std::vector<int> &table1,
                         const std::vector<int> &table2,
                         std::vector<unsigned char> &prune_table);
void createPTPsCrossInsC(int index3, int index2, int depth,
                         const std::vector<int> &table1,
                         const std::vector<int> &table2,
                         std::vector<unsigned char> &prune_table);
void createPTPsPair(int index1, int index2, int size1, int size2, int depth,
                    const std::vector<int> &table1,
                    const std::vector<int> &table2,
                    std::vector<unsigned char> &prune_table);

void PruneTableManager::genAllSequentially() {
  auto &mtm = MoveTableManager::getInstance();

  // 1. Cross Prune (Needs Edges2)
  if (!fileExists("pt_cross.bin")) {
    std::cout << "Generating pt_cross.bin..." << std::endl;
    mtm.loadMTEdge2();
    genPTCross();
    std::vector<unsigned char>().swap(pt_cross);
    mtm.releaseMTEdge2();
  }

  // 2. Cross Ins C4 Prune (Needs Cross, Corner)
  if (!fileExists("pt_cross_ins_C4.bin")) {
    mtm.loadMTEdge4();
    mtm.loadMTCorn();
    genPTCrossInsC4();
    std::vector<unsigned char>().swap(pt_cross_ins_C4);
    mtm.releaseMTEdge4();
    mtm.releaseMTCorn();
  }

  // 3. Pair C4 E0 Prune (Needs Edge, Corner)
  if (!fileExists("pt_pair_C4E0.bin")) {
    mtm.loadMTEdge();
    mtm.loadMTCorn();
    genPTPairC4E0();
    std::vector<unsigned char>().swap(pt_pair_C4E0);
    mtm.releaseMTEdge();
    mtm.releaseMTCorn();
  }

  // 4. XCross C4 E0 Prune (Needs Cross, Corner, Edge)
  if (!fileExists("pt_cross_C4E0.bin")) {
    mtm.loadMTEdge4();
    mtm.loadMTCorn();
    mtm.loadMTEdge();
    genPTCrossC4E0();
    std::vector<unsigned char>().swap(pt_cross_C4E0);
    mtm.releaseMTEdge4();
    mtm.releaseMTCorn();
    mtm.releaseMTEdge();
  }

  // 5. Huge Neighbor (Needs Edge6, Corner2)
  if (!fileExists("pt_cross_C4C5E0E1.bin")) {
    mtm.loadMTEdge6();
    mtm.loadMTCorn2();
    genPTCrossC4C5E0E1();
    std::vector<unsigned char>().swap(pt_cross_C4C5E0E1);
    mtm.releaseMTEdge6();
    mtm.releaseMTCorn2();
  }

  // 6. Huge Diagonal (Needs Edge6, Corner2)
  // 只要任意 analyzer 需要 Diagonal 表就生成
  if (ENABLE_DIAGONAL_STD || ENABLE_DIAGONAL_PAIR || ENABLE_DIAGONAL_EO_CROSS) {
    if (!fileExists("pt_cross_C4C6E0E2.bin")) {
      mtm.loadMTEdge6();
      mtm.loadMTCorn2();
      genPTCrossC4C6E0E2();
      std::vector<unsigned char>().swap(pt_cross_C4C6E0E2);
      mtm.releaseMTEdge6();
      mtm.releaseMTCorn2();
    }
  }

  // 7. EO Dependency+EO Prune (Needs EOCross Move Tables: EP4 + EO_Alt)
  if (!fileExists("pt_ep4eo12.bin")) {
    mtm.loadMTEOCross();
    genPTEP4EO12();
    std::vector<unsigned char>().swap(pt_ep4eo12);
    // NOTE: EOCross 移动表较小，保持加载不释放
  }

  // 7.1 EOCross Plus Edge (3 张, 每张 ~1.22GB, Needs Cross, Corner, Edge)
  {
    bool need_plus = !fileExists("pt_cross_C4E0E1.bin") ||
                     !fileExists("pt_cross_C4E0E2.bin") ||
                     !fileExists("pt_cross_C4E0E3.bin");
    if (need_plus) {
      mtm.loadMTEdge4();
      mtm.loadMTCorn();
      mtm.loadMTEdge();
      for (int i = 0; i < 3; ++i) {
        genPTCrossCEE(i);
        std::vector<unsigned char>().swap(pt_cross_CEE[i]);
      }
      mtm.releaseMTEdge4();
      mtm.releaseMTCorn();
      mtm.releaseMTEdge();
    }
  }

  // 7.2 EOCross Plus Corner (3 x ~1.22GB, Needs Cross, Corner, Edge)
  {
    bool need_plus = !fileExists("pt_cross_C4C5E0.bin") ||
                     !fileExists("pt_cross_C4C6E0.bin") ||
                     !fileExists("pt_cross_C4C7E0.bin");
    if (need_plus) {
      mtm.loadMTEdge4();
      mtm.loadMTCorn();
      mtm.loadMTEdge();
      for (int i = 0; i < 3; ++i) {
        genPTCrossCCE(i);
        std::vector<unsigned char>().swap(pt_cross_CCE[i]);
      }
      mtm.releaseMTEdge4();
      mtm.releaseMTCorn();
      mtm.releaseMTEdge();
    }
  }

  // 7.3 EOCross 3-Corner (1 张, ~1.22GB, Needs Cross, Corner)
  if (!fileExists("pt_cross_C4C5C6.bin")) {
    mtm.loadMTEdge4();
    mtm.loadMTCorn();
    genPTCrossC4C5C6();
    std::vector<unsigned char>().swap(pt_cross_C4C5C6);
    mtm.releaseMTEdge4();
    mtm.releaseMTCorn();
  }

  // 8. Pseudo Cross Prune
  if (!fileExists("pt_pscross.bin")) {
    mtm.loadMTEdge2();
    genPTPsCross();
    std::vector<unsigned char>().swap(pt_pscross);
    mtm.releaseMTEdge2();
  }

  // 8. Pseudo XCross Prunes
  for (int i = 0; i < 4; ++i) {
    std::string fn = "pt_pscross_C4E" + std::to_string(i) + ".bin";
    if (!fileExists(fn)) {
      mtm.loadMTEdge4();
      mtm.loadMTCorn();
      mtm.loadMTEdge();
      genPTPsCrossC4E(i);
      std::vector<unsigned char>().swap(pt_pscross_C4E[i]);
      mtm.releaseMTEdge4();
      mtm.releaseMTCorn();
      mtm.releaseMTEdge();
    }
  }

  // 9. Pseudo Edge2 表 (6张, 需要 Edge4 + Edge2)
  {
    // C(4,2)=6 种组合: {0,1},{0,2},{0,3},{1,2},{1,3},{2,3}
    const int E2_PAIRS[][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
    for (auto &p : E2_PAIRS) {
      std::string fn = "pt_pscross_E" + std::to_string(p[0]) + "E" +
                       std::to_string(p[1]) + ".bin";
      if (!fileExists(fn)) {
        mtm.loadMTEdge4();
        mtm.loadMTEdge2();
        genPTPsCrossEdge2(p[0], p[1]);
        std::vector<unsigned char>().swap(
            pt_pscross_Edge2[pairIdx(p[0], p[1])]);
        mtm.releaseMTEdge4();
        mtm.releaseMTEdge2();
      }
    }
  }

  // 10. Pseudo Edge3 表 (4张, 需要 Edge4 + Edge3)
  {
    const int E3_TRIPLES[][3] = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
    for (auto &t : E3_TRIPLES) {
      std::string fn = "pt_pscross_E" + std::to_string(t[0]) + "E" +
                       std::to_string(t[1]) + "E" + std::to_string(t[2]) +
                       ".bin";
      if (!fileExists(fn)) {
        mtm.loadMTEdge4();
        mtm.loadMTEdge3();
        genPTPsCrossEdge3(t[0], t[1], t[2]);
        std::vector<unsigned char>().swap(
            pt_pscross_Edge3[tripleIdx(t[0], t[1], t[2])]);
        mtm.releaseMTEdge4();
        mtm.releaseMTEdge3();
      }
    }
  }

  // 11. Pseudo Corner2 表 (6张, 需要 Edge4 + Corner2)
  {
    const int C2_PAIRS[][2] = {{4, 5}, {4, 6}, {4, 7}, {5, 6}, {5, 7}, {6, 7}};
    for (auto &p : C2_PAIRS) {
      std::string fn = "pt_pscross_C" + std::to_string(p[0]) + "C" +
                       std::to_string(p[1]) + ".bin";
      if (!fileExists(fn)) {
        mtm.loadMTEdge4();
        mtm.loadMTCorn2();
        genPTPsCrossCorner2(p[0], p[1]);
        std::vector<unsigned char>().swap(
            pt_pscross_Corner2[pairIdx(p[0] - 4, p[1] - 4)]);
        mtm.releaseMTEdge4();
        mtm.releaseMTCorn2();
      }
    }
  }

  // 12. Pseudo Corner3 表 (4张, 需要 Edge4 + Corner3)
  {
    const int C3_TRIPLES[][3] = {{4, 5, 6}, {4, 5, 7}, {4, 6, 7}, {5, 6, 7}};
    for (auto &t : C3_TRIPLES) {
      std::string fn = "pt_pscross_C" + std::to_string(t[0]) + "C" +
                       std::to_string(t[1]) + "C" + std::to_string(t[2]) +
                       ".bin";
      if (!fileExists(fn)) {
        mtm.loadMTEdge4();
        mtm.loadMTCorn3();
        genPTPsCrossCorner3(t[0], t[1], t[2]);
        std::vector<unsigned char>().swap(
            pt_pscross_Corner3[tripleIdx(t[0] - 4, t[1] - 4, t[2] - 4)]);
        mtm.releaseMTEdge4();
        mtm.releaseMTCorn3();
      }
    }
  }

  // --- Pseudo Cross/XCross/Pair 变体表 (共36张) ---
  // 22. Cross + Corner 变体 (4张: C4~C7)
  for (int c = 0; c < 4; ++c)
    genPTPsCrossC(c);
  // 23. XCross 变体 (16张: C{4-7}_diff{0-3})
  for (int c = 0; c < 4; ++c)
    for (int e = 0; e < 4; ++e)
      genPTPsCrossInsCDiff(c, e);
  // 24. Pair 变体 (16张: C{4-7}_E{0-3})
  for (int c = 0; c < 4; ++c)
    for (int e = 0; e < 4; ++e)
      genPTPsPairCE(c, e);
}

// === 变体表生成函数实现 ===

// 角块索引映射表 (C4=12, C5=15, C6=18, C7=21)
static const int CORNER_INDICES[4] = {12, 15, 18, 21};
// 棱块索引映射表 (E0=0, E1=2, E2=4, E3=6)
static const int EDGE_INDICES[4] = {0, 2, 4, 6};

void PruneTableManager::genPTPsCrossC(int c) {
  std::string fn = "pt_pscross_C" + std::to_string(c + 4) + ".bin";
  if (fileExists(fn))
    return;
  auto &mtm = MoveTableManager::getInstance();
  mtm.loadMTEdge4();
  mtm.loadMTCorn();
  GenerationTimer timer;
  std::cout << "  Generating " << fn << "..." << std::endl;
  std::vector<unsigned char> temp;
  createPTPsCrossCorn(CORNER_INDICES[c], 10, mtm.getMTEdge4(), mtm.getMTCorn(),
                      temp);
  saveTable(temp, fn);
  timer.printElapsed(fn);
  mtm.releaseMTEdge4();
  mtm.releaseMTCorn();
}

void PruneTableManager::genPTPsCrossInsCDiff(int c, int e) {
  std::string fn = "pt_pscross_ins_C" + std::to_string(c + 4) + "_diff" +
                   std::to_string(e) + ".bin";
  if (fileExists(fn))
    return;
  auto &mtm = MoveTableManager::getInstance();
  mtm.loadMTEdge4();
  mtm.loadMTCorn();
  GenerationTimer timer;
  std::cout << "  Generating " << fn << "..." << std::endl;
  std::vector<unsigned char> temp;
  createPTPsCrossInsC(EDGE_INDICES[e], CORNER_INDICES[c], 10, mtm.getMTEdge4(),
                      mtm.getMTCorn(), temp);
  saveTable(temp, fn);
  timer.printElapsed(fn);
  mtm.releaseMTEdge4();
  mtm.releaseMTCorn();
}

void PruneTableManager::genPTPsPairCE(int c, int e) {
  std::string fn =
      "pt_pspair_C" + std::to_string(c + 4) + "_E" + std::to_string(e) + ".bin";
  if (fileExists(fn))
    return;
  auto &mtm = MoveTableManager::getInstance();
  mtm.loadMTEdge();
  mtm.loadMTCorn();
  GenerationTimer timer;
  std::cout << "  Generating " << fn << "..." << std::endl;
  std::vector<unsigned char> temp;
  createPTPsPair(EDGE_INDICES[e], CORNER_INDICES[c], StateSpace::EDGE,
                 StateSpace::CORNER, 8, mtm.getMTEdge(), mtm.getMTCorn(), temp);
  saveTable(temp, fn);
  timer.printElapsed(fn);
  mtm.releaseMTEdge();
  mtm.releaseMTCorn();
}

bool PruneTableManager::loadTable(std::vector<unsigned char> &table,
                                  const std::string &filename) {
  // NOTE: 自动拼接 TABLE_DIR 前缀，调用方只需传裸文件名
  std::string path = std::string(TABLE_DIR) + filename;
  if (load_vector_chunked(table, path)) {
    // 计算并打印表大小
    size_t size_bytes = table.size() * sizeof(unsigned char);
    std::cout << TAG_COLOR << "[LOAD]" << ANSI_RESET << " ("
              << formatFileSize(size_bytes) << ") " << filename << std::endl;
    return true;
  }
  return false;
}

void PruneTableManager::saveTable(const std::vector<unsigned char> &table,
                                  const std::string &filename) {
  // NOTE: 自动拼接 TABLE_DIR 前缀
  std::string path = std::string(TABLE_DIR) + filename;
  save_vector_chunked(table, path);
}

void PruneTableManager::genPTCross() {
  if (loadTable(pt_cross, "pt_cross.bin"))
    return;
  GenerationTimer timer;
  std::cout << "Generating pt_cross.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  const auto &edge2_mt = mtm.getMTEdge2();
  long long sz = (long long)StateSpace::EDGE2 * StateSpace::EDGE2;
  std::vector<unsigned char> tmp(sz, 255);
  int i1 = StateSpace::EDGE2_A_SOLVED, i2 = StateSpace::EDGE2_B_SOLVED;
  tmp[(long long)i1 * StateSpace::EDGE2 + i2] = 0;
  DistributionPrinter dp(sz);
  for (int d = 0; d < 10; ++d) {
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < sz; ++i) {
      dp.progress(i, sz, d);
      if (tmp[i] == d) {
        cnt++;
        int idx1 = (i / StateSpace::EDGE2) * 18,
            idx2 = (i % StateSpace::EDGE2) * 18;
        for (int j = 0; j < 18; ++j) {
          long long ni = (long long)edge2_mt[idx1 + j] * StateSpace::EDGE2 +
                         edge2_mt[idx2 + j];
          // NOTE: 使用 CAS 避免竞态条件
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&tmp[ni], expected,
                                      (unsigned char)(d + 1));
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  int c_sz = (sz + 1) / 2;
  pt_cross.assign(c_sz, 0xFF);
  for (long long i = 0; i < sz; ++i)
    set_prune(pt_cross, i, tmp[i]);
  saveTable(pt_cross, "pt_cross.bin");
  timer.printElapsed("pt_cross.bin");
}

void PruneTableManager::genPTCrossInsC4() {
  if (loadTable(pt_cross_ins_C4, "pt_cross_ins_C4.bin"))
    return;
  GenerationTimer timer;
  std::cout << "Generating pt_cross_ins_C4.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  pt_cross_ins_C4.resize(
      ((long long)StateSpace::CROSS * StateSpace::CORNER + 1) / 2, 0xFF);
  createPTCrossInsC(StateSpace::CROSS_SOLVED, 12, StateSpace::CROSS,
                    StateSpace::CORNER, 10, mtm.getMTEdge4(), mtm.getMTCorn(),
                    pt_cross_ins_C4);
  saveTable(pt_cross_ins_C4, "pt_cross_ins_C4.bin");
  timer.printElapsed("pt_cross_ins_C4.bin");
}

void PruneTableManager::genPTPairC4E0() {
  if (loadTable(pt_pair_C4E0, "pt_pair_C4E0.bin"))
    return;
  GenerationTimer timer;
  std::cout << "Generating pt_pair_C4E0.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  pt_pair_C4E0.resize(
      ((long long)StateSpace::EDGE * StateSpace::CORNER + 1) / 2, 0xFF);
  createPTPair(0, 12, StateSpace::EDGE, StateSpace::CORNER, 8, mtm.getMTEdge(),
               mtm.getMTCorn(), pt_pair_C4E0);
  saveTable(pt_pair_C4E0, "pt_pair_C4E0.bin");
  timer.printElapsed("pt_pair_C4E0.bin");
}

void PruneTableManager::genPTCrossC4E0() {
  if (loadTable(pt_cross_C4E0, "pt_cross_C4E0.bin"))
    return;
  GenerationTimer timer;
  std::cout << "Generating pt_cross_C4E0.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  long long c_sz =
      ((long long)StateSpace::CROSS * StateSpace::CORNER * StateSpace::EDGE +
       1) /
      2;
  pt_cross_C4E0.resize(c_sz, 0xFF);
  createPTCrossCE(StateSpace::CROSS_SOLVED, 12, 0, StateSpace::CROSS,
                  StateSpace::CORNER, StateSpace::EDGE, 11, mtm.getMTEdge4(),
                  mtm.getMTCorn(), mtm.getMTEdge(), pt_cross_C4E0);
  saveTable(pt_cross_C4E0, "pt_cross_C4E0.bin");
  timer.printElapsed("pt_cross_C4E0.bin");
}

void PruneTableManager::genPTCrossC4C5E0E1() {
  if (loadTable(pt_cross_C4C5E0E1, "pt_cross_C4C5E0E1.bin"))
    return;
  GenerationTimer timer;
  std::cout << "Generating pt_cross_C4C5E0E1.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  createPTEdge6Corn2(StateSpace::EDGE6, StateSpace::CORNER2, 15,
                     {0, 2, 16, 18, 20, 22}, {12, 15}, mtm.getMTEdge6(),
                     mtm.getMTCorn2(), pt_cross_C4C5E0E1);
  saveTable(pt_cross_C4C5E0E1, "pt_cross_C4C5E0E1.bin");
  timer.printElapsed("pt_cross_C4C5E0E1.bin");
}

void PruneTableManager::genPTCrossC4C6E0E2() {
  if (!ENABLE_DIAGONAL_STD && !ENABLE_DIAGONAL_PAIR &&
      !ENABLE_DIAGONAL_EO_CROSS)
    return;
  if (loadTable(pt_cross_C4C6E0E2, "pt_cross_C4C6E0E2.bin"))
    return;
  GenerationTimer timer;
  std::cout << "Generating pt_cross_C4C6E0E2.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  createPTEdge6Corn2(StateSpace::EDGE6, StateSpace::CORNER2, 15,
                     {0, 4, 16, 18, 20, 22}, {12, 18}, mtm.getMTEdge6(),
                     mtm.getMTCorn2(), pt_cross_C4C6E0E2);
  saveTable(pt_cross_C4C6E0E2, "pt_cross_C4C6E0E2.bin");
  timer.printElapsed("pt_cross_C4C6E0E2.bin");
}

void PruneTableManager::genPTEP4EO12() {
  if (loadTable(pt_ep4eo12, "pt_ep4eo12.bin"))
    return;
  GenerationTimer timer;
  std::cout << "Generating pt_ep4eo12.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  // 状态空间: EP4 (12*11*10*9 = 11880) x EO12 (2^11 = 2048)
  // 初始状态: EP4_SOLVED=11720, EO_SOLVED=0
  // 使用 MoveTableManager 中已加载的移动表
  createPTDim2(StateSpace::EP4_SOLVED, 0, StateSpace::EP4, StateSpace::EO12, 11,
               mtm.getMTEP4(), mtm.getMTEOAlt(), pt_ep4eo12);
  saveTable(pt_ep4eo12, "pt_ep4eo12.bin");
  timer.printElapsed("pt_ep4eo12.bin");
}

// === EOCross Plus Edge 生成函数 (参数化) ===
// NOTE: 状态空间 Cross(187560) × C4(24) × E0(24) × Extra(24)
// i=0: E1(Right), i=1: E2(Diag), i=2: E3(Left)
void PruneTableManager::genPTCrossCEE(int i) {
  // .bin 文件名映射: 0→E1, 1→E2, 2→E3
  static const char *FILENAMES[] = {
      "pt_cross_C4E0E1.bin", "pt_cross_C4E0E2.bin", "pt_cross_C4E0E3.bin"};
  const char *fn = FILENAMES[i];
  if (loadTable(pt_cross_CEE[i], fn))
    return;
  GenerationTimer timer;
  std::cout << "Generating " << fn << "..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  // idx_extra: E1=2, E2=4, E3=6 → EDGE_INDICES[i+1]
  int idx_extra = EDGE_INDICES[i + 1];
  createPTCrossCEX(StateSpace::CROSS_SOLVED, 12, 0, idx_extra,
                   StateSpace::CROSS, StateSpace::CORNER, StateSpace::EDGE,
                   StateSpace::EDGE, 14, mtm.getMTEdge4(), mtm.getMTCorn(),
                   mtm.getMTEdge(), mtm.getMTEdge(), pt_cross_CEE[i]);
  saveTable(pt_cross_CEE[i], fn);
  timer.printElapsed(fn);
}

// === EOCross Plus Corner 生成函数 (参数化) ===
// NOTE: 与 Plus Edge 结构相同，但 idx_extra 是角块初始状态，t4=CornMT
// i=0: C5(Right), i=1: C6(Diag), i=2: C7(Left)
void PruneTableManager::genPTCrossCCE(int i) {
  // .bin 文件名映射: 0→C5, 1→C6, 2→C7
  static const char *FILENAMES[] = {
      "pt_cross_C4C5E0.bin", "pt_cross_C4C6E0.bin", "pt_cross_C4C7E0.bin"};
  const char *fn = FILENAMES[i];
  if (loadTable(pt_cross_CCE[i], fn))
    return;
  GenerationTimer timer;
  std::cout << "Generating " << fn << "..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  // idx_extra: C5=15, C6=18, C7=21 → CORNER_INDICES[i+1]
  int idx_extra = CORNER_INDICES[i + 1];
  createPTCrossCEX(StateSpace::CROSS_SOLVED, 12, 0, idx_extra,
                   StateSpace::CROSS, StateSpace::CORNER, StateSpace::EDGE,
                   StateSpace::CORNER, 14, mtm.getMTEdge4(), mtm.getMTCorn(),
                   mtm.getMTEdge(), mtm.getMTCorn(), pt_cross_CCE[i]);
  saveTable(pt_cross_CCE[i], fn);
  timer.printElapsed(fn);
}

// === EOCross 3-Corner 生成函数 ===
// NOTE: 使用 createPTCrossCCC，跟踪 C4+C5+C6 三个角块
// 参数来源: eo_cross_analyzer_old.cpp:L310-L313

void PruneTableManager::genPTCrossC4C5C6() {
  if (loadTable(pt_cross_C4C5C6, "pt_cross_C4C5C6.bin"))
    return;
  GenerationTimer timer;
  std::cout << "Generating pt_cross_C4C5C6.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  // 3-Corner: idx_c5=15, idx_c6=18, t_c5=CornMT, t_c6=CornMT
  createPTCrossCCC(StateSpace::CROSS_SOLVED, 12, 15, 18, StateSpace::CROSS,
                   StateSpace::CORNER, StateSpace::CORNER, StateSpace::CORNER,
                   14, mtm.getMTEdge4(), mtm.getMTCorn(), mtm.getMTCorn(),
                   mtm.getMTCorn(), pt_cross_C4C5C6);
  saveTable(pt_cross_C4C5C6, "pt_cross_C4C5C6.bin");
  timer.printElapsed("pt_cross_C4C5C6.bin");
}

void PruneTableManager::genPTPsCross() {
  if (loadTable(pt_pscross, "pt_pscross.bin"))
    return;
  GenerationTimer timer;
  std::cout << "Generating pt_pscross.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  const auto &edge2_mt = mtm.getMTEdge2();
  long long sz = (long long)StateSpace::EDGE2 * StateSpace::EDGE2;
  std::vector<unsigned char> tmp(sz, 255);
  int d_moves[] = {-1, 3, 4, 5};
  for (int k = 0; k < 4; ++k) {
    int i1 = StateSpace::EDGE2_A_SOLVED, i2 = StateSpace::EDGE2_B_SOLVED;
    if (k > 0) {
      i1 = edge2_mt[i1 * 18 + d_moves[k]];
      i2 = edge2_mt[i2 * 18 + d_moves[k]];
    }
    tmp[(long long)i1 * StateSpace::EDGE2 + i2] = 0;
  }
  DistributionPrinter dp(sz);
  for (int d = 0; d < 10; ++d) {
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < sz; ++i) {
      dp.progress(i, sz, d);
      if (tmp[i] == d) {
        cnt++;
        int idx1 = (i / StateSpace::EDGE2) * 18,
            idx2 = (i % StateSpace::EDGE2) * 18;
        for (int j = 0; j < 18; ++j) {
          long long ni = (long long)edge2_mt[idx1 + j] * StateSpace::EDGE2 +
                         edge2_mt[idx2 + j];
          // NOTE: 使用 CAS 避免竞态条件
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&tmp[ni], expected,
                                      (unsigned char)(d + 1));
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  int c_sz = (sz + 1) / 2;
  pt_pscross.assign(c_sz, 0xFF);
  for (long long i = 0; i < sz; ++i)
    set_prune(pt_pscross, i, tmp[i]);
  saveTable(pt_pscross, "pt_pscross.bin");
  timer.printElapsed("pt_pscross.bin");
}

void PruneTableManager::genPTPsCrossC4E(int i) {
  std::string fn = "pt_pscross_C4E" + std::to_string(i) + ".bin";
  if (loadTable(pt_pscross_C4E[i], fn))
    return;
  GenerationTimer timer;
  std::cout << "Generating " << fn << "..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  int e_diffs[] = {0, 2, 4, 6};
  pt_pscross_C4E[i].resize(
      ((long long)StateSpace::CROSS * StateSpace::CORNER * StateSpace::EDGE +
       1) /
          2,
      0xFF);
  createPTCrossCE(StateSpace::CROSS_SOLVED, 12, e_diffs[i], StateSpace::CROSS,
                  StateSpace::CORNER, StateSpace::EDGE, 11, mtm.getMTEdge4(),
                  mtm.getMTCorn(), mtm.getMTEdge(), pt_pscross_C4E[i], true);
  saveTable(pt_pscross_C4E[i], fn);
  timer.printElapsed(fn);
}

// === Edge2 参数化生成: a,b ∈ {0..3} ===
void PruneTableManager::genPTPsCrossEdge2(int a, int b) {
  // 文件名保持向后兼容: pt_pscross_E{a}E{b}.bin
  std::string fn =
      "pt_pscross_E" + std::to_string(a) + "E" + std::to_string(b) + ".bin";
  int idx = pairIdx(a, b);
  if (loadTable(pt_pscross_Edge2[idx], fn))
    return;
  GenerationTimer timer;
  std::cout << "Generating " << fn << "..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  // target: 每个棱块编号乘以步长 2
  std::vector<int> target = {a * 2, b * 2};
  std::sort(target.begin(), target.end());
  int idx_solved = array_to_index(target, 2, 2, 12);
  pt_pscross_Edge2[idx].resize(
      ((long long)StateSpace::CROSS * StateSpace::EDGE2 + 1) / 2, 0xFF);
  createPTPsCrossEdge2(StateSpace::CROSS_SOLVED, idx_solved, StateSpace::CROSS,
                       StateSpace::EDGE2, 11, mtm.getMTEdge4(),
                       mtm.getMTEdge2(), pt_pscross_Edge2[idx]);
  saveTable(pt_pscross_Edge2[idx], fn);
  timer.printElapsed(fn.c_str());
}

// === Corner2 参数化生成: a,b ∈ {4..7} ===
void PruneTableManager::genPTPsCrossCorner2(int a, int b) {
  std::string fn =
      "pt_pscross_C" + std::to_string(a) + "C" + std::to_string(b) + ".bin";
  int idx = pairIdx(a - 4, b - 4);
  if (loadTable(pt_pscross_Corner2[idx], fn))
    return;
  GenerationTimer timer;
  std::cout << "Generating " << fn << "..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  // target: 每个角块编号乘以步长 3
  std::vector<int> target = {a * 3, b * 3};
  std::sort(target.begin(), target.end());
  int idx_solved = array_to_index(target, 2, 3, 8);
  pt_pscross_Corner2[idx].resize(
      ((long long)StateSpace::CROSS * StateSpace::CORNER2 + 1) / 2, 0xFF);
  createPTPsCrossCorn2(StateSpace::CROSS_SOLVED, idx_solved, StateSpace::CROSS,
                       StateSpace::CORNER2, 11, mtm.getMTEdge4(),
                       mtm.getMTCorn2(), pt_pscross_Corner2[idx]);
  saveTable(pt_pscross_Corner2[idx], fn);
  timer.printElapsed(fn.c_str());
}

// === Edge3 参数化生成: a,b,c ∈ {0..3} ===
void PruneTableManager::genPTPsCrossEdge3(int a, int b, int c) {
  std::string fn = "pt_pscross_E" + std::to_string(a) + "E" +
                   std::to_string(b) + "E" + std::to_string(c) + ".bin";
  int idx = tripleIdx(a, b, c);
  if (loadTable(pt_pscross_Edge3[idx], fn))
    return;
  GenerationTimer timer;
  std::cout << "Generating " << fn << "..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {a * 2, b * 2, c * 2};
  std::sort(target.begin(), target.end());
  int idx_solved = array_to_index(target, 3, 2, 12);
  pt_pscross_Edge3[idx].resize(
      ((long long)StateSpace::CROSS * StateSpace::EDGE3 + 1) / 2, 0xFF);
  createPTPsCrossEdge3(StateSpace::CROSS_SOLVED, idx_solved, StateSpace::CROSS,
                       StateSpace::EDGE3, 12, mtm.getMTEdge4(),
                       mtm.getMTEdge3(), pt_pscross_Edge3[idx]);
  saveTable(pt_pscross_Edge3[idx], fn);
  timer.printElapsed(fn.c_str());
}

// === Corner3 参数化生成: a,b,c ∈ {4..7} ===
void PruneTableManager::genPTPsCrossCorner3(int a, int b, int c) {
  std::string fn = "pt_pscross_C" + std::to_string(a) + "C" +
                   std::to_string(b) + "C" + std::to_string(c) + ".bin";
  int idx = tripleIdx(a - 4, b - 4, c - 4);
  if (loadTable(pt_pscross_Corner3[idx], fn))
    return;
  GenerationTimer timer;
  std::cout << "Generating " << fn << "..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {a * 3, b * 3, c * 3};
  std::sort(target.begin(), target.end());
  int idx_solved = array_to_index(target, 3, 3, 8);
  pt_pscross_Corner3[idx].resize(
      ((long long)StateSpace::CROSS * StateSpace::CORNER3 + 1) / 2, 0xFF);
  createPTPsCrossCorn3(StateSpace::CROSS_SOLVED, idx_solved, StateSpace::CROSS,
                       StateSpace::CORNER3, 13, mtm.getMTEdge4(),
                       mtm.getMTCorn3(), pt_pscross_Corner3[idx]);
  saveTable(pt_pscross_Corner3[idx], fn);
  timer.printElapsed(fn.c_str());
}
