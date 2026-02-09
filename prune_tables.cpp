/*
 * prune_tables.cpp - 剪枝表实现
 */

#include "prune_tables.h"
#include "move_tables.h"
#include <iomanip>
#include <omp.h>

#ifdef _WIN32
#include <windows.h>
#endif

PruneTableManager *PruneTableManager::instance = nullptr;

// 深度分布打印器- 用于打印带百分比的深度分布表格
// NOTE: 使用 ANSI 转义码在 done() 时覆盖输出，修正百分比为 Count/Total
struct DistributionPrinter {
  long long total_size;                           // 状态空间大小
  long long accumulated;                          // 累计计数
  std::vector<std::pair<int, long long>> records; // 存储 (depth, count)

  // 格式化数字为带千分位的字符串
  static std::string formatWithCommas(long long n) {
    std::string s = std::to_string(n);
    int insertPos = s.length() - 3;
    while (insertPos > 0) {
      s.insert(insertPos, ",");
      insertPos -= 3;
    }
    return s;
  }

  DistributionPrinter(long long total) : total_size(total), accumulated(0) {
    // 打印状态空间大小
    std::cout << "  State Space: " << formatWithCommas(total_size) << std::endl;
    // 打印表头（上下都有横线，类似 cloc 风格)
    std::cout << "  -------------------------------------------------"
              << std::endl;
    std::cout << "  Depth         Count           Pct         Cum" << std::endl;
    std::cout << "  -------------------------------------------------"
              << std::endl;
  }

  void print(int depth, long long count) {
    if (count == 0)
      return; // 跳过空行
    accumulated += count;
    records.push_back({depth, count});
    // 临时打印（百分比稍后修正)
    double pct = (total_size > 0) ? (100.0 * count / total_size) : 0.0;
    double cumPct = (total_size > 0) ? (100.0 * accumulated / total_size) : 0.0;
    std::cout << "  " << std::setw(5) << std::right << depth << "  "
              << std::setw(14) << std::right << formatWithCommas(count) << "  "
              << std::fixed << std::setprecision(6) << std::setw(10)
              << std::right << pct << "%  " << std::setw(10) << std::right
              << cumPct << "%" << std::endl;
  }

  void done() {
    // 光标上移 records.size() 行，重新打印正确的百分比
    int lineCount = records.size();
    std::cout << "\033[" << lineCount << "A"; // ANSI: 上移 lineCount 行

    long long total = accumulated; // 最终 Total
    long long cumSum = 0;
    for (auto &p : records) {
      cumSum += p.second;
      double pct = (total > 0) ? (100.0 * p.second / total) : 0.0;
      double cumPct = (total > 0) ? (100.0 * cumSum / total) : 0.0;
      std::cout << "\033[2K"; // ANSI: 清除当前行
      std::cout << "  " << std::setw(5) << std::right << p.first << "  "
                << std::setw(14) << std::right << formatWithCommas(p.second)
                << "  " << std::fixed << std::setprecision(6) << std::setw(10)
                << std::right << pct << "%  " << std::setw(10) << std::right
                << cumPct << "%" << std::endl;
    }

    std::cout << "  -------------------------------------------------"
              << std::endl;
    std::cout << "  Total  " << std::setw(14) << std::right
              << formatWithCommas(accumulated) << "  100.000000%" << std::endl;
    std::cout << std::endl; // 空行分隔
  }

  // 在并发BFS 循环内部显示扫描进度
  // 位掩码检查放最前面（最便宜），绝大多数迭代在此处 return
  void progress(long long i, long long loopTotal, int depth) {
    // 每 ~100 万次迭代检查一次（便宜的位运算，放第一位）
    if ((i & 0xFFFFF) != 0)
      return;
    // 仅线程 0 报告（避免cout 竞争)
    if (omp_get_thread_num() != 0)
      return;
    int numThreads = omp_get_num_threads();
    int pct = (int)(i * numThreads * 100 / loopTotal);
    if (pct > 99)
      pct = 99;
    std::cout << "\033[?25l\r  Scanning depth " << depth << ": " << pct << "%  "
              << std::flush;
  }

  // 在 dp.print() 前调用，清除进度行
  void clearProgress() { std::cout << "\r\033[2K\033[?25h" << std::flush; }
};

PruneTableManager &PruneTableManager::getInstance() {
  if (instance == nullptr) {
    instance = new PruneTableManager();
  }
  return *instance;
}

void PruneTableManager::initialize() {
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Initializing prune tables..." << std::endl;

  genPTCross();
  genPTCrossC4();
  genPTPairC4E0();
  genPTCrossC4E0();
  genPTCrossC4C5E0E1();
  genPTCrossC4C6E0E2();
}

unsigned char *PruneTableManager::loadTableMMap(const std::string &filename) {
#ifdef _WIN32
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET << " MMap loading "
            << filename << "..." << std::endl;
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
  if (!loadTable(pt_cross, "pt_cross.bin"))
    return false;
  if (!loadTable(pt_cross_C4, "pt_cross_C4.bin"))
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
  if (!loadTable(pt_pscross_E0E2, "pt_pscross_E0E2.bin")) {
    std::cout << "Warning: pt_pscross_E0E2.bin not found." << std::endl;
  }
  // 新增邻棱表加载
  if (!loadTable(pt_pscross_E0E1, "pt_pscross_E0E1.bin")) {
    std::cout << "Warning: pt_pscross_E0E1.bin not found." << std::endl;
  }

  // Edge3 Tables Loading
  if (!loadTable(pt_pscross_E0E1E2, "pt_pscross_E0E1E2.bin")) {
    std::cout << "Warning: pt_pscross_E0E1E2.bin not found." << std::endl;
  }
  // NOTE: E1_E2_E3, E0_E2_E3, E0_E1_E3 通过 conj 复用 E0_E1_E2，不再加载

  // 对角表加载
  if (!loadTable(pt_pscross_C4C6, "pt_pscross_C4C6.bin")) {
    std::cout << "Warning: pt_pscross_C4C6.bin not found." << std::endl;
  }
  // 邻角表加载
  if (!loadTable(pt_pscross_C4C5, "pt_pscross_C4C5.bin")) {
    std::cout << "Warning: pt_pscross_C4C5.bin not found." << std::endl;
  }

  // Corner3 Tables Loading
  if (!loadTable(pt_pscross_C4C5C6, "pt_pscross_C4C5C6.bin")) {
    std::cout << "Warning: pt_pscross_C4C5C6.bin not found." << std::endl;
  }
  // NOTE: C4_C5_C7, C4_C6_C7, C5_C6_C7 通过 conj 复用 C4_C5_C6，不再加载

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
      std::string fn = "pt_pscross_C" + std::to_string(c + 4) + "_diff" +
                       std::to_string(e) + ".bin";
      if (!loadTable(pt_pscross_C_diff[idx], fn))
        return false;
    }
  }

  // 3. 加载 EC ?(Pair C{4-7}_E{0-3})
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
  // 这组表用?Conj 优化，从 C4 视角追踪 XCross 状?
  for (int e = 0; e < 4; ++e) {
    std::string fn = "pt_pscross_C4E" + std::to_string(e) + ".bin";
    if (!loadTable(pt_pscross_C4E[e], fn))
      return false;
  }

  // 5. 加载 Aux ?(复用 Pseudo Analyzer ?Aux ?
  // E2/C2/E3/C3 表已?loadPseudoTables 加载，此处确保已加载
  if (pt_pscross_E0E1.empty()) {
    if (!loadTable(pt_pscross_E0E1, "pt_pscross_E0E1.bin")) {
      std::cout << "Warning: pt_pscross_E0E1.bin not found." << std::endl;
    }
  }
  if (pt_pscross_E0E2.empty()) {
    if (!loadTable(pt_pscross_E0E2, "pt_pscross_E0E2.bin")) {
      std::cout << "Warning: pt_pscross_E0E2.bin not found." << std::endl;
    }
  }
  if (pt_pscross_C4C5.empty()) {
    if (!loadTable(pt_pscross_C4C5, "pt_pscross_C4C5.bin")) {
      std::cout << "Warning: pt_pscross_C4C5.bin not found." << std::endl;
    }
  }
  if (pt_pscross_C4C6.empty()) {
    if (!loadTable(pt_pscross_C4C6, "pt_pscross_C4C6.bin")) {
      std::cout << "Warning: pt_pscross_C4C6.bin not found." << std::endl;
    }
  }
  if (pt_pscross_E0E1E2.empty()) {
    if (!loadTable(pt_pscross_E0E1E2, "pt_pscross_E0E1E2.bin")) {
      std::cout << "Warning: pt_pscross_E0E1E2.bin not found." << std::endl;
    }
  }
  if (pt_pscross_C4C5C6.empty()) {
    if (!loadTable(pt_pscross_C4C5C6, "pt_pscross_C4C5C6.bin")) {
      std::cout << "Warning: pt_pscross_C4C5C6.bin not found." << std::endl;
    }
  }

  return true;
}

// 加载 EOCross Analyzer 所需的表
// NOTE: 仅加载表到PruneTableManager，不负责生成
// 表生成逻辑仍保留在eo_cross_analyzer.cpp?阶段2会迁?
bool PruneTableManager::loadEOCrossTables() {
  // 1. Cross+C4 (EOCross专用版本 - 使用不同生成算法)
  if (!loadTable(pt_eoc_C4, "pt_cross_C4.bin"))
    return false;

  // 2. Dependency+EO ?
  if (!loadTable(pt_ep4eo12, "pt_ep4eo12.bin"))
    return false;

  // 3. Plus Edge ?(Right/Diag/Left)
  const char *edge_files[] = {"pt_cross_C4E0E1.bin", "pt_cross_C4E0E2.bin",
                              "pt_cross_C4E0E3.bin"};
  for (int i = 0; i < 3; ++i) {
    if (!loadTable(pt_cross_CEE[i], edge_files[i]))
      return false;
  }

  // 4. Plus Corner ?(Right/Diag/Left)
  const char *corn_files[] = {"pt_cross_C4C5E0.bin", "pt_cross_C4C6E0.bin",
                              "pt_cross_C4C7E0.bin"};
  for (int i = 0; i < 3; ++i) {
    if (!loadTable(pt_cross_CCE[i], corn_files[i]))
      return false;
  }

  // 5. 3-Corner ?
  if (!loadTable(pt_cross_C4C5C6, "pt_cross_C4C5C6.bin"))
    return false;

  return true;
}

// --- 前向声明: Pseudo 表生成函?---
void create_pt_pscross_corner(int index2, int depth,
                              const std::vector<int> &table1,
                              const std::vector<int> &table2,
                              std::vector<unsigned char> &prune_table);
void create_pt_pscross_xcross(int index3, int index2, int depth,
                              const std::vector<int> &table1,
                              const std::vector<int> &table2,
                              std::vector<unsigned char> &prune_table);
void create_pt_pspair(int index1, int index2, int size1, int size2, int depth,
                      const std::vector<int> &table1,
                      const std::vector<int> &table2,
                      std::vector<unsigned char> &prune_table);

void PruneTableManager::genAllSequentially() {
  auto &mtm = MoveTableManager::getInstance();

  // 1. Cross Prune (Needs Edges2)
  if (!fileExists("pt_cross.bin")) {
    std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
              << " Generating pt_cross.bin..." << std::endl;
    mtm.loadMTEdge2();
    genPTCross();
    std::vector<unsigned char>().swap(pt_cross);
    mtm.releaseMTEdge2();
  }

  // 2. Cross C4 Prune (Needs Cross, Corner)
  if (!fileExists("pt_cross_C4.bin")) {
    mtm.loadMTEdge4();
    mtm.loadMTCorn();
    genPTCrossC4();
    std::vector<unsigned char>().swap(pt_cross_C4);
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
  // 只要任意 analyzer 需?Diagonal 表就生成
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
    // NOTE: EOCross 移动表较小，保持加载不释?
  }

  // 7.1 EOCross Plus Edge (3 ? ?~1.22GB, Needs Cross, Corner, Edge)
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

  // 7.3 EOCross 3-Corner (1 ? ~1.22GB, Needs Cross, Corner)
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

  // 9. Pseudo Cross + E0,E2 Prune
  if (!fileExists("pt_pscross_E0E2.bin")) {
    mtm.loadMTEdge4();
    mtm.loadMTEdge2();
    genPTPsCrossE0E2();
    std::vector<unsigned char>().swap(pt_pscross_E0E2);
    mtm.releaseMTEdge4();
    mtm.releaseMTEdge2();
  }

  // 11. Pseudo Cross + E0,E1 Prune
  if (!fileExists("pt_pscross_E0E1.bin")) {
    mtm.loadMTEdge4();
    mtm.loadMTEdge2();
    genPTPsCrossE0E1();
    std::vector<unsigned char>().swap(pt_pscross_E0E1);
    mtm.releaseMTEdge4();
    mtm.releaseMTEdge2();
  }

  // 11.1 Pseudo Cross + E1,E3 Prune (Edge2 对棱)
  if (!fileExists("pt_pscross_E1E3.bin")) {
    mtm.loadMTEdge4();
    mtm.loadMTEdge2();
    genPTPsCrossE1E3();
    std::vector<unsigned char>().swap(pt_pscross_E1E3);
    mtm.releaseMTEdge4();
    mtm.releaseMTEdge2();
  }

  // 11.2 Pseudo Cross + E0,E3 Prune (Edge2 邻棱)
  if (!fileExists("pt_pscross_E0E3.bin")) {
    mtm.loadMTEdge4();
    mtm.loadMTEdge2();
    genPTPsCrossE0E3();
    std::vector<unsigned char>().swap(pt_pscross_E0E3);
    mtm.releaseMTEdge4();
    mtm.releaseMTEdge2();
  }

  // 11.3 Pseudo Cross + E1,E2 Prune (Edge2 邻棱)
  if (!fileExists("pt_pscross_E1E2.bin")) {
    mtm.loadMTEdge4();
    mtm.loadMTEdge2();
    genPTPsCrossE1E2();
    std::vector<unsigned char>().swap(pt_pscross_E1E2);
    mtm.releaseMTEdge4();
    mtm.releaseMTEdge2();
  }

  // 11.4 Pseudo Cross + E2,E3 Prune (Edge2 邻棱)
  if (!fileExists("pt_pscross_E2E3.bin")) {
    mtm.loadMTEdge4();
    mtm.loadMTEdge2();
    genPTPsCrossE2E3();
    std::vector<unsigned char>().swap(pt_pscross_E2E3);
    mtm.releaseMTEdge4();
    mtm.releaseMTEdge2();
  }

  // 14.1 Pseudo Cross + E0,E1,E2 Prune (Edge3)
  if (!fileExists("pt_pscross_E0E1E2.bin")) {
    mtm.loadMTEdge4();
    mtm.loadMTEdge3();
    genPTPsCrossE0E1E2();
    std::vector<unsigned char>().swap(pt_pscross_E0E1E2);
    mtm.releaseMTEdge4();
    mtm.releaseMTEdge3();
  }

  // 14.2 Pseudo Cross + E1,E2,E3 Prune (Edge3)
  if (!fileExists("pt_pscross_E1E2E3.bin")) {
    mtm.loadMTEdge4();
    mtm.loadMTEdge3();
    genPTPsCrossE1E2E3();
    std::vector<unsigned char>().swap(pt_pscross_E1E2E3);
    mtm.releaseMTEdge4();
    mtm.releaseMTEdge3();
  }

  // 14.3 Pseudo Cross + E0,E2,E3 Prune (Edge3)
  if (!fileExists("pt_pscross_E0E2E3.bin")) {
    mtm.loadMTEdge4();
    mtm.loadMTEdge3();
    genPTPsCrossE0E2E3();
    std::vector<unsigned char>().swap(pt_pscross_E0E2E3);
    mtm.releaseMTEdge4();
    mtm.releaseMTEdge3();
  }

  // 14.4 Pseudo Cross + E0,E1,E3 Prune (Edge3)
  if (!fileExists("pt_pscross_E0E1E3.bin")) {
    mtm.loadMTEdge4();
    mtm.loadMTEdge3();
    genPTPsCrossE0E1E3();
    std::vector<unsigned char>().swap(pt_pscross_E0E1E3);
    mtm.releaseMTEdge4();
    mtm.releaseMTEdge3();
  }

  // 15. Pseudo Cross + C4,C6 Prune (Corner2)
  if (!fileExists("pt_pscross_C4C6.bin")) {
    mtm.loadMTEdge4();
    mtm.loadMTCorn2();
    genPTPsCrossC4C6();
    std::vector<unsigned char>().swap(pt_pscross_C4C6);
    mtm.releaseMTEdge4();
    mtm.releaseMTCorn2();
  }

  // 17. Pseudo Cross + C4,C5 Prune (Corner2)
  if (!fileExists("pt_pscross_C4C5.bin")) {
    mtm.loadMTEdge4();
    mtm.loadMTCorn2();
    genPTPsCrossC4C5();
    std::vector<unsigned char>().swap(pt_pscross_C4C5);
    mtm.releaseMTEdge4();
    mtm.releaseMTCorn2();
  }

  // 17.1 Pseudo Cross + C4,C7 Prune (Corner2 邻角)
  if (!fileExists("pt_pscross_C4C7.bin")) {
    mtm.loadMTEdge4();
    mtm.loadMTCorn2();
    genPTPsCrossC4C7();
    std::vector<unsigned char>().swap(pt_pscross_C4C7);
    mtm.releaseMTEdge4();
    mtm.releaseMTCorn2();
  }

  // 17.2 Pseudo Cross + C5,C6 Prune (Corner2 邻角)
  if (!fileExists("pt_pscross_C5C6.bin")) {
    mtm.loadMTEdge4();
    mtm.loadMTCorn2();
    genPTPsCrossC5C6();
    std::vector<unsigned char>().swap(pt_pscross_C5C6);
    mtm.releaseMTEdge4();
    mtm.releaseMTCorn2();
  }

  // 16. Pseudo Cross + C5,C7 Prune (Corner2 对角)
  if (!fileExists("pt_pscross_C5C7.bin")) {
    mtm.loadMTEdge4();
    mtm.loadMTCorn2();
    genPTPsCrossC5C7();
    std::vector<unsigned char>().swap(pt_pscross_C5C7);
    mtm.releaseMTEdge4();
    mtm.releaseMTCorn2();
  }

  // 17.3 Pseudo Cross + C6,C7 Prune (Corner2 邻角)
  if (!fileExists("pt_pscross_C6C7.bin")) {
    mtm.loadMTEdge4();
    mtm.loadMTCorn2();
    genPTPsCrossC6C7();
    std::vector<unsigned char>().swap(pt_pscross_C6C7);
    mtm.releaseMTEdge4();
    mtm.releaseMTCorn2();
  }

  // 21. Pseudo Cross + C4,C5,C6 Prune (Corner3)
  if (!fileExists("pt_pscross_C4C5C6.bin")) {
    mtm.loadMTEdge4();
    mtm.loadMTCorn3();
    genPTPsCrossC4C5C6();
    std::vector<unsigned char>().swap(pt_pscross_C4C5C6);
    mtm.releaseMTEdge4();
    mtm.releaseMTCorn3();
  }

  // 21.2 Pseudo Cross + C4,C5,C7 Prune (Corner3)
  if (!fileExists("pt_pscross_C4C5C7.bin")) {
    mtm.loadMTEdge4();
    mtm.loadMTCorn3();
    genPTPsCrossC4C5C7();
    std::vector<unsigned char>().swap(pt_pscross_C4C5C7);
    mtm.releaseMTEdge4();
    mtm.releaseMTCorn3();
  }

  // 21.3 Pseudo Cross + C4,C6,C7 Prune (Corner3)
  if (!fileExists("pt_pscross_C4C6C7.bin")) {
    mtm.loadMTEdge4();
    mtm.loadMTCorn3();
    genPTPsCrossC4C6C7();
    std::vector<unsigned char>().swap(pt_pscross_C4C6C7);
    mtm.releaseMTEdge4();
    mtm.releaseMTCorn3();
  }

  // 21.4 Pseudo Cross + C5,C6,C7 Prune (Corner3)
  if (!fileExists("pt_pscross_C5C6C7.bin")) {
    mtm.loadMTEdge4();
    mtm.loadMTCorn3();
    genPTPsCrossC5C6C7();
    std::vector<unsigned char>().swap(pt_pscross_C5C6C7);
    mtm.releaseMTEdge4();
    mtm.releaseMTCorn3();
  }

  // --- Pseudo Cross/XCross/Pair 变体表 (共36张) ---
  // 22. Cross + Corner 变体 (4张: C4~C7)
  for (int c = 0; c < 4; ++c)
    genPTPsCrossC(c);
  // 23. XCross 变体 (16张: C{4-7}_diff{0-3})
  for (int c = 0; c < 4; ++c)
    for (int e = 0; e < 4; ++e)
      genPTPsCrossCDiff(c, e);
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
  if (loadTable(pt_pscross_C[c], fn))
    return;
  if (fileExists(fn))
    return;
  auto &mtm = MoveTableManager::getInstance();
  mtm.loadMTEdge4();
  mtm.loadMTCorn();
  GenerationTimer timer;
  std::cout << "  Generating " << fn << "..." << std::endl;
  std::vector<unsigned char> temp;
  create_pt_pscross_corner(CORNER_INDICES[c], 10, mtm.getEdge4MT(),
                           mtm.getCornMT(), temp);
  saveTable(temp, fn);
  timer.printElapsed(fn);
  mtm.releaseMTEdge4();
  mtm.releaseMTCorn();
}

void PruneTableManager::genPTPsCrossCDiff(int c, int e) {
  std::string fn = "pt_pscross_C" + std::to_string(c + 4) + "_diff" +
                   std::to_string(e) + ".bin";
  int idx = e * 4 + c;
  if (loadTable(pt_pscross_C_diff[idx], fn))
    return;
  if (fileExists(fn))
    return;
  auto &mtm = MoveTableManager::getInstance();
  mtm.loadMTEdge4();
  mtm.loadMTCorn();
  GenerationTimer timer;
  std::cout << "  Generating " << fn << "..." << std::endl;
  std::vector<unsigned char> temp;
  create_pt_pscross_xcross(EDGE_INDICES[e], CORNER_INDICES[c], 10,
                           mtm.getEdge4MT(), mtm.getCornMT(), temp);
  saveTable(temp, fn);
  timer.printElapsed(fn);
  mtm.releaseMTEdge4();
  mtm.releaseMTCorn();
}

void PruneTableManager::genPTPsPairCE(int c, int e) {
  std::string fn =
      "pt_pspair_C" + std::to_string(c + 4) + "_E" + std::to_string(e) + ".bin";
  int idx = e * 4 + c;
  if (loadTable(pt_pspair_CE[idx], fn))
    return;
  if (fileExists(fn))
    return;
  auto &mtm = MoveTableManager::getInstance();
  mtm.loadMTEdge();
  mtm.loadMTCorn();
  GenerationTimer timer;
  std::cout << "  Generating " << fn << "..." << std::endl;
  std::vector<unsigned char> temp;
  create_pt_pspair(EDGE_INDICES[e], CORNER_INDICES[c], 24, 24, 8,
                   mtm.getEdgeMT(), mtm.getCornMT(), temp);
  saveTable(temp, fn);
  timer.printElapsed(fn);
  mtm.releaseMTEdge();
  mtm.releaseMTCorn();
}

bool PruneTableManager::loadTable(std::vector<unsigned char> &table,
                                  const std::string &filename) {
  if (load_vector_chunked(table, filename)) {
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
  save_vector_chunked(table, filename);
}

void PruneTableManager::genPTCross() {
  if (loadTable(pt_cross, "pt_cross.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_cross.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  const auto &edge2_mt = mtm.getEdge2MT();
  long long sz = 24LL * 22 * 24 * 22;
  std::vector<unsigned char> tmp(sz, 255);
  int i1 = 416, i2 = 520;
  tmp[(long long)i1 * 528 + i2] = 0;
  DistributionPrinter dp(sz);
  for (int d = 0; d < 10; ++d) {
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < sz; ++i) {
      dp.progress(i, sz, d);
      if (tmp[i] == d) {
        cnt++;
        int idx1 = (i / 528) * 18, idx2 = (i % 528) * 18;
        for (int j = 0; j < 18; ++j) {
          long long ni =
              (long long)edge2_mt[idx1 + j] * 528 + edge2_mt[idx2 + j];
          // NOTE: 使用 CAS 避免竞态条?
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

void PruneTableManager::genPTCrossC4() {
  if (loadTable(pt_cross_C4, "pt_cross_C4.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_cross_C4.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  pt_cross_C4.resize((long long)24 * 22 * 20 * 18 * 24, 255);
  create_pt_cross_c4(187520, 12, 24 * 22 * 20 * 18, 24, 10, mtm.getEdge4MT(),
                     mtm.getCornMT(), pt_cross_C4);
  saveTable(pt_cross_C4, "pt_cross_C4.bin");
  timer.printElapsed("pt_cross_C4.bin");
}

void PruneTableManager::genPTPairC4E0() {
  if (loadTable(pt_pair_C4E0, "pt_pair_C4E0.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_pair_C4E0.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  pt_pair_C4E0.resize(24 * 24, 255);
  create_pt_pair_base(0, 12, 24, 24, 8, mtm.getEdgeMT(), mtm.getCornMT(),
                      pt_pair_C4E0);
  saveTable(pt_pair_C4E0, "pt_pair_C4E0.bin");
  timer.printElapsed("pt_pair_C4E0.bin");
}

void PruneTableManager::genPTCrossC4E0() {
  if (loadTable(pt_cross_C4E0, "pt_cross_C4E0.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_cross_C4E0.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  long long c_sz = ((long long)24 * 22 * 20 * 18 * 24 * 24 + 1) / 2;
  pt_cross_C4E0.resize(c_sz, 0xFF);
  create_pt_xcross_full(187520, 12, 0, 24 * 22 * 20 * 18, 24, 24, 11,
                        mtm.getEdge4MT(), mtm.getCornMT(), mtm.getEdgeMT(),
                        pt_cross_C4E0);
  saveTable(pt_cross_C4E0, "pt_cross_C4E0.bin");
  timer.printElapsed("pt_cross_C4E0.bin");
}

void PruneTableManager::genPTCrossC4C5E0E1() {
  if (loadTable(pt_cross_C4C5E0E1, "pt_cross_C4C5E0E1.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_cross_C4C5E0E1.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  create_pt_huge(42577920, 504, 15, {0, 2, 16, 18, 20, 22}, {12, 15},
                 mtm.getEdge6MT(), mtm.getCorn2MT(), pt_cross_C4C5E0E1);
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
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_cross_C4C6E0E2.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  create_pt_huge(42577920, 504, 15, {0, 4, 16, 18, 20, 22}, {12, 18},
                 mtm.getEdge6MT(), mtm.getCorn2MT(), pt_cross_C4C6E0E2);
  saveTable(pt_cross_C4C6E0E2, "pt_cross_C4C6E0E2.bin");
  timer.printElapsed("pt_cross_C4C6E0E2.bin");
}

void PruneTableManager::genPTEP4EO12() {
  if (loadTable(pt_ep4eo12, "pt_ep4eo12.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_ep4eo12.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  // 状态空? EP4 (12*11*10*9 = 11880) x EO12 (2^11 = 2048)
  // 初始状? EP4_SOLVED=11720, EO_SOLVED=0
  // 使用 MoveTableManager 中已加载的移动表
  create_cascaded_pt3(11720, 0, 12 * 11 * 10 * 9, 2048, 11, mtm.getEP4MT(),
                      mtm.getEOAltMT(), pt_ep4eo12);
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
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET << " Generating " << fn
            << "..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  // idx_extra: E1=2, E2=4, E3=6 → EDGE_INDICES[i+1]
  int idx_extra = EDGE_INDICES[i + 1];
  create_pt_xcross_plus(187520, 12, 0, idx_extra, 24 * 22 * 20 * 18, 24, 24, 24,
                        14, mtm.getEdge4MT(), mtm.getCornMT(), mtm.getEdgeMT(),
                        mtm.getEdgeMT(), pt_cross_CEE[i]);
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
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET << " Generating " << fn
            << "..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  // idx_extra: C5=15, C6=18, C7=21 → CORNER_INDICES[i+1]
  int idx_extra = CORNER_INDICES[i + 1];
  create_pt_xcross_plus(187520, 12, 0, idx_extra, 24 * 22 * 20 * 18, 24, 24, 24,
                        14, mtm.getEdge4MT(), mtm.getCornMT(), mtm.getEdgeMT(),
                        mtm.getCornMT(), pt_cross_CCE[i]);
  saveTable(pt_cross_CCE[i], fn);
  timer.printElapsed(fn);
}

// === EOCross 3-Corner 生成函数 ===
// NOTE: 使用 create_pt_xcross_corn3，跟?C4+C5+C6 三个角块
// 参数来源: eo_cross_analyzer_old.cpp:L310-L313

void PruneTableManager::genPTCrossC4C5C6() {
  if (loadTable(pt_cross_C4C5C6, "pt_cross_C4C5C6.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_cross_C4C5C6.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  // 3-Corner: idx_c5=15, idx_c6=18, t_c5=CornMT, t_c6=CornMT
  create_pt_xcross_corn3(187520, 12, 15, 18, 24 * 22 * 20 * 18, 24, 24, 24, 14,
                         mtm.getEdge4MT(), mtm.getCornMT(), mtm.getCornMT(),
                         mtm.getCornMT(), pt_cross_C4C5C6);
  saveTable(pt_cross_C4C5C6, "pt_cross_C4C5C6.bin");
  timer.printElapsed("pt_cross_C4C5C6.bin");
}

void PruneTableManager::genPTPsCross() {
  if (loadTable(pt_pscross, "pt_pscross.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_pscross.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  const auto &edge2_mt = mtm.getEdge2MT();
  long long sz = 24LL * 22 * 24 * 22;
  std::vector<unsigned char> tmp(sz, 255);
  int d_moves[] = {-1, 3, 4, 5};
  for (int k = 0; k < 4; ++k) {
    int i1 = 416, i2 = 520;
    if (k > 0) {
      i1 = edge2_mt[i1 * 18 + d_moves[k]];
      i2 = edge2_mt[i2 * 18 + d_moves[k]];
    }
    tmp[(long long)i1 * 528 + i2] = 0;
  }
  DistributionPrinter dp(sz);
  for (int d = 0; d < 10; ++d) {
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < sz; ++i) {
      dp.progress(i, sz, d);
      if (tmp[i] == d) {
        cnt++;
        int idx1 = (i / 528) * 18, idx2 = (i % 528) * 18;
        for (int j = 0; j < 18; ++j) {
          long long ni =
              (long long)edge2_mt[idx1 + j] * 528 + edge2_mt[idx2 + j];
          // NOTE: 使用 CAS 避免竞态条?
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
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET << " Generating " << fn
            << "..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  int e_diffs[] = {0, 2, 4, 6};
  pt_pscross_C4E[i].resize(((long long)24 * 22 * 20 * 18 * 24 * 24 + 1) / 2,
                           0xFF);
  create_pt_xcross_full(187520, 12, e_diffs[i], 24 * 22 * 20 * 18, 24, 24, 11,
                        mtm.getEdge4MT(), mtm.getCornMT(), mtm.getEdgeMT(),
                        pt_pscross_C4E[i], true);
  saveTable(pt_pscross_C4E[i], fn);
  timer.printElapsed(fn);
}

void PruneTableManager::genPTPsCrossE0E2() {
  if (loadTable(pt_pscross_E0E2, "pt_pscross_E0E2.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_pscross_E0E2.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {0, 4};
  int idx_e0_e2_solved = array_to_index(target, 2, 2, 12);
  pt_pscross_E0E2.resize(((long long)190080 * 528 + 1) / 2, 0xFF);
  create_pt_pscross_edges2(187520, idx_e0_e2_solved, 190080, 528, 11,
                           mtm.getEdge4MT(), mtm.getEdge2MT(), pt_pscross_E0E2);
  saveTable(pt_pscross_E0E2, "pt_pscross_E0E2.bin");
  timer.printElapsed("pt_pscross_E0E2.bin");
}

void PruneTableManager::genPTPsCrossE0E1() {
  if (loadTable(pt_pscross_E0E1, "pt_pscross_E0E1.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_pscross_E0E1.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {0, 2}; // E0(0*2=0), E1(1*2=2) - 邻棱
  int idx_solved = array_to_index(target, 2, 2, 12);
  pt_pscross_E0E1.resize(((long long)190080 * 528 + 1) / 2, 0xFF);
  create_pt_pscross_edges2(187520, idx_solved, 190080, 528, 11,
                           mtm.getEdge4MT(), mtm.getEdge2MT(), pt_pscross_E0E1);
  saveTable(pt_pscross_E0E1, "pt_pscross_E0E1.bin");
  timer.printElapsed("pt_pscross_E0E1.bin");
}

void PruneTableManager::genPTPsCrossE1E3() {
  if (loadTable(pt_pscross_E1E3, "pt_pscross_E1E3.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_pscross_E1E3.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {2, 6}; // E1(1*2=2), E3(3*2=6) - 对棱
  int idx_solved = array_to_index(target, 2, 2, 12);
  pt_pscross_E1E3.resize(((long long)190080 * 528 + 1) / 2, 0xFF);
  create_pt_pscross_edges2(187520, idx_solved, 190080, 528, 11,
                           mtm.getEdge4MT(), mtm.getEdge2MT(), pt_pscross_E1E3);
  saveTable(pt_pscross_E1E3, "pt_pscross_E1E3.bin");
  timer.printElapsed("pt_pscross_E1E3.bin");
}

void PruneTableManager::genPTPsCrossE0E3() {
  if (loadTable(pt_pscross_E0E3, "pt_pscross_E0E3.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_pscross_E0E3.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {0, 6}; // E0(0*2=0), E3(3*2=6) - 邻棱
  int idx_solved = array_to_index(target, 2, 2, 12);
  pt_pscross_E0E3.resize(((long long)190080 * 528 + 1) / 2, 0xFF);
  create_pt_pscross_edges2(187520, idx_solved, 190080, 528, 11,
                           mtm.getEdge4MT(), mtm.getEdge2MT(), pt_pscross_E0E3);
  saveTable(pt_pscross_E0E3, "pt_pscross_E0E3.bin");
  timer.printElapsed("pt_pscross_E0E3.bin");
}

void PruneTableManager::genPTPsCrossE1E2() {
  if (loadTable(pt_pscross_E1E2, "pt_pscross_E1E2.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_pscross_E1E2.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {2, 4}; // E1(1*2=2), E2(2*2=4) - 邻棱
  int idx_solved = array_to_index(target, 2, 2, 12);
  pt_pscross_E1E2.resize(((long long)190080 * 528 + 1) / 2, 0xFF);
  create_pt_pscross_edges2(187520, idx_solved, 190080, 528, 11,
                           mtm.getEdge4MT(), mtm.getEdge2MT(), pt_pscross_E1E2);
  saveTable(pt_pscross_E1E2, "pt_pscross_E1E2.bin");
  timer.printElapsed("pt_pscross_E1E2.bin");
}

void PruneTableManager::genPTPsCrossE2E3() {
  if (loadTable(pt_pscross_E2E3, "pt_pscross_E2E3.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_pscross_E2E3.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {4, 6}; // E2(2*2=4), E3(3*2=6) - 邻棱
  int idx_solved = array_to_index(target, 2, 2, 12);
  pt_pscross_E2E3.resize(((long long)190080 * 528 + 1) / 2, 0xFF);
  create_pt_pscross_edges2(187520, idx_solved, 190080, 528, 11,
                           mtm.getEdge4MT(), mtm.getEdge2MT(), pt_pscross_E2E3);
  saveTable(pt_pscross_E2E3, "pt_pscross_E2E3.bin");
  timer.printElapsed("pt_pscross_E2E3.bin");
}

void PruneTableManager::genPTPsCrossE0E1E2() {
  if (loadTable(pt_pscross_E0E1E2, "pt_pscross_E0E1E2.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_pscross_E0E1E2.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {0, 2, 4}; // E0, E1, E2 (0, 2, 4)
  int idx_solved = array_to_index(target, 3, 2, 12);
  pt_pscross_E0E1E2.resize(((long long)190080 * 10560 + 1) / 2, 0xFF);
  create_pt_pscross_edges3(187520, idx_solved, 190080, 10560, 12,
                           mtm.getEdge4MT(), mtm.getEdge3MT(),
                           pt_pscross_E0E1E2);
  saveTable(pt_pscross_E0E1E2, "pt_pscross_E0E1E2.bin");
  timer.printElapsed("pt_pscross_E0E1E2.bin");
}

void PruneTableManager::genPTPsCrossE1E2E3() {
  if (loadTable(pt_pscross_E1E2E3, "pt_pscross_E1E2E3.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_pscross_E1E2E3.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {2, 4, 6}; // E1, E2, E3
  int idx_solved = array_to_index(target, 3, 2, 12);
  pt_pscross_E1E2E3.resize(((long long)190080 * 10560 + 1) / 2, 0xFF);
  create_pt_pscross_edges3(187520, idx_solved, 190080, 10560, 12,
                           mtm.getEdge4MT(), mtm.getEdge3MT(),
                           pt_pscross_E1E2E3);
  saveTable(pt_pscross_E1E2E3, "pt_pscross_E1E2E3.bin");
  timer.printElapsed("pt_pscross_E1E2E3.bin");
}

void PruneTableManager::genPTPsCrossE0E2E3() {
  if (loadTable(pt_pscross_E0E2E3, "pt_pscross_E0E2E3.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_pscross_E0E2E3.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {0, 4, 6}; // E0, E2, E3
  int idx_solved = array_to_index(target, 3, 2, 12);
  pt_pscross_E0E2E3.resize(((long long)190080 * 10560 + 1) / 2, 0xFF);
  create_pt_pscross_edges3(187520, idx_solved, 190080, 10560, 12,
                           mtm.getEdge4MT(), mtm.getEdge3MT(),
                           pt_pscross_E0E2E3);
  saveTable(pt_pscross_E0E2E3, "pt_pscross_E0E2E3.bin");
  timer.printElapsed("pt_pscross_E0E2E3.bin");
}

void PruneTableManager::genPTPsCrossE0E1E3() {
  if (loadTable(pt_pscross_E0E1E3, "pt_pscross_E0E1E3.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_pscross_E0E1E3.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {0, 2, 6}; // E0, E1, E3
  int idx_solved = array_to_index(target, 3, 2, 12);
  pt_pscross_E0E1E3.resize(((long long)190080 * 10560 + 1) / 2, 0xFF);
  create_pt_pscross_edges3(187520, idx_solved, 190080, 10560, 12,
                           mtm.getEdge4MT(), mtm.getEdge3MT(),
                           pt_pscross_E0E1E3);
  saveTable(pt_pscross_E0E1E3, "pt_pscross_E0E1E3.bin");
  timer.printElapsed("pt_pscross_E0E1E3.bin");
}

void PruneTableManager::genPTPsCrossC4C6() {
  if (loadTable(pt_pscross_C4C6, "pt_pscross_C4C6.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_pscross_C4C6.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {12, 18}; // C4(4*3=12), C6(6*3=18) - 对角
  int idx_solved = array_to_index(target, 2, 3, 8);
  pt_pscross_C4C6.resize(((long long)190080 * 504 + 1) / 2, 0xFF);
  create_pt_pscross_corners2(187520, idx_solved, 190080, 504, 11,
                             mtm.getEdge4MT(), mtm.getCorn2MT(),
                             pt_pscross_C4C6);
  saveTable(pt_pscross_C4C6, "pt_pscross_C4C6.bin");
  timer.printElapsed("pt_pscross_C4C6.bin");
}

void PruneTableManager::genPTPsCrossC5C7() {
  if (loadTable(pt_pscross_C5C7, "pt_pscross_C5C7.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_pscross_C5C7.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {15, 21}; // C5(5*3=15), C7(7*3=21) - 对角
  int idx_solved = array_to_index(target, 2, 3, 8);
  pt_pscross_C5C7.resize(((long long)190080 * 504 + 1) / 2, 0xFF);
  create_pt_pscross_corners2(187520, idx_solved, 190080, 504, 11,
                             mtm.getEdge4MT(), mtm.getCorn2MT(),
                             pt_pscross_C5C7);
  saveTable(pt_pscross_C5C7, "pt_pscross_C5C7.bin");
  timer.printElapsed("pt_pscross_C5C7.bin");
}

void PruneTableManager::genPTPsCrossC4C5() {
  if (loadTable(pt_pscross_C4C5, "pt_pscross_C4C5.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_pscross_C4C5.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {12, 15}; // C4(4*3=12), C5(5*3=15)
  int idx_solved = array_to_index(target, 2, 3, 8);
  pt_pscross_C4C5.resize(((long long)190080 * 504 + 1) / 2, 0xFF);
  create_pt_pscross_corners2(187520, idx_solved, 190080, 504, 11,
                             mtm.getEdge4MT(), mtm.getCorn2MT(),
                             pt_pscross_C4C5);
  saveTable(pt_pscross_C4C5, "pt_pscross_C4C5.bin");
  timer.printElapsed("pt_pscross_C4C5.bin");
}

void PruneTableManager::genPTPsCrossC4C7() {
  if (loadTable(pt_pscross_C4C7, "pt_pscross_C4C7.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_pscross_C4C7.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {12, 21}; // C4(4*3=12), C7(7*3=21)
  int idx_solved = array_to_index(target, 2, 3, 8);
  pt_pscross_C4C7.resize(((long long)190080 * 504 + 1) / 2, 0xFF);
  create_pt_pscross_corners2(187520, idx_solved, 190080, 504, 11,
                             mtm.getEdge4MT(), mtm.getCorn2MT(),
                             pt_pscross_C4C7);
  saveTable(pt_pscross_C4C7, "pt_pscross_C4C7.bin");
  timer.printElapsed("pt_pscross_C4C7.bin");
}

void PruneTableManager::genPTPsCrossC5C6() {
  if (loadTable(pt_pscross_C5C6, "pt_pscross_C5C6.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_pscross_C5C6.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {15, 18}; // C5(5*3=15), C6(6*3=18)
  int idx_solved = array_to_index(target, 2, 3, 8);
  pt_pscross_C5C6.resize(((long long)190080 * 504 + 1) / 2, 0xFF);
  create_pt_pscross_corners2(187520, idx_solved, 190080, 504, 11,
                             mtm.getEdge4MT(), mtm.getCorn2MT(),
                             pt_pscross_C5C6);
  saveTable(pt_pscross_C5C6, "pt_pscross_C5C6.bin");
  timer.printElapsed("pt_pscross_C5C6.bin");
}

void PruneTableManager::genPTPsCrossC6C7() {
  if (loadTable(pt_pscross_C6C7, "pt_pscross_C6C7.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_pscross_C6C7.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {18, 21}; // C6(6*3=18), C7(7*3=21)
  int idx_solved = array_to_index(target, 2, 3, 8);
  pt_pscross_C6C7.resize(((long long)190080 * 504 + 1) / 2, 0xFF);
  create_pt_pscross_corners2(187520, idx_solved, 190080, 504, 11,
                             mtm.getEdge4MT(), mtm.getCorn2MT(),
                             pt_pscross_C6C7);
  saveTable(pt_pscross_C6C7, "pt_pscross_C6C7.bin");
  timer.printElapsed("pt_pscross_C6C7.bin");
}

void PruneTableManager::genPTPsCrossC4C5C6() {
  if (loadTable(pt_pscross_C4C5C6, "pt_pscross_C4C5C6.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_pscross_C4C5C6.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {12, 15, 18}; // C4, C5, C6
  int idx_solved = array_to_index(target, 3, 3, 8);
  pt_pscross_C4C5C6.resize(((long long)190080 * 9072 + 1) / 2, 0xFF);
  create_pt_pscross_corners3(187520, idx_solved, 190080, 9072, 13,
                             mtm.getEdge4MT(), mtm.getCorn3MT(),
                             pt_pscross_C4C5C6);
  saveTable(pt_pscross_C4C5C6, "pt_pscross_C4C5C6.bin");
  timer.printElapsed("pt_pscross_C4C5C6.bin");
}

void PruneTableManager::genPTPsCrossC4C5C7() {
  if (loadTable(pt_pscross_C4C5C7, "pt_pscross_C4C5C7.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_pscross_C4C5C7.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {12, 15, 21}; // C4, C5, C7
  int idx_solved = array_to_index(target, 3, 3, 8);
  pt_pscross_C4C5C7.resize(((long long)190080 * 9072 + 1) / 2, 0xFF);
  create_pt_pscross_corners3(187520, idx_solved, 190080, 9072, 13,
                             mtm.getEdge4MT(), mtm.getCorn3MT(),
                             pt_pscross_C4C5C7);
  saveTable(pt_pscross_C4C5C7, "pt_pscross_C4C5C7.bin");
  timer.printElapsed("pt_pscross_C4C5C7.bin");
}

void PruneTableManager::genPTPsCrossC4C6C7() {
  if (loadTable(pt_pscross_C4C6C7, "pt_pscross_C4C6C7.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_pscross_C4C6C7.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {12, 18, 21}; // C4, C6, C7
  int idx_solved = array_to_index(target, 3, 3, 8);
  pt_pscross_C4C6C7.resize(((long long)190080 * 9072 + 1) / 2, 0xFF);
  create_pt_pscross_corners3(187520, idx_solved, 190080, 9072, 13,
                             mtm.getEdge4MT(), mtm.getCorn3MT(),
                             pt_pscross_C4C6C7);
  saveTable(pt_pscross_C4C6C7, "pt_pscross_C4C6C7.bin");
  timer.printElapsed("pt_pscross_C4C6C7.bin");
}

void PruneTableManager::genPTPsCrossC5C6C7() {
  if (loadTable(pt_pscross_C5C6C7, "pt_pscross_C5C6C7.bin"))
    return;
  GenerationTimer timer;
  std::cout << TAG_COLOR << "[PRUNE]" << ANSI_RESET
            << " Generating pt_pscross_C5C6C7.bin..." << std::endl;
  auto &mtm = MoveTableManager::getInstance();
  std::vector<int> target = {15, 18, 21}; // C5, C6, C7
  int idx_solved = array_to_index(target, 3, 3, 8);
  pt_pscross_C5C6C7.resize(((long long)190080 * 9072 + 1) / 2, 0xFF);
  create_pt_pscross_corners3(187520, idx_solved, 190080, 9072, 13,
                             mtm.getEdge4MT(), mtm.getCorn3MT(),
                             pt_pscross_C5C6C7);
  saveTable(pt_pscross_C5C6C7, "pt_pscross_C5C6C7.bin");
  timer.printElapsed("pt_pscross_C5C6C7.bin");
}

// ... existing helper functions ...

void create_pt_pscross_corners2(int idx_cr, int idx_c2, int sz_cr, int sz_c2,
                                int depth, const std::vector<int> &t_cr,
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

  DistributionPrinter dp(total);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      dp.progress(i, total, d);
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
          // NOTE: 使用 CAS 避免竞态条?
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&tmp[ni], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  std::fill(pt.begin(), pt.end(), 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

void create_pt_pscross_corners3(int idx_cr, int idx_c3, int sz_cr, int sz_c3,
                                int depth, const std::vector<int> &t_cr,
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

  DistributionPrinter dp(total);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      dp.progress(i, total, d);
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
          // NOTE: 使用 CAS 避免竞态条?
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&tmp[ni], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  std::fill(pt.begin(), pt.end(), 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

void create_pt_pscross_edges3(int idx_cr, int idx_e3, int sz_cr, int sz_e3,
                              int depth, const std::vector<int> &t_cr,
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

  DistributionPrinter dp(total);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      dp.progress(i, total, d);
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
          // NOTE: 使用 CAS 避免竞态条?
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&tmp[ni], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  std::fill(pt.begin(), pt.end(), 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

// 1. Cross + C4 (Base)
void create_pt_cross_c4(int idx1, int idx2, int sz1, int sz2, int depth,
                        const std::vector<int> &t1, const std::vector<int> &t2,
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
  DistributionPrinter dp(total);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      dp.progress(i, total, d);
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
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
}

// 2. Pair C4 + E0 (Base)
void create_pt_pair_base(int idx_e, int idx_c, int sz_e, int sz_c, int depth,
                         const std::vector<int> &t_edge,
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
  DistributionPrinter dp(total);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      dp.progress(i, total, d);
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
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
}

void create_pt_xcross_full(int idx_cr, int idx_cn, int idx_ed, int sz_cr,
                           int sz_cn, int sz_ed, int depth,
                           const std::vector<int> &t1,
                           const std::vector<int> &t2,
                           const std::vector<int> &t3,
                           std::vector<unsigned char> &pt, bool is_pseudo) {
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

  DistributionPrinter dp(total);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      dp.progress(i, total, d);
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
          // NOTE: 使用 CAS 避免竞态条?
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&tmp[ni], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  std::fill(pt.begin(), pt.end(), 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

void create_pt_huge(int sz_e6, int sz_c2, int depth,
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

  DistributionPrinter dp(total);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      dp.progress(i, total, d);
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
          // NOTE: 使用 CAS 避免竞态条?
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&tmp[ni], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  long long c_sz = (total + 1) / 2;
  pt.assign(c_sz, 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
  std::vector<unsigned char>().swap(tmp);
}

void create_pt_pscross_edges2(int idx_cr, int idx_e2, int sz_cr, int sz_e2,
                              int depth, const std::vector<int> &t_cr,
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

  DistributionPrinter dp(total);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      dp.progress(i, total, d);
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
          // NOTE: 使用 CAS 避免竞态条?
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&tmp[ni], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  std::fill(pt.begin(), pt.end(), 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

// --- 级联剪枝表生成函数实现 (from eo_cross_analyzer) ---

void create_cascaded_pt3(int i1, int i2, int s1, int s2, int depth,
                         const std::vector<int> &t1, const std::vector<int> &t2,
                         std::vector<unsigned char> &pt) {
  long long sz = (long long)s1 * s2;
  std::vector<unsigned char> tmp(sz, 255);
  tmp[(long long)i1 * s2 + i2] = 0;

  DistributionPrinter dp(sz);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < sz; ++i) {
      dp.progress(i, sz, d);
      if (tmp[i] == d) {
        cnt++;
        int tb1 = (i / s2) * 18, tb2 = (i % s2) * 18;
        for (int j = 0; j < 18; ++j) {
          long long ni = (long long)t1[tb1 + j] * s2 + t2[tb2 + j];
          // NOTE: 使用 CAS 避免竞态条?
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&tmp[ni], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  pt.assign((sz + 1) / 2, 0xFF);
  for (long long i = 0; i < sz; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

void create_pt_xcross_plus(int idx_cr, int idx_cn, int idx_ed, int idx_extra,
                           int sz_cr, int sz_cn, int sz_ed, int sz_ex,
                           int depth, const std::vector<int> &t1,
                           const std::vector<int> &t2,
                           const std::vector<int> &t3,
                           const std::vector<int> &t4,
                           std::vector<unsigned char> &pt) {
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

  DistributionPrinter dp(total);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      dp.progress(i, total, d);
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
          // NOTE: 使用 CAS 避免竞态条?
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&tmp[ni], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  long long c_sz = (total + 1) / 2;
  pt.assign(c_sz, 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

void create_pt_xcross_corn3(int idx_cr, int idx_cn, int idx_c5, int idx_c6,
                            int sz_cr, int sz_cn, int sz_c5, int sz_c6,
                            int depth, const std::vector<int> &t1,
                            const std::vector<int> &t2,
                            const std::vector<int> &t_c5,
                            const std::vector<int> &t_c6,
                            std::vector<unsigned char> &pt) {
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

  DistributionPrinter dp(total);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      dp.progress(i, total, d);
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
          // NOTE: 使用 CAS 避免竞态条?
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&tmp[ni], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  long long c_sz = (total + 1) / 2;
  pt.assign(c_sz, 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

// --- Pseudo Cross/XCross/Pair 变体表生成函?---
// NOTE: 以下函数用于生成 C{4-7} 变体的剪枝表

// 生成 Pseudo Cross + Corner ?(例如: prune_table_pseudo_cross_C4.bin)
// index2: corner 初始索引 (?12=C4, 15=C5, 18=C6, 21=C7)
void create_pt_pscross_corner(int index2, int depth,
                              const std::vector<int> &table1,
                              const std::vector<int> &table2,
                              std::vector<unsigned char> &prune_table) {
  long long size1 = 190080, size2 = 24, size = size1 * size2;
  std::vector<unsigned char> temp_table(size, 255);
  std::vector<int> a = {16, 18, 20, 22};
  int index1 = array_to_index(a, 4, 2, 12);
  temp_table[index1 * size2 + index2] = 0;
  temp_table[table1[index1 * 24 + 3] + table2[index2 * 18 + 3]] = 0;
  temp_table[table1[index1 * 24 + 4] + table2[index2 * 18 + 4]] = 0;
  temp_table[table1[index1 * 24 + 5] + table2[index2 * 18 + 5]] = 0;

  DistributionPrinter dp(size);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < size; ++i) {
      dp.progress(i, size, d);
      if (temp_table[i] == d) {
        cnt++;
        int index1_tmp = (i / size2) * 24;
        int index2_tmp = (i % size2) * 18;
        for (int j = 0; j < 18; ++j) {
          int next_i = table1[index1_tmp + j] + table2[index2_tmp + j];
          // NOTE: 使用 CAS 避免竞态条?
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&temp_table[next_i], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  prune_table.resize((size + 1) / 2);
  std::fill(prune_table.begin(), prune_table.end(), 0xFF);
  for (long long i = 0; i < size; ++i)
    if (temp_table[i] != 255)
      set_prune(prune_table, i, temp_table[i]);
}

// 生成 Pseudo XCross ?(例如: pt_pscross_C4_diff0.bin)
// index3: edge 初始索引 (0, 2, 4, 6 for E0-E3)
// index2: corner 初始索引 (12, 15, 18, 21 for C4-C7)
void create_pt_pscross_xcross(int index3, int index2, int depth,
                              const std::vector<int> &table1,
                              const std::vector<int> &table2,
                              std::vector<unsigned char> &prune_table) {
  long long size1 = 190080, size2 = 24, size = size1 * size2;
  std::vector<unsigned char> temp_table(size, 255);

  // 根据边块位置选择初始化序列
  std::vector<std::string> appl_moves;
  std::vector<int> tmp_moves;
  if (index3 == 0) {
    appl_moves = {"L U L'", "L U' L'", "B' U B", "B' U' B"};
    tmp_moves = {0, 3, 4, 5};
  }
  if (index3 == 2) {
    appl_moves = {"R' U R", "R' U' R", "B U B'", "B U' B'"};
    tmp_moves = {5, 0, 3, 4};
  }
  if (index3 == 4) {
    appl_moves = {"R U R'", "R U' R'", "F' U F", "F' U' F"};
    tmp_moves = {4, 5, 0, 3};
  }
  if (index3 == 6) {
    appl_moves = {"L' U L", "L' U' L", "F U F'", "F U' F'"};
    tmp_moves = {3, 4, 5, 0};
  }

  std::vector<int> a = {16, 18, 20, 22};
  int index1 = array_to_index(a, 4, 2, 12);
  temp_table[index1 * size2 + index2] = 0;
  temp_table[table1[index1 * 24 + 3] + table2[index2 * 18 + 3]] = 0;
  temp_table[table1[index1 * 24 + 4] + table2[index2 * 18 + 4]] = 0;
  temp_table[table1[index1 * 24 + 5] + table2[index2 * 18 + 5]] = 0;

  // 初始化所有pseudo 等效状态
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

  DistributionPrinter dp(size);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < size; ++i) {
      dp.progress(i, size, d);
      if (temp_table[i] == d) {
        cnt++;
        int index1_tmp = (i / size2) * 24;
        int index2_tmp = (i % size2) * 18;
        for (int j = 0; j < 18; ++j) {
          int next_i = table1[index1_tmp + j] + table2[index2_tmp + j];
          // NOTE: 使用 CAS 避免竞态条件
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&temp_table[next_i], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  prune_table.resize((size + 1) / 2);
  std::fill(prune_table.begin(), prune_table.end(), 0xFF);
  for (long long i = 0; i < size; ++i)
    if (temp_table[i] != 255)
      set_prune(prune_table, i, temp_table[i]);
}

// 生成 Pseudo Pair 表(例如: prune_table_pseudo_pair_C4_E0.bin)
// index1: edge 初始索引 (0, 2, 4, 6)
// index2: corner 初始索引 (12, 15, 18, 21)
void create_pt_pspair(int index1, int index2, int size1, int size2, int depth,
                      const std::vector<int> &table1,
                      const std::vector<int> &table2,
                      std::vector<unsigned char> &prune_table) {
  long long size = (long long)size1 * size2;
  std::vector<unsigned char> temp_table(size, 255);
  long long start = (long long)index1 * size2 + index2;
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

  DistributionPrinter dp(size);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < size; ++i) {
      dp.progress(i, size, d);
      if (temp_table[i] == d) {
        cnt++;
        int index1_tmp = (i / size2) * 18;
        int index2_tmp = (i % size2) * 18;
        for (int j = 0; j < 18; ++j) {
          int next_i = table1[index1_tmp + j] * size2 + table2[index2_tmp + j];
          // NOTE: 使用 CAS 避免竞态条件
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&temp_table[next_i], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  prune_table.resize((size + 1) / 2);
  std::fill(prune_table.begin(), prune_table.end(), 0xFF);
  for (long long i = 0; i < size; ++i)
    if (temp_table[i] != 255)
      set_prune(prune_table, i, temp_table[i]);
}
