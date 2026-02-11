/*
 * prune_tables.h - 剪枝表相关功能
 */

#ifndef PRUNE_TABLES_H
#define PRUNE_TABLES_H

#include "cube_common.h"
#include <iomanip>
#include <omp.h>

// 深度分布打印器 - 用于打印带百分比的深度分布表格
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
    // 计算加权平均深度: sum(depth * count) / total
    double avg = 0.0;
    for (auto &p : records) {
      avg += (double)p.first * p.second;
    }
    avg /= accumulated;
    std::cout << "  " << std::fixed << std::setprecision(2) << std::setw(5)
              << std::right << avg << "  " << std::setw(14) << std::right
              << formatWithCommas(accumulated) << "  100.000000%" << std::endl;
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

// --- 剪枝表操作 ---
inline void set_prune(std::vector<unsigned char> &table, long long index,
                      int value) {
  int shift = (index & 1) << 2;
  table[index >> 1] &= ~(0xF << shift);
  table[index >> 1] |= (value & 0xF) << shift;
}

inline int get_prune(const std::vector<unsigned char> &table, long long index) {
  return (table[index >> 1] >> ((index & 1) << 2)) & 0xF;
}

inline int get_prune(const unsigned char *table, long long index) {
  return (table[index >> 1] >> ((index & 1) << 2)) & 0xF;
}

// Huge 表剪枝检查：对 (Edge6, Corner2) 做 conj 变换后查表
// 返回 true 表示应剪枝（即 prune >= depth，调用方应 continue）
// out_e6/out_c2 接收变换后的新索引，用于递归传递
inline bool hugeTablePrunes(int conj, const unsigned char *table, int e6,
                            int c2, int move, int depth, const int *mt_e6,
                            const int *mt_c2, int &out_e6, int &out_c2) {
  if (conj == -1 || !table)
    return false;
  int mx = conj_moves_flat[move][conj];
  out_e6 = mt_e6[e6 * 18 + mx];
  out_c2 = mt_c2[c2 * 18 + mx];
  return get_prune(table, (long long)out_e6 * StateSpace::CORNER2 + out_c2) >=
         depth;
}

// --- 组合索引函数 ---
// C(4,2) 字典序索引: {0,1}->0, {0,2}->1, {0,3}->2, {1,2}->3, {1,3}->4, {2,3}->5
inline int pairIdx(int a, int b) {
  int lo = std::min(a, b), hi = std::max(a, b);
  return lo * (7 - lo) / 2 + hi - lo - 1;
}

// C(4,3) 字典序索引: {0,1,2}->0, {0,1,3}->1, {0,2,3}->2, {1,2,3}->3
// 等价于 a+b+c-3 (因为三元素之和唯一确定缺失元素)
inline int tripleIdx(int a, int b, int c) { return a + b + c - 3; }

// --- 剪枝表管理器 ---
class PruneTableManager {
private:
  // Cross相关剪枝表
  std::vector<unsigned char> pt_cross;        // Cross基础剪枝表
  std::vector<unsigned char> pt_cross_ins_C4; // Cross + C4 F2L insertion剪枝表
  std::vector<unsigned char> pt_pair_C4E0;    // Pair C4+E0剪枝表
  std::vector<unsigned char> pt_cross_C4E0;   // XCross C4+E0剪枝表

  // 巨型剪枝表
  std::vector<unsigned char> pt_cross_C4C5E0E1; // 相邻槽剪枝表
  std::vector<unsigned char> pt_cross_C4C6E0E2; // 对角槽剪枝表

  // Pseudo 相关剪枝表
  std::vector<unsigned char> pt_pscross; // Pseudo Cross 剪枝表
  std::vector<unsigned char>
      pt_pscross_C4E[4]; // Pseudo Cross 基础表 (C4+E0..E3)

  // Pseudo 组合表 (数组化，按 pairIdx/tripleIdx 索引)
  std::vector<unsigned char> pt_pscross_Edge2[6];   // Edge2: 6种棱对组合
  std::vector<unsigned char> pt_pscross_Corner2[6]; // Corner2: 6种角对组合
  std::vector<unsigned char> pt_pscross_Edge3[4]; // Edge3: 4种棱三元组合
  std::vector<unsigned char> pt_pscross_Corner3[4]; // Corner3: 4种角三元组合

  // === PseudoPair 专用表 ===
  std::vector<unsigned char> pt_pscross_C[4]; // Cross + C{4-7}
  std::vector<unsigned char>
      pt_pscross_ins_C_diff[16];               // XC: diff*4+corner_offset
  std::vector<unsigned char> pt_pspair_CE[16]; // EC Pair: edge*4+corner

  // === EOCross 专用表 ===
  // NOTE: eo_cross使用不同生成函数，与pt_cross_ins_C4不同
  std::vector<unsigned char> pt_eoc_ins_C4;   // Cross+C4 (EOCross版)
  std::vector<unsigned char> pt_ep4eo12;      // Dependency+EO
  std::vector<unsigned char> pt_cross_CEE[3]; // Plus Edge: Right/Diag/Left
  std::vector<unsigned char> pt_cross_CCE[3]; // Plus Corner: Right/Diag/Left
  std::vector<unsigned char> pt_cross_C4C5C6; // 3-Corner (C4+C5+C6)

  // 单例模式
  static PruneTableManager *instance;
  PruneTableManager() = default;

public:
  static PruneTableManager &getInstance();

  // 初始化所有剪枝表
  void initialize();

  // 尝试加载所有剪枝表
  bool loadAll();

  // Windows MMap Loader helper
  unsigned char *loadTableMMap(const std::string &filename);

  // 仅加载 Pseudo Analyzer 所需的表
  bool loadPseudoTables();

  // 加载 PseudoPair Analyzer 所需的表
  bool loadPseudoPairTables();

  // 加载 EOCross Analyzer 所需的表
  bool loadEOCrossTables();

  // 顺序生成所有表
  void genAllSequentially();

  // === 新 Ptr Getter ===
  const unsigned char *getCrossPTPtr() const { return pt_cross.data(); }
  const unsigned char *getCrossInsC4PTPtr() const {
    return pt_cross_ins_C4.data();
  }
  const unsigned char *getPairC4E0PTPtr() const { return pt_pair_C4E0.data(); }
  const unsigned char *getCrossC4E0PTPtr() const {
    return pt_cross_C4E0.data();
  }
  const unsigned char *getCrossC4C5E0E1PTPtr() const {
    return pt_cross_C4C5E0E1.data();
  }
  const unsigned char *getCrossC4C6E0E2PTPtr() const {
    return pt_cross_C4C6E0E2.data();
  }

  // === Pseudo Getter/Has (数组化) ===
  const unsigned char *getPsCrossPTPtr() const { return pt_pscross.data(); }
  const unsigned char *getPsCrossC4EPTPtr(int i) const {
    return pt_pscross_C4E[i].data();
  }

  // Edge2: 参数为棱块编号 (0-3)
  const unsigned char *getPsCrossEdge2PTPtr(int a, int b) const {
    return pt_pscross_Edge2[pairIdx(a, b)].data();
  }
  bool hasPsCrossEdge2PT(int a, int b) const {
    return !pt_pscross_Edge2[pairIdx(a, b)].empty();
  }

  // Corner2: 参数为角块编号 (4-7)
  const unsigned char *getPsCrossCorner2PTPtr(int a, int b) const {
    return pt_pscross_Corner2[pairIdx(a - 4, b - 4)].data();
  }
  bool hasPsCrossCorner2PT(int a, int b) const {
    return !pt_pscross_Corner2[pairIdx(a - 4, b - 4)].empty();
  }

  // Edge3: 参数为棱块编号 (0-3)
  const unsigned char *getPsCrossEdge3PTPtr(int a, int b, int c) const {
    return pt_pscross_Edge3[tripleIdx(a, b, c)].data();
  }
  bool hasPsCrossEdge3PT(int a, int b, int c) const {
    return !pt_pscross_Edge3[tripleIdx(a, b, c)].empty();
  }

  // Corner3: 参数为角块编号 (4-7)
  const unsigned char *getPsCrossCorner3PTPtr(int a, int b, int c) const {
    return pt_pscross_Corner3[tripleIdx(a - 4, b - 4, c - 4)].data();
  }
  bool hasPsCrossCorner3PT(int a, int b, int c) const {
    return !pt_pscross_Corner3[tripleIdx(a - 4, b - 4, c - 4)].empty();
  }

  // === PseudoPair Getters (新名) ===
  const unsigned char *getPsCrossCPTPtr(int c) const {
    return pt_pscross_C[c].data();
  }
  const unsigned char *getPsCrossInsCDiffPTPtr(int idx) const {
    return pt_pscross_ins_C_diff[idx].data();
  }
  const unsigned char *getPsPairECPTPtr(int idx) const {
    return pt_pspair_CE[idx].data();
  }

  // === EOCross Getters (新名) ===

  const unsigned char *getEP4EO12PTPtr() const { return pt_ep4eo12.data(); }
  const unsigned char *getCrossCEEPTPtr(int i) const {
    return pt_cross_CEE[i].data();
  }
  const unsigned char *getCrossCCEPTPtr(int i) const {
    return pt_cross_CCE[i].data();
  }
  const unsigned char *getCrossC4C5C6PTPtr() const {
    return pt_cross_C4C5C6.data();
  }

  // 生成函数 (新名)
  void genPTCross();
  void genPTCrossInsC4();
  void genPTPairC4E0();
  void genPTCrossC4E0();
  void genPTCrossC4C5E0E1();
  void genPTCrossC4C6E0E2();

  // EOCross 专用生成函数
  void genPTEP4EO12(); // EP4 + EO12 联合剪枝表
  // EOCross Plus 生成函数 (参数化: i=0 Right, 1 Diag, 2 Left)
  void genPTCrossCEE(int i); // Plus Edge
  void genPTCrossCCE(int i); // Plus Corner
  // EOCross 3-Corner 生成函数
  void genPTCrossC4C5C6(); // 3-Corner (C5+C6)

  // Pseudo 生成函数
  void genPTPsCross();
  void genPTPsCrossC4E(int offset_idx);
  // Pseudo 组合表生成函数 (参数化)
  void genPTPsCrossEdge2(int a, int b);          // 棱对: a,b ∈ {0..3}
  void genPTPsCrossCorner2(int a, int b);        // 角对: a,b ∈ {4..7}
  void genPTPsCrossEdge3(int a, int b, int c);   // 棱三元: a,b,c ∈ {0..3}
  void genPTPsCrossCorner3(int a, int b, int c); // 角三元: a,b,c ∈ {4..7}
  // PseudoPair 变体生成函数
  void genPTPsCrossC(int c); // Cross+Corner 变体 (c=0..3 → C4..C7)
  void genPTPsCrossInsCDiff(int c, int e); // XCross 变体 (c=0..3, e=0..3)
  void genPTPsPairCE(int c, int e);        // Pair 变体 (c=0..3, e=0..3)

private:
  // 文件操作
  bool loadTable(std::vector<unsigned char> &table,
                 const std::string &filename);
  void saveTable(const std::vector<unsigned char> &table,
                 const std::string &filename);
};

// --- 剪枝表生成函数 ---
void createPTCrossInsC(int idx_cr, int idx_cn, int sz_cr, int sz_cn, int depth,
                       const std::vector<int> &t_cr,
                       const std::vector<int> &t_cn,
                       std::vector<unsigned char> &pt);

void createPTPair(int idx_ed, int idx_cn, int sz_ed, int sz_cn, int depth,
                  const std::vector<int> &t_ed, const std::vector<int> &t_cn,
                  std::vector<unsigned char> &pt);

void createPTCrossCE(int idx_cr, int idx_cn, int idx_ed, int sz_cr, int sz_cn,
                     int sz_ed, int depth, const std::vector<int> &t1,
                     const std::vector<int> &t2, const std::vector<int> &t3,
                     std::vector<unsigned char> &pt, bool is_pseudo = false);

void createPTPsCrossEdge2(int idx_cr, int idx_e2, int sz_cr, int sz_e2,
                          int depth, const std::vector<int> &t_cr,
                          const std::vector<int> &t_e2,
                          std::vector<unsigned char> &pt);

// 新增 helper: 针对角块对
void createPTPsCrossCorn2(int idx_cr, int idx_c2, int sz_cr, int sz_c2,
                          int depth, const std::vector<int> &t_cr,
                          const std::vector<int> &t_c2,
                          std::vector<unsigned char> &pt);

void createPTPsCrossCorn3(int idx_cr, int idx_c3, int sz_cr, int sz_c3,
                          int depth, const std::vector<int> &t_cr,
                          const std::vector<int> &t_c3,
                          std::vector<unsigned char> &pt);

void createPTPsCrossEdge3(int idx_cr, int idx_e3, int sz_cr, int sz_e3,
                          int depth, const std::vector<int> &t_cr,
                          const std::vector<int> &t_e3,
                          std::vector<unsigned char> &pt);

void createPTEdge6Corn2(int sz_e6, int sz_c2, int depth,
                        const std::vector<int> &target_e_ids,
                        const std::vector<int> &target_c_ids,
                        const std::vector<int> &mt_e6,
                        const std::vector<int> &mt_c2,
                        std::vector<unsigned char> &pt);

// --- 剪枝表生成函数 (from eo_cross_analyzer) ---
void createPTDim2(int idx1, int idx2, int sz1, int sz2, int depth,
                  const std::vector<int> &t1, const std::vector<int> &t2,
                  std::vector<unsigned char> &pt);
void createPTCrossCEX(int idx_cr, int idx_cn, int idx_ed, int idx_extra,
                      int sz_cr, int sz_cn, int sz_ed, int sz_ex, int depth,
                      const std::vector<int> &t1, const std::vector<int> &t2,
                      const std::vector<int> &t3, const std::vector<int> &t4,
                      std::vector<unsigned char> &pt);
void createPTCrossCCC(int idx_cr, int idx_cn1, int idx_cn2, int idx_cn3,
                      int sz_cr, int sz_cn1, int sz_cn2, int sz_cn3, int depth,
                      const std::vector<int> &t_cr,
                      const std::vector<int> &t_cn1,
                      const std::vector<int> &t_cn2,
                      const std::vector<int> &t_cn3,
                      std::vector<unsigned char> &pt);

#endif // PRUNE_TABLES_H
