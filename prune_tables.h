/*
 * prune_tables.h - 剪枝表相关功能
 */

#ifndef PRUNE_TABLES_H
#define PRUNE_TABLES_H

#include "cube_common.h"

// --- 剪枝表操作 ---
inline void set_prune(std::vector<unsigned char> &table, long long index,
                      int value) {
  int shift = (index & 1) << 2;
  table[index >> 1] &= ~(0xF << shift);
  table[index >> 1] |= (value & 0xF) << shift;
}

inline int get_prune_4bit(const unsigned char *table, long long index) {
  return (table[index >> 1] >> ((index & 1) << 2)) & 0xF;
}

inline int get_prune(const std::vector<unsigned char> &table, long long index) {
  return (table[index >> 1] >> ((index & 1) << 2)) & 0xF;
}

inline int get_prune_ptr(const unsigned char *table, long long index) {
  return (table[index >> 1] >> ((index & 1) << 2)) & 0xF;
}

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
  std::vector<unsigned char>
      pt_pscross_E0E2; // Pseudo Cross + E0,E2 对棱表 (已有)
  std::vector<unsigned char> pt_pscross_E1E3; // 对棱表 (新增)

  // 邻棱表
  std::vector<unsigned char> pt_pscross_E0E1; // 邻棱 (已有)
  std::vector<unsigned char> pt_pscross_E0E3; // 邻棱 (新增)
  std::vector<unsigned char> pt_pscross_E1E2; // 邻棱 (新增)
  std::vector<unsigned char> pt_pscross_E2E3; // 邻棱 (新增)

  // Edge3 Triples
  std::vector<unsigned char> pt_pscross_E0E1E2; // 基准
  std::vector<unsigned char> pt_pscross_E0E1E3; // Newly Added
  std::vector<unsigned char> pt_pscross_E0E2E3; // Newly Added
  std::vector<unsigned char> pt_pscross_E1E2E3; // Newly Added

  // 新增对角表 (F2L Corner Pairs)
  std::vector<unsigned char> pt_pscross_C4C6; // 对角 (已有)
  std::vector<unsigned char> pt_pscross_C5C7; // 对角 (新增)

  // 新增邻角表
  std::vector<unsigned char> pt_pscross_C4C5; // 邻角 (已有)
  std::vector<unsigned char> pt_pscross_C4C7; // 邻角 (新增)
  std::vector<unsigned char> pt_pscross_C5C6; // 邻角 (新增)
  std::vector<unsigned char> pt_pscross_C6C7; // 邻角 (新增)

  // Corner3 Triples
  std::vector<unsigned char> pt_pscross_C4C5C6; // 基准
  std::vector<unsigned char> pt_pscross_C4C5C7; // Newly Added
  std::vector<unsigned char> pt_pscross_C4C6C7; // Newly Added
  std::vector<unsigned char> pt_pscross_C5C6C7; // Newly Added

  // === PseudoPair 专用表 ===
  std::vector<unsigned char> pt_pscross_C[4];       // Cross + C{4-7}
  std::vector<unsigned char> pt_pscross_C_diff[16]; // XC: diff*4+corner_offset
  std::vector<unsigned char> pt_pspair_CE[16];      // EC Pair: edge*4+corner

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

  // === 新 Getter (返回 vector 引用) ===
  const std::vector<unsigned char> &getCrossPT() const { return pt_cross; }
  const std::vector<unsigned char> &getCrossInsC4PT() const {
    return pt_cross_ins_C4;
  }
  const std::vector<unsigned char> &getPairC4E0PT() const {
    return pt_pair_C4E0;
  }
  const std::vector<unsigned char> &getCrossC4E0PT() const {
    return pt_cross_C4E0;
  }
  const std::vector<unsigned char> &getCrossC4C5E0E1PT() const {
    return pt_cross_C4C5E0E1;
  }
  const std::vector<unsigned char> &getCrossC4C6E0E2PT() const {
    return pt_cross_C4C6E0E2;
  }

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

  // === 新 Pseudo Getter (Ptr) ===
  const unsigned char *getPsCrossPTPtr() const { return pt_pscross.data(); }
  const unsigned char *getPsCrossC4EPTPtr(int i) const {
    return pt_pscross_C4E[i].data();
  }
  const unsigned char *getPsCrossE0E2PTPtr() const {
    return pt_pscross_E0E2.data();
  }
  const unsigned char *getPsCrossE1E3PTPtr() const {
    return pt_pscross_E1E3.data();
  }

  // 邻棱表 Getters
  const unsigned char *getPsCrossE0E1PTPtr() const {
    return pt_pscross_E0E1.data();
  }
  const unsigned char *getPsCrossE0E3PTPtr() const {
    return pt_pscross_E0E3.data();
  }
  const unsigned char *getPsCrossE1E2PTPtr() const {
    return pt_pscross_E1E2.data();
  }
  const unsigned char *getPsCrossE2E3PTPtr() const {
    return pt_pscross_E2E3.data();
  }

  // Edge3 Getters
  const unsigned char *getPsCrossE0E1E2PTPtr() const {
    return pt_pscross_E0E1E2.data();
  }
  const unsigned char *getPsCrossE1E2E3PTPtr() const {
    return pt_pscross_E1E2E3.data();
  }
  const unsigned char *getPsCrossE0E2E3PTPtr() const {
    return pt_pscross_E0E2E3.data();
  }
  const unsigned char *getPsCrossE0E1E3PTPtr() const {
    return pt_pscross_E0E1E3.data();
  }

  // 对角表 Getters
  const unsigned char *getPsCrossC4C6PTPtr() const {
    return pt_pscross_C4C6.data();
  }
  const unsigned char *getPsCrossC5C7PTPtr() const {
    return pt_pscross_C5C7.data();
  }
  // 邻角表 Getters
  const unsigned char *getPsCrossC4C5PTPtr() const {
    return pt_pscross_C4C5.data();
  }
  const unsigned char *getPsCrossC4C7PTPtr() const {
    return pt_pscross_C4C7.data();
  }
  const unsigned char *getPsCrossC5C6PTPtr() const {
    return pt_pscross_C5C6.data();
  }
  const unsigned char *getPsCrossC6C7PTPtr() const {
    return pt_pscross_C6C7.data();
  }

  // Corner3 Getters
  const unsigned char *getPsCrossC4C5C6PTPtr() const {
    return pt_pscross_C4C5C6.data();
  }
  const unsigned char *getPsCrossC4C5C7PTPtr() const {
    return pt_pscross_C4C5C7.data();
  }
  const unsigned char *getPsCrossC4C6C7PTPtr() const {
    return pt_pscross_C4C6C7.data();
  }
  const unsigned char *getPsCrossC5C6C7PTPtr() const {
    return pt_pscross_C5C6C7.data();
  }

  // HasChecks (新名)
  bool hasPsCrossE0E2PT() const { return !pt_pscross_E0E2.empty(); }
  bool hasPsCrossE1E3PT() const { return !pt_pscross_E1E3.empty(); }
  bool hasPsCrossE0E1PT() const { return !pt_pscross_E0E1.empty(); }
  bool hasPsCrossE0E3PT() const { return !pt_pscross_E0E3.empty(); }
  bool hasPsCrossE1E2PT() const { return !pt_pscross_E1E2.empty(); }
  bool hasPsCrossE2E3PT() const { return !pt_pscross_E2E3.empty(); }
  bool hasPsCrossE0E1E2PT() const { return !pt_pscross_E0E1E2.empty(); }
  bool hasPsCrossE1E2E3PT() const { return !pt_pscross_E1E2E3.empty(); }
  bool hasPsCrossE0E2E3PT() const { return !pt_pscross_E0E2E3.empty(); }
  bool hasPsCrossE0E1E3PT() const { return !pt_pscross_E0E1E3.empty(); }
  bool hasPsCrossC4C6PT() const { return !pt_pscross_C4C6.empty(); }
  bool hasPsCrossC5C7PT() const { return !pt_pscross_C5C7.empty(); }
  bool hasPsCrossC4C5PT() const { return !pt_pscross_C4C5.empty(); }
  bool hasPsCrossC4C7PT() const { return !pt_pscross_C4C7.empty(); }
  bool hasPsCrossC5C6PT() const { return !pt_pscross_C5C6.empty(); }
  bool hasPsCrossC6C7PT() const { return !pt_pscross_C6C7.empty(); }
  bool hasPsCrossC4C5C6PT() const { return !pt_pscross_C4C5C6.empty(); }
  bool hasPsCrossC4C5C7PT() const { return !pt_pscross_C4C5C7.empty(); }
  bool hasPsCrossC4C6C7PT() const { return !pt_pscross_C4C6C7.empty(); }
  bool hasPsCrossC5C6C7PT() const { return !pt_pscross_C5C6C7.empty(); }

  // === PseudoPair Getters (新名) ===
  const unsigned char *getPsCrossCPTPtr(int c) const {
    return pt_pscross_C[c].data();
  }
  const unsigned char *getPsCrossCDiffPTPtr(int idx) const {
    return pt_pscross_C_diff[idx].data();
  }
  const unsigned char *getPsPairECPTPtr(int idx) const {
    return pt_pspair_CE[idx].data();
  }

  // === EOCross Getters (新名) ===
  const unsigned char *getEOCInsC4PTPtr() const { return pt_eoc_ins_C4.data(); }
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
  // 对棱表生成函数
  void genPTPsCrossE0E2();
  void genPTPsCrossE1E3();
  // 邻棱表生成函数
  void genPTPsCrossE0E1();
  void genPTPsCrossE0E3();
  void genPTPsCrossE1E2();
  void genPTPsCrossE2E3();
  // Edge3 Generators
  void genPTPsCrossE0E1E2();
  void genPTPsCrossE1E2E3();
  void genPTPsCrossE0E2E3();
  void genPTPsCrossE0E1E3();
  // 对角生成函数
  void genPTPsCrossC4C6();
  void genPTPsCrossC5C7();
  // 邻角生成函数
  void genPTPsCrossC4C5();
  void genPTPsCrossC4C7();
  void genPTPsCrossC5C6();
  void genPTPsCrossC6C7();
  // Corner3 Generators
  void genPTPsCrossC4C5C6();
  void genPTPsCrossC4C5C7();
  void genPTPsCrossC4C6C7();
  void genPTPsCrossC5C6C7();
  // PseudoPair 变体生成函数
  void genPTPsCrossC(int c); // Cross+Corner 变体 (c=0..3 → C4..C7)
  void genPTPsCrossCDiff(int c, int e); // XCross 变体 (c=0..3, e=0..3)
  void genPTPsPairCE(int c, int e);     // Pair 变体 (c=0..3, e=0..3)

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

void createPTPsCrossEdges2(int idx_cr, int idx_e2, int sz_cr, int sz_e2,
                           int depth, const std::vector<int> &t_cr,
                           const std::vector<int> &t_e2,
                           std::vector<unsigned char> &pt);

// 新增 helper: 针对角块对
void createPTPsCrossCorners2(int idx_cr, int idx_c2, int sz_cr, int sz_c2,
                             int depth, const std::vector<int> &t_cr,
                             const std::vector<int> &t_c2,
                             std::vector<unsigned char> &pt);

void createPTPsCrossCorners3(int idx_cr, int idx_c3, int sz_cr, int sz_c3,
                             int depth, const std::vector<int> &t_cr,
                             const std::vector<int> &t_c3,
                             std::vector<unsigned char> &pt);

void createPTPsCrossEdges3(int idx_cr, int idx_e3, int sz_cr, int sz_e3,
                           int depth, const std::vector<int> &t_cr,
                           const std::vector<int> &t_e3,
                           std::vector<unsigned char> &pt);

void createPTEdge6Corner2(int sz_e6, int sz_c2, int depth,
                          const std::vector<int> &target_e_ids,
                          const std::vector<int> &target_c_ids,
                          const std::vector<int> &mt_e6,
                          const std::vector<int> &mt_c2,
                          std::vector<unsigned char> &pt);

// --- 级联剪枝表生成函数 (from eo_cross_analyzer) ---
void createCascadedPT3(int i1, int i2, int s1, int s2, int depth,
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
