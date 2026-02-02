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
  std::vector<unsigned char> cross_prune;        // Cross基础剪枝表
  std::vector<unsigned char> cross_c4_prune;     // Cross + C4剪枝表
  std::vector<unsigned char> pair_c4_e0_prune;   // Pair C4+E0剪枝表
  std::vector<unsigned char> xcross_c4_e0_prune; // XCross C4+E0剪枝表

  // 巨型剪枝表
  std::vector<unsigned char> huge_neighbor_prune; // 相邻槽剪枝表
  std::vector<unsigned char> huge_diagonal_prune; // 对角槽剪枝表

  // Pseudo 相关剪枝表
  std::vector<unsigned char> pseudo_cross_prune; // Pseudo Cross 剪枝表
  std::vector<unsigned char>
      pseudo_cross_base_prune[4]; // Pseudo Cross 基础表 (Offset 0,1,2,3)
  std::vector<unsigned char>
      pseudo_cross_E0_E2_prune; // Pseudo Cross + E0,E2 对棱表 (已有)
  std::vector<unsigned char> pseudo_cross_E1_E3_prune; // 对棱表 (新增)

  // 邻棱表
  std::vector<unsigned char> pseudo_cross_E0_E1_prune; // 邻棱 (已有)
  std::vector<unsigned char> pseudo_cross_E0_E3_prune; // 邻棱 (新增)
  std::vector<unsigned char> pseudo_cross_E1_E2_prune; // 邻棱 (新增)
  std::vector<unsigned char> pseudo_cross_E2_E3_prune; // 邻棱 (新增)

  // Edge3 Triples
  std::vector<unsigned char> pseudo_cross_E0_E1_E2_prune; // 基准
  std::vector<unsigned char> pseudo_cross_E0_E1_E3_prune; // Newly Added
  std::vector<unsigned char> pseudo_cross_E0_E2_E3_prune; // Newly Added
  std::vector<unsigned char> pseudo_cross_E1_E2_E3_prune; // Newly Added

  // 新增对角表 (F2L Corner Pairs)
  std::vector<unsigned char> pseudo_cross_C4_C6_prune; // 对角 (已有)
  std::vector<unsigned char> pseudo_cross_C5_C7_prune; // 对角 (新增)

  // 新增邻角表
  std::vector<unsigned char> pseudo_cross_C4_C5_prune; // 邻角 (已有)
  std::vector<unsigned char> pseudo_cross_C4_C7_prune; // 邻角 (新增)
  std::vector<unsigned char> pseudo_cross_C5_C6_prune; // 邻角 (新增)
  std::vector<unsigned char> pseudo_cross_C6_C7_prune; // 邻角 (新增)

  // Corner3 Triples
  std::vector<unsigned char> pseudo_cross_C4_C5_C6_prune; // 基准
  std::vector<unsigned char> pseudo_cross_C4_C5_C7_prune; // Newly Added
  std::vector<unsigned char> pseudo_cross_C4_C6_C7_prune; // Newly Added
  std::vector<unsigned char> pseudo_cross_C5_C6_C7_prune; // Newly Added

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

  // 顺序生成所有表
  void generateAllSequentially();

  // 获取剪枝表的只读访问
  const std::vector<unsigned char> &getCrossPrune() const {
    return cross_prune;
  }
  const std::vector<unsigned char> &getCrossC4Prune() const {
    return cross_c4_prune;
  }
  const std::vector<unsigned char> &getPairC4E0Prune() const {
    return pair_c4_e0_prune;
  }
  const std::vector<unsigned char> &getXCrossC4E0Prune() const {
    return xcross_c4_e0_prune;
  }
  const std::vector<unsigned char> &getHugeNeighborPrune() const {
    return huge_neighbor_prune;
  }
  const std::vector<unsigned char> &getHugeDiagonalPrune() const {
    return huge_diagonal_prune;
  }

  // 获取 Pseudo 剪枝表
  const unsigned char *getPseudoCrossPrunePtr() const {
    return pseudo_cross_prune.data();
  }
  const unsigned char *getPseudoCrossBasePrunePtr(int i) const {
    return pseudo_cross_base_prune[i].data();
  }
  const unsigned char *getPseudoCrossE0E2PrunePtr() const {
    return pseudo_cross_E0_E2_prune.data();
  }
  const unsigned char *getPseudoCrossE1E3PrunePtr() const {
    return pseudo_cross_E1_E3_prune.data();
  }

  // 邻棱表 Getters
  const unsigned char *getPseudoCrossE0E1PrunePtr() const {
    return pseudo_cross_E0_E1_prune.data();
  }
  const unsigned char *getPseudoCrossE0E3PrunePtr() const {
    return pseudo_cross_E0_E3_prune.data();
  }
  const unsigned char *getPseudoCrossE1E2PrunePtr() const {
    return pseudo_cross_E1_E2_prune.data();
  }
  const unsigned char *getPseudoCrossE2E3PrunePtr() const {
    return pseudo_cross_E2_E3_prune.data();
  }

  // Edge3 Getters
  const unsigned char *getPseudoCrossE0E1E2PrunePtr() const {
    return pseudo_cross_E0_E1_E2_prune.data();
  }
  const unsigned char *getPseudoCrossE1E2E3PrunePtr() const {
    return pseudo_cross_E1_E2_E3_prune.data();
  }
  const unsigned char *getPseudoCrossE0E2E3PrunePtr() const {
    return pseudo_cross_E0_E2_E3_prune.data();
  }
  const unsigned char *getPseudoCrossE0E1E3PrunePtr() const {
    return pseudo_cross_E0_E1_E3_prune.data();
  }

  // 对角表 Getters
  const unsigned char *getPseudoCrossC4C6PrunePtr() const {
    return pseudo_cross_C4_C6_prune.data();
  }
  const unsigned char *getPseudoCrossC5C7PrunePtr() const {
    return pseudo_cross_C5_C7_prune.data();
  }
  // 邻角表 Getters
  const unsigned char *getPseudoCrossC4C5PrunePtr() const {
    return pseudo_cross_C4_C5_prune.data();
  }
  const unsigned char *getPseudoCrossC4C7PrunePtr() const {
    return pseudo_cross_C4_C7_prune.data();
  }
  const unsigned char *getPseudoCrossC5C6PrunePtr() const {
    return pseudo_cross_C5_C6_prune.data();
  }
  const unsigned char *getPseudoCrossC6C7PrunePtr() const {
    return pseudo_cross_C6_C7_prune.data();
  }

  // Corner3 Getters
  const unsigned char *getPseudoCrossC4C5C6PrunePtr() const {
    return pseudo_cross_C4_C5_C6_prune.data();
  }
  const unsigned char *getPseudoCrossC4C5C7PrunePtr() const {
    return pseudo_cross_C4_C5_C7_prune.data();
  }
  const unsigned char *getPseudoCrossC4C6C7PrunePtr() const {
    return pseudo_cross_C4_C6_C7_prune.data();
  }
  const unsigned char *getPseudoCrossC5C6C7PrunePtr() const {
    return pseudo_cross_C5_C6_C7_prune.data();
  }

  bool hasPseudoCrossE0E2Prune() const {
    return !pseudo_cross_E0_E2_prune.empty();
  }
  bool hasPseudoCrossE1E3Prune() const {
    return !pseudo_cross_E1_E3_prune.empty();
  }
  // 邻棱表 HasChecks
  bool hasPseudoCrossE0E1Prune() const {
    return !pseudo_cross_E0_E1_prune.empty();
  }
  bool hasPseudoCrossE0E3Prune() const {
    return !pseudo_cross_E0_E3_prune.empty();
  }
  bool hasPseudoCrossE1E2Prune() const {
    return !pseudo_cross_E1_E2_prune.empty();
  }
  bool hasPseudoCrossE2E3Prune() const {
    return !pseudo_cross_E2_E3_prune.empty();
  }
  // Edge3 HasChecks
  bool hasPseudoCrossE0E1E2Prune() const {
    return !pseudo_cross_E0_E1_E2_prune.empty();
  }
  bool hasPseudoCrossE1E2E3Prune() const {
    return !pseudo_cross_E1_E2_E3_prune.empty();
  }
  bool hasPseudoCrossE0E2E3Prune() const {
    return !pseudo_cross_E0_E2_E3_prune.empty();
  }
  bool hasPseudoCrossE0E1E3Prune() const {
    return !pseudo_cross_E0_E1_E3_prune.empty();
  }
  // 对角表 HasChecks
  bool hasPseudoCrossC4C6Prune() const {
    return !pseudo_cross_C4_C6_prune.empty();
  }
  bool hasPseudoCrossC5C7Prune() const {
    return !pseudo_cross_C5_C7_prune.empty();
  }
  // 邻角表 HasChecks
  bool hasPseudoCrossC4C5Prune() const {
    return !pseudo_cross_C4_C5_prune.empty();
  }
  bool hasPseudoCrossC4C7Prune() const {
    return !pseudo_cross_C4_C7_prune.empty();
  }
  bool hasPseudoCrossC5C6Prune() const {
    return !pseudo_cross_C5_C6_prune.empty();
  }
  bool hasPseudoCrossC6C7Prune() const {
    return !pseudo_cross_C6_C7_prune.empty();
  }
  // Corner3 HasChecks
  bool hasPseudoCrossC4C5C6Prune() const {
    return !pseudo_cross_C4_C5_C6_prune.empty();
  }
  bool hasPseudoCrossC4C5C7Prune() const {
    return !pseudo_cross_C4_C5_C7_prune.empty();
  }
  bool hasPseudoCrossC4C6C7Prune() const {
    return !pseudo_cross_C4_C6_C7_prune.empty();
  }
  bool hasPseudoCrossC5C6C7Prune() const {
    return !pseudo_cross_C5_C6_C7_prune.empty();
  }

  // 获取指针
  const unsigned char *getCrossPrunePtr() const { return cross_prune.data(); }
  const unsigned char *getCrossC4PrunePtr() const {
    return cross_c4_prune.data();
  }
  const unsigned char *getPairC4E0PrunePtr() const {
    return pair_c4_e0_prune.data();
  }
  const unsigned char *getXCrossC4E0PrunePtr() const {
    return xcross_c4_e0_prune.data();
  }
  const unsigned char *getHugeNeighborPrunePtr() const {
    return huge_neighbor_prune.data();
  }
  const unsigned char *getHugeDiagonalPrunePtr() const {
    return huge_diagonal_prune.data();
  }

  // 生成函数
  void generateCrossPrune();
  void generateCrossC4Prune();
  void generatePairC4E0Prune();
  void generateXCrossC4E0Prune();
  void generateHugeNeighborPrune();
  void generateHugeDiagonalPrune();

  // Pseudo 生成函数
  void generatePseudoCrossPrune();
  void generatePseudoCrossBasePrune(int offset_idx);
  // 对棱表生成函数
  void generatePseudoCrossE0E2Prune();
  void generatePseudoCrossE1E3Prune(); // 新增
  // 邻棱表生成函数
  void generatePseudoCrossE0E1Prune();
  void generatePseudoCrossE0E3Prune(); // 新增
  void generatePseudoCrossE1E2Prune(); // 新增
  void generatePseudoCrossE2E3Prune(); // 新增
  // Edge3 Generators
  void generatePseudoCrossE0E1E2Prune();
  void generatePseudoCrossE1E2E3Prune(); // New
  void generatePseudoCrossE0E2E3Prune(); // New
  void generatePseudoCrossE0E1E3Prune(); // New
  // 新增对角生成函数
  void generatePseudoCrossC4C6Prune();
  void generatePseudoCrossC5C7Prune(); // 新增
  // 新增邻角生成函数
  void generatePseudoCrossC4C5Prune();
  void generatePseudoCrossC4C7Prune(); // 新增
  void generatePseudoCrossC5C6Prune(); // 新增
  void generatePseudoCrossC6C7Prune(); // 新增

  // Corner3 Generators
  void generatePseudoCrossC4C5C6Prune();
  void generatePseudoCrossC4C5C7Prune(); // New
  void generatePseudoCrossC4C6C7Prune(); // New
  void generatePseudoCrossC5C6C7Prune(); // New

private:
  // 文件操作
  bool loadTable(std::vector<unsigned char> &table,
                 const std::string &filename);
  void saveTable(const std::vector<unsigned char> &table,
                 const std::string &filename);
};

// --- 剪枝表生成函数 ---
void create_prune_table_cross_c4(int idx1, int idx2, int sz1, int sz2,
                                 int depth, const std::vector<int> &t1,
                                 const std::vector<int> &t2,
                                 std::vector<unsigned char> &pt);

void create_prune_table_pair_base(int idx_e, int idx_c, int sz_e, int sz_c,
                                  int depth, const std::vector<int> &t_edge,
                                  const std::vector<int> &t_corn,
                                  std::vector<unsigned char> &pt);

void create_prune_table_xcross_base(int idx_cr, int idx_cn, int idx_ex,
                                    int sz_cr, int sz_cn, int sz_ex, int depth,
                                    const std::vector<int> &t1,
                                    const std::vector<int> &t2,
                                    const std::vector<int> &t3,
                                    std::vector<unsigned char> &pt);

void create_prune_table_xcross_full(int idx_cr, int idx_cn, int idx_ed,
                                    int sz_cr, int sz_cn, int sz_ed, int depth,
                                    const std::vector<int> &t1,
                                    const std::vector<int> &t2,
                                    const std::vector<int> &t3,
                                    std::vector<unsigned char> &pt,
                                    bool is_pseudo = false);

void create_prune_table_pseudo_cross_edges2(int idx_cr, int idx_e2, int sz_cr,
                                            int sz_e2, int depth,
                                            const std::vector<int> &t_cr,
                                            const std::vector<int> &t_e2,
                                            std::vector<unsigned char> &pt);

// 新增 helper: 针对角块对
void create_prune_table_pseudo_cross_corners2(int idx_cr, int idx_c2, int sz_cr,
                                              int sz_c2, int depth,
                                              const std::vector<int> &t_cr,
                                              const std::vector<int> &t_c2,
                                              std::vector<unsigned char> &pt);

void create_prune_table_pseudo_cross_corners3(int idx_cr, int idx_c3, int sz_cr,
                                              int sz_c3, int depth,
                                              const std::vector<int> &t_cr,
                                              const std::vector<int> &t_c3,
                                              std::vector<unsigned char> &pt);

void create_prune_table_pseudo_cross_edges3(int idx_cr, int idx_e3, int sz_cr,
                                            int sz_e3, int depth,
                                            const std::vector<int> &t_cr,
                                            const std::vector<int> &t_e3,
                                            std::vector<unsigned char> &pt);

void create_prune_table_huge(int sz_e6, int sz_c2, int depth,
                             const std::vector<int> &target_e_ids,
                             const std::vector<int> &target_c_ids,
                             const std::vector<int> &mt_e6,
                             const std::vector<int> &mt_c2,
                             std::vector<unsigned char> &pt);

// --- 级联剪枝表生成函数 (from eo_cross_analyzer) ---
std::vector<unsigned char>
create_cascaded_prune_table(int i1, int i2, int s1, int s2, int depth,
                            const std::vector<int> &t1,
                            const std::vector<int> &t2);
void create_cascaded_prune_table2(int i1, int i2, int s1, int s2, int depth,
                                  const std::vector<int> &t1,
                                  const std::vector<int> &t2,
                                  std::vector<unsigned char> &pt);
void create_cascaded_prune_table3(int i1, int i2, int s1, int s2, int depth,
                                  const std::vector<int> &t1,
                                  const std::vector<int> &t2,
                                  std::vector<unsigned char> &pt);
void create_prune_table_xcross_plus(
    int idx_cr, int idx_cn, int idx_ed, int idx_extra, int sz_cr, int sz_cn,
    int sz_ed, int sz_ex, int depth, const std::vector<int> &t1,
    const std::vector<int> &t2, const std::vector<int> &t3,
    const std::vector<int> &t4, std::vector<unsigned char> &pt);
void create_prune_table_xcross_corn3(
    int idx_cr, int idx_cn, int idx_c5, int idx_c6, int sz_cr, int sz_cn,
    int sz_c5, int sz_c6, int depth, const std::vector<int> &t1,
    const std::vector<int> &t2, const std::vector<int> &t_c5,
    const std::vector<int> &t_c6, std::vector<unsigned char> &pt);

#endif // PRUNE_TABLES_H
