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
  std::vector<unsigned char> pt_cross;      // Cross基础剪枝表
  std::vector<unsigned char> pt_cross_C4;   // Cross + C4剪枝表
  std::vector<unsigned char> pt_pair_C4E0;  // Pair C4+E0剪枝表
  std::vector<unsigned char> pt_cross_C4E0; // XCross C4+E0剪枝表

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
  std::vector<unsigned char> pt_pspair_ec[16];      // EC Pair: edge*4+corner

  // === EOCross 专用表 ===
  // NOTE: eo_cross_c4使用不同生成函数，与pt_cross_C4不同
  std::vector<unsigned char> pt_eoc_C4;  // Cross+C4 (EOCross版)
  std::vector<unsigned char> pt_ep4eo12; // Dependency+EO
  std::vector<unsigned char>
      pt_cross_C4E0E1_C4E0E2_C4E0E3[3]; // Plus Edge: Right/Diag/Left
  std::vector<unsigned char>
      pt_cross_C4C5E0_C4C6E0_C4C7E0[3];       // Plus Corner: Right/Diag/Left
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
  void generateAllSequentially();

  // === 新 Getter (返回 vector 引用) ===
  const std::vector<unsigned char> &getCrossPT() const { return pt_cross; }
  const std::vector<unsigned char> &getCrossC4PT() const { return pt_cross_C4; }
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
  const unsigned char *getCrossC4PTPtr() const { return pt_cross_C4.data(); }
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
    return pt_pspair_ec[idx].data();
  }

  // === EOCross Getters (新名) ===
  const unsigned char *getEOCC4PTPtr() const { return pt_eoc_C4.data(); }
  const unsigned char *getEP4EO12PTPtr() const { return pt_ep4eo12.data(); }
  const unsigned char *getCrossC4E0E1PTPtr() const {
    return pt_cross_C4E0E1_C4E0E2_C4E0E3[0].data();
  }
  const unsigned char *getCrossC4E0E2PTPtr() const {
    return pt_cross_C4E0E1_C4E0E2_C4E0E3[1].data();
  }
  const unsigned char *getCrossC4E0E3PTPtr() const {
    return pt_cross_C4E0E1_C4E0E2_C4E0E3[2].data();
  }
  const unsigned char *getCrossC4C5E0PTPtr() const {
    return pt_cross_C4C5E0_C4C6E0_C4C7E0[0].data();
  }
  const unsigned char *getCrossC4C6E0PTPtr() const {
    return pt_cross_C4C5E0_C4C6E0_C4C7E0[1].data();
  }
  const unsigned char *getCrossC4C7E0PTPtr() const {
    return pt_cross_C4C5E0_C4C6E0_C4C7E0[2].data();
  }
  const unsigned char *getCrossC4C5C6PTPtr() const {
    return pt_cross_C4C5C6.data();
  }

  // === 旧名别名 (Analyzer 层兼容, 后续迁移后删除) ===
  const std::vector<unsigned char> &getCrossPrune() const {
    return getCrossPT();
  }
  const std::vector<unsigned char> &getCrossC4Prune() const {
    return getCrossC4PT();
  }
  const std::vector<unsigned char> &getPairC4E0Prune() const {
    return getPairC4E0PT();
  }
  const std::vector<unsigned char> &getXCrossC4E0Prune() const {
    return getCrossC4E0PT();
  }
  const std::vector<unsigned char> &getHugeNeighborPrune() const {
    return getCrossC4C5E0E1PT();
  }
  const std::vector<unsigned char> &getHugeDiagonalPrune() const {
    return getCrossC4C6E0E2PT();
  }
  const unsigned char *getCrossPrunePtr() const { return getCrossPTPtr(); }
  const unsigned char *getCrossC4PrunePtr() const { return getCrossC4PTPtr(); }
  const unsigned char *getPairC4E0PrunePtr() const {
    return getPairC4E0PTPtr();
  }
  const unsigned char *getXCrossC4E0PrunePtr() const {
    return getCrossC4E0PTPtr();
  }
  const unsigned char *getHugeNeighborPrunePtr() const {
    return getCrossC4C5E0E1PTPtr();
  }
  const unsigned char *getHugeDiagonalPrunePtr() const {
    return getCrossC4C6E0E2PTPtr();
  }
  const unsigned char *getPseudoCrossPrunePtr() const {
    return getPsCrossPTPtr();
  }
  const unsigned char *getPseudoCrossBasePrunePtr(int i) const {
    return getPsCrossC4EPTPtr(i);
  }
  const unsigned char *getPseudoCrossC4EPrunePtr(int i) const {
    return getPsCrossC4EPTPtr(i);
  }
  const unsigned char *getPseudoCrossE0E2PrunePtr() const {
    return getPsCrossE0E2PTPtr();
  }
  const unsigned char *getPseudoCrossE1E3PrunePtr() const {
    return getPsCrossE1E3PTPtr();
  }
  const unsigned char *getPseudoCrossE0E1PrunePtr() const {
    return getPsCrossE0E1PTPtr();
  }
  const unsigned char *getPseudoCrossE0E3PrunePtr() const {
    return getPsCrossE0E3PTPtr();
  }
  const unsigned char *getPseudoCrossE1E2PrunePtr() const {
    return getPsCrossE1E2PTPtr();
  }
  const unsigned char *getPseudoCrossE2E3PrunePtr() const {
    return getPsCrossE2E3PTPtr();
  }
  const unsigned char *getPseudoCrossE0E1E2PrunePtr() const {
    return getPsCrossE0E1E2PTPtr();
  }
  const unsigned char *getPseudoCrossE1E2E3PrunePtr() const {
    return getPsCrossE1E2E3PTPtr();
  }
  const unsigned char *getPseudoCrossE0E2E3PrunePtr() const {
    return getPsCrossE0E2E3PTPtr();
  }
  const unsigned char *getPseudoCrossE0E1E3PrunePtr() const {
    return getPsCrossE0E1E3PTPtr();
  }
  const unsigned char *getPseudoCrossC4C6PrunePtr() const {
    return getPsCrossC4C6PTPtr();
  }
  const unsigned char *getPseudoCrossC5C7PrunePtr() const {
    return getPsCrossC5C7PTPtr();
  }
  const unsigned char *getPseudoCrossC4C5PrunePtr() const {
    return getPsCrossC4C5PTPtr();
  }
  const unsigned char *getPseudoCrossC4C7PrunePtr() const {
    return getPsCrossC4C7PTPtr();
  }
  const unsigned char *getPseudoCrossC5C6PrunePtr() const {
    return getPsCrossC5C6PTPtr();
  }
  const unsigned char *getPseudoCrossC6C7PrunePtr() const {
    return getPsCrossC6C7PTPtr();
  }
  const unsigned char *getPseudoCrossC4C5C6PrunePtr() const {
    return getPsCrossC4C5C6PTPtr();
  }
  const unsigned char *getPseudoCrossC4C5C7PrunePtr() const {
    return getPsCrossC4C5C7PTPtr();
  }
  const unsigned char *getPseudoCrossC4C6C7PrunePtr() const {
    return getPsCrossC4C6C7PTPtr();
  }
  const unsigned char *getPseudoCrossC5C6C7PrunePtr() const {
    return getPsCrossC5C6C7PTPtr();
  }
  bool hasPseudoCrossE0E2Prune() const { return hasPsCrossE0E2PT(); }
  bool hasPseudoCrossE1E3Prune() const { return hasPsCrossE1E3PT(); }
  bool hasPseudoCrossE0E1Prune() const { return hasPsCrossE0E1PT(); }
  bool hasPseudoCrossE0E3Prune() const { return hasPsCrossE0E3PT(); }
  bool hasPseudoCrossE1E2Prune() const { return hasPsCrossE1E2PT(); }
  bool hasPseudoCrossE2E3Prune() const { return hasPsCrossE2E3PT(); }
  bool hasPseudoCrossE0E1E2Prune() const { return hasPsCrossE0E1E2PT(); }
  bool hasPseudoCrossE1E2E3Prune() const { return hasPsCrossE1E2E3PT(); }
  bool hasPseudoCrossE0E2E3Prune() const { return hasPsCrossE0E2E3PT(); }
  bool hasPseudoCrossE0E1E3Prune() const { return hasPsCrossE0E1E3PT(); }
  bool hasPseudoCrossC4C6Prune() const { return hasPsCrossC4C6PT(); }
  bool hasPseudoCrossC5C7Prune() const { return hasPsCrossC5C7PT(); }
  bool hasPseudoCrossC4C5Prune() const { return hasPsCrossC4C5PT(); }
  bool hasPseudoCrossC4C7Prune() const { return hasPsCrossC4C7PT(); }
  bool hasPseudoCrossC5C6Prune() const { return hasPsCrossC5C6PT(); }
  bool hasPseudoCrossC6C7Prune() const { return hasPsCrossC6C7PT(); }
  bool hasPseudoCrossC4C5C6Prune() const { return hasPsCrossC4C5C6PT(); }
  bool hasPseudoCrossC4C5C7Prune() const { return hasPsCrossC4C5C7PT(); }
  bool hasPseudoCrossC4C6C7Prune() const { return hasPsCrossC4C6C7PT(); }
  bool hasPseudoCrossC5C6C7Prune() const { return hasPsCrossC5C6C7PT(); }
  const unsigned char *getPseudoPairBasePrunePtr(int c) const {
    return getPsCrossCPTPtr(c);
  }
  const unsigned char *getPseudoPairCPrunePtr(int c) const {
    return getPsCrossCPTPtr(c);
  }
  const unsigned char *getPseudoPairXCPrunePtr(int idx) const {
    return getPsCrossCDiffPTPtr(idx);
  }
  const unsigned char *getPseudoPairECPrunePtr(int idx) const {
    return getPsPairECPTPtr(idx);
  }
  const unsigned char *getEOCrossC4PrunePtr() const { return getEOCC4PTPtr(); }
  const unsigned char *getEODepEOPrunePtr() const { return getEP4EO12PTPtr(); }
  const unsigned char *getEOPlusEdgePrunePtr(int idx) const {
    return pt_cross_C4E0E1_C4E0E2_C4E0E3[idx].data();
  }
  const unsigned char *getEOPlusCornerPrunePtr(int idx) const {
    return pt_cross_C4C5E0_C4C6E0_C4C7E0[idx].data();
  }
  const unsigned char *getEO3CornerPrunePtr() const {
    return getCrossC4C5C6PTPtr();
  }

  // 生成函数 (新名)
  void generateCrossPT();
  void generateCrossC4PT();
  void generatePairC4E0PT();
  void generateCrossC4E0PT();
  void generateCrossC4C5E0E1PT();
  void generateCrossC4C6E0E2PT();

  // EOCross 专用生成函数
  void generateEP4EO12PT(); // EP4 + EO12 联合剪枝表

  // Pseudo 生成函数
  void generatePsCrossPT();
  void generatePsCrossC4EPT(int offset_idx);
  // 对棱表生成函数
  void generatePsCrossE0E2PT();
  void generatePsCrossE1E3PT();
  // 邻棱表生成函数
  void generatePsCrossE0E1PT();
  void generatePsCrossE0E3PT();
  void generatePsCrossE1E2PT();
  void generatePsCrossE2E3PT();
  // Edge3 Generators
  void generatePsCrossE0E1E2PT();
  void generatePsCrossE1E2E3PT();
  void generatePsCrossE0E2E3PT();
  void generatePsCrossE0E1E3PT();
  // 对角生成函数
  void generatePsCrossC4C6PT();
  void generatePsCrossC5C7PT();
  // 邻角生成函数
  void generatePsCrossC4C5PT();
  void generatePsCrossC4C7PT();
  void generatePsCrossC5C6PT();
  void generatePsCrossC6C7PT();
  // Corner3 Generators
  void generatePsCrossC4C5C6PT();
  void generatePsCrossC4C5C7PT();
  void generatePsCrossC4C6C7PT();
  void generatePsCrossC5C6C7PT();

  // === 旧生成函数别名 ===
  void generateCrossPrune() { generateCrossPT(); }
  void generateCrossC4Prune() { generateCrossC4PT(); }
  void generatePairC4E0Prune() { generatePairC4E0PT(); }
  void generateXCrossC4E0Prune() { generateCrossC4E0PT(); }
  void generateHugeNeighborPrune() { generateCrossC4C5E0E1PT(); }
  void generateHugeDiagonalPrune() { generateCrossC4C6E0E2PT(); }
  void generateEODepEOPrune() { generateEP4EO12PT(); }
  void generatePseudoCrossPrune() { generatePsCrossPT(); }
  void generatePseudoCrossBasePrune(int i) { generatePsCrossC4EPT(i); }
  void generatePseudoCrossC4EPrune(int i) { generatePsCrossC4EPT(i); }
  void generatePseudoCrossE0E2Prune() { generatePsCrossE0E2PT(); }
  void generatePseudoCrossE1E3Prune() { generatePsCrossE1E3PT(); }
  void generatePseudoCrossE0E1Prune() { generatePsCrossE0E1PT(); }
  void generatePseudoCrossE0E3Prune() { generatePsCrossE0E3PT(); }
  void generatePseudoCrossE1E2Prune() { generatePsCrossE1E2PT(); }
  void generatePseudoCrossE2E3Prune() { generatePsCrossE2E3PT(); }
  void generatePseudoCrossE0E1E2Prune() { generatePsCrossE0E1E2PT(); }
  void generatePseudoCrossE1E2E3Prune() { generatePsCrossE1E2E3PT(); }
  void generatePseudoCrossE0E2E3Prune() { generatePsCrossE0E2E3PT(); }
  void generatePseudoCrossE0E1E3Prune() { generatePsCrossE0E1E3PT(); }
  void generatePseudoCrossC4C6Prune() { generatePsCrossC4C6PT(); }
  void generatePseudoCrossC5C7Prune() { generatePsCrossC5C7PT(); }
  void generatePseudoCrossC4C5Prune() { generatePsCrossC4C5PT(); }
  void generatePseudoCrossC4C7Prune() { generatePsCrossC4C7PT(); }
  void generatePseudoCrossC5C6Prune() { generatePsCrossC5C6PT(); }
  void generatePseudoCrossC6C7Prune() { generatePsCrossC6C7PT(); }
  void generatePseudoCrossC4C5C6Prune() { generatePsCrossC4C5C6PT(); }
  void generatePseudoCrossC4C5C7Prune() { generatePsCrossC4C5C7PT(); }
  void generatePseudoCrossC4C6C7Prune() { generatePsCrossC4C6C7PT(); }
  void generatePseudoCrossC5C6C7Prune() { generatePsCrossC5C6C7PT(); }

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
void create_cascaded_prune_table(int i1, int i2, int s1, int s2, int depth,
                                 const std::vector<int> &t1,
                                 const std::vector<int> &t2,
                                 std::vector<unsigned char> &pt);
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
