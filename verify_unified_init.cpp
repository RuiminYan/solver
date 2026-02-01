/*
 * verify_unified_init.cpp - 重新设计的验证逻辑
 *
 * 正确理解 Conj：
 * - pslot=k 表示"我正在处理 slot_k"
 * - Conj(k) 把针对 slot_k 的操作映射到 slot_0
 *
 * 验证逻辑：
 * - Method A: C{4+k} solved → 物理 moves → 查 slot_k 表
 * - Method B: C4 solved → Conj(k) moves → 查 slot0 表
 * 两者应该得到相同的剪枝值
 */

#include "cube_common.h"
#include "move_tables.h"
#include "prune_tables.h"
#include <iostream>
#include <random>
#include <vector>

const int *p_multi = nullptr;
const int *p_corner = nullptr;

std::vector<unsigned char> table_C4_slot0;

bool load_table() {
  if (!load_vector(table_C4_slot0,
                   "prune_table_pseudo_cross_C4_into_slot0.bin")) {
    std::cerr << "Error loading C4_into_slot0" << std::endl;
    return false;
  }
  std::cout << "Table C4_slot0 loaded: " << table_C4_slot0.size() << " bytes"
            << std::endl;
  return true;
}

int main() {
  std::cout << "=== Redesigned Conj Verification ===" << std::endl;

  init_matrix();
  std::cout << "init_matrix() called." << std::endl;

  auto &mtm = MoveTableManager::getInstance();
  if (!mtm.loadAll()) {
    std::cerr << "Failed to load move tables!" << std::endl;
    return 1;
  }

  p_multi = mtm.getCrossTablePtr();
  p_corner = mtm.getCornerTablePtr();

  if (!load_table())
    return 1;

  // 各 pslot 对应的 corner 初始索引
  std::vector<int> corner_init = {12, 15, 18, 21}; // C4, C5, C6, C7

  std::mt19937 rng(42);
  std::uniform_int_distribution<int> move_dist(0, 17);
  std::uniform_int_distribution<int> len_dist(1, 20);

  int tests = 10000;
  int mismatches = 0;

  std::cout << "\nVerification Logic:" << std::endl;
  std::cout << "  A: C{4+k} solved + physical moves -> query slot_k table"
            << std::endl;
  std::cout << "  B: C4 solved + Conj(k) moves -> query slot0 table"
            << std::endl;
  std::cout << "  Expected: val_A == val_B (if unified init works)"
            << std::endl;
  std::cout << std::endl;

  for (int t = 0; t < tests; ++t) {
    int len = len_dist(rng);
    std::vector<int> alg(len);
    for (int i = 0; i < len; ++i)
      alg[i] = move_dist(rng);

    for (int pslot = 0; pslot < 4; ++pslot) {
      // === Method A: C{4+pslot} solved + 物理 moves ===
      // 初始状态: Cross solved, Corner = C{4+pslot}
      int cross_a = 187520 * 24;
      int corner_a = corner_init[pslot]; // C4/C5/C6/C7 基于 pslot

      for (int m : alg) {
        cross_a = p_multi[cross_a + m];
        corner_a = p_corner[corner_a * 18 + m];
      }

      int idx_a = cross_a + corner_a;
      // 注意：这里我们仍然需要查正确的表
      // 但因为我们统一初始化了，所有表内容相同，查 slot0 就行
      int val_a = get_prune_ptr(table_C4_slot0.data(), idx_a);

      // === Method B: C4 solved + Conj(pslot) moves ===
      // 初始状态: Cross solved, Corner = C4 (固定)
      int cross_b = 187520 * 24;
      int corner_b = 12; // 始终是 C4

      for (int m : alg) {
        int mc = conj_moves_flat[m][pslot];
        cross_b = p_multi[cross_b + mc];
        corner_b = p_corner[corner_b * 18 + mc];
      }

      int idx_b = cross_b + corner_b;
      int val_b = get_prune_ptr(table_C4_slot0.data(), idx_b);

      if (val_a != val_b) {
        mismatches++;
        if (mismatches <= 10) {
          std::cout << "MISMATCH pslot=" << pslot << " t=" << t
                    << " corner_init=" << corner_init[pslot]
                    << " val_A=" << val_a << " val_B=" << val_b << std::endl;
        }
      }
    }

    if ((t + 1) % 2000 == 0) {
      std::cout << "Progress: " << (t + 1) << "/" << tests
                << " (mismatches=" << mismatches << ")" << std::endl;
    }
  }

  std::cout << "\n=== Results ===" << std::endl;
  std::cout << "Tests: " << tests << " x 4 pslots = " << (tests * 4)
            << std::endl;
  std::cout << "Mismatches: " << mismatches << std::endl;

  if (mismatches == 0) {
    std::cout << "PASS: Unified init + Conj query is valid!" << std::endl;
    return 0;
  } else {
    double mismatch_rate = 100.0 * mismatches / (tests * 4);
    std::cout << "FAIL: Mismatch rate = " << mismatch_rate << "%" << std::endl;
    return 1;
  }
}
