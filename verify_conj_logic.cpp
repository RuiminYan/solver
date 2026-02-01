/*
 * verify_conj_logic.cpp - 验证 Conj 查表逻辑 (修正版)
 *
 * 关键发现：
 * - _into_slot 表是 Cross + Corner 表 (2.18MB)，不包含 Edge
 * - 索引 = cross_idx + corner_idx (cross scaled by 24, corner is 0-23)
 * - "slot" 指的是初始化时的目标槽位，用于设置 pseudo 等效状态
 */

#include "cube_common.h"
#include "move_tables.h"
#include "prune_tables.h"
#include <iostream>
#include <random>
#include <vector>

const int *p_multi = nullptr;
const int *p_corner = nullptr;

// Cross + Corner 表 (不含 Edge)
std::vector<unsigned char> table_C4_slot0; // C4_into_slot0
std::vector<unsigned char> table_C5_slot1; // C5_into_slot1

bool load_tables() {
  if (!load_vector(table_C4_slot0,
                   "prune_table_pseudo_cross_C4_into_slot0.bin")) {
    std::cerr << "Error loading C4_into_slot0" << std::endl;
    return false;
  }
  if (!load_vector(table_C5_slot1,
                   "prune_table_pseudo_cross_C5_into_slot1.bin")) {
    std::cerr << "Error loading C5_into_slot1" << std::endl;
    return false;
  }
  std::cout << "Tables loaded: C4_slot0=" << table_C4_slot0.size()
            << " C5_slot1=" << table_C5_slot1.size() << std::endl;
  std::cout.flush();
  return true;
}

int main() {
  std::cout << "=== Simple Conj Verification (Cross+Corner Only) ==="
            << std::endl;

  auto &mtm = MoveTableManager::getInstance();
  if (!mtm.loadAll()) {
    std::cerr << "Failed to load move tables!" << std::endl;
    return 1;
  }

  p_multi = mtm.getCrossTablePtr();
  p_corner = mtm.getCornerTablePtr();

  if (!load_tables())
    return 1;

  std::mt19937 rng(42);
  std::uniform_int_distribution<int> move_dist(0, 17);
  std::uniform_int_distribution<int> len_dist(1, 20);

  int tests = 10000;
  int mismatches = 0;

  std::cout << "Starting " << tests << " tests..." << std::endl;
  std::cout.flush();

  for (int t = 0; t < tests; ++t) {
    int len = len_dist(rng);
    std::vector<int> alg(len);
    for (int i = 0; i < len; ++i)
      alg[i] = move_dist(rng);

    // === Method A: 直接查 C5_into_slot1 ===
    // 初始状态：Cross solved (187520*24), C5 corner index=15
    int cross_a = 187520 * 24;
    int corner_a = 15; // C5 corner index (NOT scaled)

    for (int m : alg) {
      cross_a = p_multi[cross_a + m];
      corner_a = p_corner[corner_a * 18 + m]; // corner index NOT scaled
    }

    // 索引 = cross_idx + corner_idx
    int idx_a = cross_a + corner_a;
    int val_a = get_prune_ptr(table_C5_slot1.data(), idx_a);

    // === Method B: Conj(pslot=1) 后查 C4_into_slot0 ===
    // 初始状态：Cross solved, C4 corner index=12
    int cross_b = 187520 * 24;
    int corner_b = 12; // C4 corner index

    for (int m : alg) {
      int mc = conj_moves_flat[m][1]; // pslot=1 共轭
      cross_b = p_multi[cross_b + mc];
      corner_b = p_corner[corner_b * 18 + mc];
    }

    int idx_b = cross_b + corner_b;
    int val_b = get_prune_ptr(table_C4_slot0.data(), idx_b);

    if (val_a != val_b) {
      mismatches++;
      if (mismatches <= 5) {
        std::cout << "MISMATCH t=" << t << " val_A(C5_s1)=" << val_a
                  << " val_B(C4_s0)=" << val_b << std::endl;
        std::cout << "  idx_a=" << idx_a << " idx_b=" << idx_b << std::endl;
        std::cout.flush();
      }
    }

    if ((t + 1) % 1000 == 0) {
      std::cout << "Progress: " << (t + 1) << "/" << tests
                << " (mismatches=" << mismatches << ")" << std::endl;
      std::cout.flush();
    }
  }

  std::cout << "\n=== Results ===" << std::endl;
  std::cout << "Tests: " << tests << " Mismatches: " << mismatches << std::endl;
  std::cout.flush();

  if (mismatches == 0) {
    std::cout
        << "PASS: C5_into_slot1 can be replaced by Conj(1) + C4_into_slot0!"
        << std::endl;
    std::cout.flush();
    return 0;
  } else {
    std::cout << "FAIL: Tables are NOT equivalent via Conj." << std::endl;
    std::cout.flush();
    return 1;
  }
}
