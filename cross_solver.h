/*
 * cross_solver.h - Cross 搜索器（Std/Pseudo 共享）
 *
 * NOTE: 从 std_analyzer.cpp 和 pseudo_analyzer.cpp 中抽取。
 * 两者唯一差异是剪枝表（pt_cross vs pt_pscross），通过构造函数参数区分。
 * EO 版因维度和对称策略差异过大，保持独立。
 */

#pragma once

#include "cube_common.h"
#include "move_tables.h"
#include "prune_tables.h"

struct CrossSolver {
  const int *p_mt_edge2;
  const unsigned char *p_pt; // Cross 或 PsCross 剪枝表指针

  // NOTE: isPseudo=false 使用标准 Cross 表，isPseudo=true 使用 Pseudo Cross 表
  CrossSolver(bool isPseudo = false) {
    auto &mtm = MoveTableManager::getInstance();
    auto &ptm = PruneTableManager::getInstance();

    p_mt_edge2 = mtm.getMTEdge2Ptr();
    p_pt = isPseudo ? ptm.getPsCrossPTPtr() : ptm.getCrossPTPtr();
  }

  bool search(int i1, int i2, int depth, int prev) {
    const int *moves = valid_moves_flat[prev];
    const int count = valid_moves_count[prev];
    for (int k = 0; k < count; ++k) {
      int m = moves[k];
      COUNT_NODE
      int n_i1 = p_mt_edge2[i1 + m];
      int n_i2 = p_mt_edge2[i2 + m];
      long long idx = (long long)n_i1 * StateSpace::EDGE2 + n_i2;
      if (get_prune(p_pt, idx) >= depth)
        continue;
      if (depth == 1) {
        return true;
      }
      if (search(n_i1 * 18, n_i2 * 18, depth - 1, m))
        return true;
    }
    return false;
  }

  std::vector<int> get_stats(const std::vector<int> &base_alg,
                             const std::vector<std::string> &rots) {
    std::vector<int> res(rots.size(), 0);
    for (size_t i = 0; i < rots.size(); ++i) {
      std::vector<int> alg = alg_rotation(base_alg, rots[i]);
      int i1 = 416, i2 = 520;
      for (int m : alg) {
        i1 = p_mt_edge2[i1 * 18 + m];
        i2 = p_mt_edge2[i2 * 18 + m];
      }
      long long idx = (long long)i1 * StateSpace::EDGE2 + i2;
      int d_min = get_prune(p_pt, idx);
      if (d_min == 0)
        continue;
      for (int d = d_min; d <= 8; ++d) {
        if (search(i1 * 18, i2 * 18, d, 18)) {
          res[i] = d;
          break;
        }
      }
    }
    return res;
  }
};
