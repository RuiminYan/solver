/*
 * std_analyzer.cpp - 清理后的标准分析器
 */

#include "analyzer_executor.h"
#include "cube_common.h"
#include "move_tables.h"
#include "prune_tables.h"

// --- 全局统计变量 ---
// NOTE: 使用 AnalyzerStats 命名空间中的全局变量
#define global_nodes AnalyzerStats::globalNodes
#define completed_tasks AnalyzerStats::completedTasks
#define is_solving AnalyzerStats::isSolving

// NOTE: COUNT_NODE 宏已移至 analyzer_executor.h

// --- 通用结构 ---
struct SearchContext {
  std::vector<int> sol_len;
  int current_max_depth = 0;
  SearchContext() { sol_len.reserve(32); }
};

struct CrossSolver {
  const int *p_multi;
  const unsigned char *p_prune;

  CrossSolver() {
    auto &mtm = MoveTableManager::getInstance();
    auto &ptm = PruneTableManager::getInstance();

    p_multi = mtm.getEdges2TablePtr();
    p_prune = ptm.getCrossPrunePtr();
  }

  bool search(SearchContext &ctx, int i1, int i2, int depth, int prev) {
    const int *moves = valid_moves_flat[prev];
    const int count = valid_moves_count[prev];
    for (int k = 0; k < count; ++k) {
      int m = moves[k];
      COUNT_NODE
      int n_i1 = p_multi[i1 + m];
      int n_i2 = p_multi[i2 + m];
      long long idx = (long long)n_i1 * 528 + n_i2;
      if (get_prune_ptr(p_prune, idx) >= depth)
        continue;
      if (depth == 1) {
        ctx.sol_len.push_back(ctx.current_max_depth);
        return true;
      }
      if (search(ctx, n_i1 * 18, n_i2 * 18, depth - 1, m))
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
        i1 = p_multi[i1 * 18 + m];
        i2 = p_multi[i2 * 18 + m];
      }
      long long idx = (long long)i1 * 528 + i2;
      int d_min = get_prune_ptr(p_prune, idx);
      if (d_min == 0)
        continue;
      SearchContext ctx;
      for (int d = d_min; d <= 8; ++d) {
        ctx.current_max_depth = d;
        if (search(ctx, i1 * 18, i2 * 18, d, 18)) {
          res[i] = d;
          break;
        }
      }
    }
    return res;
  }
};

struct XCrossSolver {
  const int *p_multi, *p_corn, *p_edge, *p_edge6, *p_corn2;
  const unsigned char *p_prune_neighbor = nullptr, *p_prune_diagonal = nullptr;
  const unsigned char *p_prune_base;

  struct Task1 {
    int id;
    int h;
  };
  struct Task2 {
    int a, b;
    int h;
    int v;
  };
  struct Task3 {
    int a, b, c;
    int h;
    int v12, v23, v31;
  };

  XCrossSolver() {
    auto &mtm = MoveTableManager::getInstance();
    auto &ptm = PruneTableManager::getInstance();

    p_multi = mtm.getCrossTablePtr();
    p_corn = mtm.getCornerTablePtr();
    p_edge = mtm.getEdgeTablePtr();
    p_edge6 = mtm.getEdge6TablePtr();
    p_corn2 = mtm.getCorner2TablePtr();

    p_prune_base = ptm.getXCrossC4E0PrunePtr();
    p_prune_neighbor = ptm.getHugeNeighborPrunePtr();
#if ENABLE_DIAGONAL_STD
    p_prune_diagonal = ptm.getHugeDiagonalPrunePtr();
#endif
  }

  inline int get_plus_table_idx(int s_base, int s_target) {
    int diff = (s_target - s_base + 4) % 4;
    if (diff == 1)
      return 0;
    if (diff == 2)
      return 1;
    if (diff == 3)
      return 2;
    return -1;
  }
  inline int get_neighbor_view(int s1, int s2) {
    if ((s2 - s1 + 4) % 4 == 1)
      return s1;
    if ((s1 - s2 + 4) % 4 == 1)
      return s2;
    return -1;
  }
  inline int get_diagonal_view(int s1, int s2) {
    int mn = std::min(s1, s2);
    int mx = std::max(s1, s2);
    if (mn == 0 && mx == 2)
      return 0;
    if (mn == 1 && mx == 3)
      return 1;
    return -1;
  }
  int get_correct_edge_start(int idx, int e0, int e2, int e4, int e6) {
    if (idx == 0)
      return e2;
    if (idx == 1)
      return e4;
    if (idx == 2)
      return e6;
    return e0;
  }
  int get_correct_corn_start(int idx, int c5, int c6, int c7) {
    if (idx == 0)
      return c5;
    if (idx == 1)
      return c6;
    if (idx == 2)
      return c7;
    return 0;
  }

  void get_conjugated_indices_all(const std::vector<int> &alg, int slot_k,
                                  int &i_mul, int &i_cn, int &i_e0, int &i_e2,
                                  int &i_e4, int &i_e6, int &i_c5, int &i_c6,
                                  int &i_c7, int &i_e6_idx_nb, int &i_c2_idx_nb,
                                  int &i_e6_idx_dg, int &i_c2_idx_dg) {
    int cur_mul = 187520 * 24;
    int cur_cn = 12 * 18;
    int cur_e0 = 0;
    int cur_e2 = 2;
    int cur_e4 = 4;
    int cur_e6 = 6;
    int cur_c5 = 15;
    int cur_c6 = 18;
    int cur_c7 = 21;
    static int solved_e6_nb = -1, solved_c2_nb = -1;
    static int solved_e6_dg = -1, solved_c2_dg = -1;
    if (solved_e6_nb == -1) {
      solved_e6_nb = array_to_index({0, 2, 16, 18, 20, 22}, 6, 2, 12);
      solved_c2_nb = array_to_index({12, 15}, 2, 3, 8);
      solved_e6_dg = array_to_index({0, 4, 16, 18, 20, 22}, 6, 2, 12);
      solved_c2_dg = array_to_index({12, 18}, 2, 3, 8);
    }
    int cur_e6_n = solved_e6_nb * 18;
    int cur_c2_n = solved_c2_nb * 18;
    int cur_e6_d = solved_e6_dg * 18;
    int cur_c2_d = solved_c2_dg * 18;

    for (int m : alg) {
      int mc = conj_moves_flat[m][slot_k];
      cur_mul = p_multi[cur_mul + mc];
      cur_cn = p_corn[cur_cn + mc] * 18;
      cur_e0 = p_edge[cur_e0 * 18 + mc];
      cur_e2 = p_edge[cur_e2 * 18 + mc];
      cur_e4 = p_edge[cur_e4 * 18 + mc];
      cur_e6 = p_edge[cur_e6 * 18 + mc];
      cur_c5 = p_corn[cur_c5 * 18 + mc];
      cur_c6 = p_corn[cur_c6 * 18 + mc];
      cur_c7 = p_corn[cur_c7 * 18 + mc];
      cur_e6_n = p_edge6[cur_e6_n + mc] * 18;
      cur_c2_n = p_corn2[cur_c2_n + mc] * 18;
      cur_e6_d = p_edge6[cur_e6_d + mc] * 18;
      cur_c2_d = p_corn2[cur_c2_d + mc] * 18;
    }
    i_mul = cur_mul;
    i_cn = cur_cn / 18;
    i_e0 = cur_e0;
    i_e2 = cur_e2;
    i_e4 = cur_e4;
    i_e6 = cur_e6;
    i_c5 = cur_c5;
    i_c6 = cur_c6;
    i_c7 = cur_c7;
    i_e6_idx_nb = cur_e6_n / 18;
    i_c2_idx_nb = cur_c2_n / 18;
    i_e6_idx_dg = cur_e6_d / 18;
    i_c2_idx_dg = cur_c2_d / 18;
  }

  bool search_1(SearchContext &ctx, int i1, int i2, int i3, int s1, int depth,
                int prev) {
    const int *moves = valid_moves_flat[prev];
    const int count = valid_moves_count[prev];
    for (int k = 0; k < count; ++k) {
      int m = moves[k];
      COUNT_NODE
      int m1 = conj_moves_flat[m][s1];
      int n_i1 = p_multi[i1 + m1];
      int n_i2 = p_corn[i2 + m1];
      int n_i3 = p_edge[i3 + m1];
      long long idx = (long long)(n_i1 + n_i2) * 24 + n_i3;
      if (get_prune_ptr(p_prune_base, idx) >= depth)
        continue;
      if (depth == 1) {
        ctx.sol_len.push_back(ctx.current_max_depth);
        return true;
      }
      if (search_1(ctx, n_i1, n_i2 * 18, n_i3 * 18, s1, depth - 1, m))
        return true;
    }
    return false;
  }

  bool search_2_optimized(SearchContext &ctx, int i1a, int i2a, int i3a,
                          int i4a, int i5a, int s1, int i1b, int i2b, int i3b,
                          int i4b, int i5b, int s2, int t_idx1, int t_idx2,
                          int i_e6, int i_c2, int v_adj,
                          const unsigned char *p_prune_active, int depth,
                          int prev) {
    const int *moves = valid_moves_flat[prev];
    const int count = valid_moves_count[prev];
    for (int k = 0; k < count; ++k) {
      int m = moves[k];
      COUNT_NODE
      if (v_adj != -1 && p_prune_active) {
        int mv = conj_moves_flat[m][v_adj];
        int n_ie6 = p_edge6[i_e6 * 18 + mv];
        int n_ic2 = p_corn2[i_c2 * 18 + mv];
        if (get_prune_ptr(p_prune_active, (long long)n_ie6 * 504 + n_ic2) >=
            depth)
          continue;
      }
      int m1 = conj_moves_flat[m][s1];
      int n_i1a = p_multi[i1a + m1];
      int n_i2a = p_corn[i2a + m1];
      int n_i3a = p_edge[i3a + m1];
      int n_i4a = p_edge[i4a + m1];
      int n_i5a = p_corn[i5a + m1];
      int m2 = conj_moves_flat[m][s2];
      int n_i1b = p_multi[i1b + m2];
      int n_i2b = p_corn[i2b + m2];
      int n_i3b = p_edge[i3b + m2];
      int n_i4b = p_edge[i4b + m2];
      int n_i5b = p_corn[i5b + m2];
      if (depth == 1) {
        ctx.sol_len.push_back(ctx.current_max_depth);
        return true;
      }
      if (search_2_optimized(
              ctx, n_i1a, n_i2a * 18, n_i3a * 18, n_i4a * 18, n_i5a * 18, s1,
              n_i1b, n_i2b * 18, n_i3b * 18, n_i4b * 18, n_i5b * 18, s2, t_idx1,
              t_idx2,
              (v_adj != -1) ? p_edge6[i_e6 * 18 + conj_moves_flat[m][v_adj]]
                            : -1,
              (v_adj != -1) ? p_corn2[i_c2 * 18 + conj_moves_flat[m][v_adj]]
                            : -1,
              v_adj, p_prune_active, depth - 1, m))
        return true;
    }
    return false;
  }

  bool search_3_optimized(
      SearchContext &ctx, int i1a, int i2a, int i3a, int i4a_1, int i5a_1,
      int i6a, int i4a_2, int i5a_2, int s1, int i1b, int i2b, int i3b,
      int i4b_1, int i5b_1, int i6b, int i4b_2, int i5b_2, int s2, int i1c,
      int i2c, int i3c, int i4c_1, int i5c_1, int i6c, int i4c_2, int i5c_2,
      int s3, int t12, int t21, int t23, int t32, int i_e6_12, int i_c2_12,
      int v12, const unsigned char *p_table_12, int i_e6_23, int i_c2_23,
      int v23, const unsigned char *p_table_23, int i_e6_31, int i_c2_31,
      int v31, const unsigned char *p_table_31, int depth, int prev) {
    const int *moves = valid_moves_flat[prev];
    const int count = valid_moves_count[prev];
    for (int k = 0; k < count; ++k) {
      int m = moves[k];
      COUNT_NODE
      if (v12 != -1 && p_table_12) {
        int mx = conj_moves_flat[m][v12];
        if (get_prune_ptr(p_table_12,
                          (long long)p_edge6[i_e6_12 * 18 + mx] * 504 +
                              p_corn2[i_c2_12 * 18 + mx]) >= depth)
          continue;
      }
      if (v23 != -1 && p_table_23) {
        int mx = conj_moves_flat[m][v23];
        if (get_prune_ptr(p_table_23,
                          (long long)p_edge6[i_e6_23 * 18 + mx] * 504 +
                              p_corn2[i_c2_23 * 18 + mx]) >= depth)
          continue;
      }
      if (v31 != -1 && p_table_31) {
        int mx = conj_moves_flat[m][v31];
        if (get_prune_ptr(p_table_31,
                          (long long)p_edge6[i_e6_31 * 18 + mx] * 504 +
                              p_corn2[i_c2_31 * 18 + mx]) >= depth)
          continue;
      }

      int m1 = conj_moves_flat[m][s1];
      int n_i1a = p_multi[i1a + m1];
      int n_i2a = p_corn[i2a + m1];
      int n_i3a = p_edge[i3a + m1];
      int n_i5a_1 = p_corn[i5a_1 + m1];
      int n_i6a = p_corn[i6a + m1];
      int n_i4a_1 = p_edge[i4a_1 + m1];
      int n_i4a_2 = p_edge[i4a_2 + m1];
      int n_i5a_2 = p_corn[i5a_2 + m1];
      int m2 = conj_moves_flat[m][s2];
      int n_i1b = p_multi[i1b + m2];
      int n_i2b = p_corn[i2b + m2];
      int n_i3b = p_edge[i3b + m2];
      int n_i6b = p_corn[i6b + m2];
      int n_i4b_1 = p_edge[i4b_1 + m2];
      int n_i5b_1 = p_corn[i5b_1 + m2];
      int n_i4b_2 = p_edge[i4b_2 + m2];
      int n_i5b_2 = p_corn[i5b_2 + m2];
      int m3 = conj_moves_flat[m][s3];
      int n_i1c = p_multi[i1c + m3];
      int n_i2c = p_corn[i2c + m3];
      int n_i3c = p_edge[i3c + m3];
      int n_i6c = p_corn[i6c + m3];
      int n_i4c_1 = p_edge[i4c_1 + m3];
      int n_i5c_1 = p_corn[i5c_1 + m3];
      int n_i4c_2 = p_edge[i4c_2 + m3];
      int n_i5c_2 = p_corn[i5c_2 + m3];

      if (depth == 1) {
        ctx.sol_len.push_back(ctx.current_max_depth);
        return true;
      }
      if (search_3_optimized(
              ctx, n_i1a, n_i2a * 18, n_i3a * 18, n_i4a_1 * 18, n_i5a_1 * 18,
              n_i6a * 18, n_i4a_2 * 18, n_i5a_2 * 18, s1, n_i1b, n_i2b * 18,
              n_i3b * 18, n_i4b_1 * 18, n_i5b_1 * 18, n_i6b * 18, n_i4b_2 * 18,
              n_i5b_2 * 18, s2, n_i1c, n_i2c * 18, n_i3c * 18, n_i4c_1 * 18,
              n_i5c_1 * 18, n_i6c * 18, n_i4c_2 * 18, n_i5c_2 * 18, s3, t12,
              t21, t23, t32,
              (v12 != -1) ? p_edge6[i_e6_12 * 18 + conj_moves_flat[m][v12]]
                          : -1,
              (v12 != -1) ? p_corn2[i_c2_12 * 18 + conj_moves_flat[m][v12]]
                          : -1,
              v12, p_table_12,
              (v23 != -1) ? p_edge6[i_e6_23 * 18 + conj_moves_flat[m][v23]]
                          : -1,
              (v23 != -1) ? p_corn2[i_c2_23 * 18 + conj_moves_flat[m][v23]]
                          : -1,
              v23, p_table_23,
              (v31 != -1) ? p_edge6[i_e6_31 * 18 + conj_moves_flat[m][v31]]
                          : -1,
              (v31 != -1) ? p_corn2[i_c2_31 * 18 + conj_moves_flat[m][v31]]
                          : -1,
              v31, p_table_31, depth - 1, m))
        return true;
    }
    return false;
  }

  bool search_4_optimized(SearchContext &ctx, int i1a, int i2a, int i3a,
                          int i4a, int i5a, int i6a, int i1b, int i2b, int i3b,
                          int i4b, int i5b, int i6b, int i1c, int i2c, int i3c,
                          int i4c, int i5c, int i6c, int i1d, int i2d, int i3d,
                          int i4d, int i5d, int i6d, int nb01_e, int nb01_c,
                          int nb12_e, int nb12_c, int nb23_e, int nb23_c,
                          int nb30_e, int nb30_c, int dg02_e, int dg02_c,
                          int dg13_e, int dg13_c, int depth, int prev) {
    const int *moves = valid_moves_flat[prev];
    const int count = valid_moves_count[prev];
    for (int k = 0; k < count; ++k) {
      int m = moves[k];
      COUNT_NODE

      int m0 = conj_moves_flat[m][0];
      int n_nb01_e = p_edge6[nb01_e * 18 + m0];
      int n_nb01_c = p_corn2[nb01_c * 18 + m0];
      if (get_prune_ptr(p_prune_neighbor,
                        (long long)n_nb01_e * 504 + n_nb01_c) >= depth)
        continue;
      int m1 = conj_moves_flat[m][1];
      int n_nb12_e = p_edge6[nb12_e * 18 + m1];
      int n_nb12_c = p_corn2[nb12_c * 18 + m1];
      if (get_prune_ptr(p_prune_neighbor,
                        (long long)n_nb12_e * 504 + n_nb12_c) >= depth)
        continue;
      int m2 = conj_moves_flat[m][2];
      int n_nb23_e = p_edge6[nb23_e * 18 + m2];
      int n_nb23_c = p_corn2[nb23_c * 18 + m2];
      if (get_prune_ptr(p_prune_neighbor,
                        (long long)n_nb23_e * 504 + n_nb23_c) >= depth)
        continue;
      int m3 = conj_moves_flat[m][3];
      int n_nb30_e = p_edge6[nb30_e * 18 + m3];
      int n_nb30_c = p_corn2[nb30_c * 18 + m3];
      if (get_prune_ptr(p_prune_neighbor,
                        (long long)n_nb30_e * 504 + n_nb30_c) >= depth)
        continue;

      int n_dg02_e = p_edge6[dg02_e * 18 + m0];
      int n_dg02_c = p_corn2[dg02_c * 18 + m0];
      if (p_prune_diagonal) {
        if (get_prune_ptr(p_prune_diagonal,
                          (long long)n_dg02_e * 504 + n_dg02_c) >= depth)
          continue;
      }
      int n_dg13_e = p_edge6[dg13_e * 18 + m1];
      int n_dg13_c = p_corn2[dg13_c * 18 + m1];
      if (p_prune_diagonal) {
        if (get_prune_ptr(p_prune_diagonal,
                          (long long)n_dg13_e * 504 + n_dg13_c) >= depth)
          continue;
      }

      int n_i1a = p_multi[i1a + m0];
      int n_i2a = p_corn[i2a + m0];
      int n_i3a = p_edge[i3a + m0];
      int n_i4a = p_edge[i4a + m0];
      int n_i5a = p_corn[i5a + m0];
      int n_i6a = p_corn[i6a + m0];
      int n_i1b = p_multi[i1b + m1];
      int n_i2b = p_corn[i2b + m1];
      int n_i3b = p_edge[i3b + m1];
      int n_i4b = p_edge[i4b + m1];
      int n_i5b = p_corn[i5b + m1];
      int n_i6b = p_corn[i6b + m1];
      int n_i1c = p_multi[i1c + m2];
      int n_i2c = p_corn[i2c + m2];
      int n_i3c = p_edge[i3c + m2];
      int n_i4c = p_edge[i4c + m2];
      int n_i5c = p_corn[i5c + m2];
      int n_i6c = p_corn[i6c + m2];
      int n_i1d = p_multi[i1d + m3];
      int n_i2d = p_corn[i2d + m3];
      int n_i3d = p_edge[i3d + m3];
      int n_i4d = p_edge[i4d + m3];
      int n_i5d = p_corn[i5d + m3];
      int n_i6d = p_corn[i6d + m3];

      if (depth == 1) {
        ctx.sol_len.push_back(ctx.current_max_depth);
        return true;
      }
      if (search_4_optimized(
              ctx, n_i1a, n_i2a * 18, n_i3a * 18, n_i4a * 18, n_i5a * 18,
              n_i6a * 18, n_i1b, n_i2b * 18, n_i3b * 18, n_i4b * 18, n_i5b * 18,
              n_i6b * 18, n_i1c, n_i2c * 18, n_i3c * 18, n_i4c * 18, n_i5c * 18,
              n_i6c * 18, n_i1d, n_i2d * 18, n_i3d * 18, n_i4d * 18, n_i5d * 18,
              n_i6d * 18, n_nb01_e, n_nb01_c, n_nb12_e, n_nb12_c, n_nb23_e,
              n_nb23_c, n_nb30_e, n_nb30_c, n_dg02_e, n_dg02_c, n_dg13_e,
              n_dg13_c, depth - 1, m))
        return true;
    }
    return false;
  }

  std::vector<int> get_stats(const std::vector<int> &base_alg,
                             const std::vector<std::string> &rots) {
    std::vector<int> all_results;
    all_results.reserve(48);

    { // 1. XC
      std::vector<int> stage_min(rots.size(), 99);
      for (size_t r = 0; r < rots.size(); ++r) {
        std::vector<int> alg = alg_rotation(base_alg, rots[r]);
        struct SlotState {
          int im;
          int ic;
          int ie;
          int nb_e;
          int nb_c;
          int dg_e;
          int dg_c;
        };
        std::vector<SlotState> initial_states(4);
        std::vector<Task1> tasks;
        int d1, d2, d3, c5, c6, c7;
        for (int k = 0; k < 4; ++k) {
          get_conjugated_indices_all(
              alg, k, initial_states[k].im, initial_states[k].ic,
              initial_states[k].ie, d1, d2, d3, c5, c6, c7,
              initial_states[k].nb_e, initial_states[k].nb_c,
              initial_states[k].dg_e, initial_states[k].dg_c);
          long long idx =
              (long long)(initial_states[k].im + initial_states[k].ic) * 24 +
              initial_states[k].ie;
          int h = get_prune_ptr(p_prune_base, idx);
          tasks.push_back({k, h});
        }
        std::sort(tasks.begin(), tasks.end(),
                  [](const Task1 &a, const Task1 &b) { return a.h < b.h; });
        int current_best = 99;
        for (auto &t : tasks) {
          if (t.h >= current_best)
            break;
          int res = 99;
          if (t.h > 0) {
            SearchContext ctx;
            int max_search = std::min(12, current_best - 1);
            for (int d = t.h; d <= max_search; ++d) {
              ctx.current_max_depth = d;
              if (search_1(ctx, initial_states[t.id].im,
                           initial_states[t.id].ic * 18,
                           initial_states[t.id].ie * 18, t.id, d, 18)) {
                res = d;
                break;
              }
            }
          } else {
            res = 0;
          }
          if (res < current_best)
            current_best = res;
        }
        stage_min[r] = current_best;
      }
      all_results.insert(all_results.end(), stage_min.begin(), stage_min.end());
    }

    { // 2. XX-Cross
      std::vector<int> stage_min(rots.size(), 99);
      for (size_t r = 0; r < rots.size(); ++r) {
        std::vector<int> alg = alg_rotation(base_alg, rots[r]);
        struct FullSlot {
          int im;
          int ic;
          int e0;
          int e2;
          int e4;
          int e6;
          int c5;
          int c6;
          int c7;
          int ie6_nb;
          int ic2_nb;
          int ie6_dg;
          int ic2_dg;
        };
        std::vector<FullSlot> st(4);
        for (int k = 0; k < 4; ++k)
          get_conjugated_indices_all(alg, k, st[k].im, st[k].ic, st[k].e0,
                                     st[k].e2, st[k].e4, st[k].e6, st[k].c5,
                                     st[k].c6, st[k].c7, st[k].ie6_nb,
                                     st[k].ic2_nb, st[k].ie6_dg, st[k].ic2_dg);
        std::vector<Task2> tasks;
        struct P2 {
          int a;
          int b;
        };
        std::vector<P2> pairs = {{0, 1}, {0, 2}, {0, 3},
                                 {1, 2}, {1, 3}, {2, 3}};
        for (auto &p : pairs) {
          int v_nb = get_neighbor_view(p.a, p.b);
          int v_dg = get_diagonal_view(p.a, p.b);
          int h_val = 0;
          int view_used = -1;
          if (v_nb != -1) {
            h_val = get_prune_ptr(p_prune_neighbor,
                                  (long long)st[v_nb].ie6_nb * 504 +
                                      st[v_nb].ic2_nb);
            view_used = v_nb;
          } else if (v_dg != -1 && p_prune_diagonal) {
            h_val = get_prune_ptr(p_prune_diagonal,
                                  (long long)st[v_dg].ie6_dg * 504 +
                                      st[v_dg].ic2_dg);
            view_used = v_dg;
          }
          tasks.push_back({p.a, p.b, h_val, view_used});
        }
        std::sort(tasks.begin(), tasks.end(),
                  [](const Task2 &a, const Task2 &b) { return a.h < b.h; });
        int current_best = 99;
        for (auto &t : tasks) {
          if (t.h >= current_best)
            break;
          int res = 99;
          if (t.h > 0) {
            SearchContext ctx;
            int max_search = std::min(14, current_best - 1);
            int t1 = get_plus_table_idx(t.a, t.b);
            int t2 = get_plus_table_idx(t.b, t.a);
            int ea = get_correct_edge_start(t1, st[t.a].e0, st[t.a].e2,
                                            st[t.a].e4, st[t.a].e6);
            int ca =
                get_correct_corn_start(t1, st[t.a].c5, st[t.a].c6, st[t.a].c7);
            int eb = get_correct_edge_start(t2, st[t.b].e0, st[t.b].e2,
                                            st[t.b].e4, st[t.b].e6);
            int cb =
                get_correct_corn_start(t2, st[t.b].c5, st[t.b].c6, st[t.b].c7);
            int ie6_use = -1, ic2_use = -1;
            const unsigned char *p_table_use = nullptr;
            if (get_neighbor_view(t.a, t.b) != -1) {
              ie6_use = st[t.v].ie6_nb;
              ic2_use = st[t.v].ic2_nb;
              p_table_use = p_prune_neighbor;
            } else if (p_prune_diagonal) {
              ie6_use = st[t.v].ie6_dg;
              ic2_use = st[t.v].ic2_dg;
              p_table_use = p_prune_diagonal;
            }
            if (p_table_use) {
              for (int d = t.h; d <= max_search; ++d) {
                ctx.current_max_depth = d;
                if (search_2_optimized(
                        ctx, st[t.a].im, st[t.a].ic * 18, st[t.a].e0 * 18,
                        ea * 18, ca * 18, t.a, st[t.b].im, st[t.b].ic * 18,
                        st[t.b].e0 * 18, eb * 18, cb * 18, t.b, t1, t2, ie6_use,
                        ic2_use, t.v, p_table_use, d, 18)) {
                  res = d;
                  break;
                }
              }
            }
          } else {
            res = 0;
          }
          if (res < current_best)
            current_best = res;
        }
        stage_min[r] = current_best;
      }
      all_results.insert(all_results.end(), stage_min.begin(), stage_min.end());
    }

    { // 3. XXX-Cross
      std::vector<int> stage_min(rots.size(), 99);
      for (size_t r = 0; r < rots.size(); ++r) {
        std::vector<int> alg = alg_rotation(base_alg, rots[r]);
        struct FullSlot {
          int im;
          int ic;
          int e0;
          int e2;
          int e4;
          int e6;
          int c5;
          int c6;
          int c7;
          int ie6_nb;
          int ic2_nb;
          int ie6_dg;
          int ic2_dg;
        };
        std::vector<FullSlot> st(4);
        for (int k = 0; k < 4; ++k)
          get_conjugated_indices_all(alg, k, st[k].im, st[k].ic, st[k].e0,
                                     st[k].e2, st[k].e4, st[k].e6, st[k].c5,
                                     st[k].c6, st[k].c7, st[k].ie6_nb,
                                     st[k].ic2_nb, st[k].ie6_dg, st[k].ic2_dg);
        std::vector<Task3> tasks;
        struct P3 {
          int a;
          int b;
          int c;
        };
        std::vector<P3> trips = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
        for (auto &t : trips) {
          int h_max = 0;
          int v1 = get_neighbor_view(t.a, t.b);
          int d1 = 0;
          if (v1 != -1)
            d1 = get_prune_ptr(p_prune_neighbor,
                               (long long)st[v1].ie6_nb * 504 + st[v1].ic2_nb);
          else if (p_prune_diagonal) {
            v1 = get_diagonal_view(t.a, t.b);
            d1 = get_prune_ptr(p_prune_diagonal,
                               (long long)st[v1].ie6_dg * 504 + st[v1].ic2_dg);
          }
          int v2 = get_neighbor_view(t.b, t.c);
          int d2 = 0;
          if (v2 != -1)
            d2 = get_prune_ptr(p_prune_neighbor,
                               (long long)st[v2].ie6_nb * 504 + st[v2].ic2_nb);
          else if (p_prune_diagonal) {
            v2 = get_diagonal_view(t.b, t.c);
            d2 = get_prune_ptr(p_prune_diagonal,
                               (long long)st[v2].ie6_dg * 504 + st[v2].ic2_dg);
          }
          int v3 = get_neighbor_view(t.c, t.a);
          int d3 = 0;
          if (v3 != -1)
            d3 = get_prune_ptr(p_prune_neighbor,
                               (long long)st[v3].ie6_nb * 504 + st[v3].ic2_nb);
          else if (p_prune_diagonal) {
            v3 = get_diagonal_view(t.c, t.a);
            d3 = get_prune_ptr(p_prune_diagonal,
                               (long long)st[v3].ie6_dg * 504 + st[v3].ic2_dg);
          }
          h_max = std::max({d1, d2, d3});
          tasks.push_back({t.a, t.b, t.c, h_max, v1, v2, v3});
        }
        std::sort(tasks.begin(), tasks.end(),
                  [](const Task3 &a, const Task3 &b) { return a.h < b.h; });
        int current_best = 99;
        for (auto &t : tasks) {
          if (t.h >= current_best)
            break;
          int res = 99;
          if (t.h > 0) {
            SearchContext ctx;
            int max_search = std::min(16, current_best - 1);
            int t12 = get_plus_table_idx(t.a, t.b);
            int t21 = get_plus_table_idx(t.b, t.a);
            int t23 = get_plus_table_idx(t.b, t.c);
            int t32 = get_plus_table_idx(t.c, t.b);
            int t31 = get_plus_table_idx(t.c, t.a);
            int t13 = get_plus_table_idx(t.a, t.c);
            int ea_b = get_correct_edge_start(t12, st[t.a].e0, st[t.a].e2,
                                              st[t.a].e4, st[t.a].e6);
            int ca_b =
                get_correct_corn_start(t12, st[t.a].c5, st[t.a].c6, st[t.a].c7);
            int ea_c = get_correct_edge_start(t13, st[t.a].e0, st[t.a].e2,
                                              st[t.a].e4, st[t.a].e6);
            int ca_c =
                get_correct_corn_start(t13, st[t.a].c5, st[t.a].c6, st[t.a].c7);
            int eb_a = get_correct_edge_start(t21, st[t.b].e0, st[t.b].e2,
                                              st[t.b].e4, st[t.b].e6);
            int cb_a =
                get_correct_corn_start(t21, st[t.b].c5, st[t.b].c6, st[t.b].c7);
            int eb_c = get_correct_edge_start(t23, st[t.b].e0, st[t.b].e2,
                                              st[t.b].e4, st[t.b].e6);
            int cb_c =
                get_correct_corn_start(t23, st[t.b].c5, st[t.b].c6, st[t.b].c7);
            int ec_b = get_correct_edge_start(t32, st[t.c].e0, st[t.c].e2,
                                              st[t.c].e4, st[t.c].e6);
            int cc_b =
                get_correct_corn_start(t32, st[t.c].c5, st[t.c].c6, st[t.c].c7);
            int ec_a = get_correct_edge_start(t31, st[t.c].e0, st[t.c].e2,
                                              st[t.c].e4, st[t.c].e6);
            int cc_a =
                get_correct_corn_start(t31, st[t.c].c5, st[t.c].c6, st[t.c].c7);

            int i_e6_1 = -1, i_c2_1 = -1;
            const unsigned char *p_1 = nullptr;
            if (get_neighbor_view(t.a, t.b) != -1) {
              i_e6_1 = st[t.v12].ie6_nb;
              i_c2_1 = st[t.v12].ic2_nb;
              p_1 = p_prune_neighbor;
            } else if (p_prune_diagonal) {
              i_e6_1 = st[t.v12].ie6_dg;
              i_c2_1 = st[t.v12].ic2_dg;
              p_1 = p_prune_diagonal;
            }
            int i_e6_2 = -1, i_c2_2 = -1;
            const unsigned char *p_2 = nullptr;
            if (get_neighbor_view(t.b, t.c) != -1) {
              i_e6_2 = st[t.v23].ie6_nb;
              i_c2_2 = st[t.v23].ic2_nb;
              p_2 = p_prune_neighbor;
            } else if (p_prune_diagonal) {
              i_e6_2 = st[t.v23].ie6_dg;
              i_c2_2 = st[t.v23].ic2_dg;
              p_2 = p_prune_diagonal;
            }
            int i_e6_3 = -1, i_c2_3 = -1;
            const unsigned char *p_3 = nullptr;
            if (get_neighbor_view(t.c, t.a) != -1) {
              i_e6_3 = st[t.v31].ie6_nb;
              i_c2_3 = st[t.v31].ic2_nb;
              p_3 = p_prune_neighbor;
            } else if (p_prune_diagonal) {
              i_e6_3 = st[t.v31].ie6_dg;
              i_c2_3 = st[t.v31].ic2_dg;
              p_3 = p_prune_diagonal;
            }

            for (int d = t.h; d <= max_search; ++d) {
              ctx.current_max_depth = d;
              if (search_3_optimized(
                      ctx, st[t.a].im, st[t.a].ic * 18, st[t.a].e0 * 18,
                      ea_b * 18, ca_b * 18, st[t.a].c6 * 18, ea_c * 18,
                      ca_c * 18, t.a, st[t.b].im, st[t.b].ic * 18,
                      st[t.b].e0 * 18, eb_a * 18, cb_a * 18, st[t.b].c6 * 18,
                      eb_c * 18, cb_c * 18, t.b, st[t.c].im, st[t.c].ic * 18,
                      st[t.c].e0 * 18, ec_b * 18, cc_b * 18, st[t.c].c6 * 18,
                      ec_a * 18, cc_a * 18, t.c, t12, t21, t23, t32, i_e6_1,
                      i_c2_1, t.v12, p_1, i_e6_2, i_c2_2, t.v23, p_2, i_e6_3,
                      i_c2_3, t.v31, p_3, d, 18)) {
                res = d;
                break;
              }
            }
          } else {
            res = 0;
          }
          if (res < current_best)
            current_best = res;
        }
        stage_min[r] = current_best;
      }
      all_results.insert(all_results.end(), stage_min.begin(), stage_min.end());
    }

    { // 4. F2L
      std::vector<int> stage_res(rots.size());
      for (size_t r = 0; r < rots.size(); ++r) {
        std::vector<int> alg = alg_rotation(base_alg, rots[r]);
        struct FullSlot {
          int im;
          int ic;
          int e0;
          int e2;
          int e4;
          int e6;
          int c5;
          int c6;
          int c7;
          int ie6_nb;
          int ic2_nb;
          int ie6_dg;
          int ic2_dg;
        };
        std::vector<FullSlot> st(4);
        for (int k = 0; k < 4; ++k)
          get_conjugated_indices_all(alg, k, st[k].im, st[k].ic, st[k].e0,
                                     st[k].e2, st[k].e4, st[k].e6, st[k].c5,
                                     st[k].c6, st[k].c7, st[k].ie6_nb,
                                     st[k].ic2_nb, st[k].ie6_dg, st[k].ic2_dg);
        int d_nb0 = get_prune_ptr(p_prune_neighbor,
                                  (long long)st[0].ie6_nb * 504 + st[0].ic2_nb);
        int d_nb1 = get_prune_ptr(p_prune_neighbor,
                                  (long long)st[1].ie6_nb * 504 + st[1].ic2_nb);
        int d_nb2 = get_prune_ptr(p_prune_neighbor,
                                  (long long)st[2].ie6_nb * 504 + st[2].ic2_nb);
        int d_nb3 = get_prune_ptr(p_prune_neighbor,
                                  (long long)st[3].ie6_nb * 504 + st[3].ic2_nb);
        int d_dg0 =
            p_prune_diagonal
                ? get_prune_ptr(p_prune_diagonal,
                                (long long)st[0].ie6_dg * 504 + st[0].ic2_dg)
                : 0;
        int d_dg1 =
            p_prune_diagonal
                ? get_prune_ptr(p_prune_diagonal,
                                (long long)st[1].ie6_dg * 504 + st[1].ic2_dg)
                : 0;
        int max_h = std::max({d_nb0, d_nb1, d_nb2, d_nb3, d_dg0, d_dg1});

        int res = 0;
        if (max_h <= 16) {
          if (max_h > 0) {
            SearchContext ctx;
            for (int d = max_h; d <= 16; ++d) {
              ctx.current_max_depth = d;
              if (search_4_optimized(
                      ctx, st[0].im, st[0].ic * 18, st[0].e0 * 18,
                      st[0].e2 * 18, st[0].c5 * 18, st[0].c6 * 18, st[1].im,
                      st[1].ic * 18, st[1].e0 * 18, st[1].e2 * 18,
                      st[1].c5 * 18, st[1].c6 * 18, st[2].im, st[2].ic * 18,
                      st[2].e0 * 18, st[2].e2 * 18, st[2].c5 * 18,
                      st[2].c6 * 18, st[3].im, st[3].ic * 18, st[3].e0 * 18,
                      st[3].e2 * 18, st[3].c5 * 18, st[3].c6 * 18, st[0].ie6_nb,
                      st[0].ic2_nb, st[1].ie6_nb, st[1].ic2_nb, st[2].ie6_nb,
                      st[2].ic2_nb, st[3].ie6_nb, st[3].ic2_nb, st[0].ie6_dg,
                      st[0].ic2_dg, st[1].ie6_dg, st[1].ic2_dg, d, 18)) {
                res = d;
                break;
              }
            }
          }
        } else {
          res = 17;
        }
        stage_res[r] = res;
      }
      all_results.insert(all_results.end(), stage_res.begin(), stage_res.end());
    }
    return all_results;
  }
};

// --- StdSolver: 封装 CrossSolver 和 XCrossSolver 的统一接口 ---
// NOTE: 此类满足 analyzer_executor.h 中 run_analyzer_app 模板的要求
struct StdSolver {
  // 旋转列表：对应 _z0, _z1, _z2, _z3, _x1, _x3 后缀
  static inline std::vector<std::string> rots = {"",  "z2", "z'",
                                                 "z", "x'", "x"};

  CrossSolver crossSolver;
  XCrossSolver xcrossSolver;

  // 全局初始化：加载移动表和剪枝表
  static void global_init() {
    printCuberootLogo();
    init_matrix();

    auto &mtm = MoveTableManager::getInstance();
    auto &ptm = PruneTableManager::getInstance();

    if (!mtm.loadAll()) {
      std::cerr << ANSI_RED
                << "[ERROR] Move tables missing. Please run "
                   "table_generator.exe first."
                << ANSI_RESET << std::endl;
      exit(1);
    }

    if (!ptm.loadAll()) {
      std::cerr << ANSI_RED
                << "[ERROR] Prune tables missing. Please run "
                   "table_generator.exe first."
                << ANSI_RESET << std::endl;
      exit(1);
    }
  }

  // CSV 表头：使用 snake_case 和对称旋转后缀
  // NOTE: 格式为 id,cross_z0,cross_z1,...,xcross_z0,...,f2l_x3
  static std::string get_csv_header() {
    std::vector<std::string> suffixes = {"_z0", "_z1", "_z2",
                                         "_z3", "_x1", "_x3"};
    std::ostringstream oss;
    oss << "id";
    for (const auto &s : suffixes)
      oss << ",cross" << s;
    for (const auto &s : suffixes)
      oss << ",xcross" << s;
    for (const auto &s : suffixes)
      oss << ",xxcross" << s;
    for (const auto &s : suffixes)
      oss << ",xxxcross" << s;
    for (const auto &s : suffixes)
      oss << ",xxxxcross" << s;
    return oss.str();
  }

  // 求解单个任务
  // NOTE: 返回不含换行的 CSV 行
  std::string solve(const std::vector<int> &alg, const std::string &id) {
    std::vector<int> crossStats = crossSolver.get_stats(alg, rots);
    std::vector<int> xcrossStats = xcrossSolver.get_stats(alg, rots);

    std::ostringstream oss;
    oss << id;
    for (int v : crossStats)
      oss << "," << v;
    for (int v : xcrossStats)
      oss << "," << v;
    return oss.str();
  }

  // 可选：最终统计
  static void print_stats() {}
};

int main() {
  run_analyzer_app<StdSolver>("_std");
  return 0;
}
