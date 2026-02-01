/*
 * pair_analyzer.cpp - 清理后的Pair分析器
 */

#include "analyzer_executor.h"
#include "cube_common.h"
#include "move_tables.h"
#include "prune_tables.h"

// --- 全局统计变量重定向到 AnalyzerStats ---
#define global_nodes AnalyzerStats::globalNodes
#define completed_tasks AnalyzerStats::completedTasks
#define is_solving AnalyzerStats::isSolving

// NOTE: COUNT_NODE 宏已移至 analyzer_executor.h

// --- 主要求解器类 ---
struct PairSolver {
  // 快速指针
  const int *p_multi, *p_corn, *p_edge;
  const int *p_edge6, *p_corn2;
  const unsigned char *p_cross, *p_pair, *p_xcross;
  const unsigned char *p_prune_neighbor, *p_prune_diagonal;

  // 常量定义 (以 Slot 0 为基准)
  const int IDX_MULTI_BASE = 187520; // Cross 已还原
  const int IDX_C4 = 12;             // 基准角块 (DBL)
  const int IDX_E0 = 0;              // 基准棱块 (BL)

  // Huge Table Solved Indices
  int IDX_SOLVED_E6_NB;
  int IDX_SOLVED_E6_DG;
  int IDX_SOLVED_C2_NB;
  int IDX_SOLVED_C2_DG;

  struct Task1 {
    int s1;
    int h;
  };
  struct Task2 {
    int s1, s2;
    int h;
  };
  struct Task3 {
    int s1, s2, s3;
    int h;
  };
  struct Task4 {
    int s1, s2, s3, s4;
    int h;
  };

  // 静态初始化标志和指针
  static inline bool s_initialized = false;
  static inline const int *s_p_multi = nullptr;
  static inline const int *s_p_corn = nullptr;
  static inline const int *s_p_edge = nullptr;
  static inline const int *s_p_edge6 = nullptr;
  static inline const int *s_p_corn2 = nullptr;
  static inline const unsigned char *s_p_cross = nullptr;
  static inline const unsigned char *s_p_pair = nullptr;
  static inline const unsigned char *s_p_xcross = nullptr;
  static inline const unsigned char *s_p_prune_neighbor = nullptr;
  static inline const unsigned char *s_p_prune_diagonal = nullptr;
  static inline int s_IDX_SOLVED_E6_NB = 0;
  static inline int s_IDX_SOLVED_E6_DG = 0;
  static inline int s_IDX_SOLVED_C2_NB = 0;
  static inline int s_IDX_SOLVED_C2_DG = 0;

  static void static_init() {
    if (s_initialized)
      return;

    auto &mtm = MoveTableManager::getInstance();
    auto &ptm = PruneTableManager::getInstance();

    // 获取移动表指针
    s_p_multi = mtm.getCrossTablePtr();
    s_p_corn = mtm.getCornerTablePtr();
    s_p_edge = mtm.getEdgeTablePtr();
    s_p_edge6 = mtm.getEdge6TablePtr();
    s_p_corn2 = mtm.getCorner2TablePtr();

    // 获取剪枝表指针
    s_p_cross = ptm.getCrossC4PrunePtr();
    s_p_pair = ptm.getPairC4E0PrunePtr();
    s_p_xcross = ptm.getXCrossC4E0PrunePtr();
    s_p_prune_neighbor = ptm.getHugeNeighborPrunePtr();
    s_p_prune_diagonal = ptm.getHugeDiagonalPrunePtr();

    // 计算解决状态索引
    s_IDX_SOLVED_E6_NB = array_to_index({0, 2, 16, 18, 20, 22}, 6, 2, 12);
    s_IDX_SOLVED_E6_DG = array_to_index({0, 4, 16, 18, 20, 22}, 6, 2, 12);
    s_IDX_SOLVED_C2_NB = array_to_index({12, 15}, 2, 3, 8);
    s_IDX_SOLVED_C2_DG = array_to_index({12, 18}, 2, 3, 8);

    s_initialized = true;
  }

  PairSolver() {
    // 仅复制指针引用
    p_multi = s_p_multi;
    p_corn = s_p_corn;
    p_edge = s_p_edge;
    p_edge6 = s_p_edge6;
    p_corn2 = s_p_corn2;
    p_cross = s_p_cross;
    p_pair = s_p_pair;
    p_xcross = s_p_xcross;
    p_prune_neighbor = s_p_prune_neighbor;
    p_prune_diagonal = s_p_prune_diagonal;
    IDX_SOLVED_E6_NB = s_IDX_SOLVED_E6_NB;
    IDX_SOLVED_E6_DG = s_IDX_SOLVED_E6_DG;
    IDX_SOLVED_C2_NB = s_IDX_SOLVED_C2_NB;
    IDX_SOLVED_C2_DG = s_IDX_SOLVED_C2_DG;
  }

  // 虚拟状态：用于追踪共轭后的位置
  struct VirtState {
    int im;     // Multi (Cross)
    int ic;     // Corner (C4)
    int ie;     // Edge (E0)
    int ie6_nb; // Edge6 index (Neighbor)
    int ic2_nb; // Corn2 index (Neighbor)
    int ie6_dg; // Edge6 index (Diagonal)
    int ic2_dg; // Corn2 index (Diagonal)
  };

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

  void get_conjugated_indices_full(const std::vector<int> &alg, int slot_k,
                                   VirtState &vs) {
    int cur_mul = IDX_MULTI_BASE * 24;
    int cur_corn = IDX_C4 * 18;
    int cur_e0 = IDX_E0 * 18;
    int cur_e6_n = IDX_SOLVED_E6_NB * 18;
    int cur_c2_n = IDX_SOLVED_C2_NB * 18;
    int cur_e6_d = IDX_SOLVED_E6_DG * 18;
    int cur_c2_d = IDX_SOLVED_C2_DG * 18;

    for (int m : alg) {
      int mc = conj_moves_flat[m][slot_k];
      cur_mul = p_multi[cur_mul + mc];
      cur_corn = p_corn[cur_corn + mc] * 18;
      cur_e0 = p_edge[cur_e0 + mc] * 18;
      cur_e6_n = p_edge6[cur_e6_n + mc] * 18;
      cur_c2_n = p_corn2[cur_c2_n + mc] * 18;
      cur_e6_d = p_edge6[cur_e6_d + mc] * 18;
      cur_c2_d = p_corn2[cur_c2_d + mc] * 18;
    }
    vs.im = cur_mul;
    vs.ic = cur_corn / 18;
    vs.ie = cur_e0 / 18;
    vs.ie6_nb = cur_e6_n / 18;
    vs.ic2_nb = cur_c2_n / 18;
    vs.ie6_dg = cur_e6_d / 18;
    vs.ic2_dg = cur_c2_d / 18;
  }

  // --- Search Logic ---

  bool search_1(int i1, int i2, int i3, int depth, int prev, int slot_s1) {
    const int *moves = valid_moves_flat[prev];
    const int count = valid_moves_count[prev];
    for (int k = 0; k < count; ++k) {
      COUNT_NODE
      int m = moves[k];
      int mc = conj_moves_flat[m][slot_s1];
      int n1 = p_multi[i1 + mc];
      int n2 = p_corn[i2 + mc];
      if (p_cross[n1 + n2] >= depth)
        continue;
      int n3 = p_edge[i3 + mc];
      if (p_pair[n3 * 24 + n2] >= depth)
        continue;
      if (depth == 1) {
        if (p_cross[n1 + n2] == 0 && p_pair[n3 * 24 + n2] == 0)
          return true;
      } else {
        if (search_1(n1, n2 * 18, n3 * 18, depth - 1, m, slot_s1))
          return true;
      }
    }
    return false;
  }

  bool search_2(int im_p, int ic_p, int ie_p, int im_x, int ic_x, int ie_x,
                int depth, int prev, int s_p, int s_x) {
    const int *moves = valid_moves_flat[prev];
    const int count = valid_moves_count[prev];
    for (int k = 0; k < count; ++k) {
      COUNT_NODE
      int m = moves[k];
      int mc_p = conj_moves_flat[m][s_p];
      int n_im_p = p_multi[im_p + mc_p], n_ic_p = p_corn[ic_p + mc_p];
      if (p_cross[n_im_p + n_ic_p] >= depth)
        continue;
      int n_ie_p = p_edge[ie_p + mc_p];
      if (p_pair[n_ie_p * 24 + n_ic_p] >= depth)
        continue;
      int mc_x = conj_moves_flat[m][s_x];
      int n_im_x = p_multi[im_x + mc_x], n_ic_x = p_corn[ic_x + mc_x],
          n_ie_x = p_edge[ie_x + mc_x];
      if (get_prune_4bit(p_xcross,
                         (long long)(n_im_x + n_ic_x) * 24 + n_ie_x) >= depth)
        continue;
      if (depth == 1) {
        if (p_pair[n_ie_p * 24 + n_ic_p] == 0 &&
            get_prune_4bit(p_xcross,
                           (long long)(n_im_x + n_ic_x) * 24 + n_ie_x) == 0)
          return true;
      } else {
        if (search_2(n_im_p, n_ic_p * 18, n_ie_p * 18, n_im_x, n_ic_x * 18,
                     n_ie_x * 18, depth - 1, m, s_p, s_x))
          return true;
      }
    }
    return false;
  }

  bool search_3(int im_p, int ic_p, int ie_p, int im_x1, int ic_x1, int ie_x1,
                int im_x2, int ic_x2, int ie_x2, int i_e6, int i_c2, int s_v,
                const unsigned char *p_table_huge, int depth, int prev, int s_p,
                int s_x1, int s_x2) {
    const int *moves = valid_moves_flat[prev];
    const int count = valid_moves_count[prev];
    for (int k = 0; k < count; ++k) {
      COUNT_NODE
      int m = moves[k];
      if (s_v != -1 && p_table_huge) {
        int mv = conj_moves_flat[m][s_v];
        if (get_prune_4bit(p_table_huge,
                           (long long)p_edge6[i_e6 * 18 + mv] * 504 +
                               p_corn2[i_c2 * 18 + mv]) >= depth)
          continue;
      }
      int mc_p = conj_moves_flat[m][s_p];
      int n_im_p = p_multi[im_p + mc_p], n_ic_p = p_corn[ic_p + mc_p];
      if (p_cross[n_im_p + n_ic_p] >= depth)
        continue;
      int n_ie_p = p_edge[ie_p + mc_p];
      if (p_pair[n_ie_p * 24 + n_ic_p] >= depth)
        continue;
      int mc_x1 = conj_moves_flat[m][s_x1];
      int n_im_x1 = p_multi[im_x1 + mc_x1], n_ic_x1 = p_corn[ic_x1 + mc_x1],
          n_ie_x1 = p_edge[ie_x1 + mc_x1];
      if (get_prune_4bit(p_xcross, (long long)(n_im_x1 + n_ic_x1) * 24 +
                                       n_ie_x1) >= depth)
        continue;
      int mc_x2 = conj_moves_flat[m][s_x2];
      int n_im_x2 = p_multi[im_x2 + mc_x2], n_ic_x2 = p_corn[ic_x2 + mc_x2],
          n_ie_x2 = p_edge[ie_x2 + mc_x2];
      if (get_prune_4bit(p_xcross, (long long)(n_im_x2 + n_ic_x2) * 24 +
                                       n_ie_x2) >= depth)
        continue;
      if (depth == 1) {
        if (p_pair[n_ie_p * 24 + n_ic_p] == 0)
          return true;
      } else {
        if (search_3(
                n_im_p, n_ic_p * 18, n_ie_p * 18, n_im_x1, n_ic_x1 * 18,
                n_ie_x1 * 18, n_im_x2, n_ic_x2 * 18, n_ie_x2 * 18,
                (s_v != -1) ? p_edge6[i_e6 * 18 + conj_moves_flat[m][s_v]] : -1,
                (s_v != -1) ? p_corn2[i_c2 * 18 + conj_moves_flat[m][s_v]] : -1,
                s_v, p_table_huge, depth - 1, m, s_p, s_x1, s_x2))
          return true;
      }
    }
    return false;
  }

  bool search_4(int im_p, int ic_p, int ie_p, int im_x1, int ic_x1, int ie_x1,
                int im_x2, int ic_x2, int ie_x2, int im_x3, int ic_x3,
                int ie_x3, int ie6_1, int ic2_1, int v1,
                const unsigned char *p1, int ie6_2, int ic2_2, int v2,
                const unsigned char *p2, int ie6_3, int ic2_3, int v3,
                const unsigned char *p3, int depth, int prev, int s_p, int s_x1,
                int s_x2, int s_x3) {
    const int *moves = valid_moves_flat[prev];
    const int count = valid_moves_count[prev];
    for (int k = 0; k < count; ++k) {
      COUNT_NODE
      int m = moves[k];
      if (v1 != -1 && p1) {
        int mx = conj_moves_flat[m][v1];
        if (get_prune_4bit(p1, (long long)p_edge6[ie6_1 * 18 + mx] * 504 +
                                   p_corn2[ic2_1 * 18 + mx]) >= depth)
          continue;
      }
      if (v2 != -1 && p2) {
        int mx = conj_moves_flat[m][v2];
        if (get_prune_4bit(p2, (long long)p_edge6[ie6_2 * 18 + mx] * 504 +
                                   p_corn2[ic2_2 * 18 + mx]) >= depth)
          continue;
      }
      if (v3 != -1 && p3) {
        int mx = conj_moves_flat[m][v3];
        if (get_prune_4bit(p3, (long long)p_edge6[ie6_3 * 18 + mx] * 504 +
                                   p_corn2[ic2_3 * 18 + mx]) >= depth)
          continue;
      }
      int mc_p = conj_moves_flat[m][s_p];
      int n_im_p = p_multi[im_p + mc_p], n_ic_p = p_corn[ic_p + mc_p];
      if (p_cross[n_im_p + n_ic_p] >= depth)
        continue;
      int n_ie_p = p_edge[ie_p + mc_p];
      if (p_pair[n_ie_p * 24 + n_ic_p] >= depth)
        continue;
      int mc_x1 = conj_moves_flat[m][s_x1];
      int n_im_x1 = p_multi[im_x1 + mc_x1], n_ic_x1 = p_corn[ic_x1 + mc_x1],
          n_ie_x1 = p_edge[ie_x1 + mc_x1];
      if (get_prune_4bit(p_xcross, (long long)(n_im_x1 + n_ic_x1) * 24 +
                                       n_ie_x1) >= depth)
        continue;
      int mc_x2 = conj_moves_flat[m][s_x2];
      int n_im_x2 = p_multi[im_x2 + mc_x2], n_ic_x2 = p_corn[ic_x2 + mc_x2],
          n_ie_x2 = p_edge[ie_x2 + mc_x2];
      if (get_prune_4bit(p_xcross, (long long)(n_im_x2 + n_ic_x2) * 24 +
                                       n_ie_x2) >= depth)
        continue;
      int mc_x3 = conj_moves_flat[m][s_x3];
      int n_im_x3 = p_multi[im_x3 + mc_x3], n_ic_x3 = p_corn[ic_x3 + mc_x3],
          n_ie_x3 = p_edge[ie_x3 + mc_x3];
      if (get_prune_4bit(p_xcross, (long long)(n_im_x3 + n_ic_x3) * 24 +
                                       n_ie_x3) >= depth)
        continue;
      if (depth == 1) {
        if (p_pair[n_ie_p * 24 + n_ic_p] == 0)
          return true;
      } else if (search_4(
                     n_im_p, n_ic_p * 18, n_ie_p * 18, n_im_x1, n_ic_x1 * 18,
                     n_ie_x1 * 18, n_im_x2, n_ic_x2 * 18, n_ie_x2 * 18, n_im_x3,
                     n_ic_x3 * 18, n_ie_x3 * 18,
                     (v1 != -1) ? p_edge6[ie6_1 * 18 + conj_moves_flat[m][v1]]
                                : -1,
                     (v1 != -1) ? p_corn2[ic2_1 * 18 + conj_moves_flat[m][v1]]
                                : -1,
                     v1, p1,
                     (v2 != -1) ? p_edge6[ie6_2 * 18 + conj_moves_flat[m][v2]]
                                : -1,
                     (v2 != -1) ? p_corn2[ic2_2 * 18 + conj_moves_flat[m][v2]]
                                : -1,
                     v2, p2,
                     (v3 != -1) ? p_edge6[ie6_3 * 18 + conj_moves_flat[m][v3]]
                                : -1,
                     (v3 != -1) ? p_corn2[ic2_3 * 18 + conj_moves_flat[m][v3]]
                                : -1,
                     v3, p3, depth - 1, m, s_p, s_x1, s_x2, s_x3))
        return true;
    }
    return false;
  }

  // --- Solvers ---

  int solve_1_group(const std::vector<int> &alg, int bound) {
    std::vector<Task1> tasks;
    VirtState st;
    for (int s1 = 0; s1 < 4; ++s1) {
      get_conjugated_indices_full(alg, s1, st);
      tasks.push_back({s1, (int)p_cross[st.im + st.ic]});
    }
    std::sort(tasks.begin(), tasks.end(),
              [](const Task1 &a, const Task1 &b) { return a.h < b.h; });
    int min_v = bound;
    for (const auto &t : tasks) {
      if (t.h >= min_v)
        continue;
      get_conjugated_indices_full(alg, t.s1, st);
      if (t.h == 0 && p_pair[st.ie * 24 + st.ic] == 0)
        return 0;
      int max_search = std::min(18, min_v - 1);
      for (int d = t.h; d <= max_search; ++d) {
        if (search_1(st.im, st.ic * 18, st.ie * 18, d, 18, t.s1)) {
          if (d < min_v)
            min_v = d;
          break;
        }
      }
    }
    return min_v;
  }

  int solve_2_group(const std::vector<int> &alg, int bound) {
    std::vector<Task2> tasks;
    VirtState sp, sx;
    for (int fix = 0; fix < 4; ++fix) {
      for (int tgt = 0; tgt < 4; ++tgt) {
        if (fix == tgt)
          continue;
        get_conjugated_indices_full(alg, tgt, sp);
        get_conjugated_indices_full(alg, fix, sx);
        int h1 = p_cross[sp.im + sp.ic];
        int h2 =
            get_prune_4bit(p_xcross, (long long)(sx.im + sx.ic) * 24 + sx.ie);
        tasks.push_back({tgt, fix, std::max(h1, h2)});
      }
    }
    std::sort(tasks.begin(), tasks.end(),
              [](const Task2 &a, const Task2 &b) { return a.h < b.h; });
    int min_v = bound;
    for (const auto &t : tasks) {
      if (t.h >= min_v)
        continue;
      get_conjugated_indices_full(alg, t.s1, sp);
      get_conjugated_indices_full(alg, t.s2, sx);
      if (t.h == 0 && p_pair[sp.ie * 24 + sp.ic] == 0)
        return 0;
      int max_search = std::min(18, min_v - 1);
      for (int d = t.h; d <= max_search; ++d) {
        if (search_2(sp.im, sp.ic * 18, sp.ie * 18, sx.im, sx.ic * 18,
                     sx.ie * 18, d, 18, t.s1, t.s2)) {
          if (d < min_v)
            min_v = d;
          break;
        }
      }
    }
    return min_v;
  }

  int solve_3_group(const std::vector<int> &alg, int bound) {
    std::vector<Task3> tasks;
    std::vector<std::vector<int>> pairs = {{0, 1}, {0, 2}, {0, 3},
                                           {1, 2}, {1, 3}, {2, 3}};
    VirtState sp, sx1, sx2, st_v;
    for (auto &p : pairs) {
      for (int tgt = 0; tgt < 4; ++tgt) {
        if (tgt == p[0] || tgt == p[1])
          continue;
        get_conjugated_indices_full(alg, tgt, sp);
        get_conjugated_indices_full(alg, p[0], sx1);
        get_conjugated_indices_full(alg, p[1], sx2);
        int h1 = p_cross[sp.im + sp.ic];
        int h2 = get_prune_4bit(p_xcross,
                                (long long)(sx1.im + sx1.ic) * 24 + sx1.ie);
        int h3 = get_prune_4bit(p_xcross,
                                (long long)(sx2.im + sx2.ic) * 24 + sx2.ie);
        int h_huge = 0;
        int v = get_neighbor_view(p[0], p[1]);
        if (v != -1) {
          get_conjugated_indices_full(alg, v, st_v);
          h_huge = get_prune_4bit(p_prune_neighbor,
                                  (long long)st_v.ie6_nb * 504 + st_v.ic2_nb);
        } else if (p_prune_diagonal) {
          v = get_diagonal_view(p[0], p[1]);
          get_conjugated_indices_full(alg, v, st_v);
          h_huge = get_prune_4bit(p_prune_diagonal,
                                  (long long)st_v.ie6_dg * 504 + st_v.ic2_dg);
        }
        tasks.push_back({tgt, p[0], p[1], std::max({h1, h2, h3, h_huge})});
      }
    }
    std::sort(tasks.begin(), tasks.end(),
              [](const Task3 &a, const Task3 &b) { return a.h < b.h; });
    int min_v = bound;
    for (const auto &t : tasks) {
      if (t.h >= min_v)
        continue;
      get_conjugated_indices_full(alg, t.s1, sp);
      get_conjugated_indices_full(alg, t.s2, sx1);
      get_conjugated_indices_full(alg, t.s3, sx2);
      if (t.h == 0 && p_pair[sp.ie * 24 + sp.ic] == 0)
        return 0;
      int ie6_use = -1, ic2_use = -1, v_use = -1;
      const unsigned char *p_use = nullptr;
      VirtState st_tmp;
      if (get_neighbor_view(t.s2, t.s3) != -1) {
        v_use = get_neighbor_view(t.s2, t.s3);
        get_conjugated_indices_full(alg, v_use, st_tmp);
        ie6_use = st_tmp.ie6_nb;
        ic2_use = st_tmp.ic2_nb;
        p_use = p_prune_neighbor;
      } else if (p_prune_diagonal) {
        v_use = get_diagonal_view(t.s2, t.s3);
        get_conjugated_indices_full(alg, v_use, st_tmp);
        ie6_use = st_tmp.ie6_dg;
        ic2_use = st_tmp.ic2_dg;
        p_use = p_prune_diagonal;
      }
      int max_search = std::min(18, min_v - 1);
      for (int d = t.h; d <= max_search; ++d) {
        if (search_3(sp.im, sp.ic * 18, sp.ie * 18, sx1.im, sx1.ic * 18,
                     sx1.ie * 18, sx2.im, sx2.ic * 18, sx2.ie * 18, ie6_use,
                     ic2_use, v_use, p_use, d, 18, t.s1, t.s2, t.s3)) {
          if (d < min_v)
            min_v = d;
          break;
        }
      }
    }
    return min_v;
  }

  int solve_4_group(const std::vector<int> &alg, int bound) {
    std::vector<Task4> tasks;
    VirtState sp, s[3], st_v;
    for (int tgt = 0; tgt < 4; ++tgt) {
      std::vector<int> fix;
      for (int k = 0; k < 4; ++k)
        if (k != tgt)
          fix.push_back(k);
      get_conjugated_indices_full(alg, tgt, sp);
      int h_val = p_cross[sp.im + sp.ic];
      for (int i = 0; i < 3; ++i) {
        get_conjugated_indices_full(alg, fix[i], s[i]);
        h_val =
            std::max(h_val, get_prune_4bit(p_xcross,
                                           (long long)(s[i].im + s[i].ic) * 24 +
                                               s[i].ie));
      }
      for (int i = 0; i < 3; ++i) {
        for (int j = i + 1; j < 3; ++j) {
          int v = get_neighbor_view(fix[i], fix[j]);
          if (v != -1) {
            get_conjugated_indices_full(alg, v, st_v);
            h_val =
                std::max(h_val, get_prune_4bit(p_prune_neighbor,
                                               (long long)st_v.ie6_nb * 504 +
                                                   st_v.ic2_nb));
          } else if (p_prune_diagonal) {
            v = get_diagonal_view(fix[i], fix[j]);
            get_conjugated_indices_full(alg, v, st_v);
            h_val =
                std::max(h_val, get_prune_4bit(p_prune_diagonal,
                                               (long long)st_v.ie6_dg * 504 +
                                                   st_v.ic2_dg));
          }
        }
      }
      tasks.push_back({tgt, fix[0], fix[1], fix[2], h_val});
    }
    std::sort(tasks.begin(), tasks.end(),
              [](const Task4 &a, const Task4 &b) { return a.h < b.h; });
    int min_v = bound;
    for (const auto &t : tasks) {
      if (t.h >= min_v)
        continue;
      get_conjugated_indices_full(alg, t.s1, sp);
      get_conjugated_indices_full(alg, t.s2, s[0]);
      get_conjugated_indices_full(alg, t.s3, s[1]);
      get_conjugated_indices_full(alg, t.s4, s[2]);
      if (t.h == 0 && p_pair[sp.ie * 24 + sp.ic] == 0)
        return 0;
      int ie6[3], ic2[3], v[3];
      const unsigned char *p[3] = {nullptr, nullptr, nullptr};
      int pairs[3][2] = {{t.s2, t.s3}, {t.s3, t.s4}, {t.s4, t.s2}};
      VirtState st_tmp;
      for (int i = 0; i < 3; ++i) {
        v[i] = -1;
        if (get_neighbor_view(pairs[i][0], pairs[i][1]) != -1) {
          v[i] = get_neighbor_view(pairs[i][0], pairs[i][1]);
          get_conjugated_indices_full(alg, v[i], st_tmp);
          ie6[i] = st_tmp.ie6_nb;
          ic2[i] = st_tmp.ic2_nb;
          p[i] = p_prune_neighbor;
        } else if (p_prune_diagonal) {
          v[i] = get_diagonal_view(pairs[i][0], pairs[i][1]);
          get_conjugated_indices_full(alg, v[i], st_tmp);
          ie6[i] = st_tmp.ie6_dg;
          ic2[i] = st_tmp.ic2_dg;
          p[i] = p_prune_diagonal;
        }
      }
      int max_search = std::min(18, min_v - 1);
      for (int d = t.h; d <= max_search; ++d) {
        if (search_4(sp.im, sp.ic * 18, sp.ie * 18, s[0].im, s[0].ic * 18,
                     s[0].ie * 18, s[1].im, s[1].ic * 18, s[1].ie * 18, s[2].im,
                     s[2].ic * 18, s[2].ie * 18, ie6[0], ic2[0], v[0], p[0],
                     ie6[1], ic2[1], v[1], p[1], ie6[2], ic2[2], v[2], p[2], d,
                     18, t.s1, t.s2, t.s3, t.s4)) {
          if (d < min_v)
            min_v = d;
          break;
        }
      }
    }
    return min_v;
  }
};

// --- PairSolverWrapper: 封装 PairSolver 的统一接口 ---
// NOTE: 此类满足 analyzer_executor.h 中 run_analyzer_app 模板的要求
struct PairSolverWrapper {
  // 旋转列表：对应 _z0, _z1, _z2, _z3, _x1, _x3 后缀
  static inline std::vector<std::string> rots = {"",  "z2", "z'",
                                                 "z", "x'", "x"};

  PairSolver solver;

  // 全局初始化：加载移动表和剪枝表
  static void global_init() {
    init_matrix();

    auto &mtm = MoveTableManager::getInstance();
    auto &ptm = PruneTableManager::getInstance();

    std::cout << ANSI_CYAN << "[INIT] " << ANSI_RESET
              << "Loading Move Tables..." << std::endl;
    if (!mtm.loadAll()) {
      std::cerr << ANSI_RED
                << "[ERROR] Move tables missing. Please run "
                   "table_generator.exe first."
                << ANSI_RESET << std::endl;
      exit(1);
    }
    std::cout << ANSI_CYAN << "[INIT] " << ANSI_RESET
              << "Loading Move Tables... Done." << std::endl;

    std::cout << ANSI_CYAN << "[INIT] " << ANSI_RESET
              << "Loading Prune Tables..." << std::endl;
    if (!ptm.loadAll()) {
      std::cerr << ANSI_RED
                << "[ERROR] Prune tables missing. Please run "
                   "table_generator.exe first."
                << ANSI_RESET << std::endl;
      exit(1);
    }
    std::cout << ANSI_CYAN << "[INIT] " << ANSI_RESET
              << "Loading Prune Tables... Done." << std::endl;

    // 初始化静态指针
    PairSolver::static_init();
  }

  // CSV 表头：使用 snake_case 和对称旋转后缀
  static std::string get_csv_header() {
    std::vector<std::string> suffixes = {"_z0", "_z1", "_z2",
                                         "_z3", "_x1", "_x3"};
    std::ostringstream oss;
    oss << "id";
    for (const auto &s : suffixes)
      oss << ",cross_pair" << s;
    for (const auto &s : suffixes)
      oss << ",xcross_pair" << s;
    for (const auto &s : suffixes)
      oss << ",xxcross_pair" << s;
    for (const auto &s : suffixes)
      oss << ",xxxcross_pair" << s;
    return oss.str();
  }

  // 求解单个任务
  std::string solve(const std::vector<int> &alg, const std::string &id) {
    std::ostringstream oss;
    oss << id;
    // Cross + Pair
    for (const auto &r : rots) {
      std::vector<int> a = alg_rotation(alg, r);
      oss << "," << solver.solve_1_group(a, 99);
    }
    // XCross + Pair
    for (const auto &r : rots) {
      std::vector<int> a = alg_rotation(alg, r);
      oss << "," << solver.solve_2_group(a, 99);
    }
    // XXCross + Pair
    for (const auto &r : rots) {
      std::vector<int> a = alg_rotation(alg, r);
      oss << "," << solver.solve_3_group(a, 99);
    }
    // XXXCross + Pair
    for (const auto &r : rots) {
      std::vector<int> a = alg_rotation(alg, r);
      oss << "," << solver.solve_4_group(a, 99);
    }
    return oss.str();
  }

  static void print_stats() {}
};

int main() {
  run_analyzer_app<PairSolverWrapper>("_pair");
  return 0;
}
