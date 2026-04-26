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

// --- 剪枝统计 (通过 prune_stats.h 统一开关控制) ---
#include "prune_stats.h"

// Search 1: Cross+XCross
STAT_DECL(s1_cross);  // S1: Cross C4 剪枝表
STAT_DECL(s1_xcross); // S1: XCross 剪枝表

// Search 2: XCross×2 + Huge
STAT_DECL(s2_xcross1); // S2: XCross 1 剪枝表
STAT_DECL(s2_xcross2); // S2: XCross 2 剪枝表
STAT_DECL(s2_huge);    // S2: Huge 表

// Search 3: Huge×2 + XCross×3
STAT_DECL(s3_huge1);   // S3: Huge 表 1
STAT_DECL(s3_huge2);   // S3: Huge 表 2
STAT_DECL(s3_xcross1); // S3: XCross 1 剪枝表
STAT_DECL(s3_xcross2); // S3: XCross 2 剪枝表
STAT_DECL(s3_xcross3); // S3: XCross 3 剪枝表

// Search 4: Huge×3 + XCross×4
STAT_DECL(s4_huge1);   // S4: Huge 表 1
STAT_DECL(s4_huge2);   // S4: Huge 表 2
STAT_DECL(s4_huge3);   // S4: Huge 表 3
STAT_DECL(s4_xcross1); // S4: XCross 1 剪枝表
STAT_DECL(s4_xcross2); // S4: XCross 2 剪枝表
STAT_DECL(s4_xcross3); // S4: XCross 3 剪枝表
STAT_DECL(s4_xcross4); // S4: XCross 4 剪枝表

// NOTE: CrossSolver 已抽取到 cross_solver.h（Std/Pseudo 共享）
#include "cross_solver.h"

struct XCrossSolver {
  const int *p_mt_edge4, *p_mt_corn, *p_mt_edge, *p_mt_edge6, *p_mt_corn2;
  const unsigned char *p_pt_cross_C4C5E0E1 = nullptr,
                      *p_pt_cross_C4C6E0E2 = nullptr;
  const unsigned char *p_pt_cross_C4E0;

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

  // NOTE: search_4 每个视角的 6 个状态索引
  // 隐含 slot = 数组下标 (0,1,2,3)
  struct SlotView4 {
    int im;  // Edge4（已含步幅偏移，直接与 p_mt_edge4 配合）
    int ic;  // Corner
    int ie;  // Edge
    int ex;  // Extra Edge
    int cx1; // Extra Corner 1
    int cx2; // Extra Corner 2
  };

  // NOTE: Huge 剪枝的 (Edge6, Corner2) 状态对
  struct HugePair {
    int e6; // Edge6 状态索引
    int c2; // Corner2 状态索引
  };

  // NOTE: search_3 每个视角的 8 个状态索引 + slot 编号
  struct SlotView3 {
    int im;     // Edge4
    int ic;     // Corner
    int ie;     // Edge
    int ex1;    // Extra Edge - Plus 1
    int cx1;    // Extra Corner - Plus 1
    int cx_mid; // Corner 中间值 (c6)
    int ex2;    // Extra Edge - Plus 2
    int cx2;    // Extra Corner - Plus 2
    int slot;   // conj slot 号
  };

  // NOTE: search_3 的 Huge 状态（含 conj 视角和表指针）
  struct HugeState {
    int e6;                     // Edge6 状态索引
    int c2;                     // Corner2 状态索引
    int conj;                   // conj slot 号
    const unsigned char *table; // 剪枝表指针
  };

  XCrossSolver() {
    auto &mtm = MoveTableManager::getInstance();
    auto &ptm = PruneTableManager::getInstance();

    p_mt_edge4 = mtm.getMTEdge4Ptr();
    p_mt_corn = mtm.getMTCornPtr();
    p_mt_edge = mtm.getMTEdgePtr();
    p_mt_edge6 = mtm.getMTEdge6Ptr();
    p_mt_corn2 = mtm.getMTCorn2Ptr();

    p_pt_cross_C4E0 = ptm.getCrossC4E0PTPtr();
    p_pt_cross_C4C5E0E1 = ptm.getCrossC4C5E0E1PTPtr();
#if ENABLE_DIAGONAL_STD
    p_pt_cross_C4C6E0E2 = ptm.getCrossC4C6E0E2PTPtr();
#endif
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
    int cur_mul = StateSpace::CROSS_SOLVED * StateSpace::CORNER;
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
      cur_mul = p_mt_edge4[cur_mul + mc];
      cur_cn = p_mt_corn[cur_cn + mc] * 18;
      cur_e0 = p_mt_edge[cur_e0 * 18 + mc];
      cur_e2 = p_mt_edge[cur_e2 * 18 + mc];
      cur_e4 = p_mt_edge[cur_e4 * 18 + mc];
      cur_e6 = p_mt_edge[cur_e6 * 18 + mc];
      cur_c5 = p_mt_corn[cur_c5 * 18 + mc];
      cur_c6 = p_mt_corn[cur_c6 * 18 + mc];
      cur_c7 = p_mt_corn[cur_c7 * 18 + mc];
      cur_e6_n = p_mt_edge6[cur_e6_n + mc] * 18;
      cur_c2_n = p_mt_corn2[cur_c2_n + mc] * 18;
      cur_e6_d = p_mt_edge6[cur_e6_d + mc] * 18;
      cur_c2_d = p_mt_corn2[cur_c2_d + mc] * 18;
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

  bool search_1(int i1, int i2, int i3, int s1, int depth, int prev) {
    const int *moves = valid_moves_flat[prev];
    const int count = valid_moves_count[prev];
    for (int k = 0; k < count; ++k) {
      int m = moves[k];
      COUNT_NODE
      int m1 = conj_moves_flat[m][s1];
      int n_i1 = p_mt_edge4[i1 + m1];
      int n_i2 = p_mt_corn[i2 + m1];
      int n_i3 = p_mt_edge[i3 + m1];
      long long idx = (long long)(n_i1 + n_i2) * 24 + n_i3;
      S1_CHECK(s1_xcross);
      if (get_prune(p_pt_cross_C4E0, idx) >= depth) {
        S1_HIT(s1_xcross);
        continue;
      }
      if (depth == 1) {
        return true;
      }
      if (search_1(n_i1, n_i2 * 18, n_i3 * 18, s1, depth - 1, m))
        return true;
    }
    return false;
  }

  bool search_2(int i1a, int i2a, int i3a, int i4a, int i5a, int s1, int i1b,
                int i2b, int i3b, int i4b, int i5b, int s2, int t_idx1,
                int t_idx2, int i_e6, int i_c2, int v_adj,
                const unsigned char *p_prune_active, int depth, int prev) {
    const int *moves = valid_moves_flat[prev];
    const int count = valid_moves_count[prev];
    for (int k = 0; k < count; ++k) {
      int m = moves[k];
      COUNT_NODE
      int n_ie6 = -1, n_ic2 = -1;
      if (hugeTablePrunes(v_adj, p_prune_active, i_e6, i_c2, m, depth,
                          p_mt_edge6, p_mt_corn2, n_ie6, n_ic2))
        continue;
      int m1 = conj_moves_flat[m][s1];
      int n_i1a = p_mt_edge4[i1a + m1];
      int n_i2a = p_mt_corn[i2a + m1];
      int n_i3a = p_mt_edge[i3a + m1];
      int n_i4a = p_mt_edge[i4a + m1];
      int n_i5a = p_mt_corn[i5a + m1];
      int m2 = conj_moves_flat[m][s2];
      int n_i1b = p_mt_edge4[i1b + m2];
      int n_i2b = p_mt_corn[i2b + m2];
      int n_i3b = p_mt_edge[i3b + m2];
      int n_i4b = p_mt_edge[i4b + m2];
      int n_i5b = p_mt_corn[i5b + m2];
      if (depth == 1) {
        return true;
      }
      if (search_2(n_i1a, n_i2a * 18, n_i3a * 18, n_i4a * 18, n_i5a * 18, s1,
                   n_i1b, n_i2b * 18, n_i3b * 18, n_i4b * 18, n_i5b * 18, s2,
                   t_idx1, t_idx2, (v_adj != -1) ? n_ie6 : -1,
                   (v_adj != -1) ? n_ic2 : -1, v_adj, p_prune_active, depth - 1,
                   m))
        return true;
    }
    return false;
  }

  // NOTE: search_3 参数结构化 — 42 参数 → views[3] + huge[3] + depth + prev
  // t12/t21/t23/t32 在函数体内未被使用，已移除
  bool search_3(const SlotView3 views[3], HugeState huge[3], int depth,
                int prev) {
    const int *moves = valid_moves_flat[prev];
    const int count = valid_moves_count[prev];
    for (int k = 0; k < count; ++k) {
      int m = moves[k];
      COUNT_NODE

      // --- Huge 剪枝 (3 对) ---
      HugeState n_huge[3];
      bool pruned = false;
      for (int i = 0; i < 3; ++i) {
        n_huge[i].conj = huge[i].conj;
        n_huge[i].table = huge[i].table;
        int n_e6 = -1, n_c2 = -1;
        if (hugeTablePrunes(huge[i].conj, huge[i].table, huge[i].e6, huge[i].c2,
                            m, depth, p_mt_edge6, p_mt_corn2, n_e6, n_c2)) {
          pruned = true;
          break;
        }
        n_huge[i].e6 = (huge[i].conj != -1) ? n_e6 : -1;
        n_huge[i].c2 = (huge[i].conj != -1) ? n_c2 : -1;
      }
      if (pruned)
        continue;

      // --- 3 个视角的 move-table lookup ---
      SlotView3 nv[3];
      for (int i = 0; i < 3; ++i) {
        int mi = conj_moves_flat[m][views[i].slot];
        nv[i].im = p_mt_edge4[views[i].im + mi];
        nv[i].ic = p_mt_corn[views[i].ic + mi];
        nv[i].ie = p_mt_edge[views[i].ie + mi];
        nv[i].ex1 = p_mt_edge[views[i].ex1 + mi];
        nv[i].cx1 = p_mt_corn[views[i].cx1 + mi];
        nv[i].cx_mid = p_mt_corn[views[i].cx_mid + mi];
        nv[i].ex2 = p_mt_edge[views[i].ex2 + mi];
        nv[i].cx2 = p_mt_corn[views[i].cx2 + mi];
        nv[i].slot = views[i].slot;
      }

      if (depth == 1) {
        return true;
      }
      // 递归：新状态需要乘步幅 18
      SlotView3 nv_rec[3];
      for (int i = 0; i < 3; ++i) {
        nv_rec[i] = {nv[i].im,       nv[i].ic * 18,  nv[i].ie * 18,
                     nv[i].ex1 * 18, nv[i].cx1 * 18, nv[i].cx_mid * 18,
                     nv[i].ex2 * 18, nv[i].cx2 * 18, nv[i].slot};
      }
      if (search_3(nv_rec, n_huge, depth - 1, m))
        return true;
    }
    return false;
  }

  // NOTE: search_4 参数结构化 — 38 参数 → views[4] + nb[4] + dg[2] + depth +
  // prev
  bool search_4(const SlotView4 views[4], HugePair nb[4], HugePair dg[2],
                int depth, int prev) {
    const int *moves = valid_moves_flat[prev];
    const int count = valid_moves_count[prev];
    for (int k = 0; k < count; ++k) {
      int m = moves[k];
      COUNT_NODE

      // --- Huge Neighbor 剪枝 (4对，conj slot = 数组下标) ---
      HugePair n_nb[4];
      bool pruned = false;
      for (int i = 0; i < 4; ++i) {
        int mi = conj_moves_flat[m][i];
        n_nb[i].e6 = p_mt_edge6[nb[i].e6 * 18 + mi];
        n_nb[i].c2 = p_mt_corn2[nb[i].c2 * 18 + mi];
        if (get_prune(p_pt_cross_C4C5E0E1,
                      (long long)n_nb[i].e6 * StateSpace::CORNER2 +
                          n_nb[i].c2) >= depth) {
          pruned = true;
          break;
        }
      }
      if (pruned)
        continue;

      // --- Huge Diagonal 剪枝 (2对，conj slot = 0, 1) ---
      HugePair n_dg[2];
      for (int i = 0; i < 2; ++i) {
        int mi = conj_moves_flat[m][i];
        n_dg[i].e6 = p_mt_edge6[dg[i].e6 * 18 + mi];
        n_dg[i].c2 = p_mt_corn2[dg[i].c2 * 18 + mi];
        if (p_pt_cross_C4C6E0E2) {
          if (get_prune(p_pt_cross_C4C6E0E2,
                        (long long)n_dg[i].e6 * StateSpace::CORNER2 +
                            n_dg[i].c2) >= depth) {
            pruned = true;
            break;
          }
        }
      }
      if (pruned)
        continue;

      // --- 4 个视角的 move-table lookup (conj slot = 数组下标) ---
      SlotView4 nv[4];
      for (int i = 0; i < 4; ++i) {
        int mi = conj_moves_flat[m][i];
        nv[i].im = p_mt_edge4[views[i].im + mi];
        nv[i].ic = p_mt_corn[views[i].ic + mi];
        nv[i].ie = p_mt_edge[views[i].ie + mi];
        nv[i].ex = p_mt_edge[views[i].ex + mi];
        nv[i].cx1 = p_mt_corn[views[i].cx1 + mi];
        nv[i].cx2 = p_mt_corn[views[i].cx2 + mi];
      }

      if (depth == 1) {
        return true;
      }
      // 递归：新状态需要乘步幅 18
      SlotView4 nv_rec[4];
      for (int i = 0; i < 4; ++i) {
        nv_rec[i] = {nv[i].im,      nv[i].ic * 18,  nv[i].ie * 18,
                     nv[i].ex * 18, nv[i].cx1 * 18, nv[i].cx2 * 18};
      }
      if (search_4(nv_rec, n_nb, n_dg, depth - 1, m))
        return true;
    }
    return false;
  }

  std::vector<int> get_stats(const std::vector<int> &base_alg,
                             const std::vector<std::string> &rots) {
    std::vector<int> all_results;
    all_results.reserve(48);
    // NOTE: 跨阶段 Early Exit — 前阶段 best 作为后阶段搜索起始深度下界
    std::vector<int> xc_min, xxc_min, xxxc_min;

    { // 1. XC
      xc_min.assign(rots.size(), 99);
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
          int h = get_prune(p_pt_cross_C4E0, idx);
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
            int max_search = std::min(12, current_best - 1);
            for (int d = t.h; d <= max_search; ++d) {
              if (search_1(initial_states[t.id].im,
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
        xc_min[r] = current_best;
      }
      all_results.insert(all_results.end(), xc_min.begin(), xc_min.end());
    }

    { // 2. XX-Cross
      xxc_min.assign(rots.size(), 99);
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
          int v_nb = getNeighborView(p.a, p.b);
          int v_dg = getDiagonalView(p.a, p.b);
          int h_val = 0;
          int view_used = -1;
          if (v_nb != -1) {
            h_val = get_prune(p_pt_cross_C4C5E0E1,
                              (long long)st[v_nb].ie6_nb * StateSpace::CORNER2 +
                                  st[v_nb].ic2_nb);
            view_used = v_nb;
          } else if (v_dg != -1 && p_pt_cross_C4C6E0E2) {
            h_val = get_prune(p_pt_cross_C4C6E0E2,
                              (long long)st[v_dg].ie6_dg * StateSpace::CORNER2 +
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
            int max_search = std::min(14, current_best - 1);
            int t1 = getPlusTableIdx(t.a, t.b);
            int t2 = getPlusTableIdx(t.b, t.a);
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
            if (getNeighborView(t.a, t.b) != -1) {
              ie6_use = st[t.v].ie6_nb;
              ic2_use = st[t.v].ic2_nb;
              p_table_use = p_pt_cross_C4C5E0E1;
            } else if (p_pt_cross_C4C6E0E2) {
              ie6_use = st[t.v].ie6_dg;
              ic2_use = st[t.v].ic2_dg;
              p_table_use = p_pt_cross_C4C6E0E2;
            }
            if (p_table_use) {
              // 跨阶段下界：XXC 解 ≥ XC 解
              int startD = std::max(t.h, xc_min[r]);
              for (int d = startD; d <= max_search; ++d) {
                if (search_2(st[t.a].im, st[t.a].ic * 18, st[t.a].e0 * 18,
                             ea * 18, ca * 18, t.a, st[t.b].im, st[t.b].ic * 18,
                             st[t.b].e0 * 18, eb * 18, cb * 18, t.b, t1, t2,
                             ie6_use, ic2_use, t.v, p_table_use, d, 18)) {
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
        xxc_min[r] = current_best;
      }
      all_results.insert(all_results.end(), xxc_min.begin(), xxc_min.end());
    }

    { // 3. XXX-Cross
      xxxc_min.assign(rots.size(), 99);
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
          int v1 = getNeighborView(t.a, t.b);
          int d1 = 0;
          if (v1 != -1)
            d1 = get_prune(p_pt_cross_C4C5E0E1,
                           (long long)st[v1].ie6_nb * StateSpace::CORNER2 +
                               st[v1].ic2_nb);
          else if (p_pt_cross_C4C6E0E2) {
            v1 = getDiagonalView(t.a, t.b);
            d1 = get_prune(p_pt_cross_C4C6E0E2,
                           (long long)st[v1].ie6_dg * StateSpace::CORNER2 +
                               st[v1].ic2_dg);
          }
          int v2 = getNeighborView(t.b, t.c);
          int d2 = 0;
          if (v2 != -1)
            d2 = get_prune(p_pt_cross_C4C5E0E1,
                           (long long)st[v2].ie6_nb * StateSpace::CORNER2 +
                               st[v2].ic2_nb);
          else if (p_pt_cross_C4C6E0E2) {
            v2 = getDiagonalView(t.b, t.c);
            d2 = get_prune(p_pt_cross_C4C6E0E2,
                           (long long)st[v2].ie6_dg * StateSpace::CORNER2 +
                               st[v2].ic2_dg);
          }
          int v3 = getNeighborView(t.c, t.a);
          int d3 = 0;
          if (v3 != -1)
            d3 = get_prune(p_pt_cross_C4C5E0E1,
                           (long long)st[v3].ie6_nb * StateSpace::CORNER2 +
                               st[v3].ic2_nb);
          else if (p_pt_cross_C4C6E0E2) {
            v3 = getDiagonalView(t.c, t.a);
            d3 = get_prune(p_pt_cross_C4C6E0E2,
                           (long long)st[v3].ie6_dg * StateSpace::CORNER2 +
                               st[v3].ic2_dg);
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
            int max_search = std::min(16, current_best - 1);
            int t12 = getPlusTableIdx(t.a, t.b);
            int t21 = getPlusTableIdx(t.b, t.a);
            int t23 = getPlusTableIdx(t.b, t.c);
            int t32 = getPlusTableIdx(t.c, t.b);
            int t31 = getPlusTableIdx(t.c, t.a);
            int t13 = getPlusTableIdx(t.a, t.c);
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
            if (getNeighborView(t.a, t.b) != -1) {
              i_e6_1 = st[t.v12].ie6_nb;
              i_c2_1 = st[t.v12].ic2_nb;
              p_1 = p_pt_cross_C4C5E0E1;
            } else if (p_pt_cross_C4C6E0E2) {
              i_e6_1 = st[t.v12].ie6_dg;
              i_c2_1 = st[t.v12].ic2_dg;
              p_1 = p_pt_cross_C4C6E0E2;
            }
            int i_e6_2 = -1, i_c2_2 = -1;
            const unsigned char *p_2 = nullptr;
            if (getNeighborView(t.b, t.c) != -1) {
              i_e6_2 = st[t.v23].ie6_nb;
              i_c2_2 = st[t.v23].ic2_nb;
              p_2 = p_pt_cross_C4C5E0E1;
            } else if (p_pt_cross_C4C6E0E2) {
              i_e6_2 = st[t.v23].ie6_dg;
              i_c2_2 = st[t.v23].ic2_dg;
              p_2 = p_pt_cross_C4C6E0E2;
            }
            int i_e6_3 = -1, i_c2_3 = -1;
            const unsigned char *p_3 = nullptr;
            if (getNeighborView(t.c, t.a) != -1) {
              i_e6_3 = st[t.v31].ie6_nb;
              i_c2_3 = st[t.v31].ic2_nb;
              p_3 = p_pt_cross_C4C5E0E1;
            } else if (p_pt_cross_C4C6E0E2) {
              i_e6_3 = st[t.v31].ie6_dg;
              i_c2_3 = st[t.v31].ic2_dg;
              p_3 = p_pt_cross_C4C6E0E2;
            }

            // 跨阶段下界：XXXC 解 ≥ XXC 解
            int startD = std::max(t.h, xxc_min[r]);
            SlotView3 s3_views[3] = {
                {st[t.a].im, st[t.a].ic * 18, st[t.a].e0 * 18, ea_b * 18,
                 ca_b * 18, st[t.a].c6 * 18, ea_c * 18, ca_c * 18, t.a},
                {st[t.b].im, st[t.b].ic * 18, st[t.b].e0 * 18, eb_a * 18,
                 cb_a * 18, st[t.b].c6 * 18, eb_c * 18, cb_c * 18, t.b},
                {st[t.c].im, st[t.c].ic * 18, st[t.c].e0 * 18, ec_b * 18,
                 cc_b * 18, st[t.c].c6 * 18, ec_a * 18, cc_a * 18, t.c}};
            HugeState s3_huge[3] = {{i_e6_1, i_c2_1, t.v12, p_1},
                                    {i_e6_2, i_c2_2, t.v23, p_2},
                                    {i_e6_3, i_c2_3, t.v31, p_3}};
            for (int d = startD; d <= max_search; ++d) {
              if (search_3(s3_views, s3_huge, d, 18)) {
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
        xxxc_min[r] = current_best;
      }
      all_results.insert(all_results.end(), xxxc_min.begin(), xxxc_min.end());
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
        int d_nb0 = get_prune(p_pt_cross_C4C5E0E1,
                              (long long)st[0].ie6_nb * StateSpace::CORNER2 +
                                  st[0].ic2_nb);
        int d_nb1 = get_prune(p_pt_cross_C4C5E0E1,
                              (long long)st[1].ie6_nb * StateSpace::CORNER2 +
                                  st[1].ic2_nb);
        int d_nb2 = get_prune(p_pt_cross_C4C5E0E1,
                              (long long)st[2].ie6_nb * StateSpace::CORNER2 +
                                  st[2].ic2_nb);
        int d_nb3 = get_prune(p_pt_cross_C4C5E0E1,
                              (long long)st[3].ie6_nb * StateSpace::CORNER2 +
                                  st[3].ic2_nb);
        int d_dg0 =
            p_pt_cross_C4C6E0E2
                ? get_prune(p_pt_cross_C4C6E0E2,
                            (long long)st[0].ie6_dg * StateSpace::CORNER2 +
                                st[0].ic2_dg)
                : 0;
        int d_dg1 =
            p_pt_cross_C4C6E0E2
                ? get_prune(p_pt_cross_C4C6E0E2,
                            (long long)st[1].ie6_dg * StateSpace::CORNER2 +
                                st[1].ic2_dg)
                : 0;
        int max_h = std::max({d_nb0, d_nb1, d_nb2, d_nb3, d_dg0, d_dg1});

        int res = 0;
        if (max_h <= 16) {
          if (max_h > 0) {
            // 跨阶段下界：F2L 解 ≥ XXXC 解
            int startD = std::max(max_h, xxxc_min[r]);
            SlotView4 views[4];
            for (int i = 0; i < 4; ++i) {
              views[i] = {st[i].im,      st[i].ic * 18, st[i].e0 * 18,
                          st[i].e2 * 18, st[i].c5 * 18, st[i].c6 * 18};
            }
            HugePair nb[4] = {{st[0].ie6_nb, st[0].ic2_nb},
                              {st[1].ie6_nb, st[1].ic2_nb},
                              {st[2].ie6_nb, st[2].ic2_nb},
                              {st[3].ie6_nb, st[3].ic2_nb}};
            HugePair dg[2] = {{st[0].ie6_dg, st[0].ic2_dg},
                              {st[1].ie6_dg, st[1].ic2_dg}};
            for (int d = startD; d <= 16; ++d) {
              if (search_4(views, nb, dg, d, 18)) {
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
  // NOTE: 标准 cubing 记号,index k 对应 z^k / x^k:
  //   _z0=identity → Y, _z1=z → R, _z2=z2 → W, _z3=z' → O, _x1=x → B, _x3=x' → G
  static inline std::vector<std::string> rots = {"",   "z",  "z2",
                                                 "z'", "x",  "x'"};

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
  static void print_stats() {
#if ENABLE_PRUNE_STATS
    printf("\n=== Std Analyzer Pruning Stats ===\n");
#if ENABLE_STATS_S1
    PRINT_STAT(s1_xcross);
#endif
#if ENABLE_STATS_S2
    PRINT_STAT(s2_xcross1);
    PRINT_STAT(s2_xcross2);
    PRINT_STAT(s2_huge);
#endif
#if ENABLE_STATS_S3
    PRINT_STAT(s3_huge1);
    PRINT_STAT(s3_huge2);
    PRINT_STAT(s3_xcross1);
    PRINT_STAT(s3_xcross2);
    PRINT_STAT(s3_xcross3);
#endif
#if ENABLE_STATS_S4
    PRINT_STAT(s4_huge1);
    PRINT_STAT(s4_huge2);
    PRINT_STAT(s4_huge3);
    PRINT_STAT(s4_xcross1);
    PRINT_STAT(s4_xcross2);
    PRINT_STAT(s4_xcross3);
    PRINT_STAT(s4_xcross4);
#endif
#endif
  }
};

int main() {
  run_analyzer_app<StdSolver>("_std");
  return 0;
}
