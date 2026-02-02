#include "analyzer_executor.h"
#include "cube_common.h"
#include "move_tables.h"
#include "prune_tables.h"

// --- 全局统计变量重定向到 AnalyzerStats ---
#define global_nodes AnalyzerStats::globalNodes
#define completed_tasks AnalyzerStats::completedTasks
#define is_solving AnalyzerStats::isSolving

// NOTE: COUNT_NODE 宏已移至 analyzer_executor.h

// --- Cross Analyzer (EO Cross) ---
struct cross_analyzer {
  // 静态成员：所有实例共享
  static inline bool s_initialized = false;
  static inline std::vector<int> s_eo_mt;
  static inline const int *s_p_multi = nullptr;
  static inline const unsigned char *s_p_prune = nullptr;

  // 实例成员（指向静态数据）
  const int *p_multi, *p_eo;
  const unsigned char *p_prune;

  static void static_init() {
    if (s_initialized)
      return;
    auto &mm = MoveTableManager::getInstance();
    auto &pm = PruneTableManager::getInstance();
    mm.loadEdgeTable();
    mm.loadEdges2Table();
    if (!load_vector(s_eo_mt, "move_table_eo_12.bin")) {
      s_eo_mt = create_eo_move_table();
      save_vector(s_eo_mt, "move_table_eo_12.bin");
    }
    pm.generateCrossPrune();
    s_p_multi = mm.getEdges2TablePtr();
    s_p_prune = pm.getCrossPrunePtr();
    s_initialized = true;
  }

  cross_analyzer() {
    // 仅复制指针引用
    p_multi = s_p_multi;
    p_eo = s_eo_mt.data();
    p_prune = s_p_prune;
  }

  void get_indices_sym(const std::vector<int> &alg, int sym_idx, int &i1,
                       int &i2, int &i_eo) {
    i1 = 416;
    i2 = 520;
    i_eo = 0;
    for (int m : alg) {
      int conj_m = sym_moves_flat[m][sym_idx];
      i1 = p_multi[i1 * 18 + conj_m];
      i2 = p_multi[i2 * 18 + conj_m];
      i_eo = p_eo[i_eo + conj_m];
    }
  }

  bool search(int i1, int i2, int i_eo, int depth, int prev, int &found_len,
              int max_d) {
    const int *moves = valid_moves_flat[prev];
    const int count = valid_moves_count[prev];

    for (int k = 0; k < count; ++k) {
      COUNT_NODE
      int m = moves[k];

      // 级联: 先 Cross 查表
      int n1 = p_multi[i1 + m], n2 = p_multi[i2 + m];
      long long idx = (long long)n1 * 528 + n2;
      int pr = get_prune_ptr(p_prune, idx);
      if (pr >= depth)
        continue;

      // 后 EO 计算
      int neo = p_eo[i_eo + m];

      if (depth == 1) {
        if (pr == 0 && neo == 0) {
          found_len = max_d;
          return true;
        }
      } else if (search(n1 * 18, n2 * 18, neo, depth - 1, m, found_len, max_d))
        return true;
    }
    return false;
  }

  std::vector<int> get_stats(const std::vector<int> &base_alg) {
    std::vector<int> res(12, 99);
    std::vector<std::pair<int, int>> tasks;
    for (int s = 0; s < 12; ++s) {
      int i1, i2, ieo;
      get_indices_sym(base_alg, s, i1, i2, ieo);
      long long idx = (long long)i1 * 528 + i2;
      int h = get_prune_ptr(p_prune, idx);
      if (h == 0 && ieo != 0)
        h = 1;
      if (h == 0 && ieo == 0) {
        res[s] = 0;
        continue;
      }
      tasks.push_back({h, s});
    }
    std::sort(tasks.begin(), tasks.end());
    for (auto &t : tasks) {
      int s = t.second;
      int i1, i2, ieo;
      get_indices_sym(base_alg, s, i1, i2, ieo);
      int found = 99;
      for (int d = t.first; d <= 12; ++d) {
        if (search(i1 * 18, i2 * 18, ieo, d, 18, found, d)) {
          res[s] = found;
          break;
        }
      }
    }
    return res;
  }
};

// --- XCross Analyzer (Optimized with Symmetry + Single Base Tables) ---
struct xcross_analyzer {
  // 静态成员：所有实例共享
  static inline bool s_initialized = false;
  static inline std::vector<int> s_dep_mt, s_eo_mt;
  static inline std::vector<unsigned char> s_prune_t, s_prune_dep_eo;
  static inline std::vector<unsigned char> s_prune_xcross_base;
  static inline std::vector<unsigned char> s_prune_3c;

  static inline const int *s_p_multi = nullptr;
  static inline const int *s_p_corner = nullptr;
  static inline const int *s_p_edge = nullptr;
  static inline const int *s_p_edge6 =
      nullptr; // Edge6 Move Table (用于 Huge 表)
  static inline const int *s_p_corn2 =
      nullptr; // Corner2 Move Table (用于 Huge 表)
  static inline const unsigned char *s_p_prune = nullptr;
  static inline const unsigned char *s_p_prune_dep_eo = nullptr;
  static inline const unsigned char *s_p_prune_base = nullptr;
  static inline const unsigned char *s_p_prune_3c = nullptr;
  static inline const unsigned char *s_p_huge_neighbor =
      nullptr; // Huge Neighbor 表
  static inline const unsigned char *s_p_huge_diagonal =
      nullptr; // Huge Diagonal 表

  // 实例成员（指向静态数据）
  const int *p_multi, *p_corner, *p_edge, *p_dep, *p_eo;
  const int *p_edge6 = nullptr, *p_corn2 = nullptr; // Edge6/Corner2 Move Tables
  const unsigned char *p_prune, *p_prune_dep_eo;
  const unsigned char *p_prune_base = nullptr;
  const unsigned char *p_prune_3c = nullptr;
  const unsigned char *p_huge_neighbor = nullptr; // Huge Neighbor 表
  const unsigned char *p_huge_diagonal = nullptr; // Huge Diagonal 表

  const int SOLVED_MULTI = 187520 * 24;
  const int SOLVED_CORNER = 12;
  const int SOLVED_EDGE = 0;

  static void static_init() {
    if (s_initialized)
      return;

    MoveTableManager &mm = MoveTableManager::getInstance();
    mm.loadEdgeTable();
    mm.loadCornerTable();
    mm.loadCrossTable();
    mm.loadEdge6Table();   // 用于 Huge 表状态追踪
    mm.loadCorner2Table(); // 用于 Huge 表状态追踪

    std::vector<int> ep_mt;
    if (!load_vector(ep_mt, "move_table_ep_1.bin")) {
      ep_mt = create_ep_move_table();
      save_vector(ep_mt, "move_table_ep_1.bin");
    }
    if (!load_vector(s_eo_mt, "move_table_eo_12_alt.bin")) {
      s_eo_mt = create_eo_move_table2();
      save_vector(s_eo_mt, "move_table_eo_12_alt.bin");
    }
    if (!load_vector(s_dep_mt, "move_table_ep_4.bin")) {
      s_dep_mt = create_multi_move_table(4, 1, 12, 12 * 11 * 10 * 9, ep_mt);
      save_vector(s_dep_mt, "move_table_ep_4.bin");
    }

    const char *pn_c4 = "prune_table_cross_C4.bin";
    if (!load_vector(s_prune_t, pn_c4)) {
      std::cout << "  Generating " << pn_c4 << " ..." << std::endl;
      create_cascaded_prune_table2(187520, 12, 24 * 22 * 20 * 18, 24, 10,
                                   mm.getCrossTable(), mm.getCornerTable(),
                                   s_prune_t);
      save_vector(s_prune_t, pn_c4);
    }
    s_p_prune = s_prune_t.data();

    if (!load_vector(s_prune_dep_eo, "prune_table_ep_4_eo_12.bin")) {
      std::cout << "  Generating prune_table_ep_4_eo_12.bin ..." << std::endl;
      create_cascaded_prune_table3(11720, 0, 12 * 11 * 10 * 9, 2048, 11,
                                   s_dep_mt, s_eo_mt, s_prune_dep_eo);
      save_vector(s_prune_dep_eo, "prune_table_ep_4_eo_12.bin");
    }

    s_p_multi = mm.getCrossTablePtr();
    s_p_corner = mm.getCornerTablePtr();
    s_p_edge = mm.getEdgeTablePtr();
    s_p_edge6 = mm.getEdge6TablePtr();   // Edge6 Move Table
    s_p_corn2 = mm.getCorner2TablePtr(); // Corner2 Move Table
    s_p_prune_dep_eo = s_prune_dep_eo.data();

    // 加载 Huge Neighbor/Diagonal Prune Tables
    auto &ptm = PruneTableManager::getInstance();
    ptm.generateHugeNeighborPrune();
    s_p_huge_neighbor = ptm.getHugeNeighborPrunePtr();
    if (ENABLE_DIAGONAL_TABLE) {
      ptm.generateHugeDiagonalPrune();
      s_p_huge_diagonal = ptm.getHugeDiagonalPrunePtr();
    }

    // 1. 基准 XCross Table (C4 + E0)
    std::cout << "[Init] Checking Base XCross Table (C4 + E0)..." << std::endl;
    std::string fn_base = "prune_table_cross_C4_E0.bin";
    if (!load_vector(s_prune_xcross_base, fn_base)) {
      std::cout << "  Generating " << fn_base << " ..." << std::endl;
      create_prune_table_xcross_full(
          187520, 12, 0, 24 * 22 * 20 * 18, 24, 24, 11, mm.getCrossTable(),
          mm.getCornerTable(), mm.getEdgeTable(), s_prune_xcross_base);
      save_vector(s_prune_xcross_base, fn_base);
    }
    s_p_prune_base = s_prune_xcross_base.data();

    // NOTE: Plus Tables (E1/E2/E3/C5/C6/C7) 已移除，改用 Huge Neighbor/Diagonal
    // 表

    // 3. 3-Corner Table (C4+C5+C6)
    std::cout << "[Init] Checking XCross+C4+C5+C6 Table..." << std::endl;
    std::string fn_3c = "prune_table_cross_C4_C5_C6.bin";
    if (!load_vector(s_prune_3c, fn_3c)) {
      std::cout << "  Generating " << fn_3c << " (Depth 14) ..." << std::endl;
      int c5 = 15;
      int c6 = 18;
      create_prune_table_xcross_corn3(187520, 12, c5, c6, 24 * 22 * 20 * 18, 24,
                                      24, 24, 14, mm.getCrossTable(),
                                      mm.getCornerTable(), mm.getCornerTable(),
                                      mm.getCornerTable(), s_prune_3c);
      save_vector(s_prune_3c, fn_3c);
    }
    s_p_prune_3c = s_prune_3c.data();

    s_initialized = true;
  }

  xcross_analyzer() {
    // 仅复制指针引用
    p_multi = s_p_multi;
    p_corner = s_p_corner;
    p_edge = s_p_edge;
    p_edge6 = s_p_edge6; // Edge6 Move Table
    p_corn2 = s_p_corn2; // Corner2 Move Table
    p_dep = s_dep_mt.data();
    p_eo = s_eo_mt.data();
    p_prune = s_p_prune;
    p_prune_dep_eo = s_p_prune_dep_eo;
    p_prune_base = s_p_prune_base;
    p_prune_3c = s_p_prune_3c;
    p_huge_neighbor = s_p_huge_neighbor; // Huge Neighbor 表
    p_huge_diagonal = s_p_huge_diagonal; // Huge Diagonal 表
  }

  // NOTE: get_plus_table_idx 已移除，Plus Tables 不再使用

  // 判断两个 slot 是否为相邻关系，返回用于 Huge Neighbor 表的 Conj View
  // 返回值: 应作为 conj 基准的 slot，-1 表示非相邻
  inline int get_neighbor_view(int s1, int s2) {
    if ((s2 - s1 + 4) % 4 == 1)
      return s1; // s2 是 s1 的右邻 → View=s1
    if ((s1 - s2 + 4) % 4 == 1)
      return s2; // s1 是 s2 的右邻 → View=s2
    return -1;   // 非相邻
  }

  // 判断两个 slot 是否为对角关系，返回用于 Huge Diagonal 表的 Conj View
  // 返回值: 应作为 conj 基准的 slot，-1 表示非对角
  inline int get_diagonal_view(int s1, int s2) {
    int mn = std::min(s1, s2), mx = std::max(s1, s2);
    if (mn == 0 && mx == 2)
      return 0; // (0,2) 对角 → View=0
    if (mn == 1 && mx == 3)
      return 1; // (1,3) 对角 → View=1
    return -1;  // 非对角
  }

  void get_indices_conj_full(const std::vector<int> &alg, int sym_idx,
                             int slot_idx, int &i1, int &i2, int &i3,
                             int &i_dep, int &i_eo, int *track_e, int *track_c,
                             int &i_e6_nb, int &i_c2_nb, // Huge Neighbor 状态
                             int &i_e6_dg, int &i_c2_dg) { // Huge Diagonal 状态
    i1 = SOLVED_MULTI;
    i2 = SOLVED_CORNER;
    i3 = SOLVED_EDGE;
    i_dep = 11720;
    i_eo = 0;

    track_e[0] = 2;
    track_e[1] = 4;
    track_e[2] = 6;
    track_c[0] = 15;
    track_c[1] = 18;
    track_c[2] = 21;

    // 初始化 Huge 表索引 (参考 std_analyzer.cpp)
    static int solved_e6_nb = -1, solved_c2_nb = -1;
    static int solved_e6_dg = -1, solved_c2_dg = -1;
    if (solved_e6_nb == -1) {
      solved_e6_nb = array_to_index({0, 2, 16, 18, 20, 22}, 6, 2, 12);
      solved_c2_nb = array_to_index({12, 15}, 2, 3, 8);
      solved_e6_dg = array_to_index({0, 4, 16, 18, 20, 22}, 6, 2, 12);
      solved_c2_dg = array_to_index({12, 18}, 2, 3, 8);
    }
    int cur_e6_nb = solved_e6_nb, cur_c2_nb = solved_c2_nb;
    int cur_e6_dg = solved_e6_dg, cur_c2_dg = solved_c2_dg;

    for (int m : alg) {
      int m_global = sym_moves_flat[m][sym_idx];
      int m_slot = conj_moves_flat[m_global][slot_idx];

      i1 = p_multi[i1 + m_slot];
      i2 = p_corner[i2 * 18 + m_slot];
      i3 = p_edge[i3 * 18 + m_slot];

      i_dep = p_dep[i_dep * 18 + m_global];
      i_eo = p_eo[i_eo * 18 + m_global];

      for (int k = 0; k < 3; ++k) {
        track_e[k] = p_edge[track_e[k] * 18 + m_slot];
        track_c[k] = p_corner[track_c[k] * 18 + m_slot];
      }

      // 追踪 Huge 表状态
      cur_e6_nb = p_edge6[cur_e6_nb * 18 + m_slot];
      cur_c2_nb = p_corn2[cur_c2_nb * 18 + m_slot];
      cur_e6_dg = p_edge6[cur_e6_dg * 18 + m_slot];
      cur_c2_dg = p_corn2[cur_c2_dg * 18 + m_slot];
    }

    i_e6_nb = cur_e6_nb;
    i_c2_nb = cur_c2_nb;
    i_e6_dg = cur_e6_dg;
    i_c2_dg = cur_c2_dg;
  }

  // --- Search 1: XCross+EO (Optimized) ---
  bool search_1(int i1, int i2, int i3, int i_dep, int i_eo, int depth,
                int prev, int slot, int bound) {
    if (depth > bound)
      return false;

    const int *moves = valid_moves_flat[prev];
    const int count = valid_moves_count[prev];

    for (int k = 0; k < count; ++k) {
      COUNT_NODE
      int m = moves[k];

      // 级联 Check 1: Dependency (EO + Partial Cross)
      int nd = p_dep[i_dep + m], neo = p_eo[i_eo + m];
      if (get_prune_ptr(p_prune_dep_eo, (long long)nd * 2048 + neo) >= depth)
        continue;

      // 级联 Check 2: Main XCross
      int m_slot = conj_moves_flat[m][slot];
      int n1 = p_multi[i1 + m_slot], n2 = p_corner[i2 + m_slot],
          n3 = p_edge[i3 + m_slot];
      long long idx_xc = (long long)(n1 + n2) * 24 + n3;
      if (get_prune_ptr(p_prune_base, idx_xc) >= depth)
        continue;

      if (depth == 1)
        return true;
      else if (search_1(n1, n2 * 18, n3 * 18, nd * 18, neo * 18, depth - 1, m,
                        slot, bound))
        return true;
    }
    return false;
  }

  // --- Search 2: XXCross+EO (Optimized) ---
  // NOTE: Plus Tables 已移除，仅使用 Huge 表 + Base 表剪枝
  bool search_2(int i1a, int i2a, int i3a, int i1b, int i2b, int i3b, int i_dep,
                int i_eo, int depth, int prev, int s1, int s2, int bound,
                // Huge 表参数
                int v_huge, const unsigned char *p_huge_active, int i_e6,
                int i_c2) {
    if (depth > bound)
      return false;

    const int *moves = valid_moves_flat[prev];
    const int count = valid_moves_count[prev];

    for (int k = 0; k < count; ++k) {
      COUNT_NODE
      int m = moves[k];

      // 级联 Check 0: Huge 表 (最前置，剪枝力最强)
      int n_ie6 = -1, n_ic2 = -1;
      if (v_huge != -1 && p_huge_active) {
        int mv = conj_moves_flat[m][v_huge];
        n_ie6 = p_edge6[i_e6 * 18 + mv];
        n_ic2 = p_corn2[i_c2 * 18 + mv];
        if (get_prune_ptr(p_huge_active, (long long)n_ie6 * 504 + n_ic2) >=
            depth)
          continue;
      }

      // 级联 Check 1: Dep + EO
      int nd = p_dep[i_dep + m], neo = p_eo[i_eo + m];
      if (get_prune_ptr(p_prune_dep_eo, (long long)nd * 2048 + neo) >= depth)
        continue;

      // 级联 Check 2: View A (Base only)
      int m1 = conj_moves_flat[m][s1];
      int n1a = p_multi[i1a + m1], n2a = p_corner[i2a + m1],
          n3a = p_edge[i3a + m1];
      long long idx_a = (long long)(n1a + n2a) * 24 + n3a;
      if (get_prune_ptr(p_prune_base, idx_a) >= depth)
        continue;

      // 级联 Check 3: View B (Base only)
      int m2 = conj_moves_flat[m][s2];
      int n1b = p_multi[i1b + m2], n2b = p_corner[i2b + m2],
          n3b = p_edge[i3b + m2];
      long long idx_b = (long long)(n1b + n2b) * 24 + n3b;
      if (get_prune_ptr(p_prune_base, idx_b) >= depth)
        continue;

      if (depth == 1)
        return true;
      else if (search_2(n1a, n2a * 18, n3a * 18, n1b, n2b * 18, n3b * 18,
                        nd * 18, neo * 18, depth - 1, m, s1, s2, bound, v_huge,
                        p_huge_active, (v_huge != -1) ? n_ie6 : -1,
                        (v_huge != -1) ? n_ic2 : -1))
        return true;
    }
    return false;
  }

  // --- Search 3: XXXCross+EO (Optimized) ---
  // NOTE: Plus Tables 已移除，仅使用 Huge 表 + Base 表 + 3-Corner 表剪枝
  bool search_3(int i1a, int i2a, int i3a, int i1b, int i2b, int i3b, int i1c,
                int i2c, int i3c, int i_dep, int i_eo, int depth, int prev,
                int s1, int s2, int s3, int bound,
                // Huge 表参数
                int v_huge, const unsigned char *p_huge_active, int i_e6,
                int i_c2) {
    if (depth > bound)
      return false;

    const int *moves = valid_moves_flat[prev];
    const int count = valid_moves_count[prev];

    for (int k = 0; k < count; ++k) {
      COUNT_NODE
      int m = moves[k];

      // 级联 Check 0: Huge 表 (最前置)
      int n_ie6 = -1, n_ic2 = -1;
      if (v_huge != -1 && p_huge_active) {
        int mv = conj_moves_flat[m][v_huge];
        n_ie6 = p_edge6[i_e6 * 18 + mv];
        n_ic2 = p_corn2[i_c2 * 18 + mv];
        if (get_prune_ptr(p_huge_active, (long long)n_ie6 * 504 + n_ic2) >=
            depth)
          continue;
      }

      // 级联 Check 1: Dep + EO
      int nd = p_dep[i_dep + m], neo = p_eo[i_eo + m];
      if (get_prune_ptr(p_prune_dep_eo, (long long)nd * 2048 + neo) >= depth)
        continue;

      // --- View A (Base only) ---
      int m1 = conj_moves_flat[m][s1];
      int n1a = p_multi[i1a + m1], n2a = p_corner[i2a + m1],
          n3a = p_edge[i3a + m1];
      long long idx_a = (long long)(n1a + n2a) * 24 + n3a;
      if (get_prune_ptr(p_prune_base, idx_a) >= depth)
        continue;

      // --- View B (Base only) ---
      int m2 = conj_moves_flat[m][s2];
      int n1b = p_multi[i1b + m2], n2b = p_corner[i2b + m2],
          n3b = p_edge[i3b + m2];
      long long idx_b = (long long)(n1b + n2b) * 24 + n3b;
      if (get_prune_ptr(p_prune_base, idx_b) >= depth)
        continue;

      // --- View C (Base only) ---
      int m3 = conj_moves_flat[m][s3];
      int n1c = p_multi[i1c + m3], n2c = p_corner[i2c + m3],
          n3c = p_edge[i3c + m3];
      long long idx_c = (long long)(n1c + n2c) * 24 + n3c;
      if (get_prune_ptr(p_prune_base, idx_c) >= depth)
        continue;

      if (depth == 1)
        return true;
      else if (search_3(n1a, n2a * 18, n3a * 18, n1b, n2b * 18, n3b * 18, n1c,
                        n2c * 18, n3c * 18, nd * 18, neo * 18, depth - 1, m, s1,
                        s2, s3, bound, v_huge, p_huge_active,
                        (v_huge != -1) ? n_ie6 : -1,
                        (v_huge != -1) ? n_ic2 : -1))
        return true;
    }
    return false;
  }

  std::vector<int> get_stats(const std::vector<int> &base_alg) {
    std::vector<int> res(36, 99);

    for (int sym = 0; sym < 12; ++sym) {
      // Data Prep
      struct SlotState {
        int i1, i2, i3, idep, ieo;
        int e_trk[3], c_trk[3];
        int i_e6_nb, i_c2_nb; // Huge Neighbor 状态
        int i_e6_dg, i_c2_dg; // Huge Diagonal 状态
      };
      std::vector<SlotState> st(4);
      for (int s = 0; s < 4; ++s) {
        get_indices_conj_full(base_alg, sym, s, st[s].i1, st[s].i2, st[s].i3,
                              st[s].idep, st[s].ieo, st[s].e_trk, st[s].c_trk,
                              st[s].i_e6_nb, st[s].i_c2_nb, st[s].i_e6_dg,
                              st[s].i_c2_dg);
      }

      // --- 1. XCross+EO ---
      {
        std::vector<std::pair<int, int>> tasks;
        for (int s = 0; s < 4; ++s) {
          long long idx_xc = (long long)(st[s].i1 + st[s].i2) * 24 + st[s].i3;
          int pr_xc = get_prune_ptr(p_prune_base, idx_xc);
          int pr_de = get_prune_ptr(p_prune_dep_eo,
                                    (long long)st[s].idep * 2048 + st[s].ieo);
          tasks.push_back({std::max(pr_xc, pr_de), s});
        }
        std::sort(tasks.begin(), tasks.end());

        int best = 99;
        for (auto &t : tasks) {
          if (t.first >= best)
            break;
          if (t.first == 0) {
            best = 0;
            break;
          }
          int s = t.second;
          for (int d = t.first; d <= std::min(20, best - 1); ++d) {
            if (search_1(st[s].i1, st[s].i2 * 18, st[s].i3 * 18,
                         st[s].idep * 18, st[s].ieo * 18, d, 18, s, best - 1)) {
              best = d;
              break;
            }
          }
        }
        res[sym] = best;

        // --- 2. XXCross+EO ---
        {
          int pairs[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
          std::vector<std::pair<int, int>> tasks_xx;
          for (int p = 0; p < 6; ++p) {
            int s1 = pairs[p][0], s2 = pairs[p][1];

            // View A + B (仅 Base 表)
            long long idx1 =
                (long long)(st[s1].i1 + st[s1].i2) * 24 + st[s1].i3;
            int h1 = get_prune_ptr(p_prune_base, idx1);

            long long idx2 =
                (long long)(st[s2].i1 + st[s2].i2) * 24 + st[s2].i3;
            int h2 = get_prune_ptr(p_prune_base, idx2);

            int h_de = get_prune_ptr(
                p_prune_dep_eo, (long long)st[s1].idep * 2048 + st[s1].ieo);
            int h = std::max({h1, h2, h_de});
            tasks_xx.push_back({h, p});
          }
          std::sort(tasks_xx.begin(), tasks_xx.end());

          int best_xx = 99;
          for (auto &t : tasks_xx) {
            if (t.first >= best_xx)
              break;
            if (t.first == 0) {
              best_xx = 0;
              break;
            }
            int s1 = pairs[t.second][0], s2 = pairs[t.second][1];

            for (int d = t.first; d <= std::min(20, best_xx - 1); ++d) {
              // 确定 Huge 表视角和初始状态
              int v_nb = get_neighbor_view(s1, s2);
              int v_dg = get_diagonal_view(s1, s2);

              // 选择使用 Neighbor 或 Diagonal 表
              int v_huge = (v_nb != -1) ? v_nb : v_dg;
              const unsigned char *p_huge = nullptr;
              int init_e6 = -1, init_c2 = -1;

              if (v_nb != -1 && p_huge_neighbor) {
                p_huge = p_huge_neighbor;
                init_e6 = st[v_nb].i_e6_nb;
                init_c2 = st[v_nb].i_c2_nb;
              } else if (v_dg != -1 && p_huge_diagonal) {
                p_huge = p_huge_diagonal;
                init_e6 = st[v_dg].i_e6_dg;
                init_c2 = st[v_dg].i_c2_dg;
              }

              if (search_2(st[s1].i1, st[s1].i2 * 18, st[s1].i3 * 18, st[s2].i1,
                           st[s2].i2 * 18, st[s2].i3 * 18, st[s1].idep * 18,
                           st[s1].ieo * 18, d, 18, s1, s2, best_xx - 1, v_huge,
                           p_huge, init_e6, init_c2)) {
                best_xx = d;
                break;
              }
            }
          }
          res[12 + sym] = best_xx;
        }

        // --- 3. XXXCross+EO ---
        {
          int trips[4][3] = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
          std::vector<std::pair<int, int>> tasks_xxx;
          for (int tr = 0; tr < 4; ++tr) {
            int s1 = trips[tr][0], s2 = trips[tr][1], s3 = trips[tr][2];

            // Pruning check for all 3 views (仅 Base 表)
            long long idx1 =
                (long long)(st[s1].i1 + st[s1].i2) * 24 + st[s1].i3;
            int h1 = get_prune_ptr(p_prune_base, idx1);

            long long idx2 =
                (long long)(st[s2].i1 + st[s2].i2) * 24 + st[s2].i3;
            int h2 = get_prune_ptr(p_prune_base, idx2);

            long long idx3 =
                (long long)(st[s3].i1 + st[s3].i2) * 24 + st[s3].i3;
            int h3 = get_prune_ptr(p_prune_base, idx3);

            int pr_de = get_prune_ptr(
                p_prune_dep_eo, (long long)st[s1].idep * 2048 + st[s1].ieo);
            int h = std::max({h1, h2, h3, pr_de});
            tasks_xxx.push_back({h, tr});
          }
          std::sort(tasks_xxx.begin(), tasks_xxx.end());

          int best_xxx = 99;
          for (auto &t : tasks_xxx) {
            if (t.first >= best_xxx)
              break;
            if (t.first == 0) {
              best_xxx = 0;
              break;
            }
            int s1 = trips[t.second][0], s2 = trips[t.second][1],
                s3 = trips[t.second][2];

            for (int d = t.first; d <= std::min(20, best_xxx - 1); ++d) {
              // 确定 Huge 表视角和初始状态 (选择第一对 s1-s2)
              int v_nb = get_neighbor_view(s1, s2);
              int v_dg = get_diagonal_view(s1, s2);

              int v_huge = (v_nb != -1) ? v_nb : v_dg;
              const unsigned char *p_huge = nullptr;
              int init_e6 = -1, init_c2 = -1;

              if (v_nb != -1 && p_huge_neighbor) {
                p_huge = p_huge_neighbor;
                init_e6 = st[v_nb].i_e6_nb;
                init_c2 = st[v_nb].i_c2_nb;
              } else if (v_dg != -1 && p_huge_diagonal) {
                p_huge = p_huge_diagonal;
                init_e6 = st[v_dg].i_e6_dg;
                init_c2 = st[v_dg].i_c2_dg;
              }

              if (search_3(st[s1].i1, st[s1].i2 * 18, st[s1].i3 * 18, st[s2].i1,
                           st[s2].i2 * 18, st[s2].i3 * 18, st[s3].i1,
                           st[s3].i2 * 18, st[s3].i3 * 18, st[s1].idep * 18,
                           st[s1].ieo * 18, d, 18, s1, s2, s3, best_xxx - 1,
                           v_huge, p_huge, init_e6, init_c2)) {
                best_xxx = d;
                break;
              }
            }
          }
          res[24 + sym] = best_xxx;
        }
      }
    }
    return res;
  }
};

// --- EOCrossSolverWrapper: 封装 EO Cross 求解器的统一接口 ---
struct EOCrossSolverWrapper {
  cross_analyzer crossSolver;
  xcross_analyzer xcrossSolver;

  static void global_init() {
    init_matrix();

    std::cout << ANSI_CYAN << "[INIT] " << ANSI_RESET
              << "Loading EO Cross analyzers..." << std::endl;
    // 调用静态初始化方法，加载所有表（只执行一次）
    cross_analyzer::static_init();
    xcross_analyzer::static_init();
    std::cout << ANSI_CYAN << "[INIT] " << ANSI_RESET << "System Ready."
              << std::endl;
  }

  static std::string get_csv_header() {
    std::vector<std::string> suffixes = {"_z0", "_z1", "_z2",
                                         "_z3", "_x1", "_x3"};
    std::ostringstream oss;
    oss << "id";
    for (const auto &s : suffixes)
      oss << ",eo_cross" << s;
    for (const auto &s : suffixes)
      oss << ",eo_xcross" << s;
    for (const auto &s : suffixes)
      oss << ",eo_xxcross" << s;
    for (const auto &s : suffixes)
      oss << ",eo_xxxcross" << s;
    return oss.str();
  }

  std::string solve(const std::vector<int> &alg, const std::string &id) {
    std::vector<int> cr = crossSolver.get_stats(alg);
    std::vector<int> xr = xcrossSolver.get_stats(alg);

    std::ostringstream oss;
    oss << id;
    // Cross+EO
    for (int c = 0; c < 6; ++c)
      oss << "," << std::min(cr[2 * c], cr[2 * c + 1]);
    // XC+EO
    for (int c = 0; c < 6; ++c)
      oss << "," << std::min(xr[2 * c], xr[2 * c + 1]);
    // XXC+EO
    for (int c = 0; c < 6; ++c)
      oss << "," << std::min(xr[12 + 2 * c], xr[12 + 2 * c + 1]);
    // XXXC+EO
    for (int c = 0; c < 6; ++c)
      oss << "," << std::min(xr[24 + 2 * c], xr[24 + 2 * c + 1]);

    return oss.str();
  }

  static void print_stats() {}
};

int main() {
  run_analyzer_app<EOCrossSolverWrapper>("_eo");
  return 0;
}
