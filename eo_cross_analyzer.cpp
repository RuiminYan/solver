#include "analyzer_executor.h"
#include "cube_common.h"
#include "move_tables.h"
#include "prune_tables.h"

// --- 全局统计变量重定向到 AnalyzerStats ---
#define global_nodes AnalyzerStats::globalNodes
#define completed_tasks AnalyzerStats::completedTasks
#define is_solving AnalyzerStats::isSolving

// NOTE: COUNT_NODE 宏已移至 analyzer_executor.h

// --- 剪枝统计 (通过 prune_stats.h 统一开关控制) ---
#include "prune_stats.h"

// Search 1: dep_eo + xcross
STAT_DECL(s1_dep_eo); // S1: Dependency+EO 剪枝表
STAT_DECL(s1_xcross); // S1: XCross 剪枝表

// Search 2: huge + dep_eo + xcross
STAT_DECL(s2_huge);    // S2: Huge 表
STAT_DECL(s2_dep_eo);  // S2: Dependency+EO 剪枝表
STAT_DECL(s2_xcross1); // S2: XCross 1 剪枝表
STAT_DECL(s2_xcross2); // S2: XCross 2 剪枝表

// Search 3: huge×2 + dep_eo + xcross×3
STAT_DECL(s3_huge1);   // S3: Huge 表 1
STAT_DECL(s3_huge2);   // S3: Huge 表 2
STAT_DECL(s3_dep_eo);  // S3: Dependency+EO 剪枝表
STAT_DECL(s3_xcross1); // S3: XCross 1 剪枝表
STAT_DECL(s3_xcross2); // S3: XCross 2 剪枝表
STAT_DECL(s3_xcross3); // S3: XCross 3 剪枝表

// Search 4: huge×3 + dep_eo + xcross×4
STAT_DECL(s4_huge1);   // S4: Huge 表 1
STAT_DECL(s4_huge2);   // S4: Huge 表 2
STAT_DECL(s4_huge3);   // S4: Huge 表 3
STAT_DECL(s4_dep_eo);  // S4: Dependency+EO 剪枝表
STAT_DECL(s4_xcross1); // S4: XCross 1 剪枝表
STAT_DECL(s4_xcross2); // S4: XCross 2 剪枝表
STAT_DECL(s4_xcross3); // S4: XCross 3 剪枝表
STAT_DECL(s4_xcross4); // S4: XCross 4 剪枝表

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
  // NOTE: 移动表暂保留在此（阶段2迁移到MoveTableManager）
  static inline std::vector<int> s_dep_mt, s_eo_mt;
  // NOTE: 剪枝表已迁移到PruneTableManager，此处仅保留指针

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
  static inline std::vector<const unsigned char *> s_p_plus_edge;
  static inline std::vector<const unsigned char *> s_p_plus_corn;
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
  std::vector<const unsigned char *> p_plus_edge;
  std::vector<const unsigned char *> p_plus_corn;
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

    // 设置移动表指针
    s_p_multi = mm.getCrossTablePtr();
    s_p_corner = mm.getCornerTablePtr();
    s_p_edge = mm.getEdgeTablePtr();
    s_p_edge6 = mm.getEdge6TablePtr();   // Edge6 Move Table
    s_p_corn2 = mm.getCorner2TablePtr(); // Corner2 Move Table

    // === 剪枝表：使用 PruneTableManager ===
    auto &ptm = PruneTableManager::getInstance();

    // 先加载复用的xcross_c4_e0表（与std_analyzer/pair_analyzer共享）
    ptm.generateXCrossC4E0Prune();

    // 加载 EOCross 专用剪枝表
    ptm.loadEOCrossTables();

    // 获取剪枝表指针
    s_p_prune = ptm.getEOCrossC4PrunePtr();
    s_p_prune_dep_eo = ptm.getEODepEOPrunePtr();
    s_p_prune_base = ptm.getXCrossC4E0PrunePtr(); // 复用已有表

    s_p_plus_edge.resize(3);
    s_p_plus_corn.resize(3);
    for (int i = 0; i < 3; ++i) {
      s_p_plus_edge[i] = ptm.getEOPlusEdgePrunePtr(i);
      s_p_plus_corn[i] = ptm.getEOPlusCornerPrunePtr(i);
    }
    s_p_prune_3c = ptm.getEO3CornerPrunePtr();

    // 加载 Huge Neighbor/Diagonal Prune Tables
    ptm.generateHugeNeighborPrune();
    s_p_huge_neighbor = ptm.getHugeNeighborPrunePtr();
    if (ENABLE_DIAGONAL_EO_CROSS) {
      ptm.generateHugeDiagonalPrune();
      s_p_huge_diagonal = ptm.getHugeDiagonalPrunePtr();
    }

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
    p_plus_edge = s_p_plus_edge;
    p_plus_corn = s_p_plus_corn;
    p_prune_3c = s_p_prune_3c;
    p_huge_neighbor = s_p_huge_neighbor; // Huge Neighbor 表
    p_huge_diagonal = s_p_huge_diagonal; // Huge Diagonal 表
  }

  inline int get_plus_table_idx(int s_base, int s_target) {
    int diff = (s_target - s_base + 4) % 4;
    if (diff == 1)
      return 0; // Right
    if (diff == 2)
      return 1; // Diag
    if (diff == 3)
      return 2; // Left
    return -1;
  }

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
      S1_CHECK(s1_dep_eo);
      if (get_prune_ptr(p_prune_dep_eo, (long long)nd * 2048 + neo) >= depth) {
        S1_HIT(s1_dep_eo);
        continue;
      }

      // 级联 Check 2: Main XCross
      int m_slot = conj_moves_flat[m][slot];
      int n1 = p_multi[i1 + m_slot], n2 = p_corner[i2 + m_slot],
          n3 = p_edge[i3 + m_slot];
      long long idx_xc = (long long)(n1 + n2) * 24 + n3;
      S1_CHECK(s1_xcross);
      if (get_prune_ptr(p_prune_base, idx_xc) >= depth) {
        S1_HIT(s1_xcross);
        continue;
      }

      if (depth == 1)
        return true;
      else if (search_1(n1, n2 * 18, n3 * 18, nd * 18, neo * 18, depth - 1, m,
                        slot, bound))
        return true;
    }
    return false;
  }

  // --- Search 2: XXCross+EO (Optimized) ---
  bool search_2(int i1a, int i2a, int i3a, int i1b, int i2b, int i3b, int i_dep,
                int i_eo, int depth, int prev, int s1, int s2, int bound,
                int tab, int tba, int ea_rel, int ca_rel, int eb_rel,
                int cb_rel,
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

      // 级联 Check 2: View A (Base + Plus)
      int m1 = conj_moves_flat[m][s1];
      int n1a = p_multi[i1a + m1], n2a = p_corner[i2a + m1],
          n3a = p_edge[i3a + m1];
      long long idx_a = (long long)(n1a + n2a) * 24 + n3a;
      if (get_prune_ptr(p_prune_base, idx_a) >= depth)
        continue;

      int n_ea_rel = p_edge[ea_rel * 18 + m1];
      if (get_prune_ptr(p_plus_edge[tab], idx_a * 24 + n_ea_rel) >= depth)
        continue;
      int n_ca_rel = p_corner[ca_rel * 18 + m1];
      if (get_prune_ptr(p_plus_corn[tab], idx_a * 24 + n_ca_rel) >= depth)
        continue;

      // 级联 Check 3: View B (Base + Plus)
      int m2 = conj_moves_flat[m][s2];
      int n1b = p_multi[i1b + m2], n2b = p_corner[i2b + m2],
          n3b = p_edge[i3b + m2];
      long long idx_b = (long long)(n1b + n2b) * 24 + n3b;
      if (get_prune_ptr(p_prune_base, idx_b) >= depth)
        continue;

      int n_eb_rel = p_edge[eb_rel * 18 + m2];
      if (get_prune_ptr(p_plus_edge[tba], idx_b * 24 + n_eb_rel) >= depth)
        continue;
      int n_cb_rel = p_corner[cb_rel * 18 + m2];
      if (get_prune_ptr(p_plus_corn[tba], idx_b * 24 + n_cb_rel) >= depth)
        continue;

      if (depth == 1)
        return true;
      else if (search_2(n1a, n2a * 18, n3a * 18, n1b, n2b * 18, n3b * 18,
                        nd * 18, neo * 18, depth - 1, m, s1, s2, bound, tab,
                        tba, n_ea_rel, n_ca_rel, n_eb_rel, n_cb_rel, v_huge,
                        p_huge_active, (v_huge != -1) ? n_ie6 : -1,
                        (v_huge != -1) ? n_ic2 : -1))
        return true;
    }
    return false;
  }

  // --- Search 3: XXXCross+EO (Optimized) ---
  bool search_3(int i1a, int i2a, int i3a, int i1b, int i2b, int i3b, int i1c,
                int i2c, int i3c, int i_dep, int i_eo, int depth, int prev,
                int s1, int s2, int s3, int bound, int t_ab, int t_ba, int t_bc,
                int t_cb, int t_ac, int t_ca, int ea_b, int ca_b, int ea_c,
                int ca_c, int eb_a, int cb_a, int eb_c, int cb_c, int ec_a,
                int cc_a, int ec_b, int cc_b,
                // Huge 表参数 (仅追踪第一对 s1-s2)
                int v_huge, const unsigned char *p_huge_active, int i_e6,
                int i_c2) {
    if (depth > bound)
      return false;

    bool check_3c_A = (t_ab == 0 && t_ac == 1);
    bool check_3c_B = (t_ba == 0 && t_bc == 1);
    bool check_3c_C = (t_ca == 0 && t_cb == 1);

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

      // --- View A ---
      int m1 = conj_moves_flat[m][s1];
      int n1a = p_multi[i1a + m1], n2a = p_corner[i2a + m1],
          n3a = p_edge[i3a + m1];
      long long idx_a = (long long)(n1a + n2a) * 24 + n3a;
      if (get_prune_ptr(p_prune_base, idx_a) >= depth)
        continue;

      int n_ea_b = p_edge[ea_b * 18 + m1], n_ca_b = p_corner[ca_b * 18 + m1];
      if (get_prune_ptr(p_plus_edge[t_ab], idx_a * 24 + n_ea_b) >= depth)
        continue;
      if (get_prune_ptr(p_plus_corn[t_ab], idx_a * 24 + n_ca_b) >= depth)
        continue;

      int n_ea_c = p_edge[ea_c * 18 + m1], n_ca_c = p_corner[ca_c * 18 + m1];
      if (get_prune_ptr(p_plus_edge[t_ac], idx_a * 24 + n_ea_c) >= depth)
        continue;
      if (get_prune_ptr(p_plus_corn[t_ac], idx_a * 24 + n_ca_c) >= depth)
        continue;

      if (check_3c_A) {
        long long idx_3c = ((long long)(n1a + n2a) * 24 + n_ca_b) * 24 + n_ca_c;
        if (get_prune_ptr(p_prune_3c, idx_3c) >= depth)
          continue;
      }

      // --- View B ---
      int m2 = conj_moves_flat[m][s2];
      int n1b = p_multi[i1b + m2], n2b = p_corner[i2b + m2],
          n3b = p_edge[i3b + m2];
      long long idx_b = (long long)(n1b + n2b) * 24 + n3b;
      if (get_prune_ptr(p_prune_base, idx_b) >= depth)
        continue;

      int n_eb_a = p_edge[eb_a * 18 + m2], n_cb_a = p_corner[cb_a * 18 + m2];
      if (get_prune_ptr(p_plus_edge[t_ba], idx_b * 24 + n_eb_a) >= depth)
        continue;
      if (get_prune_ptr(p_plus_corn[t_ba], idx_b * 24 + n_cb_a) >= depth)
        continue;

      int n_eb_c = p_edge[eb_c * 18 + m2], n_cb_c = p_corner[cb_c * 18 + m2];
      if (get_prune_ptr(p_plus_edge[t_bc], idx_b * 24 + n_eb_c) >= depth)
        continue;
      if (get_prune_ptr(p_plus_corn[t_bc], idx_b * 24 + n_cb_c) >= depth)
        continue;

      if (check_3c_B) {
        long long idx_3c = ((long long)(n1b + n2b) * 24 + n_cb_a) * 24 + n_cb_c;
        if (get_prune_ptr(p_prune_3c, idx_3c) >= depth)
          continue;
      }

      // --- View C ---
      int m3 = conj_moves_flat[m][s3];
      int n1c = p_multi[i1c + m3], n2c = p_corner[i2c + m3],
          n3c = p_edge[i3c + m3];
      long long idx_c = (long long)(n1c + n2c) * 24 + n3c;
      if (get_prune_ptr(p_prune_base, idx_c) >= depth)
        continue;

      int n_ec_a = p_edge[ec_a * 18 + m3], n_cc_a = p_corner[cc_a * 18 + m3];
      if (get_prune_ptr(p_plus_edge[t_ca], idx_c * 24 + n_ec_a) >= depth)
        continue;
      if (get_prune_ptr(p_plus_corn[t_ca], idx_c * 24 + n_cc_a) >= depth)
        continue;

      int n_ec_b = p_edge[ec_b * 18 + m3], n_cc_b = p_corner[cc_b * 18 + m3];
      if (get_prune_ptr(p_plus_edge[t_cb], idx_c * 24 + n_ec_b) >= depth)
        continue;
      if (get_prune_ptr(p_plus_corn[t_cb], idx_c * 24 + n_cc_b) >= depth)
        continue;

      if (check_3c_C) {
        long long idx_3c = ((long long)(n1c + n2c) * 24 + n_cc_a) * 24 + n_cc_b;
        if (get_prune_ptr(p_prune_3c, idx_3c) >= depth)
          continue;
      }

      if (depth == 1)
        return true;
      else if (search_3(n1a, n2a * 18, n3a * 18, n1b, n2b * 18, n3b * 18, n1c,
                        n2c * 18, n3c * 18, nd * 18, neo * 18, depth - 1, m, s1,
                        s2, s3, bound, t_ab, t_ba, t_bc, t_cb, t_ac, t_ca,
                        n_ea_b, n_ca_b, n_ea_c, n_ca_c, n_eb_a, n_cb_a, n_eb_c,
                        n_cb_c, n_ec_a, n_cc_a, n_ec_b, n_cc_b, v_huge,
                        p_huge_active, (v_huge != -1) ? n_ie6 : -1,
                        (v_huge != -1) ? n_ic2 : -1))
        return true;
    }
    return false;
  }

  // --- Search 4: XXXXCross+EO (优化版，与 search_3 剪枝策略一致) ---
  // 4 个视角，每个视角检查对其他 3 个槽位的 Plus 表 + 3-Corner 表
  bool search_4(
      // 4 个视角的状态
      int i1_a, int i2_a, int i3_a, int i1_b, int i2_b, int i3_b, int i1_c,
      int i2_c, int i3_c, int i1_d, int i2_d, int i3_d,
      // 全局约束
      int i_dep, int i_eo, int depth, int prev, int bound,
      // 追踪状态：每视角对其他 3 个槽位的 Edge/Corner
      // View A (s0): 对 s1,s2,s3
      int ea_1, int ca_1, int ea_2, int ca_2, int ea_3, int ca_3,
      // View B (s1): 对 s0,s2,s3
      int eb_0, int cb_0, int eb_2, int cb_2, int eb_3, int cb_3,
      // View C (s2): 对 s0,s1,s3
      int ec_0, int cc_0, int ec_1, int cc_1, int ec_3, int cc_3,
      // View D (s3): 对 s0,s1,s2
      int ed_0, int cd_0, int ed_1, int cd_1, int ed_2, int cd_2,
      // Huge 表参数
      int v_huge, const unsigned char *p_huge_active, int i_e6, int i_c2) {
    if (depth > bound)
      return false;

    const int *moves = valid_moves_flat[prev];
    const int count = valid_moves_count[prev];

    for (int k = 0; k < count; ++k) {
      COUNT_NODE
      int m = moves[k];

      // --- Check 0: Huge 表 (最前置) ---
      int n_ie6 = -1, n_ic2 = -1;
      if (v_huge != -1 && p_huge_active) {
        int mv = conj_moves_flat[m][v_huge];
        n_ie6 = p_edge6[i_e6 * 18 + mv];
        n_ic2 = p_corn2[i_c2 * 18 + mv];
        if (get_prune_ptr(p_huge_active, (long long)n_ie6 * 504 + n_ic2) >=
            depth)
          continue;
      }

      // --- Check 1: Dep + EO ---
      int nd = p_dep[i_dep + m], neo = p_eo[i_eo + m];
      if (get_prune_ptr(p_prune_dep_eo, (long long)nd * 2048 + neo) >= depth)
        continue;

      // --- View A (s0): 对 s1(Right), s2(Diag), s3(Left) ---
      int m0 = conj_moves_flat[m][0];
      int n1a = p_multi[i1_a + m0], n2a = p_corner[i2_a + m0],
          n3a = p_edge[i3_a + m0];
      long long idx_a = (long long)(n1a + n2a) * 24 + n3a;
      if (get_prune_ptr(p_prune_base, idx_a) >= depth)
        continue;

      int n_ea_1 = p_edge[ea_1 * 18 + m0], n_ca_1 = p_corner[ca_1 * 18 + m0];
      if (get_prune_ptr(p_plus_edge[0], idx_a * 24 + n_ea_1) >= depth)
        continue; // s1: Right
      if (get_prune_ptr(p_plus_corn[0], idx_a * 24 + n_ca_1) >= depth)
        continue;

      int n_ea_2 = p_edge[ea_2 * 18 + m0], n_ca_2 = p_corner[ca_2 * 18 + m0];
      if (get_prune_ptr(p_plus_edge[1], idx_a * 24 + n_ea_2) >= depth)
        continue; // s2: Diag
      if (get_prune_ptr(p_plus_corn[1], idx_a * 24 + n_ca_2) >= depth)
        continue;

      int n_ea_3 = p_edge[ea_3 * 18 + m0], n_ca_3 = p_corner[ca_3 * 18 + m0];
      if (get_prune_ptr(p_plus_edge[2], idx_a * 24 + n_ea_3) >= depth)
        continue; // s3: Left
      if (get_prune_ptr(p_plus_corn[2], idx_a * 24 + n_ca_3) >= depth)
        continue;

      // 3-Corner: s1(Right) + s2(Diag) → 始终满足
      long long idx_3c_a = ((long long)(n1a + n2a) * 24 + n_ca_1) * 24 + n_ca_2;
      if (get_prune_ptr(p_prune_3c, idx_3c_a) >= depth)
        continue;

      // --- View B (s1): 对 s0(Left), s2(Right), s3(Diag) ---
      int m1 = conj_moves_flat[m][1];
      int n1b = p_multi[i1_b + m1], n2b = p_corner[i2_b + m1],
          n3b = p_edge[i3_b + m1];
      long long idx_b = (long long)(n1b + n2b) * 24 + n3b;
      if (get_prune_ptr(p_prune_base, idx_b) >= depth)
        continue;

      int n_eb_0 = p_edge[eb_0 * 18 + m1], n_cb_0 = p_corner[cb_0 * 18 + m1];
      if (get_prune_ptr(p_plus_edge[2], idx_b * 24 + n_eb_0) >= depth)
        continue; // s0: Left
      if (get_prune_ptr(p_plus_corn[2], idx_b * 24 + n_cb_0) >= depth)
        continue;

      int n_eb_2 = p_edge[eb_2 * 18 + m1], n_cb_2 = p_corner[cb_2 * 18 + m1];
      if (get_prune_ptr(p_plus_edge[0], idx_b * 24 + n_eb_2) >= depth)
        continue; // s2: Right
      if (get_prune_ptr(p_plus_corn[0], idx_b * 24 + n_cb_2) >= depth)
        continue;

      int n_eb_3 = p_edge[eb_3 * 18 + m1], n_cb_3 = p_corner[cb_3 * 18 + m1];
      if (get_prune_ptr(p_plus_edge[1], idx_b * 24 + n_eb_3) >= depth)
        continue; // s3: Diag
      if (get_prune_ptr(p_plus_corn[1], idx_b * 24 + n_cb_3) >= depth)
        continue;

      // 3-Corner: s2(Right) + s3(Diag)
      long long idx_3c_b = ((long long)(n1b + n2b) * 24 + n_cb_2) * 24 + n_cb_3;
      if (get_prune_ptr(p_prune_3c, idx_3c_b) >= depth)
        continue;

      // --- View C (s2): 对 s0(Diag), s1(Left), s3(Right) ---
      int m2 = conj_moves_flat[m][2];
      int n1c = p_multi[i1_c + m2], n2c = p_corner[i2_c + m2],
          n3c = p_edge[i3_c + m2];
      long long idx_c = (long long)(n1c + n2c) * 24 + n3c;
      if (get_prune_ptr(p_prune_base, idx_c) >= depth)
        continue;

      int n_ec_0 = p_edge[ec_0 * 18 + m2], n_cc_0 = p_corner[cc_0 * 18 + m2];
      if (get_prune_ptr(p_plus_edge[1], idx_c * 24 + n_ec_0) >= depth)
        continue; // s0: Diag
      if (get_prune_ptr(p_plus_corn[1], idx_c * 24 + n_cc_0) >= depth)
        continue;

      int n_ec_1 = p_edge[ec_1 * 18 + m2], n_cc_1 = p_corner[cc_1 * 18 + m2];
      if (get_prune_ptr(p_plus_edge[2], idx_c * 24 + n_ec_1) >= depth)
        continue; // s1: Left
      if (get_prune_ptr(p_plus_corn[2], idx_c * 24 + n_cc_1) >= depth)
        continue;

      int n_ec_3 = p_edge[ec_3 * 18 + m2], n_cc_3 = p_corner[cc_3 * 18 + m2];
      if (get_prune_ptr(p_plus_edge[0], idx_c * 24 + n_ec_3) >= depth)
        continue; // s3: Right
      if (get_prune_ptr(p_plus_corn[0], idx_c * 24 + n_cc_3) >= depth)
        continue;

      // 3-Corner: s3(Right) + s0(Diag)
      long long idx_3c_c = ((long long)(n1c + n2c) * 24 + n_cc_3) * 24 + n_cc_0;
      if (get_prune_ptr(p_prune_3c, idx_3c_c) >= depth)
        continue;

      // --- View D (s3): 对 s0(Right), s1(Diag), s2(Left) ---
      int m3 = conj_moves_flat[m][3];
      int n1d = p_multi[i1_d + m3], n2d = p_corner[i2_d + m3],
          n3d = p_edge[i3_d + m3];
      long long idx_d = (long long)(n1d + n2d) * 24 + n3d;
      if (get_prune_ptr(p_prune_base, idx_d) >= depth)
        continue;

      int n_ed_0 = p_edge[ed_0 * 18 + m3], n_cd_0 = p_corner[cd_0 * 18 + m3];
      if (get_prune_ptr(p_plus_edge[0], idx_d * 24 + n_ed_0) >= depth)
        continue; // s0: Right
      if (get_prune_ptr(p_plus_corn[0], idx_d * 24 + n_cd_0) >= depth)
        continue;

      int n_ed_1 = p_edge[ed_1 * 18 + m3], n_cd_1 = p_corner[cd_1 * 18 + m3];
      if (get_prune_ptr(p_plus_edge[1], idx_d * 24 + n_ed_1) >= depth)
        continue; // s1: Diag
      if (get_prune_ptr(p_plus_corn[1], idx_d * 24 + n_cd_1) >= depth)
        continue;

      int n_ed_2 = p_edge[ed_2 * 18 + m3], n_cd_2 = p_corner[cd_2 * 18 + m3];
      if (get_prune_ptr(p_plus_edge[2], idx_d * 24 + n_ed_2) >= depth)
        continue; // s2: Left
      if (get_prune_ptr(p_plus_corn[2], idx_d * 24 + n_cd_2) >= depth)
        continue;

      // 3-Corner: s0(Right) + s1(Diag)
      long long idx_3c_d = ((long long)(n1d + n2d) * 24 + n_cd_0) * 24 + n_cd_1;
      if (get_prune_ptr(p_prune_3c, idx_3c_d) >= depth)
        continue;

      if (depth == 1)
        return true;
      else if (search_4(n1a, n2a * 18, n3a * 18, n1b, n2b * 18, n3b * 18, n1c,
                        n2c * 18, n3c * 18, n1d, n2d * 18, n3d * 18, nd * 18,
                        neo * 18, depth - 1, m, bound, n_ea_1, n_ca_1, n_ea_2,
                        n_ca_2, n_ea_3, n_ca_3, n_eb_0, n_cb_0, n_eb_2, n_cb_2,
                        n_eb_3, n_cb_3, n_ec_0, n_cc_0, n_ec_1, n_cc_1, n_ec_3,
                        n_cc_3, n_ed_0, n_cd_0, n_ed_1, n_cd_1, n_ed_2, n_cd_2,
                        v_huge, p_huge_active, (v_huge != -1) ? n_ie6 : -1,
                        (v_huge != -1) ? n_ic2 : -1))
        return true;
    }
    return false;
  }

  std::vector<int> get_stats(const std::vector<int> &base_alg) {
    // 返回 48 个结果：12 XCross + 12 XXCross + 12 XXXCross + 12 XXXXCross
    std::vector<int> res(ENABLE_EO_SEARCH_4 ? 48 : 36, 99);

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
            int t_ab = get_plus_table_idx(s1, s2);
            int t_ba = get_plus_table_idx(s2, s1);

            // View A
            long long idx1 =
                (long long)(st[s1].i1 + st[s1].i2) * 24 + st[s1].i3;
            int h1 = get_prune_ptr(p_prune_base, idx1);
            int h1_pe = get_prune_ptr(p_plus_edge[t_ab],
                                      idx1 * 24 + st[s1].e_trk[t_ab]);
            int h1_pc = get_prune_ptr(p_plus_corn[t_ab],
                                      idx1 * 24 + st[s1].c_trk[t_ab]);

            // View B
            long long idx2 =
                (long long)(st[s2].i1 + st[s2].i2) * 24 + st[s2].i3;
            int h2 = get_prune_ptr(p_prune_base, idx2);
            int h2_pe = get_prune_ptr(p_plus_edge[t_ba],
                                      idx2 * 24 + st[s2].e_trk[t_ba]);
            int h2_pc = get_prune_ptr(p_plus_corn[t_ba],
                                      idx2 * 24 + st[s2].c_trk[t_ba]);

            int h_de = get_prune_ptr(
                p_prune_dep_eo, (long long)st[s1].idep * 2048 + st[s1].ieo);
            int h = std::max({h1, h1_pe, h1_pc, h2, h2_pe, h2_pc, h_de});
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
            int t_ab = get_plus_table_idx(s1, s2);
            int t_ba = get_plus_table_idx(s2, s1);

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
                           st[s1].ieo * 18, d, 18, s1, s2, best_xx - 1, t_ab,
                           t_ba, st[s1].e_trk[t_ab], st[s1].c_trk[t_ab],
                           st[s2].e_trk[t_ba], st[s2].c_trk[t_ba], v_huge,
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
            int t_ab = get_plus_table_idx(s1, s2),
                t_ba = get_plus_table_idx(s2, s1);
            int t_bc = get_plus_table_idx(s2, s3),
                t_cb = get_plus_table_idx(s3, s2);
            int t_ac = get_plus_table_idx(s1, s3),
                t_ca = get_plus_table_idx(s3, s1);

            // Pruning check for all 3 views
            long long idx1 =
                (long long)(st[s1].i1 + st[s1].i2) * 24 + st[s1].i3;
            int h1 = std::max({get_prune_ptr(p_prune_base, idx1),
                               get_prune_ptr(p_plus_edge[t_ab],
                                             idx1 * 24 + st[s1].e_trk[t_ab]),
                               get_prune_ptr(p_plus_corn[t_ab],
                                             idx1 * 24 + st[s1].c_trk[t_ab]),
                               get_prune_ptr(p_plus_edge[t_ac],
                                             idx1 * 24 + st[s1].e_trk[t_ac]),
                               get_prune_ptr(p_plus_corn[t_ac],
                                             idx1 * 24 + st[s1].c_trk[t_ac])});

            long long idx2 =
                (long long)(st[s2].i1 + st[s2].i2) * 24 + st[s2].i3;
            int h2 = std::max({get_prune_ptr(p_prune_base, idx2),
                               get_prune_ptr(p_plus_edge[t_ba],
                                             idx2 * 24 + st[s2].e_trk[t_ba]),
                               get_prune_ptr(p_plus_corn[t_ba],
                                             idx2 * 24 + st[s2].c_trk[t_ba]),
                               get_prune_ptr(p_plus_edge[t_bc],
                                             idx2 * 24 + st[s2].e_trk[t_bc]),
                               get_prune_ptr(p_plus_corn[t_bc],
                                             idx2 * 24 + st[s2].c_trk[t_bc])});

            long long idx3 =
                (long long)(st[s3].i1 + st[s3].i2) * 24 + st[s3].i3;
            int h3 = std::max({get_prune_ptr(p_prune_base, idx3),
                               get_prune_ptr(p_plus_edge[t_ca],
                                             idx3 * 24 + st[s3].e_trk[t_ca]),
                               get_prune_ptr(p_plus_corn[t_ca],
                                             idx3 * 24 + st[s3].c_trk[t_ca]),
                               get_prune_ptr(p_plus_edge[t_cb],
                                             idx3 * 24 + st[s3].e_trk[t_cb]),
                               get_prune_ptr(p_plus_corn[t_cb],
                                             idx3 * 24 + st[s3].c_trk[t_cb])});

            // 3-Corner Pruning
            int d_3c = 0;
            if ((t_ab == 0 && t_ac == 1) ||
                (t_ac == 0 && t_ab == 1)) { // View A
              int c_r = (t_ab == 0) ? st[s1].c_trk[t_ab] : st[s1].c_trk[t_ac];
              int c_d = (t_ab == 0) ? st[s1].c_trk[t_ac] : st[s1].c_trk[t_ab];
              d_3c = std::max(
                  d_3c, get_prune_ptr(p_prune_3c,
                                      ((long long)(st[s1].i1 + st[s1].i2) * 24 +
                                       c_r) * 24 +
                                          c_d));
            }
            if ((t_ba == 0 && t_bc == 1) ||
                (t_bc == 0 && t_ba == 1)) { // View B
              int c_r = (t_ba == 0) ? st[s2].c_trk[t_ba] : st[s2].c_trk[t_bc];
              int c_d = (t_ba == 0) ? st[s2].c_trk[t_bc] : st[s2].c_trk[t_ba];
              d_3c = std::max(
                  d_3c, get_prune_ptr(p_prune_3c,
                                      ((long long)(st[s2].i1 + st[s2].i2) * 24 +
                                       c_r) * 24 +
                                          c_d));
            }
            if ((t_ca == 0 && t_cb == 1) ||
                (t_cb == 0 && t_ca == 1)) { // View C
              int c_r = (t_ca == 0) ? st[s3].c_trk[t_ca] : st[s3].c_trk[t_cb];
              int c_d = (t_ca == 0) ? st[s3].c_trk[t_cb] : st[s3].c_trk[t_ca];
              d_3c = std::max(
                  d_3c, get_prune_ptr(p_prune_3c,
                                      ((long long)(st[s3].i1 + st[s3].i2) * 24 +
                                       c_r) * 24 +
                                          c_d));
            }

            int pr_de = get_prune_ptr(
                p_prune_dep_eo, (long long)st[s1].idep * 2048 + st[s1].ieo);
            int h = std::max({h1, h2, h3, pr_de, d_3c});
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
            int t_ab = get_plus_table_idx(s1, s2),
                t_ba = get_plus_table_idx(s2, s1);
            int t_bc = get_plus_table_idx(s2, s3),
                t_cb = get_plus_table_idx(s3, s2);
            int t_ac = get_plus_table_idx(s1, s3),
                t_ca = get_plus_table_idx(s3, s1);

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
                           t_ab, t_ba, t_bc, t_cb, t_ac, t_ca,
                           st[s1].e_trk[t_ab], st[s1].c_trk[t_ab],
                           st[s1].e_trk[t_ac], st[s1].c_trk[t_ac],
                           st[s2].e_trk[t_ba], st[s2].c_trk[t_ba],
                           st[s2].e_trk[t_bc], st[s2].c_trk[t_bc],
                           st[s3].e_trk[t_ca], st[s3].c_trk[t_ca],
                           st[s3].e_trk[t_cb], st[s3].c_trk[t_cb], v_huge,
                           p_huge, init_e6, init_c2)) {
                best_xxx = d;
                break;
              }
            }
          }
          res[24 + sym] = best_xxx;
        }

#if ENABLE_EO_SEARCH_4
        // --- 4. XXXXCross+EO ---
        // 只有 1 种组合 {0, 1, 2, 3}
        {
          // 获取初始下界：检查所有 4 个视角的 Base 表 + Plus 表 + 3-Corner 表
          int h_de = get_prune_ptr(p_prune_dep_eo,
                                   (long long)st[0].idep * 2048 + st[0].ieo);
          int h_max = h_de;
          for (int s = 0; s < 4; ++s) {
            long long idx = (long long)(st[s].i1 + st[s].i2) * 24 + st[s].i3;
            h_max = std::max(h_max, get_prune_ptr(p_prune_base, idx));
            // Plus Edge/Corner 检查
            for (int t = 0; t < 3; ++t) {
              h_max = std::max(h_max, get_prune_ptr(p_plus_edge[t],
                                                    idx * 24 + st[s].e_trk[t]));
              h_max = std::max(h_max, get_prune_ptr(p_plus_corn[t],
                                                    idx * 24 + st[s].c_trk[t]));
            }
            // 3-Corner 检查 (Right + Diag)
            long long idx_3c =
                ((long long)(st[s].i1 + st[s].i2) * 24 + st[s].c_trk[0]) * 24 +
                st[s].c_trk[1];
            h_max = std::max(h_max, get_prune_ptr(p_prune_3c, idx_3c));
          }

          // 确定 Huge 表视角 (使用 s0-s1 相邻对)
          int v_nb = get_neighbor_view(0, 1); // 始终为 0
          int v_huge = v_nb;
          const unsigned char *p_huge = nullptr;
          int init_e6 = -1, init_c2 = -1;
          if (v_nb != -1 && p_huge_neighbor) {
            p_huge = p_huge_neighbor;
            init_e6 = st[v_nb].i_e6_nb;
            init_c2 = st[v_nb].i_c2_nb;
          }

          int best_xxxx = 99;
          if (h_max == 0) {
            best_xxxx = 0;
          } else {
            for (int d = h_max; d <= std::min(20, best_xxxx - 1); ++d) {
              // 初始追踪状态：每视角对其他 3 个槽位的 Edge/Corner
              // View A (s0): 对 s1(e_trk[0],c_trk[0]), s2(e_trk[1],c_trk[1]),
              // s3(e_trk[2],c_trk[2]) View B (s1): s0→Left(2), s2→Right(0),
              // s3→Diag(1) View C (s2): s0→Diag(1), s1→Left(2), s3→Right(0)
              // View D (s3): s0→Right(0), s1→Diag(1), s2→Left(2)
              if (search_4(st[0].i1, st[0].i2 * 18, st[0].i3 * 18, st[1].i1,
                           st[1].i2 * 18, st[1].i3 * 18, st[2].i1,
                           st[2].i2 * 18, st[2].i3 * 18, st[3].i1,
                           st[3].i2 * 18, st[3].i3 * 18, st[0].idep * 18,
                           st[0].ieo * 18, d, 18, best_xxxx - 1,
                           // View A: s1(Right), s2(Diag), s3(Left)
                           st[0].e_trk[0], st[0].c_trk[0], st[0].e_trk[1],
                           st[0].c_trk[1], st[0].e_trk[2], st[0].c_trk[2],
                           // View B: s0(Left:2), s2(Right:0), s3(Diag:1)
                           st[1].e_trk[2], st[1].c_trk[2], st[1].e_trk[0],
                           st[1].c_trk[0], st[1].e_trk[1], st[1].c_trk[1],
                           // View C: s0(Diag:1), s1(Left:2), s3(Right:0)
                           st[2].e_trk[1], st[2].c_trk[1], st[2].e_trk[2],
                           st[2].c_trk[2], st[2].e_trk[0], st[2].c_trk[0],
                           // View D: s0(Right:0), s1(Diag:1), s2(Left:2)
                           st[3].e_trk[0], st[3].c_trk[0], st[3].e_trk[1],
                           st[3].c_trk[1], st[3].e_trk[2], st[3].c_trk[2],
                           // Huge 表
                           v_huge, p_huge, init_e6, init_c2)) {
                best_xxxx = d;
                break;
              }
            }
          }
          res[36 + sym] = best_xxxx;
        }
#endif
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
    printCuberootLogo();
    init_matrix();

    // 调用静态初始化方法，加载所有表（只执行一次）
    cross_analyzer::static_init();
    xcross_analyzer::static_init();
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
#if ENABLE_EO_SEARCH_4
    for (const auto &s : suffixes)
      oss << ",eo_xxxxcross" << s;
#endif
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
#if ENABLE_EO_SEARCH_4
    // XXXXC+EO
    for (int c = 0; c < 6; ++c)
      oss << "," << std::min(xr[36 + 2 * c], xr[36 + 2 * c + 1]);
#endif

    return oss.str();
  }

  static void print_stats() {
#if ENABLE_PRUNE_STATS
    printf("\n=== EO Cross Analyzer Pruning Stats ===\n");
#if ENABLE_STATS_S1
    PRINT_STAT(s1_dep_eo);
    PRINT_STAT(s1_xcross);
#endif
#if ENABLE_STATS_S2
    PRINT_STAT(s2_huge);
    PRINT_STAT(s2_dep_eo);
    PRINT_STAT(s2_xcross1);
    PRINT_STAT(s2_xcross2);
#endif
#if ENABLE_STATS_S3
    PRINT_STAT(s3_huge1);
    PRINT_STAT(s3_huge2);
    PRINT_STAT(s3_dep_eo);
    PRINT_STAT(s3_xcross1);
    PRINT_STAT(s3_xcross2);
    PRINT_STAT(s3_xcross3);
#endif
#if ENABLE_STATS_S4
    PRINT_STAT(s4_huge1);
    PRINT_STAT(s4_huge2);
    PRINT_STAT(s4_huge3);
    PRINT_STAT(s4_dep_eo);
    PRINT_STAT(s4_xcross1);
    PRINT_STAT(s4_xcross2);
    PRINT_STAT(s4_xcross3);
    PRINT_STAT(s4_xcross4);
#endif
#endif
  }
};

int main() {
  run_analyzer_app<EOCrossSolverWrapper>("_eo");
  return 0;
}
