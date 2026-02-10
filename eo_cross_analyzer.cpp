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
STAT_DECL(s3_huge1);   // S3: Huge 表1
STAT_DECL(s3_huge2);   // S3: Huge 表2
STAT_DECL(s3_dep_eo);  // S3: Dependency+EO 剪枝表
STAT_DECL(s3_xcross1); // S3: XCross 1 剪枝表
STAT_DECL(s3_xcross2); // S3: XCross 2 剪枝表
STAT_DECL(s3_xcross3); // S3: XCross 3 剪枝表

// Search 4: huge×3 + dep_eo + xcross×4
STAT_DECL(s4_huge1);   // S4: Huge 表1
STAT_DECL(s4_huge2);   // S4: Huge 表2
STAT_DECL(s4_huge3);   // S4: Huge 表3
STAT_DECL(s4_dep_eo);  // S4: Dependency+EO 剪枝表
STAT_DECL(s4_xcross1); // S4: XCross 1 剪枝表
STAT_DECL(s4_xcross2); // S4: XCross 2 剪枝表
STAT_DECL(s4_xcross3); // S4: XCross 3 剪枝表
STAT_DECL(s4_xcross4); // S4: XCross 4 剪枝表

// --- Cross Analyzer (EO Cross) ---
struct CrossSolver {
  // 静态成员：所有实例共享
  static inline bool s_initialized = false;
  static inline const int *s_p_mt_edge2 = nullptr;
  static inline const int *s_p_mt_eo12 =
      nullptr; // EO移动表指针迁移到MoveTableManager)
  static inline const unsigned char *s_p_pt_cross = nullptr;

  // 实例成员（指向静态数据）
  const int *p_mt_edge2, *p_mt_eo12;
  const unsigned char *p_pt_cross;

  static void static_init() {
    if (s_initialized)
      return;
    auto &mm = MoveTableManager::getInstance();
    auto &pm = PruneTableManager::getInstance();
    mm.loadMTEdge();
    mm.loadMTEdge2();
    mm.loadMTEO(); // 使用 MoveTableManager 加载 EO 表
    pm.genPTCross();
    s_p_mt_edge2 = mm.getMTEdge2Ptr();
    s_p_mt_eo12 = mm.getMTEOPtr();
    s_p_pt_cross = pm.getCrossPTPtr();
    s_initialized = true;
  }

  CrossSolver() {
    // 仅复制指针引用
    p_mt_edge2 = s_p_mt_edge2;
    p_mt_eo12 = s_p_mt_eo12;
    p_pt_cross = s_p_pt_cross;
  }

  void get_indices_sym(const std::vector<int> &alg, int sym_idx, int &i1,
                       int &i2, int &i_eo) {
    i1 = StateSpace::EDGE2_A_SOLVED;
    i2 = StateSpace::EDGE2_B_SOLVED;
    i_eo = 0;
    for (int m : alg) {
      int conj_m = sym_moves_flat[m][sym_idx];
      i1 = p_mt_edge2[i1 * 18 + conj_m];
      i2 = p_mt_edge2[i2 * 18 + conj_m];
      i_eo = p_mt_eo12[i_eo + conj_m];
    }
  }

  bool search(int i1, int i2, int i_eo, int depth, int prev, int &found_len,
              int max_d) {
    const int *moves = valid_moves_flat[prev];
    const int count = valid_moves_count[prev];

    for (int k = 0; k < count; ++k) {
      COUNT_NODE
      int m = moves[k];

      // : 先Cross 查表
      int n1 = p_mt_edge2[i1 + m], n2 = p_mt_edge2[i2 + m];
      long long idx = (long long)n1 * StateSpace::EDGE2 + n2;
      int pr = get_prune(p_pt_cross, idx);
      if (pr >= depth)
        continue;

      // 再 EO 计算
      int neo = p_mt_eo12[i_eo + m];

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
      long long idx = (long long)i1 * StateSpace::EDGE2 + i2;
      int h = get_prune(p_pt_cross, idx);
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
struct XCrossSolver {
  // 静态成员：所有实例共享
  static inline bool s_initialized = false;
  // NOTE: 移动表和剪枝表已迁移到Manager，此处仅保留指针
  static inline const int *s_p_mt_ep4 = nullptr;      // EP4 移动表指针
  static inline const int *s_p_mt_eo12_alt = nullptr; // EO Alt 移动表指针

  static inline const int *s_p_mt_edge4 = nullptr;
  static inline const int *s_p_mt_corn = nullptr;
  static inline const int *s_p_mt_edge = nullptr;
  static inline const int *s_p_mt_edge6 =
      nullptr; // Edge6 Move Table (用于 Huge 表
  static inline const int *s_p_mt_corn2 =
      nullptr; // Corner2 Move Table (用于 Huge 表
  static inline const unsigned char *s_p_pt_ep4eo12 = nullptr;
  static inline const unsigned char *s_p_pt_cross_C4E0 = nullptr;
  static inline std::vector<const unsigned char *> s_p_pt_cross_CEE;
  static inline std::vector<const unsigned char *> s_p_pt_cross_CCE;
  static inline const unsigned char *s_p_pt_cross_C4C5C6 = nullptr;
  static inline const unsigned char *s_p_pt_cross_C4C5E0E1 =
      nullptr; // Huge Neighbor 表
  static inline const unsigned char *s_p_pt_cross_C4C6E0E2 =
      nullptr; // Huge Diagonal 表

  // 实例成员（指向静态数据）
  const int *p_mt_edge4, *p_mt_corn, *p_mt_edge, *p_mt_ep4, *p_mt_eo12_alt;
  const int *p_mt_edge6 = nullptr,
            *p_mt_corn2 = nullptr; // Edge6/Corner2 Move Tables
  const unsigned char *p_pt_ep4eo12;
  const unsigned char *p_pt_cross_C4E0 = nullptr;
  std::vector<const unsigned char *> p_pt_cross_CEE;
  std::vector<const unsigned char *> p_pt_cross_CCE;
  const unsigned char *p_pt_cross_C4C5C6 = nullptr;
  const unsigned char *p_pt_cross_C4C5E0E1 = nullptr; // Huge Neighbor 表
  const unsigned char *p_pt_cross_C4C6E0E2 = nullptr; // Huge Diagonal 表

  const int SOLVED_CORNER = 12;
  const int SOLVED_EDGE = 0;

  static void static_init() {
    if (s_initialized)
      return;

    MoveTableManager &mm = MoveTableManager::getInstance();
    mm.loadMTEdge();
    mm.loadMTCorn();
    mm.loadMTEdge4();
    mm.loadMTEdge6(); // 用于 Huge 表状态追踪
    mm.loadMTCorn2(); // 用于 Huge 表状态追踪

    // 加载 EOCross 专用移动表
    mm.loadMTEOCross();

    // 设置移动表指针
    s_p_mt_edge4 = mm.getMTEdge4Ptr();
    s_p_mt_corn = mm.getMTCornPtr();
    s_p_mt_edge = mm.getMTEdgePtr();
    s_p_mt_edge6 = mm.getMTEdge6Ptr();    // Edge6 Move Table
    s_p_mt_corn2 = mm.getMTCorn2Ptr();    // Corner2 Move Table
    s_p_mt_ep4 = mm.getMTEP4Ptr();        // EP4 Move Table
    s_p_mt_eo12_alt = mm.getMTEOAltPtr(); // EO Alt Move Table

    // === 剪枝表：使用 PruneTableManager ===
    auto &ptm = PruneTableManager::getInstance();

    // 先加载复用的xcross_c4_e0表（与std_analyzer/pair_analyzer共享)
    ptm.genPTCrossC4E0();

    // 生成/加载 EOCross 专用剪枝表
    ptm.genPTCrossInsC4(); // Cross+C4 Insertion (EOCross 表)
    ptm.genPTEP4EO12();    // Dependency+EO
    // Plus Edge (参数化) // Plus Edge Right
    for (int i = 0; i < 3; ++i)
      ptm.genPTCrossCEE(i);
    // Plus Corner (参数化)
    for (int i = 0; i < 3; ++i)
      ptm.genPTCrossCCE(i);

    ptm.genPTCrossC4C5C6(); // 3-Corner

    // 获取剪枝表指针
    s_p_pt_ep4eo12 = ptm.getEP4EO12PTPtr();
    s_p_pt_cross_C4E0 = ptm.getCrossC4E0PTPtr(); // 复用已有表

    s_p_pt_cross_CEE.resize(3);
    s_p_pt_cross_CCE.resize(3);
    for (int i = 0; i < 3; ++i) {
      s_p_pt_cross_CEE[i] = ptm.getCrossCEEPTPtr(i);
      s_p_pt_cross_CCE[i] = ptm.getCrossCCEPTPtr(i);
    }
    s_p_pt_cross_C4C5C6 = ptm.getCrossC4C5C6PTPtr();

    // 加载 Huge Neighbor/Diagonal Prune Tables
    ptm.genPTCrossC4C5E0E1();
    s_p_pt_cross_C4C5E0E1 = ptm.getCrossC4C5E0E1PTPtr();
    if (ENABLE_DIAGONAL_EO_CROSS) {
      ptm.genPTCrossC4C6E0E2();
      s_p_pt_cross_C4C6E0E2 = ptm.getCrossC4C6E0E2PTPtr();
    }

    s_initialized = true;
  }

  XCrossSolver() {
    // 仅复制指针引用
    p_mt_edge4 = s_p_mt_edge4;
    p_mt_corn = s_p_mt_corn;
    p_mt_edge = s_p_mt_edge;
    p_mt_edge6 = s_p_mt_edge6;       // Edge6 Move Table
    p_mt_corn2 = s_p_mt_corn2;       // Corner2 Move Table
    p_mt_ep4 = s_p_mt_ep4;           // EP4 Move Table
    p_mt_eo12_alt = s_p_mt_eo12_alt; // EO Alt Move Table
    p_pt_ep4eo12 = s_p_pt_ep4eo12;
    p_pt_cross_C4E0 = s_p_pt_cross_C4E0;
    p_pt_cross_CEE = s_p_pt_cross_CEE;
    p_pt_cross_CCE = s_p_pt_cross_CCE;
    p_pt_cross_C4C5C6 = s_p_pt_cross_C4C5C6;
    p_pt_cross_C4C5E0E1 = s_p_pt_cross_C4C5E0E1; // Huge Neighbor 表
    p_pt_cross_C4C6E0E2 = s_p_pt_cross_C4C6E0E2; // Huge Diagonal 表
  }

  void get_indices_conj_full(const std::vector<int> &alg, int sym_idx,
                             int slot_idx, int &i1, int &i2, int &i3,
                             int &i_dep, int &i_eo, int *track_e, int *track_c,
                             int &i_e6_nb, int &i_c2_nb, // Huge Neighbor 状态
                             int &i_e6_dg, int &i_c2_dg) { // Huge Diagonal 状态
    i1 = StateSpace::CROSS_SOLVED * StateSpace::CORNER;
    i2 = SOLVED_CORNER;
    i3 = SOLVED_EDGE;
    i_dep = StateSpace::EP4_SOLVED;
    i_eo = 0;

    track_e[0] = 2;
    track_e[1] = 4;
    track_e[2] = 6;
    track_c[0] = 15;
    track_c[1] = 18;
    track_c[2] = 21;

    // 初始化Huge 表索引(参考std_analyzer.cpp)
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

      i1 = p_mt_edge4[i1 + m_slot];
      i2 = p_mt_corn[i2 * 18 + m_slot];
      i3 = p_mt_edge[i3 * 18 + m_slot];

      i_dep = p_mt_ep4[i_dep * 18 + m_global];
      i_eo = p_mt_eo12_alt[i_eo * 18 + m_global];

      for (int k = 0; k < 3; ++k) {
        track_e[k] = p_mt_edge[track_e[k] * 18 + m_slot];
        track_c[k] = p_mt_corn[track_c[k] * 18 + m_slot];
      }

      // 追踪 Huge 表状态
      cur_e6_nb = p_mt_edge6[cur_e6_nb * 18 + m_slot];
      cur_c2_nb = p_mt_corn2[cur_c2_nb * 18 + m_slot];
      cur_e6_dg = p_mt_edge6[cur_e6_dg * 18 + m_slot];
      cur_c2_dg = p_mt_corn2[cur_c2_dg * 18 + m_slot];
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

      // Check 1: Dependency (EO + Partial Cross)
      int nd = p_mt_ep4[i_dep + m], neo = p_mt_eo12_alt[i_eo + m];
      S1_CHECK(s1_dep_eo);
      if (get_prune(p_pt_ep4eo12, (long long)nd * StateSpace::EO12 + neo) >=
          depth) {
        S1_HIT(s1_dep_eo);
        continue;
      }

      // Check 2: Main XCross
      int m_slot = conj_moves_flat[m][slot];
      int n1 = p_mt_edge4[i1 + m_slot], n2 = p_mt_corn[i2 + m_slot],
          n3 = p_mt_edge[i3 + m_slot];
      long long idx_xc = (long long)(n1 + n2) * 24 + n3;
      S1_CHECK(s1_xcross);
      if (get_prune(p_pt_cross_C4E0, idx_xc) >= depth) {
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
                // Huge 表参考
                int v_huge, const unsigned char *p_huge_active, int i_e6,
                int i_c2) {
    if (depth > bound)
      return false;

    const int *moves = valid_moves_flat[prev];
    const int count = valid_moves_count[prev];

    for (int k = 0; k < count; ++k) {
      COUNT_NODE
      int m = moves[k];

      // Check 0: Huge 表(最前置，剪枝力最强)
      int n_ie6 = -1, n_ic2 = -1;
      if (v_huge != -1 && p_huge_active) {
        int mv = conj_moves_flat[m][v_huge];
        n_ie6 = p_mt_edge6[i_e6 * 18 + mv];
        n_ic2 = p_mt_corn2[i_c2 * 18 + mv];
        if (get_prune(p_huge_active,
                      (long long)n_ie6 * StateSpace::CORNER2 + n_ic2) >= depth)
          continue;
      }

      // Check 1: Dep + EO
      int nd = p_mt_ep4[i_dep + m], neo = p_mt_eo12_alt[i_eo + m];
      if (get_prune(p_pt_ep4eo12, (long long)nd * StateSpace::EO12 + neo) >=
          depth)
        continue;

      // Check 2: View A (Base + Plus)
      int m1 = conj_moves_flat[m][s1];
      int n1a = p_mt_edge4[i1a + m1], n2a = p_mt_corn[i2a + m1],
          n3a = p_mt_edge[i3a + m1];
      long long idx_a = (long long)(n1a + n2a) * 24 + n3a;
      if (get_prune(p_pt_cross_C4E0, idx_a) >= depth)
        continue;

      int n_ea_rel = p_mt_edge[ea_rel * 18 + m1];
      if (get_prune(p_pt_cross_CEE[tab], idx_a * 24 + n_ea_rel) >= depth)
        continue;
      int n_ca_rel = p_mt_corn[ca_rel * 18 + m1];
      if (get_prune(p_pt_cross_CCE[tab], idx_a * 24 + n_ca_rel) >= depth)
        continue;

      // Check 3: View B (Base + Plus)
      int m2 = conj_moves_flat[m][s2];
      int n1b = p_mt_edge4[i1b + m2], n2b = p_mt_corn[i2b + m2],
          n3b = p_mt_edge[i3b + m2];
      long long idx_b = (long long)(n1b + n2b) * 24 + n3b;
      if (get_prune(p_pt_cross_C4E0, idx_b) >= depth)
        continue;

      int n_eb_rel = p_mt_edge[eb_rel * 18 + m2];
      if (get_prune(p_pt_cross_CEE[tba], idx_b * 24 + n_eb_rel) >= depth)
        continue;
      int n_cb_rel = p_mt_corn[cb_rel * 18 + m2];
      if (get_prune(p_pt_cross_CCE[tba], idx_b * 24 + n_cb_rel) >= depth)
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
                // Huge 表参考(仅追踪第一对s1-s2)
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

      // Check 0: Huge 表(最前置)
      int n_ie6 = -1, n_ic2 = -1;
      if (v_huge != -1 && p_huge_active) {
        int mv = conj_moves_flat[m][v_huge];
        n_ie6 = p_mt_edge6[i_e6 * 18 + mv];
        n_ic2 = p_mt_corn2[i_c2 * 18 + mv];
        if (get_prune(p_huge_active,
                      (long long)n_ie6 * StateSpace::CORNER2 + n_ic2) >= depth)
          continue;
      }

      // Check 1: Dep + EO
      int nd = p_mt_ep4[i_dep + m], neo = p_mt_eo12_alt[i_eo + m];
      if (get_prune(p_pt_ep4eo12, (long long)nd * StateSpace::EO12 + neo) >=
          depth)
        continue;

      // --- View A ---
      int m1 = conj_moves_flat[m][s1];
      int n1a = p_mt_edge4[i1a + m1], n2a = p_mt_corn[i2a + m1],
          n3a = p_mt_edge[i3a + m1];
      long long idx_a = (long long)(n1a + n2a) * 24 + n3a;
      if (get_prune(p_pt_cross_C4E0, idx_a) >= depth)
        continue;

      int n_ea_b = p_mt_edge[ea_b * 18 + m1],
          n_ca_b = p_mt_corn[ca_b * 18 + m1];
      if (get_prune(p_pt_cross_CEE[t_ab], idx_a * 24 + n_ea_b) >= depth)
        continue;
      if (get_prune(p_pt_cross_CCE[t_ab], idx_a * 24 + n_ca_b) >= depth)
        continue;

      int n_ea_c = p_mt_edge[ea_c * 18 + m1],
          n_ca_c = p_mt_corn[ca_c * 18 + m1];
      if (get_prune(p_pt_cross_CEE[t_ac], idx_a * 24 + n_ea_c) >= depth)
        continue;
      if (get_prune(p_pt_cross_CCE[t_ac], idx_a * 24 + n_ca_c) >= depth)
        continue;

      if (check_3c_A) {
        long long idx_3c = ((long long)(n1a + n2a) * 24 + n_ca_b) * 24 + n_ca_c;
        if (get_prune(p_pt_cross_C4C5C6, idx_3c) >= depth)
          continue;
      }

      // --- View B ---
      int m2 = conj_moves_flat[m][s2];
      int n1b = p_mt_edge4[i1b + m2], n2b = p_mt_corn[i2b + m2],
          n3b = p_mt_edge[i3b + m2];
      long long idx_b = (long long)(n1b + n2b) * 24 + n3b;
      if (get_prune(p_pt_cross_C4E0, idx_b) >= depth)
        continue;

      int n_eb_a = p_mt_edge[eb_a * 18 + m2],
          n_cb_a = p_mt_corn[cb_a * 18 + m2];
      if (get_prune(p_pt_cross_CEE[t_ba], idx_b * 24 + n_eb_a) >= depth)
        continue;
      if (get_prune(p_pt_cross_CCE[t_ba], idx_b * 24 + n_cb_a) >= depth)
        continue;

      int n_eb_c = p_mt_edge[eb_c * 18 + m2],
          n_cb_c = p_mt_corn[cb_c * 18 + m2];
      if (get_prune(p_pt_cross_CEE[t_bc], idx_b * 24 + n_eb_c) >= depth)
        continue;
      if (get_prune(p_pt_cross_CCE[t_bc], idx_b * 24 + n_cb_c) >= depth)
        continue;

      if (check_3c_B) {
        long long idx_3c = ((long long)(n1b + n2b) * 24 + n_cb_a) * 24 + n_cb_c;
        if (get_prune(p_pt_cross_C4C5C6, idx_3c) >= depth)
          continue;
      }

      // --- View C ---
      int m3 = conj_moves_flat[m][s3];
      int n1c = p_mt_edge4[i1c + m3], n2c = p_mt_corn[i2c + m3],
          n3c = p_mt_edge[i3c + m3];
      long long idx_c = (long long)(n1c + n2c) * 24 + n3c;
      if (get_prune(p_pt_cross_C4E0, idx_c) >= depth)
        continue;

      int n_ec_a = p_mt_edge[ec_a * 18 + m3],
          n_cc_a = p_mt_corn[cc_a * 18 + m3];
      if (get_prune(p_pt_cross_CEE[t_ca], idx_c * 24 + n_ec_a) >= depth)
        continue;
      if (get_prune(p_pt_cross_CCE[t_ca], idx_c * 24 + n_cc_a) >= depth)
        continue;

      int n_ec_b = p_mt_edge[ec_b * 18 + m3],
          n_cc_b = p_mt_corn[cc_b * 18 + m3];
      if (get_prune(p_pt_cross_CEE[t_cb], idx_c * 24 + n_ec_b) >= depth)
        continue;
      if (get_prune(p_pt_cross_CCE[t_cb], idx_c * 24 + n_cc_b) >= depth)
        continue;

      if (check_3c_C) {
        long long idx_3c = ((long long)(n1c + n2c) * 24 + n_cc_a) * 24 + n_cc_b;
        if (get_prune(p_pt_cross_C4C5C6, idx_3c) >= depth)
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

  // 每个 View 在 search_4 中的完整状态
  // NOTE: ex/cx 统一按 [Right, Diag, Left] 顺序排列，
  //   因此 Plus 表索引固定为 CEE[0]/CCE[0] (Right), CEE[1]/CCE[1] (Diag),
  //   CEE[2]/CCE[2] (Left)， 3-Corner 也固定使用 cx[0](Right) 和 cx[1](Diag)
  struct ViewState {
    int i1, i2, i3; // Edge4, Corner, Edge 索引
    int slot;       // conj slot 号 (0,1,2,3)
    int ex[3];      // 追踪 Edge: [Right, Diag, Left]
    int cx[3];      // 追踪 Corner: [Right, Diag, Left]
  };

  // --- Search 4: XXXXCross+EO (数据驱动循环版) ---
  // 4 个视角按 viewOrder 顺序检查，每个视角检查 Base + Plus×3 + 3-Corner
  bool search_4(const ViewState views[4], const int viewOrder[4], int i_dep,
                int i_eo, int depth, int prev, int bound, int v_huge,
                const unsigned char *p_huge_active, int i_e6, int i_c2) {
    if (depth > bound)
      return false;

    const int *moves = valid_moves_flat[prev];
    const int count = valid_moves_count[prev];

    for (int k = 0; k < count; ++k) {
      COUNT_NODE
      int m = moves[k];

      // --- Check 0: Huge 表(最前置) ---
      int n_ie6 = -1, n_ic2 = -1;
      if (v_huge != -1 && p_huge_active) {
        int mv = conj_moves_flat[m][v_huge];
        n_ie6 = p_mt_edge6[i_e6 * 18 + mv];
        n_ic2 = p_mt_corn2[i_c2 * 18 + mv];
        if (get_prune(p_huge_active,
                      (long long)n_ie6 * StateSpace::CORNER2 + n_ic2) >= depth)
          continue;
      }

      // --- Check 1: Dep + EO ---
      int nd = p_mt_ep4[i_dep + m], neo = p_mt_eo12_alt[i_eo + m];
      if (get_prune(p_pt_ep4eo12, (long long)nd * StateSpace::EO12 + neo) >=
          depth)
        continue;

      // --- 按 viewOrder 顺序检查 4 个 View ---
      ViewState nv[4]; // 本轮各 View 的新状态
      bool pruned = false;

      for (int vi = 0; vi < 4; ++vi) {
        int v = viewOrder[vi];
        const ViewState &cur = views[v];
        int mv = conj_moves_flat[m][cur.slot];

        int n1 = p_mt_edge4[cur.i1 + mv];
        int n2 = p_mt_corn[cur.i2 + mv];
        int n3 = p_mt_edge[cur.i3 + mv];
        long long idx = (long long)(n1 + n2) * 24 + n3;

        // Base 表
        if (get_prune(p_pt_cross_C4E0, idx) >= depth) {
          pruned = true;
          break;
        }

        // Plus Edge/Corner 表 (3 对: Right, Diag, Left)
        int ne[3], nc[3];
        for (int t = 0; t < 3; ++t) {
          ne[t] = p_mt_edge[cur.ex[t] * 18 + mv];
          if (get_prune(p_pt_cross_CEE[t], idx * 24 + ne[t]) >= depth) {
            pruned = true;
            break;
          }
          nc[t] = p_mt_corn[cur.cx[t] * 18 + mv];
          if (get_prune(p_pt_cross_CCE[t], idx * 24 + nc[t]) >= depth) {
            pruned = true;
            break;
          }
        }
        if (pruned)
          break;

        // 3-Corner: Right(cx[0]) + Diag(cx[1])
        long long idx_3c = ((long long)(n1 + n2) * 24 + nc[0]) * 24 + nc[1];
        if (get_prune(p_pt_cross_C4C5C6, idx_3c) >= depth) {
          pruned = true;
          break;
        }

        // 保存新状态（递归用）
        nv[v] = {n1,
                 n2 * 18,
                 n3 * 18,
                 cur.slot,
                 {ne[0], ne[1], ne[2]},
                 {nc[0], nc[1], nc[2]}};
      }
      if (pruned)
        continue;

      if (depth == 1)
        return true;
      else if (search_4(nv, viewOrder, nd * 18, neo * 18, depth - 1, m, bound,
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
          int pr_xc = get_prune(p_pt_cross_C4E0, idx_xc);
          int pr_de =
              get_prune(p_pt_ep4eo12,
                        (long long)st[s].idep * StateSpace::EO12 + st[s].ieo);
          tasks.push_back({std::max(pr_xc, pr_de), s});
        }
        std::sort(tasks.begin(), tasks.end());

        // NOTE: 配对内 best 共享 —— 最终输出取 min(res[2c], res[2c+1])
        // 当 sym 为奇数时，配对的偶数 sym 已算完，用其结果做搜索上界
        int pairBound = (sym & 1) ? res[sym - 1] : 99;
        int best = pairBound;
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

        // NOTE: 提前声明，使 best_xx/best_xxx 可被后续阶段引用
        // 跨阶段不变量: XXCross >= XCross, XXXCross >= XXCross, XXXXCross >=
        // XXXCross
        int best_xx = 99;
        int best_xxx = 99;

        // --- 2. XXCross+EO ---
        {
          int pairs[6][2] = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};
          std::vector<std::pair<int, int>> tasks_xx;
          for (int p = 0; p < 6; ++p) {
            int s1 = pairs[p][0], s2 = pairs[p][1];
            int t_ab = getPlusTableIdx(s1, s2);
            int t_ba = getPlusTableIdx(s2, s1);

            // View A
            long long idx1 =
                (long long)(st[s1].i1 + st[s1].i2) * 24 + st[s1].i3;
            int h1 = get_prune(p_pt_cross_C4E0, idx1);
            int h1_pe =
                get_prune(p_pt_cross_CEE[t_ab], idx1 * 24 + st[s1].e_trk[t_ab]);
            int h1_pc =
                get_prune(p_pt_cross_CCE[t_ab], idx1 * 24 + st[s1].c_trk[t_ab]);

            // View B
            long long idx2 =
                (long long)(st[s2].i1 + st[s2].i2) * 24 + st[s2].i3;
            int h2 = get_prune(p_pt_cross_C4E0, idx2);
            int h2_pe =
                get_prune(p_pt_cross_CEE[t_ba], idx2 * 24 + st[s2].e_trk[t_ba]);
            int h2_pc =
                get_prune(p_pt_cross_CCE[t_ba], idx2 * 24 + st[s2].c_trk[t_ba]);

            int h_de = get_prune(p_pt_ep4eo12,
                                 (long long)st[s1].idep * StateSpace::EO12 +
                                     st[s1].ieo);
            int h = std::max({h1, h1_pe, h1_pc, h2, h2_pe, h2_pc, h_de});
            tasks_xx.push_back({h, p});
          }
          std::sort(tasks_xx.begin(), tasks_xx.end());

          best_xx = (sym & 1) ? res[12 + sym - 1] : 99;
          for (auto &t : tasks_xx) {
            if (t.first >= best_xx)
              break;
            if (t.first == 0) {
              best_xx = 0;
              break;
            }
            int s1 = pairs[t.second][0], s2 = pairs[t.second][1];
            int t_ab = getPlusTableIdx(s1, s2);
            int t_ba = getPlusTableIdx(s2, s1);

            // 跨阶段下界: XXCross >= XCross
            int startD = std::max(t.first, best);
            for (int d = startD; d <= std::min(20, best_xx - 1); ++d) {
              // 确定 Huge 表视角和初始状态
              int v_nb = getNeighborView(s1, s2);
              int v_dg = getDiagonalView(s1, s2);

              // 选择使用 Neighbor 或 Diagonal 表
              int v_huge = (v_nb != -1) ? v_nb : v_dg;
              const unsigned char *p_huge = nullptr;
              int init_e6 = -1, init_c2 = -1;

              if (v_nb != -1 && p_pt_cross_C4C5E0E1) {
                p_huge = p_pt_cross_C4C5E0E1;
                init_e6 = st[v_nb].i_e6_nb;
                init_c2 = st[v_nb].i_c2_nb;
              } else if (v_dg != -1 && p_pt_cross_C4C6E0E2) {
                p_huge = p_pt_cross_C4C6E0E2;
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
            int t_ab = getPlusTableIdx(s1, s2), t_ba = getPlusTableIdx(s2, s1);
            int t_bc = getPlusTableIdx(s2, s3), t_cb = getPlusTableIdx(s3, s2);
            int t_ac = getPlusTableIdx(s1, s3), t_ca = getPlusTableIdx(s3, s1);

            // Pruning check for all 3 views
            long long idx1 =
                (long long)(st[s1].i1 + st[s1].i2) * 24 + st[s1].i3;
            int h1 = std::max({get_prune(p_pt_cross_C4E0, idx1),
                               get_prune(p_pt_cross_CEE[t_ab],
                                         idx1 * 24 + st[s1].e_trk[t_ab]),
                               get_prune(p_pt_cross_CCE[t_ab],
                                         idx1 * 24 + st[s1].c_trk[t_ab]),
                               get_prune(p_pt_cross_CEE[t_ac],
                                         idx1 * 24 + st[s1].e_trk[t_ac]),
                               get_prune(p_pt_cross_CCE[t_ac],
                                         idx1 * 24 + st[s1].c_trk[t_ac])});

            long long idx2 =
                (long long)(st[s2].i1 + st[s2].i2) * 24 + st[s2].i3;
            int h2 = std::max({get_prune(p_pt_cross_C4E0, idx2),
                               get_prune(p_pt_cross_CEE[t_ba],
                                         idx2 * 24 + st[s2].e_trk[t_ba]),
                               get_prune(p_pt_cross_CCE[t_ba],
                                         idx2 * 24 + st[s2].c_trk[t_ba]),
                               get_prune(p_pt_cross_CEE[t_bc],
                                         idx2 * 24 + st[s2].e_trk[t_bc]),
                               get_prune(p_pt_cross_CCE[t_bc],
                                         idx2 * 24 + st[s2].c_trk[t_bc])});

            long long idx3 =
                (long long)(st[s3].i1 + st[s3].i2) * 24 + st[s3].i3;
            int h3 = std::max({get_prune(p_pt_cross_C4E0, idx3),
                               get_prune(p_pt_cross_CEE[t_ca],
                                         idx3 * 24 + st[s3].e_trk[t_ca]),
                               get_prune(p_pt_cross_CCE[t_ca],
                                         idx3 * 24 + st[s3].c_trk[t_ca]),
                               get_prune(p_pt_cross_CEE[t_cb],
                                         idx3 * 24 + st[s3].e_trk[t_cb]),
                               get_prune(p_pt_cross_CCE[t_cb],
                                         idx3 * 24 + st[s3].c_trk[t_cb])});

            // 3-Corner Pruning
            int d_3c = 0;
            if ((t_ab == 0 && t_ac == 1) ||
                (t_ac == 0 && t_ab == 1)) { // View A
              int c_r = (t_ab == 0) ? st[s1].c_trk[t_ab] : st[s1].c_trk[t_ac];
              int c_d = (t_ab == 0) ? st[s1].c_trk[t_ac] : st[s1].c_trk[t_ab];
              d_3c = std::max(
                  d_3c, get_prune(p_pt_cross_C4C5C6,
                                  ((long long)(st[s1].i1 + st[s1].i2) * 24 +
                                   c_r) * 24 +
                                      c_d));
            }
            if ((t_ba == 0 && t_bc == 1) ||
                (t_bc == 0 && t_ba == 1)) { // View B
              int c_r = (t_ba == 0) ? st[s2].c_trk[t_ba] : st[s2].c_trk[t_bc];
              int c_d = (t_ba == 0) ? st[s2].c_trk[t_bc] : st[s2].c_trk[t_ba];
              d_3c = std::max(
                  d_3c, get_prune(p_pt_cross_C4C5C6,
                                  ((long long)(st[s2].i1 + st[s2].i2) * 24 +
                                   c_r) * 24 +
                                      c_d));
            }
            if ((t_ca == 0 && t_cb == 1) ||
                (t_cb == 0 && t_ca == 1)) { // View C
              int c_r = (t_ca == 0) ? st[s3].c_trk[t_ca] : st[s3].c_trk[t_cb];
              int c_d = (t_ca == 0) ? st[s3].c_trk[t_cb] : st[s3].c_trk[t_ca];
              d_3c = std::max(
                  d_3c, get_prune(p_pt_cross_C4C5C6,
                                  ((long long)(st[s3].i1 + st[s3].i2) * 24 +
                                   c_r) * 24 +
                                      c_d));
            }

            int pr_de = get_prune(p_pt_ep4eo12,
                                  (long long)st[s1].idep * StateSpace::EO12 +
                                      st[s1].ieo);
            int h = std::max({h1, h2, h3, pr_de, d_3c});
            tasks_xxx.push_back({h, tr});
          }
          std::sort(tasks_xxx.begin(), tasks_xxx.end());

          best_xxx = (sym & 1) ? res[24 + sym - 1] : 99;
          for (auto &t : tasks_xxx) {
            if (t.first >= best_xxx)
              break;
            if (t.first == 0) {
              best_xxx = 0;
              break;
            }
            int s1 = trips[t.second][0], s2 = trips[t.second][1],
                s3 = trips[t.second][2];
            int t_ab = getPlusTableIdx(s1, s2), t_ba = getPlusTableIdx(s2, s1);
            int t_bc = getPlusTableIdx(s2, s3), t_cb = getPlusTableIdx(s3, s2);
            int t_ac = getPlusTableIdx(s1, s3), t_ca = getPlusTableIdx(s3, s1);

            // 跨阶段下界: XXXCross >= XXCross
            int startD = std::max(t.first, best_xx);
            for (int d = startD; d <= std::min(20, best_xxx - 1); ++d) {
              // 确定 Huge 表视角和初始状态(选择第一对s1-s2)
              int v_nb = getNeighborView(s1, s2);
              int v_dg = getDiagonalView(s1, s2);

              int v_huge = (v_nb != -1) ? v_nb : v_dg;
              const unsigned char *p_huge = nullptr;
              int init_e6 = -1, init_c2 = -1;

              if (v_nb != -1 && p_pt_cross_C4C5E0E1) {
                p_huge = p_pt_cross_C4C5E0E1;
                init_e6 = st[v_nb].i_e6_nb;
                init_c2 = st[v_nb].i_c2_nb;
              } else if (v_dg != -1 && p_pt_cross_C4C6E0E2) {
                p_huge = p_pt_cross_C4C6E0E2;
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
        // 只有 1 种组合{0, 1, 2, 3}
        {
          // 构造 4 个 ViewState，ex/cx 统一按 [Right, Diag, Left] 排列
          // NOTE: e_trk[0] 始终对应 Right, e_trk[1] 对应 Diag, e_trk[2] 对应
          // Left
          ViewState views[4];
          for (int s = 0; s < 4; ++s) {
            views[s] = {st[s].i1,
                        st[s].i2 * 18,
                        st[s].i3 * 18,
                        s,
                        {st[s].e_trk[0], st[s].e_trk[1], st[s].e_trk[2]},
                        {st[s].c_trk[0], st[s].c_trk[1], st[s].c_trk[2]}};
          }

          // 计算每个 View 的 heuristic，按从高到低排序
          int viewH[4];
          for (int v = 0; v < 4; ++v) {
            long long idx = (long long)(st[v].i1 + st[v].i2) * 24 + st[v].i3;
            viewH[v] = get_prune(p_pt_cross_C4E0, idx);
            for (int t = 0; t < 3; ++t) {
              viewH[v] =
                  std::max(viewH[v], get_prune(p_pt_cross_CEE[t],
                                               idx * 24 + st[v].e_trk[t]));
              viewH[v] =
                  std::max(viewH[v], get_prune(p_pt_cross_CCE[t],
                                               idx * 24 + st[v].c_trk[t]));
            }
            long long idx_3c =
                ((long long)(st[v].i1 + st[v].i2) * 24 + st[v].c_trk[0]) * 24 +
                st[v].c_trk[1];
            viewH[v] = std::max(viewH[v], get_prune(p_pt_cross_C4C5C6, idx_3c));
          }

          // 按 heuristic 从高到低排序，拒绝率高的 View 先检查
          int viewOrder[4] = {0, 1, 2, 3};
          std::sort(viewOrder, viewOrder + 4,
                    [&](int a, int b) { return viewH[a] > viewH[b]; });

          // 全局初始下界 = max(所有 View heuristic, Dep+EO)
          int h_de =
              get_prune(p_pt_ep4eo12,
                        (long long)st[0].idep * StateSpace::EO12 + st[0].ieo);
          int h_max = std::max({h_de, viewH[0], viewH[1], viewH[2], viewH[3]});

          // 确定 Huge 表视角(使用 s0-s1 相邻对)
          int v_nb = getNeighborView(0, 1); // 始终是0
          int v_huge = v_nb;
          const unsigned char *p_huge = nullptr;
          int init_e6 = -1, init_c2 = -1;
          if (v_nb != -1 && p_pt_cross_C4C5E0E1) {
            p_huge = p_pt_cross_C4C5E0E1;
            init_e6 = st[v_nb].i_e6_nb;
            init_c2 = st[v_nb].i_c2_nb;
          }

          int best_xxxx = (sym & 1) ? res[36 + sym - 1] : 99;
          if (h_max == 0) {
            best_xxxx = 0;
          } else {
            // 跨阶段下界: XXXXCross >= XXXCross
            int startD = std::max(h_max, best_xxx);
            for (int d = startD; d <= std::min(20, best_xxxx - 1); ++d) {
              if (search_4(views, viewOrder, st[0].idep * 18, st[0].ieo * 18, d,
                           18, best_xxxx - 1, v_huge, p_huge, init_e6,
                           init_c2)) {
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
  CrossSolver crossSolver;
  XCrossSolver xcrossSolver;

  static void global_init() {
    printCuberootLogo();
    init_matrix();

    // 调用静态初始化方法，加载所有表（只执行一次）
    CrossSolver::static_init();
    XCrossSolver::static_init();
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
