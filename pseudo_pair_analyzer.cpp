#include "analyzer_executor.h"
#include "cube_common.h"
#include "move_tables.h"
#include "prune_tables.h"

// --- 全局统计变量重定向到 AnalyzerStats ---
#define global_nodes AnalyzerStats::globalNodes
#define completed_tasks AnalyzerStats::completedTasks
#define is_solving AnalyzerStats::isSolving

// NOTE: COUNT_NODE 宏已移至 analyzer_executor.h

struct xcross_analyzer2;

// 定义静态指针，供类内初始化
struct xcross_analyzer2 {
  static bool tables_initialized;
  static std::vector<std::vector<unsigned char>> base_prune_tables;
  static std::vector<std::vector<unsigned char>> xc_prune_tables;
  static std::vector<std::vector<unsigned char>> ec_prune_tables;
  static std::vector<std::vector<unsigned char>> pseudo_base_prune_tables;

  // 静态指针成员，指向 static vector 数据，供搜索函数使用
  static const int *p_edge_move_ptr;
  static const int *p_corner_move_ptr;
  static const int *p_multi_move_ptr;

  // Edge3 剪枝相关
  static const int *p_edge3_move_ptr;
  static const unsigned char *p_edge3_prune_ptr;
  static bool edge3_prune_available;

  int slot1, slot2, slot3, slot4, pslot1, pslot2, pslot3, pslot4;
  int max_length;
  int edge_solved2, edge_solved3, edge_solved4;

  // 成员变量用于存储中间状态，避免递归中频繁声明
  int index1, index2, index3, index4, index5, index6, index7, index8, index9,
      index10, index11, index12;

  struct StageResults {
    std::vector<int> min_xc, min_xxc, min_xxxc, min_xxxxc;
    StageResults()
        : min_xc(6, 999), min_xxc(6, 999), min_xxxc(6, 999), min_xxxxc(6, 999) {
    }
  };
  StageResults stage_results;

  static void initialize_tables() {
    if (tables_initialized)
      return;

    auto &mtm = MoveTableManager::getInstance();
    // Load all standard tables first
    if (!mtm.loadAll()) {
      std::cerr << "Error: Move tables missing. Please run table_generator.exe."
                << std::endl;
      exit(1);
    }

    // Assign pointers
    p_edge_move_ptr = mtm.getEdgeTablePtr();
    p_corner_move_ptr = mtm.getCornerTablePtr();
    p_multi_move_ptr = mtm.getCrossTablePtr();

    init_matrix();

    std::cout << "[Init] Loading Pseudo Pair Tables..." << std::endl;

    // 1. Load Base Prune Tables (Cross + 1 Corner)
    base_prune_tables.resize(4);
    for (int c = 0; c < 4; ++c) {
      std::string filename =
          "prune_table_pseudo_cross_C" + std::to_string(c + 4) + ".bin";
      if (!load_vector(base_prune_tables[c], filename)) {
        std::cerr << "Error: Missing table " << filename << std::endl;
        exit(1);
      }
    }

    // 2. Load XCross & Pair Prune Tables
    xc_prune_tables.resize(16);
    ec_prune_tables.resize(16);
    for (int e = 0; e < 4; ++e) {
      for (int c = 0; c < 4; ++c) {
        int idx = e * 4 + c;

        std::string fn_xc = "prune_table_pseudo_cross_C" +
                            std::to_string(c + 4) + "_into_slot" +
                            std::to_string(e) + ".bin";
        if (!load_vector(xc_prune_tables[idx], fn_xc)) {
          std::cerr << "Error: Missing table " << fn_xc << std::endl;
          exit(1);
        }

        std::string fn_ec = "prune_table_pseudo_pair_C" +
                            std::to_string(c + 4) + "_E" + std::to_string(e) +
                            ".bin";
        if (!load_vector(ec_prune_tables[idx], fn_ec)) {
          std::cerr << "Error: Missing table " << fn_ec << std::endl;
          exit(1);
        }
      }
    }

    // 3. Load Pseudo Base Prune Tables (XCross Base)
    pseudo_base_prune_tables.resize(16);
    for (int c = 0; c < 4; ++c) {
      for (int e = 0; e < 4; ++e) {
        int idx = c * 4 + e;
        std::string filename = "prune_table_pseudo_cross_C" +
                               std::to_string(c + 4) + "_E" +
                               std::to_string(e) + ".bin";
        if (!load_vector(pseudo_base_prune_tables[idx], filename)) {
          std::cerr << "Error: Missing table " << filename << std::endl;
          exit(1);
        }
      }
    }

    // 4. Load Edge3 Move Table and Prune Table (可选, 用于增强 search_3 启发值)
    auto &ptm = PruneTableManager::getInstance();
    // NOTE: Edge3 剪枝表通过 loadPseudoTables() 加载，此处仅加载移动表并检查
    if (mtm.loadEdge3Table()) {
      p_edge3_move_ptr = mtm.getEdge3TablePtr();
      // 尝试加载 Pseudo 表 (如果尚未加载)
      ptm.loadPseudoTables();
      if (ptm.hasPseudoCrossE0E1E2Prune()) {
        p_edge3_prune_ptr = ptm.getPseudoCrossE0E1E2PrunePtr();
        edge3_prune_available = true;
        std::cout << "[Init] Edge3 Prune Table loaded." << std::endl;
      }
    }

    std::cout << "All tables initialized." << std::endl;
    tables_initialized = true;
  }

  xcross_analyzer2() {
    initialize_tables();
    stage_results = StageResults();
  }

  bool depth_limited_search_1(int arg_index1, int arg_index2, int arg_index3,
                              int depth, int prev, const unsigned char *prune1,
                              const unsigned char *edge_prune) {
    const int *moves = valid_moves_flat[prev];
    const int count_moves = valid_moves_count[prev];

    for (int k = 0; k < count_moves; ++k) {
      COUNT_NODE
      int i = moves[k];

      int index1_tmp = p_multi_move_ptr[arg_index1 + i];
      int index2_tmp = p_corner_move_ptr[arg_index2 + i];

      int prune1_tmp = get_prune_ptr(prune1, index1_tmp + index2_tmp);
      if (prune1_tmp >= depth)
        continue;

      int index3_tmp = p_edge_move_ptr[arg_index3 + i];
      int edge_prune1_tmp =
          get_prune_ptr(edge_prune, index3_tmp * 24 + index2_tmp);
      if (edge_prune1_tmp >= depth)
        continue;

      if (depth == 1) {
        if (prune1_tmp == 0 && edge_prune1_tmp == 0) {
          return true;
        }
      } else if (depth_limited_search_1(index1_tmp, index2_tmp * 18,
                                        index3_tmp * 18, depth - 1, i, prune1,
                                        edge_prune))
        return true;
    }
    return false;
  }

  // 辅助：计算旋转后的状态索引
  void get_rotated_indices(const std::vector<int> &base_alg,
                           const std::string &rot, int &idx1, int &idx2,
                           int &idx3, int s1, int ps1,
                           const std::vector<int> &edge_index,
                           const std::vector<int> &corner_index,
                           const std::vector<int> &single_edge_index) {
    idx1 = edge_index[s1] * 24;
    idx2 = corner_index[ps1];
    idx3 = single_edge_index[s1];

    // 仍然需要转换算法，但这是为了计算状态
    std::vector<int> alg = alg_rotation(base_alg, rot);
    for (int m : alg) {
      idx1 = p_multi_move_ptr[idx1 + m];
      idx2 = p_corner_move_ptr[idx2 * 18 + m];
      idx3 = p_edge_move_ptr[idx3 * 18 + m];
    }
  }

  // 计算 Edge3 状态的共轭索引 (用于 search_3 阶段)
  // alg: 已转换的公式 (通过 alg_rotation 转换后)
  // s1, s2, s3: 三个棱块槽位 (0-3)
  // slot_k: Pseudo 参考槽位 (对应第一个角块 pslot1)
  // 返回: {edge3_state, cross_state} 或 {-1, -1} 如果表不可用
  std::pair<int, int> get_edge3_conjugated_state(const std::vector<int> &alg,
                                                 int s1, int s2, int s3,
                                                 int slot_k) {
    if (!edge3_prune_available)
      return {-1, -1};

    // 计算相对虚拟 ID (相对于 slot_k)
    // 与 pseudo_analyzer.cpp 第 424-426 行一致
    int r1 = (s1 - slot_k + 4) % 4;
    int r2 = (s2 - slot_k + 4) % 4;
    int r3 = (s3 - slot_k + 4) % 4;

    // 排序得到规范 key
    std::vector<int> keys = {r1, r2, r3};
    std::sort(keys.begin(), keys.end());

    // 确定 rot_map 索引 (与 pseudo_analyzer.cpp 第 487-494 行一致)
    int rot_idx = 0;
    if (keys[0] == 0 && keys[1] == 1 && keys[2] == 2)
      rot_idx = 0; // {0,1,2} -> Id
    else if (keys[0] == 0 && keys[1] == 1 && keys[2] == 3)
      rot_idx = 1; // {0,1,3} -> y
    else if (keys[0] == 0 && keys[1] == 2 && keys[2] == 3)
      rot_idx = 2; // {0,2,3} -> y2
    else if (keys[0] == 1 && keys[1] == 2 && keys[2] == 3)
      rot_idx = 3; // {1,2,3} -> y'

    const int *mapper = rot_map[rot_idx];

    // 初始化 Edge3 索引 (基准: E0, E1, E2 -> 位置 0, 2, 4)
    std::vector<int> target_arr = {0, 2, 4};
    int cur_e3 = array_to_index(target_arr, 3, 2, 12);

    // 初始化 Cross 索引 (基准: 已解状态, *24 版本)
    int cur_cross = 187520 * 24;

    // 应用两层共轭 (与 pseudo_analyzer.cpp 第 507-511 行一致):
    // 物理移动 -> Pseudo 移动 (conj_moves_flat) -> 旋转后移动 (rot_map)
    for (int m : alg) {
      int m_pseudo = conj_moves_flat[m][slot_k]; // 第一层共轭
      int m_rot = mapper[m_pseudo];              // 第二层共轭
      cur_e3 = p_edge3_move_ptr[cur_e3 * 18 + m_rot];
      cur_cross = p_multi_move_ptr[cur_cross + m_rot];
    }

    return {cur_e3, cur_cross / 24};
  }

  void start_search_1(int arg_slot1, int arg_pslot1,
                      std::vector<unsigned char> &prune1,
                      std::vector<unsigned char> &edge_prune,
                      std::vector<std::string> rotations,
                      const std::vector<int> &base_alg) {
    slot1 = arg_slot1;
    pslot1 = arg_pslot1;
    max_length = 20;
    std::vector<int> edge_index = {187520, 187520, 187520, 187520},
                     single_edge_index = {0, 2, 4, 6},
                     corner_index = {12, 15, 18, 21};
    struct RotTask {
      int rot_idx;
      int heuristic;
    };
    std::vector<RotTask> tasks;
    const unsigned char *p_prune1 = prune1.data();
    const unsigned char *p_edge_prune = edge_prune.data();

    for (size_t r = 0; r < rotations.size(); ++r) {
      int idx1, idx2, idx3;
      get_rotated_indices(base_alg, rotations[r], idx1, idx2, idx3, slot1,
                          pslot1, edge_index, corner_index, single_edge_index);

      int h1 = get_prune_ptr(p_prune1, idx1 + idx2);
      int h2 = get_prune_ptr(p_edge_prune, idx3 * 24 + idx2);
      tasks.push_back({(int)r, std::max(h1, h2)});
    }
    std::sort(tasks.begin(), tasks.end(),
              [](const RotTask &a, const RotTask &b) {
                return a.heuristic < b.heuristic;
              });
    std::vector<int> results(rotations.size());
    for (const auto &task : tasks) {
      int r = task.rot_idx;

      int idx1, idx2, idx3;
      get_rotated_indices(base_alg, rotations[r], idx1, idx2, idx3, slot1,
                          pslot1, edge_index, corner_index, single_edge_index);

      // 设置成员变量供搜索使用
      index1 = idx1;
      index2 = idx2;
      index3 = idx3;

      int prune1_tmp = get_prune_ptr(p_prune1, index1 + index2);
      int edge_prune1_tmp = get_prune_ptr(p_edge_prune, index3 * 24 + index2);

      if (prune1_tmp == 0 && edge_prune1_tmp == 0) {
        results[r] = 0;
      } else {
        index2 *= 18;
        index3 *= 18;
        int found = 999;
        for (int d = std::max(prune1_tmp, edge_prune1_tmp); d <= max_length;
             d++) {
          if (depth_limited_search_1(index1, index2, index3, d, 18, p_prune1,
                                     p_edge_prune)) {
            found = d;
            break;
          }
        }
        results[r] = found;
      }
    }
    for (size_t r = 0; r < rotations.size(); ++r) {
      int val = (results[r] == 999) ? 0 : results[r];
      if (val < stage_results.min_xc[r])
        stage_results.min_xc[r] = val;
    }
  }

  void xcross_analyze(std::string scramble,
                      std::vector<std::string> rotations) {
    stage_results.min_xc.assign(6, 999);
    std::vector<int> base_alg = string_to_alg(scramble);
    for (int slot1_tmp = 0; slot1_tmp < 4; slot1_tmp++) {
      for (int pslot1_tmp = 0; pslot1_tmp < 4; pslot1_tmp++) {
        int idx = slot1_tmp * 4 + pslot1_tmp;
        start_search_1(slot1_tmp, pslot1_tmp, xc_prune_tables[idx],
                       ec_prune_tables[idx], rotations, base_alg);
      }
    }
  }

  bool depth_limited_search_2(int arg_index1, int arg_index2, int arg_index4,
                              int arg_index5, int arg_index6, int depth,
                              int prev, const unsigned char *prune1,
                              const unsigned char *prune2,
                              const unsigned char *edge_prune1,
                              const unsigned char *prune_xc2) {
    const int *moves = valid_moves_flat[prev];
    const int count_moves = valid_moves_count[prev];

    for (int k = 0; k < count_moves; ++k) {
      COUNT_NODE
      int i = moves[k];

      int index1_tmp = p_multi_move_ptr[arg_index1 + i];
      int index2_tmp = p_corner_move_ptr[arg_index2 + i];
      int prune1_tmp = get_prune_ptr(prune1, index1_tmp + index2_tmp);
      if (prune1_tmp >= depth)
        continue;

      int index5_tmp = p_edge_move_ptr[arg_index5 + i];
      int edge_prune1_tmp =
          get_prune_ptr(edge_prune1, index5_tmp * 24 + index2_tmp);
      if (edge_prune1_tmp >= depth)
        continue;

      int index4_tmp = p_corner_move_ptr[arg_index4 + i];
      int prune2_tmp = get_prune_ptr(prune2, index1_tmp + index4_tmp);
      if (prune2_tmp >= depth)
        continue;

      int index6_tmp = p_edge_move_ptr[arg_index6 + i];
      long long idx_xc2 =
          (long long)(index1_tmp + index4_tmp) * 24 + index6_tmp;
      int prune_xc2_tmp = get_prune_ptr(prune_xc2, idx_xc2);
      if (prune_xc2_tmp >= depth)
        continue;

      if (depth == 1) {
        if (prune1_tmp == 0 && prune2_tmp == 0 && edge_prune1_tmp == 0 &&
            prune_xc2_tmp == 0 && index6_tmp == edge_solved2) {
          return true;
        }
      } else if (depth_limited_search_2(index1_tmp, index2_tmp * 18,
                                        index4_tmp * 18, index5_tmp * 18,
                                        index6_tmp * 18, depth - 1, i, prune1,
                                        prune2, edge_prune1, prune_xc2))
        return true;
    }
    return false;
  }

  void start_search_2(int arg_slot1, int arg_slot2, int arg_pslot1,
                      int arg_pslot2, std::vector<unsigned char> &prune1,
                      std::vector<unsigned char> &prune2,
                      std::vector<unsigned char> &edge_prune1,
                      std::vector<std::string> rotations,
                      const std::vector<int> &base_alg) {
    slot1 = arg_slot1;
    slot2 = arg_slot2;
    pslot1 = arg_pslot1;
    pslot2 = arg_pslot2;
    max_length = 20;
    std::vector<int> edge_index = {187520, 187520, 187520, 187520},
                     single_edge_index = {0, 2, 4, 6},
                     corner_index = {12, 15, 18, 21};
    int xc2_table_idx = pslot2 * 4 + slot2;
    std::vector<unsigned char> &prune_xc2 =
        pseudo_base_prune_tables[xc2_table_idx];

    const unsigned char *p_prune1 = prune1.data();
    const unsigned char *p_prune2 = prune2.data();
    const unsigned char *p_edge_prune1 = edge_prune1.data();
    const unsigned char *p_prune_xc2 = prune_xc2.data();

    struct RotTask {
      int rot_idx;
      int heuristic;
    };
    std::vector<RotTask> tasks;

    for (size_t r = 0; r < rotations.size(); ++r) {
      int idx1, idx2, idx5;
      get_rotated_indices(base_alg, rotations[r], idx1, idx2, idx5, slot1,
                          pslot1, edge_index, corner_index, single_edge_index);

      int idx1_dummy, idx4, idx6;
      get_rotated_indices(base_alg, rotations[r], idx1_dummy, idx4, idx6, slot2,
                          pslot2, edge_index, corner_index, single_edge_index);

      int h1 = get_prune_ptr(p_prune1, idx1 + idx2);
      int h2 = get_prune_ptr(p_prune2, idx1 + idx4);
      int h3 = get_prune_ptr(p_edge_prune1, idx5 * 24 + idx2);
      long long idx_xc2 = (long long)(idx1 + idx4) * 24 + idx6;
      int h4 = get_prune_ptr(p_prune_xc2, idx_xc2);
      tasks.push_back({(int)r, std::max({h1, h2, h3, h4})});
    }
    std::sort(tasks.begin(), tasks.end(),
              [](const RotTask &a, const RotTask &b) {
                return a.heuristic < b.heuristic;
              });
    std::vector<int> results(rotations.size());
    for (const auto &task : tasks) {
      int r = task.rot_idx;

      int idx1, idx2, idx5;
      get_rotated_indices(base_alg, rotations[r], idx1, idx2, idx5, slot1,
                          pslot1, edge_index, corner_index, single_edge_index);
      int idx1_dummy, idx4, idx6;
      get_rotated_indices(base_alg, rotations[r], idx1_dummy, idx4, idx6, slot2,
                          pslot2, edge_index, corner_index, single_edge_index);

      index1 = idx1;
      index2 = idx2;
      index5 = idx5;

      index4 = idx4;
      index6 = idx6;
      edge_solved2 = single_edge_index[slot2];

      int prune1_tmp = get_prune_ptr(p_prune1, index1 + index2);
      int prune2_tmp = get_prune_ptr(p_prune2, index1 + index4);
      int edge_prune1_tmp = get_prune_ptr(p_edge_prune1, index5 * 24 + index2);
      int prune_xc2_tmp = get_prune_ptr(
          p_prune_xc2, (long long)(index1 + index4) * 24 + index6);
      if (prune1_tmp == 0 && prune2_tmp == 0 && edge_prune1_tmp == 0 &&
          prune_xc2_tmp == 0 && index6 == edge_solved2) {
        results[r] = 0;
      } else {
        index2 *= 18;
        index4 *= 18;
        index5 *= 18;
        index6 *= 18;
        int found = 999;
        int start_depth =
            std::max({prune1_tmp, prune2_tmp, edge_prune1_tmp, prune_xc2_tmp});
        for (int d = start_depth; d <= max_length; d++) {
          if (depth_limited_search_2(index1, index2, index4, index5, index6, d,
                                     18, p_prune1, p_prune2, p_edge_prune1,
                                     p_prune_xc2)) {
            found = d;
            break;
          }
        }
        results[r] = found;
      }
    }
    for (size_t r = 0; r < rotations.size(); ++r) {
      int val = (results[r] == 999) ? 0 : results[r];
      if (val < stage_results.min_xxc[r])
        stage_results.min_xxc[r] = val;
    }
  }

  void xxcross_analyze(std::string scramble,
                       std::vector<std::string> rotations) {
    stage_results.min_xxc.assign(6, 999);
    std::vector<int> base_alg = string_to_alg(scramble);
    for (int slot2_tmp = 0; slot2_tmp < 4; slot2_tmp++) {
      for (int pslot2_tmp = 0; pslot2_tmp < 4; pslot2_tmp++) {
        for (int slot1_tmp = 0; slot1_tmp < 4; slot1_tmp++) {
          if (slot1_tmp == slot2_tmp)
            continue;
          for (int pslot1_tmp = 0; pslot1_tmp < 4; pslot1_tmp++) {
            if (pslot1_tmp == pslot2_tmp)
              continue;
            start_search_2(slot1_tmp, slot2_tmp, pslot1_tmp, pslot2_tmp,
                           xc_prune_tables[slot1_tmp * 4 + pslot1_tmp],
                           base_prune_tables[pslot2_tmp],
                           ec_prune_tables[slot1_tmp * 4 + pslot1_tmp],
                           rotations, base_alg);
          }
        }
      }
    }
  }

  // Search 3: 移除 sol 记录
  bool depth_limited_search_3(int arg_index1, int arg_index2, int arg_index4,
                              int arg_index6, int arg_index7, int arg_index8,
                              int arg_index9, int depth, int prev,
                              const unsigned char *prune1,
                              const unsigned char *prune2,
                              const unsigned char *prune3,
                              const unsigned char *edge_prune1,
                              const unsigned char *prune_xc3) {
    const int *moves = valid_moves_flat[prev];
    const int count_moves = valid_moves_count[prev];

    for (int k = 0; k < count_moves; ++k) {
      COUNT_NODE
      int i = moves[k];

      int index1_tmp = p_multi_move_ptr[arg_index1 + i];
      int index2_tmp = p_corner_move_ptr[arg_index2 + i];
      int prune1_tmp = get_prune_ptr(prune1, index1_tmp + index2_tmp);
      if (prune1_tmp >= depth)
        continue;

      int index7_tmp = p_edge_move_ptr[arg_index7 + i];
      int edge_prune1_tmp =
          get_prune_ptr(edge_prune1, index7_tmp * 24 + index2_tmp);
      if (edge_prune1_tmp >= depth)
        continue;

      int index4_tmp = p_corner_move_ptr[arg_index4 + i];
      int prune2_tmp = get_prune_ptr(prune2, index1_tmp + index4_tmp);
      if (prune2_tmp >= depth)
        continue;

      int index6_tmp = p_corner_move_ptr[arg_index6 + i];
      int prune3_tmp = get_prune_ptr(prune3, index1_tmp + index6_tmp);
      if (prune3_tmp >= depth)
        continue;

      int index9_tmp = p_edge_move_ptr[arg_index9 + i];
      long long idx_xc3 =
          (long long)(index1_tmp + index6_tmp) * 24 + index9_tmp;
      int prune_xc3_tmp = get_prune_ptr(prune_xc3, idx_xc3);
      if (prune_xc3_tmp >= depth)
        continue;

      int index8_tmp = p_edge_move_ptr[arg_index8 + i];

      if (depth == 1) {
        if (prune1_tmp == 0 && prune2_tmp == 0 && prune3_tmp == 0 &&
            edge_prune1_tmp == 0 && prune_xc3_tmp == 0 &&
            index8_tmp == edge_solved2 && index9_tmp == edge_solved3) {
          return true;
        }
      } else if (depth_limited_search_3(index1_tmp, index2_tmp * 18,
                                        index4_tmp * 18, index6_tmp * 18,
                                        index7_tmp * 18, index8_tmp * 18,
                                        index9_tmp * 18, depth - 1, i, prune1,
                                        prune2, prune3, edge_prune1, prune_xc3))
        return true;
    }
    return false;
  }

  void start_search_3(int arg_slot1, int arg_slot2, int arg_slot3,
                      int arg_pslot1, int arg_pslot2, int arg_pslot3,
                      std::vector<unsigned char> &prune1,
                      std::vector<unsigned char> &prune2,
                      std::vector<unsigned char> &prune3,
                      std::vector<unsigned char> &edge_prune1,
                      std::vector<std::string> rotations,
                      const std::vector<int> &base_alg) {
    slot1 = arg_slot1;
    slot2 = arg_slot2;
    slot3 = arg_slot3;
    pslot1 = arg_pslot1;
    pslot2 = arg_pslot2;
    pslot3 = arg_pslot3;
    max_length = 20;
    std::vector<int> edge_index = {187520, 187520, 187520, 187520},
                     single_edge_index = {0, 2, 4, 6},
                     corner_index = {12, 15, 18, 21};
    int xc3_table_idx = pslot3 * 4 + slot3;
    std::vector<unsigned char> &prune_xc3 =
        pseudo_base_prune_tables[xc3_table_idx];

    const unsigned char *p_prune1 = prune1.data();
    const unsigned char *p_prune2 = prune2.data();
    const unsigned char *p_prune3 = prune3.data();
    const unsigned char *p_edge_prune1 = edge_prune1.data();
    const unsigned char *p_prune_xc3 = prune_xc3.data();

    struct RotTask {
      int rot_idx;
      int heuristic;
    };
    std::vector<RotTask> tasks;

    for (size_t r = 0; r < rotations.size(); ++r) {
      int idx1, idx2, idx7;
      get_rotated_indices(base_alg, rotations[r], idx1, idx2, idx7, slot1,
                          pslot1, edge_index, corner_index, single_edge_index);
      int idx1_dummy, idx4, idx8;
      get_rotated_indices(base_alg, rotations[r], idx1_dummy, idx4, idx8, slot2,
                          pslot2, edge_index, corner_index, single_edge_index);
      int idx1_dummy2, idx6, idx9;
      get_rotated_indices(base_alg, rotations[r], idx1_dummy2, idx6, idx9,
                          slot3, pslot3, edge_index, corner_index,
                          single_edge_index);

      int h1 = get_prune_ptr(p_prune1, idx1 + idx2);
      int h2 = get_prune_ptr(p_prune2, idx1 + idx4);
      int h3 = get_prune_ptr(p_prune3, idx1 + idx6);
      int h4 = get_prune_ptr(p_edge_prune1, idx7 * 24 + idx2);
      long long idx_xc3 = (long long)(idx1 + idx6) * 24 + idx9;
      int h5 = get_prune_ptr(p_prune_xc3, idx_xc3);

      // Edge3 剪枝 (使用两层共轭机制复用单张表)
      int h_e3 = 0;
      if (edge3_prune_available) {
        // 与 pseudo_analyzer.cpp 一致: slot_k = pslot1 (第一个角块槽位)
        std::vector<int> alg = alg_rotation(base_alg, rotations[r]);
        auto [e3_state, cross_state] =
            get_edge3_conjugated_state(alg, slot1, slot2, slot3, pslot1);
        if (e3_state >= 0) {
          long long idx_e3 = (long long)cross_state * 10560 + e3_state;
          h_e3 = get_prune_ptr(p_edge3_prune_ptr, idx_e3);
        }
      }

      tasks.push_back({(int)r, std::max({h1, h2, h3, h4, h5, h_e3})});
    }
    std::sort(tasks.begin(), tasks.end(),
              [](const RotTask &a, const RotTask &b) {
                return a.heuristic < b.heuristic;
              });
    std::vector<int> results(rotations.size());
    for (const auto &task : tasks) {
      int r = task.rot_idx;

      int idx1, idx2, idx7;
      get_rotated_indices(base_alg, rotations[r], idx1, idx2, idx7, slot1,
                          pslot1, edge_index, corner_index, single_edge_index);
      int idx1_dummy, idx4, idx8;
      get_rotated_indices(base_alg, rotations[r], idx1_dummy, idx4, idx8, slot2,
                          pslot2, edge_index, corner_index, single_edge_index);
      int idx1_dummy2, idx6, idx9;
      get_rotated_indices(base_alg, rotations[r], idx1_dummy2, idx6, idx9,
                          slot3, pslot3, edge_index, corner_index,
                          single_edge_index);

      index1 = idx1;
      index2 = idx2;
      index7 = idx7;

      index4 = idx4;
      index8 = idx8;
      edge_solved2 = single_edge_index[slot2];

      index6 = idx6;
      index9 = idx9;
      edge_solved3 = single_edge_index[slot3];

      int prune1_tmp = get_prune_ptr(p_prune1, index1 + index2);
      int prune2_tmp = get_prune_ptr(p_prune2, index1 + index4);
      int prune3_tmp = get_prune_ptr(p_prune3, index1 + index6);
      int edge_prune1_tmp = get_prune_ptr(p_edge_prune1, index7 * 24 + index2);
      int prune_xc3_tmp = get_prune_ptr(
          p_prune_xc3, (long long)(index1 + index6) * 24 + index9);
      if (prune1_tmp == 0 && prune2_tmp == 0 && prune3_tmp == 0 &&
          edge_prune1_tmp == 0 && prune_xc3_tmp == 0 &&
          index8 == edge_solved2 && index9 == edge_solved3) {
        results[r] = 0;
      } else {
        index2 *= 18;
        index4 *= 18;
        index6 *= 18;
        index7 *= 18;
        index8 *= 18;
        index9 *= 18;
        int found = 999;
        int start_depth = std::max({prune1_tmp, prune2_tmp, prune3_tmp,
                                    edge_prune1_tmp, prune_xc3_tmp});
        for (int d = start_depth; d <= max_length; d++) {
          if (depth_limited_search_3(index1, index2, index4, index6, index7,
                                     index8, index9, d, 18, p_prune1, p_prune2,
                                     p_prune3, p_edge_prune1, p_prune_xc3)) {
            found = d;
            break;
          }
        }
        results[r] = found;
      }
    }
    for (size_t r = 0; r < rotations.size(); ++r) {
      int val = (results[r] == 999) ? 0 : results[r];
      if (val < stage_results.min_xxxc[r])
        stage_results.min_xxxc[r] = val;
    }
  }

  void xxxcross_analyze(std::string scramble,
                        std::vector<std::string> rotations) {
    stage_results.min_xxxc.assign(6, 999);
    std::vector<int> base_alg = string_to_alg(scramble);
    std::vector<std::vector<int>> slot_tmps_set = {{0, 1}, {0, 2}, {0, 3},
                                                   {1, 2}, {1, 3}, {2, 3}},
                                  pslot_tmps_set = {{0, 1}, {0, 2}, {0, 3},
                                                    {1, 2}, {1, 3}, {2, 3}};
    for (int i = 0; i < 6; i++) {
      std::vector<int> a_slot_tmps = {0, 1, 2, 3};
      for (int i_tmp = 1; i_tmp >= 0; i_tmp--)
        a_slot_tmps.erase(std::remove(a_slot_tmps.begin(), a_slot_tmps.end(),
                                      slot_tmps_set[i][i_tmp]),
                          a_slot_tmps.end());
      for (int j = 0; j < 6; j++) {
        std::vector<int> a_pslot_tmps = {0, 1, 2, 3};
        for (int i_tmp = 1; i_tmp >= 0; i_tmp--)
          a_pslot_tmps.erase(std::remove(a_pslot_tmps.begin(),
                                         a_pslot_tmps.end(),
                                         slot_tmps_set[j][i_tmp]),
                             a_pslot_tmps.end());
        for (int slot1_tmp : a_slot_tmps) {
          for (int pslot1_tmp : a_pslot_tmps) {
            start_search_3(slot1_tmp, slot_tmps_set[i][0], slot_tmps_set[i][1],
                           pslot1_tmp, pslot_tmps_set[j][0],
                           pslot_tmps_set[j][1],
                           xc_prune_tables[slot1_tmp * 4 + pslot1_tmp],
                           base_prune_tables[pslot_tmps_set[j][0]],
                           base_prune_tables[pslot_tmps_set[j][1]],
                           ec_prune_tables[slot1_tmp * 4 + pslot1_tmp],
                           rotations, base_alg);
          }
        }
      }
    }
  }

  // Search 4
  bool depth_limited_search_4(int arg_index1, int arg_index2, int arg_index4,
                              int arg_index6, int arg_index8, int arg_index9,
                              int arg_index10, int arg_index11, int arg_index12,
                              int depth, int prev, const unsigned char *prune1,
                              const unsigned char *prune2,
                              const unsigned char *prune3,
                              const unsigned char *prune4,
                              const unsigned char *edge_prune1,
                              const unsigned char *prune_xc4) {
    const int *moves = valid_moves_flat[prev];
    const int count_moves = valid_moves_count[prev];

    for (int k = 0; k < count_moves; ++k) {
      COUNT_NODE
      int i = moves[k];

      int index1_tmp = p_multi_move_ptr[arg_index1 + i];
      int index2_tmp = p_corner_move_ptr[arg_index2 + i];
      int prune1_tmp = get_prune_ptr(prune1, index1_tmp + index2_tmp);
      if (prune1_tmp >= depth)
        continue;

      // [优化] 提前检查 Edge Pruning，通常比多表联合检查更快且过滤率高
      int index9_tmp = p_edge_move_ptr[arg_index9 + i];
      int edge_prune1_tmp =
          get_prune_ptr(edge_prune1, index9_tmp * 24 + index2_tmp);
      if (edge_prune1_tmp >= depth)
        continue;

      int index4_tmp = p_corner_move_ptr[arg_index4 + i];
      int prune2_tmp = get_prune_ptr(prune2, index1_tmp + index4_tmp);
      if (prune2_tmp >= depth)
        continue;

      int index6_tmp = p_corner_move_ptr[arg_index6 + i];
      int prune3_tmp = get_prune_ptr(prune3, index1_tmp + index6_tmp);
      if (prune3_tmp >= depth)
        continue;

      int index8_tmp = p_corner_move_ptr[arg_index8 + i];
      int prune4_tmp = get_prune_ptr(prune4, index1_tmp + index8_tmp);
      if (prune4_tmp >= depth)
        continue;

      int index12_tmp = p_edge_move_ptr[arg_index12 + i];
      long long idx_xc4 =
          (long long)(index1_tmp + index8_tmp) * 24 + index12_tmp;
      int prune_xc4_tmp = get_prune_ptr(prune_xc4, idx_xc4);
      if (prune_xc4_tmp >= depth)
        continue;

      int index10_tmp = p_edge_move_ptr[arg_index10 + i];
      int index11_tmp = p_edge_move_ptr[arg_index11 + i];

      if (depth == 1) {
        if (prune1_tmp == 0 && prune2_tmp == 0 && prune3_tmp == 0 &&
            prune4_tmp == 0 && edge_prune1_tmp == 0 && prune_xc4_tmp == 0 &&
            index10_tmp == edge_solved2 && index11_tmp == edge_solved3 &&
            index12_tmp == edge_solved4) {
          return true;
        }
      } else if (depth_limited_search_4(
                     index1_tmp, index2_tmp * 18, index4_tmp * 18,
                     index6_tmp * 18, index8_tmp * 18, index9_tmp * 18,
                     index10_tmp * 18, index11_tmp * 18, index12_tmp * 18,
                     depth - 1, i, prune1, prune2, prune3, prune4, edge_prune1,
                     prune_xc4))
        return true;
    }
    return false;
  }

  void start_search_4(
      int arg_slot1, int arg_slot2, int arg_slot3, int arg_slot4,
      int arg_pslot1, int arg_pslot2, int arg_pslot3, int arg_pslot4,
      std::vector<unsigned char> &prune1, std::vector<unsigned char> &prune2,
      std::vector<unsigned char> &prune3, std::vector<unsigned char> &prune4,
      std::vector<unsigned char> &edge_prune1,
      std::vector<std::string> rotations, const std::vector<int> &base_alg) {
    slot1 = arg_slot1;
    slot2 = arg_slot2;
    slot3 = arg_slot3;
    slot4 = arg_slot4;
    pslot1 = arg_pslot1;
    pslot2 = arg_pslot2;
    pslot3 = arg_pslot3;
    pslot4 = arg_pslot4;
    max_length = 20;
    std::vector<int> edge_index = {187520, 187520, 187520, 187520},
                     single_edge_index = {0, 2, 4, 6},
                     corner_index = {12, 15, 18, 21};
    int xc4_table_idx = pslot4 * 4 + slot4;
    std::vector<unsigned char> &prune_xc4 =
        pseudo_base_prune_tables[xc4_table_idx];

    const unsigned char *p_prune1 = prune1.data();
    const unsigned char *p_prune2 = prune2.data();
    const unsigned char *p_prune3 = prune3.data();
    const unsigned char *p_prune4 = prune4.data();
    const unsigned char *p_edge_prune1 = edge_prune1.data();
    const unsigned char *p_prune_xc4 = prune_xc4.data();

    struct RotTask {
      int rot_idx;
      int heuristic;
    };
    std::vector<RotTask> tasks;

    for (size_t r = 0; r < rotations.size(); ++r) {
      int idx1, idx2, idx9;
      get_rotated_indices(base_alg, rotations[r], idx1, idx2, idx9, slot1,
                          pslot1, edge_index, corner_index, single_edge_index);
      int idx1_dummy, idx4, idx10;
      get_rotated_indices(base_alg, rotations[r], idx1_dummy, idx4, idx10,
                          slot2, pslot2, edge_index, corner_index,
                          single_edge_index);
      int idx1_dummy2, idx6, idx11;
      get_rotated_indices(base_alg, rotations[r], idx1_dummy2, idx6, idx11,
                          slot3, pslot3, edge_index, corner_index,
                          single_edge_index);
      int idx1_dummy3, idx8, idx12;
      get_rotated_indices(base_alg, rotations[r], idx1_dummy3, idx8, idx12,
                          slot4, pslot4, edge_index, corner_index,
                          single_edge_index);

      int h1 = get_prune_ptr(p_prune1, idx1 + idx2);
      int h2 = get_prune_ptr(p_prune2, idx1 + idx4);
      int h3 = get_prune_ptr(p_prune3, idx1 + idx6);
      int h4 = get_prune_ptr(p_prune4, idx1 + idx8);
      int h5 = get_prune_ptr(p_edge_prune1, idx9 * 24 + idx2);
      long long idx_xc4 = (long long)(idx1 + idx8) * 24 + idx12;
      int h6 = get_prune_ptr(p_prune_xc4, idx_xc4);
      tasks.push_back({(int)r, std::max({h1, h2, h3, h4, h5, h6})});
    }
    std::sort(tasks.begin(), tasks.end(),
              [](const RotTask &a, const RotTask &b) {
                return a.heuristic < b.heuristic;
              });
    std::vector<int> results(rotations.size());
    for (const auto &task : tasks) {
      int r = task.rot_idx;

      int idx1, idx2, idx9;
      get_rotated_indices(base_alg, rotations[r], idx1, idx2, idx9, slot1,
                          pslot1, edge_index, corner_index, single_edge_index);
      int idx1_dummy, idx4, idx10;
      get_rotated_indices(base_alg, rotations[r], idx1_dummy, idx4, idx10,
                          slot2, pslot2, edge_index, corner_index,
                          single_edge_index);
      int idx1_dummy2, idx6, idx11;
      get_rotated_indices(base_alg, rotations[r], idx1_dummy2, idx6, idx11,
                          slot3, pslot3, edge_index, corner_index,
                          single_edge_index);
      int idx1_dummy3, idx8, idx12;
      get_rotated_indices(base_alg, rotations[r], idx1_dummy3, idx8, idx12,
                          slot4, pslot4, edge_index, corner_index,
                          single_edge_index);

      index1 = idx1;
      index2 = idx2;
      index9 = idx9;

      index4 = idx4;
      index10 = idx10;
      edge_solved2 = single_edge_index[slot2];

      index6 = idx6;
      index11 = idx11;
      edge_solved3 = single_edge_index[slot3];

      index8 = idx8;
      index12 = idx12;
      edge_solved4 = single_edge_index[slot4];

      int prune1_tmp = get_prune_ptr(p_prune1, index1 + index2);
      int prune2_tmp = get_prune_ptr(p_prune2, index1 + index4);
      int prune3_tmp = get_prune_ptr(p_prune3, index1 + index6);
      int prune4_tmp = get_prune_ptr(p_prune4, index1 + index8);
      int edge_prune1_tmp = get_prune_ptr(p_edge_prune1, index9 * 24 + index2);
      int prune_xc4_tmp = get_prune_ptr(
          p_prune_xc4, (long long)(index1 + index8) * 24 + index12);
      if (prune1_tmp == 0 && prune2_tmp == 0 && prune3_tmp == 0 &&
          prune4_tmp == 0 && edge_prune1_tmp == 0 && prune_xc4_tmp == 0 &&
          index10 == edge_solved2 && index11 == edge_solved3 &&
          index12 == edge_solved4) {
        results[r] = 0;
      } else {
        index2 *= 18;
        index4 *= 18;
        index6 *= 18;
        index8 *= 18;
        index9 *= 18;
        index10 *= 18;
        index11 *= 18;
        index12 *= 18;
        int found = 999;
        int start_depth =
            std::max({prune1_tmp, prune2_tmp, prune3_tmp, prune4_tmp,
                      edge_prune1_tmp, prune_xc4_tmp});
        for (int d = start_depth; d <= max_length; d++) {
          if (depth_limited_search_4(index1, index2, index4, index6, index8,
                                     index9, index10, index11, index12, d, 18,
                                     p_prune1, p_prune2, p_prune3, p_prune4,
                                     p_edge_prune1, p_prune_xc4)) {
            found = d;
            break;
          }
        }
        results[r] = found;
      }
    }
    for (size_t r = 0; r < rotations.size(); ++r) {
      int val = (results[r] == 999) ? 0 : results[r];
      if (val < stage_results.min_xxxxc[r])
        stage_results.min_xxxxc[r] = val;
    }
  }

  void xxxxcross_analyze(std::string scramble,
                         std::vector<std::string> rotations) {
    stage_results.min_xxxxc.assign(6, 999);
    std::vector<int> base_alg = string_to_alg(scramble);
    for (int i = 3; i >= 0; --i) {
      std::vector<int> s_rem;
      for (int k = 0; k < 4; ++k)
        if (k != i)
          s_rem.push_back(k);
      for (int j = 0; j < 4; ++j) {
        std::vector<int> p_rem;
        for (int k = 0; k < 4; ++k)
          if (k != j)
            p_rem.push_back(k);
        start_search_4(i, s_rem[0], s_rem[1], s_rem[2], j, p_rem[0], p_rem[1],
                       p_rem[2], xc_prune_tables[i * 4 + j],
                       base_prune_tables[p_rem[0]], base_prune_tables[p_rem[1]],
                       base_prune_tables[p_rem[2]], ec_prune_tables[i * 4 + j],
                       rotations, base_alg);
      }
    }
  }
};

bool xcross_analyzer2::tables_initialized = false;
std::vector<std::vector<unsigned char>> xcross_analyzer2::base_prune_tables;
std::vector<std::vector<unsigned char>> xcross_analyzer2::xc_prune_tables;
std::vector<std::vector<unsigned char>> xcross_analyzer2::ec_prune_tables;
std::vector<std::vector<unsigned char>>
    xcross_analyzer2::pseudo_base_prune_tables;

const int *xcross_analyzer2::p_edge_move_ptr = nullptr;
const int *xcross_analyzer2::p_corner_move_ptr = nullptr;
const int *xcross_analyzer2::p_multi_move_ptr = nullptr;

// Edge3 静态成员初始化
const int *xcross_analyzer2::p_edge3_move_ptr = nullptr;
const unsigned char *xcross_analyzer2::p_edge3_prune_ptr = nullptr;
bool xcross_analyzer2::edge3_prune_available = false;

std::string analyzer_compute(xcross_analyzer2 &xcs, std::string scramble,
                             std::string id) {
  xcs.stage_results = xcross_analyzer2::StageResults();

  std::vector<std::string> rotations;
  std::string rot_set = "DULRFB";
  for (char c_tmp : rot_set) {
    std::string c(1, c_tmp);
    if (c == "D")
      rotations.emplace_back("");
    else if (c == "U")
      rotations.emplace_back("z2");
    else if (c == "L")
      rotations.emplace_back("z'");
    else if (c == "R")
      rotations.emplace_back("z");
    else if (c == "F")
      rotations.emplace_back("x'");
    else
      rotations.emplace_back("x");
  }
  xcs.xcross_analyze(scramble, rotations);
  xcs.xxcross_analyze(scramble, rotations);
  xcs.xxxcross_analyze(scramble, rotations);
  xcs.xxxxcross_analyze(scramble, rotations);
  std::ostringstream oss;
  oss << id;
  for (int val : xcs.stage_results.min_xc) {
    if (val == 999)
      val = 0;
    oss << "," << val;
  }
  for (int val : xcs.stage_results.min_xxc) {
    if (val == 999)
      val = 0;
    oss << "," << val;
  }
  for (int val : xcs.stage_results.min_xxxc) {
    if (val == 999)
      val = 0;
    oss << "," << val;
  }
  for (int val : xcs.stage_results.min_xxxxc) {
    if (val == 999)
      val = 0;
    oss << "," << val;
  }

  oss << "\n";
  return oss.str();
}

// --- PseudoPairSolverWrapper: 封装xcross_analyzer2的统一接口 ---
struct PseudoPairSolverWrapper {
  static inline std::vector<std::string> rots = {"",  "z2", "z'",
                                                 "z", "x'", "x"};

  xcross_analyzer2 analyzer;

  static void global_init() {
    init_matrix();

    std::cout << ANSI_CYAN << "[INIT] " << ANSI_RESET << "Loading Tables..."
              << std::endl;
    xcross_analyzer2::initialize_tables();
    std::cout << ANSI_CYAN << "[INIT] " << ANSI_RESET
              << "Loading Tables... Done." << std::endl;
  }

  static std::string get_csv_header() {
    std::vector<std::string> suffixes = {"_z0", "_z1", "_z2",
                                         "_z3", "_x1", "_x3"};
    std::ostringstream oss;
    oss << "id";
    for (const auto &s : suffixes)
      oss << ",pseudo_xcross_pair" << s;
    for (const auto &s : suffixes)
      oss << ",pseudo_xxcross_pair" << s;
    for (const auto &s : suffixes)
      oss << ",pseudo_xxxcross_pair" << s;
    for (const auto &s : suffixes)
      oss << ",pseudo_f2l_pair" << s;
    return oss.str();
  }

  std::string solve(const std::vector<int> &alg, const std::string &id) {
    // 将alg转换为字符串 (analyzer_compute需要scramble字符串)
    std::ostringstream scrambleOss;
    for (size_t i = 0; i < alg.size(); ++i) {
      if (i > 0)
        scrambleOss << " ";
      scrambleOss << move_names[alg[i]];
    }
    std::string scramble = scrambleOss.str();

    std::string result = analyzer_compute(analyzer, scramble, id);
    // 移除尾部换行符
    if (!result.empty() && result.back() == '\n') {
      result.pop_back();
    }
    return result;
  }

  static void print_stats() {}
};

int main() {
  run_analyzer_app<PseudoPairSolverWrapper>("_pseudo_pair");
  return 0;
}
