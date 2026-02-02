#include "analyzer_executor.h"
#include "cube_common.h"
#include "move_tables.h"
#include "prune_tables.h"
#include <map>

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
  static const int *p_edge3_move_ptr;   // Edge3 移动表指针
  static const int *p_corner3_move_ptr; // [新增] Corner3 移动表指针
  static const int *p_corner2_move_ptr; // [新增] Corner2 移动表指针
  static const int *p_edge2_move_ptr;   // [新增] Edge2 移动表指针

  // Edge3 剪枝表 (用于 Search 4: 3 个归位槽位的棱块联合剪枝)
  static std::vector<unsigned char> prune_e0e1e2;
  static std::vector<unsigned char> prune_e1e2e3;
  static std::vector<unsigned char> prune_e0e2e3;
  static std::vector<unsigned char> prune_e0e1e3;

  // [新增] Corner3 剪枝表 (用于 Search 4: 3 个归位槽位的角块联合剪枝)
  static std::vector<unsigned char> prune_c4c5c6;
  static std::vector<unsigned char> prune_c4c5c7;
  static std::vector<unsigned char> prune_c4c6c7;
  static std::vector<unsigned char> prune_c5c6c7;

  // [新增] Corner2 剪枝表 (用于 Search 3)
  static std::vector<unsigned char> prune_c4c5;
  static std::vector<unsigned char> prune_c4c6;
  static std::vector<unsigned char> prune_c4c7;
  static std::vector<unsigned char> prune_c5c6;
  static std::vector<unsigned char> prune_c5c7;
  static std::vector<unsigned char> prune_c6c7;

  // [新增] Edge2 剪枝表 (用于 Search 3)
  static std::vector<unsigned char> prune_e0e1;
  static std::vector<unsigned char> prune_e0e2;
  static std::vector<unsigned char> prune_e0e3;
  static std::vector<unsigned char> prune_e1e2;
  static std::vector<unsigned char> prune_e1e3;
  static std::vector<unsigned char> prune_e2e3;

  // [新增] 辅助剪枝定义 (用于通用 AuxState 架构)
  struct AuxPrunerDef {
    const unsigned char *p_prune; // 剪枝表指针
    const int *p_move;            // 移动表指针
    int multiplier;               // 状态乘数 (Corner3=9072, Edge3=10560)
  };

  struct AuxState {
    const AuxPrunerDef *def = nullptr;
    int current_idx = 0;
    int current_cross_scaled = 0;     // 虚拟 Cross 状态 * 24
    const int *move_mapper = nullptr; // rot_map[rot_idx]
    int slot_k = 0;                   // 共轭参考槽位 (pslot1)
  };

  static constexpr int MAX_AUX = 8;
  static std::map<std::vector<int>, AuxPrunerDef> aux_registry;

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

    // 4. [新增] Load Edge3 Prune Tables for Search 4
    mtm.loadEdge3Table();
    p_edge3_move_ptr = mtm.getEdge3TablePtr();

    if (!load_vector(prune_e0e1e2, "prune_table_pseudo_cross_E0_E1_E2.bin")) {
      std::cerr << "Error: Missing prune_table_pseudo_cross_E0_E1_E2.bin"
                << std::endl;
      exit(1);
    }
    if (!load_vector(prune_e1e2e3, "prune_table_pseudo_cross_E1_E2_E3.bin")) {
      std::cerr << "Error: Missing prune_table_pseudo_cross_E1_E2_E3.bin"
                << std::endl;
      exit(1);
    }
    if (!load_vector(prune_e0e2e3, "prune_table_pseudo_cross_E0_E2_E3.bin")) {
      std::cerr << "Error: Missing prune_table_pseudo_cross_E0_E2_E3.bin"
                << std::endl;
      exit(1);
    }
    if (!load_vector(prune_e0e1e3, "prune_table_pseudo_cross_E0_E1_E3.bin")) {
      std::cerr << "Error: Missing prune_table_pseudo_cross_E0_E1_E3.bin"
                << std::endl;
      exit(1);
    }
    std::cout << "[Init] Edge3 Prune Tables loaded." << std::endl;

    // 5. [新增] Load Corner3 Prune Tables for Search 4
    mtm.loadCorner3Table();
    p_corner3_move_ptr = mtm.getCorner3TablePtr();

    if (!load_vector(prune_c4c5c6, "prune_table_pseudo_cross_C4_C5_C6.bin")) {
      std::cerr << "Error: Missing prune_table_pseudo_cross_C4_C5_C6.bin"
                << std::endl;
      exit(1);
    }
    if (!load_vector(prune_c4c5c7, "prune_table_pseudo_cross_C4_C5_C7.bin")) {
      std::cerr << "Error: Missing prune_table_pseudo_cross_C4_C5_C7.bin"
                << std::endl;
      exit(1);
    }
    if (!load_vector(prune_c4c6c7, "prune_table_pseudo_cross_C4_C6_C7.bin")) {
      std::cerr << "Error: Missing prune_table_pseudo_cross_C4_C6_C7.bin"
                << std::endl;
      exit(1);
    }
    if (!load_vector(prune_c5c6c7, "prune_table_pseudo_cross_C5_C6_C7.bin")) {
      std::cerr << "Error: Missing prune_table_pseudo_cross_C5_C6_C7.bin"
                << std::endl;
      exit(1);
    }
    std::cout << "[Init] Corner3 Prune Tables loaded." << std::endl;

    // 6. [新增] Load Corner2 Prune Tables for Search 3
    mtm.loadCorner2Table();
    p_corner2_move_ptr = mtm.getCorner2TablePtr();

    if (!load_vector(prune_c4c5, "prune_table_pseudo_cross_C4_C5.bin")) {
      std::cerr << "Error: Missing prune_table_pseudo_cross_C4_C5.bin"
                << std::endl;
      exit(1);
    }
    if (!load_vector(prune_c4c6, "prune_table_pseudo_cross_C4_C6.bin")) {
      std::cerr << "Error: Missing prune_table_pseudo_cross_C4_C6.bin"
                << std::endl;
      exit(1);
    }
    if (!load_vector(prune_c4c7, "prune_table_pseudo_cross_C4_C7.bin")) {
      std::cerr << "Error: Missing prune_table_pseudo_cross_C4_C7.bin"
                << std::endl;
      exit(1);
    }
    if (!load_vector(prune_c5c6, "prune_table_pseudo_cross_C5_C6.bin")) {
      std::cerr << "Error: Missing prune_table_pseudo_cross_C5_C6.bin"
                << std::endl;
      exit(1);
    }
    if (!load_vector(prune_c5c7, "prune_table_pseudo_cross_C5_C7.bin")) {
      std::cerr << "Error: Missing prune_table_pseudo_cross_C5_C7.bin"
                << std::endl;
      exit(1);
    }
    if (!load_vector(prune_c6c7, "prune_table_pseudo_cross_C6_C7.bin")) {
      std::cerr << "Error: Missing prune_table_pseudo_cross_C6_C7.bin"
                << std::endl;
      exit(1);
    }
    std::cout << "[Init] Corner2 Prune Tables loaded." << std::endl;

    // 7. [新增] Load Edge2 Prune Tables for Search 3
    mtm.loadEdges2Table();
    p_edge2_move_ptr = mtm.getEdges2TablePtr();

    if (!load_vector(prune_e0e1, "prune_table_pseudo_cross_E0_E1.bin")) {
      std::cerr << "Error: Missing prune_table_pseudo_cross_E0_E1.bin"
                << std::endl;
      exit(1);
    }
    if (!load_vector(prune_e0e2, "prune_table_pseudo_cross_E0_E2.bin")) {
      std::cerr << "Error: Missing prune_table_pseudo_cross_E0_E2.bin"
                << std::endl;
      exit(1);
    }
    if (!load_vector(prune_e0e3, "prune_table_pseudo_cross_E0_E3.bin")) {
      std::cerr << "Error: Missing prune_table_pseudo_cross_E0_E3.bin"
                << std::endl;
      exit(1);
    }
    if (!load_vector(prune_e1e2, "prune_table_pseudo_cross_E1_E2.bin")) {
      std::cerr << "Error: Missing prune_table_pseudo_cross_E1_E2.bin"
                << std::endl;
      exit(1);
    }
    if (!load_vector(prune_e1e3, "prune_table_pseudo_cross_E1_E3.bin")) {
      std::cerr << "Error: Missing prune_table_pseudo_cross_E1_E3.bin"
                << std::endl;
      exit(1);
    }
    if (!load_vector(prune_e2e3, "prune_table_pseudo_cross_E2_E3.bin")) {
      std::cerr << "Error: Missing prune_table_pseudo_cross_E2_E3.bin"
                << std::endl;
      exit(1);
    }
    std::cout << "[Init] Edge2 Prune Tables loaded." << std::endl;

    // 8. [新增] Register aux_registry
    // Corner3 表 (multiplier = 9072)
    aux_registry[{4, 5, 6}] = {prune_c4c5c6.data(), p_corner3_move_ptr, 9072};
    aux_registry[{4, 5, 7}] = {prune_c4c5c7.data(), p_corner3_move_ptr, 9072};
    aux_registry[{4, 6, 7}] = {prune_c4c6c7.data(), p_corner3_move_ptr, 9072};
    aux_registry[{5, 6, 7}] = {prune_c5c6c7.data(), p_corner3_move_ptr, 9072};
    // Edge3 表 (multiplier = 10560)
    aux_registry[{0, 1, 2}] = {prune_e0e1e2.data(), p_edge3_move_ptr, 10560};
    aux_registry[{0, 1, 3}] = {prune_e0e1e3.data(), p_edge3_move_ptr, 10560};
    aux_registry[{0, 2, 3}] = {prune_e0e2e3.data(), p_edge3_move_ptr, 10560};
    aux_registry[{1, 2, 3}] = {prune_e1e2e3.data(), p_edge3_move_ptr, 10560};
    // Corner2 表 (multiplier = 504)
    aux_registry[{4, 5}] = {prune_c4c5.data(), p_corner2_move_ptr, 504};
    aux_registry[{4, 6}] = {prune_c4c6.data(), p_corner2_move_ptr, 504};
    aux_registry[{4, 7}] = {prune_c4c7.data(), p_corner2_move_ptr, 504};
    aux_registry[{5, 6}] = {prune_c5c6.data(), p_corner2_move_ptr, 504};
    aux_registry[{5, 7}] = {prune_c5c7.data(), p_corner2_move_ptr, 504};
    aux_registry[{6, 7}] = {prune_c6c7.data(), p_corner2_move_ptr, 504};
    // Edge2 表 (multiplier = 528)
    aux_registry[{0, 1}] = {prune_e0e1.data(), p_edge2_move_ptr, 528};
    aux_registry[{0, 2}] = {prune_e0e2.data(), p_edge2_move_ptr, 528};
    aux_registry[{0, 3}] = {prune_e0e3.data(), p_edge2_move_ptr, 528};
    aux_registry[{1, 2}] = {prune_e1e2.data(), p_edge2_move_ptr, 528};
    aux_registry[{1, 3}] = {prune_e1e3.data(), p_edge2_move_ptr, 528};
    aux_registry[{2, 3}] = {prune_e2e3.data(), p_edge2_move_ptr, 528};

    std::cout << "All tables initialized." << std::endl;
    tables_initialized = true;
  }

  xcross_analyzer2() {
    initialize_tables();
    stage_results = StageResults();
  }

  // [新增] 为 Search 4 设置 AuxState
  // 返回值: num_aux (已设置的辅助表数量)
  int setup_aux_pruners_for_search4(int slot2_arg, int slot3_arg,
                                    int slot4_arg, // Edge 槽位 (固定位)
                                    int pslot2_arg, int pslot3_arg,
                                    int pslot4_arg, // Corner 伪槽位
                                    const std::vector<int> &alg,
                                    AuxState *out_aux) {
    int count = 0;

    // 1. Corner3 表 (优先剪枝)
    std::vector<int> corner_ids = {pslot2_arg + 4, pslot3_arg + 4,
                                   pslot4_arg + 4};
    std::sort(corner_ids.begin(), corner_ids.end());
    auto it_c = aux_registry.find(corner_ids);
    if (it_c != aux_registry.end()) {
      // 计算初始索引: (id-4)*3+12
      std::vector<int> target = {(corner_ids[0] - 4) * 3 + 12,
                                 (corner_ids[1] - 4) * 3 + 12,
                                 (corner_ids[2] - 4) * 3 + 12};
      int idx = array_to_index(target, 3, 3, 8);
      // 应用 scramble
      for (int m : alg) {
        idx = p_corner3_move_ptr[idx * 18 + m];
      }
      out_aux[count].def = &it_c->second;
      out_aux[count].current_idx = idx;
      count++;
    }

    // 2. Edge3 表
    std::vector<int> edge_ids = {slot2_arg, slot3_arg, slot4_arg};
    std::sort(edge_ids.begin(), edge_ids.end());
    auto it_e = aux_registry.find(edge_ids);
    if (it_e != aux_registry.end()) {
      // 计算初始索引: id*2
      std::vector<int> target = {edge_ids[0] * 2, edge_ids[1] * 2,
                                 edge_ids[2] * 2};
      int idx = array_to_index(target, 3, 2, 12);
      // 应用 scramble
      for (int m : alg) {
        idx = p_edge3_move_ptr[idx * 18 + m];
      }
      out_aux[count].def = &it_e->second;
      out_aux[count].current_idx = idx;
      count++;
    }

    return count;
  }

  // 返回值: num_aux (已设置的辅助表数量)
  // 顺序: Corner2 优先 → Edge2 其次
  // 使用两级映射：conj_moves_flat[m][slot_k] + rot_map[rot_idx]
  int setup_aux_pruners_for_search3(int pslot1_arg, // 参考槽位 (slot_k)
                                    int slot2_arg,
                                    int slot3_arg, // Edge 槽位 (固定位)
                                    int pslot2_arg,
                                    int pslot3_arg, // Corner 伪槽位
                                    const std::vector<int> &alg,
                                    AuxState *out_aux) {
    int count = 0;
    int slot_k = pslot1_arg; // 参考槽位

    // 1. Corner2 表 (优先剪枝)
    // 计算相对槽位 (相对于 slot_k)
    {
      int r1 = ((pslot2_arg)-slot_k + 4) % 4 + 4; // 映射到 4-7 范围
      int r2 = ((pslot3_arg)-slot_k + 4) % 4 + 4;
      int k1 = r1, k2 = r2;
      if (k1 > k2)
        std::swap(k1, k2);

      bool is_diag = (k2 - k1) == 2;
      std::vector<int> canon_key =
          is_diag ? std::vector<int>{4, 6} : std::vector<int>{4, 5};

      auto it = aux_registry.find(canon_key);
      if (it != aux_registry.end()) {
        int rot_idx = 0;
        std::vector<int> target;

        if (is_diag) {
          // 对角: {4,6} -> Id, {5,7} -> y'
          rot_idx = (k1 == 4) ? 0 : 3;
          target = {12, 18}; // C4(12), C6(18)
        } else {
          // 邻接
          if (k1 == 4 && k2 == 5)
            rot_idx = 0;
          else if (k1 == 4 && k2 == 7)
            rot_idx = 1;
          else if (k1 == 6 && k2 == 7)
            rot_idx = 2;
          else if (k1 == 5 && k2 == 6)
            rot_idx = 3;
          target = {12, 15}; // C4(12), C5(15)
        }

        const int *mapper = rot_map[rot_idx];
        int init_idx = array_to_index(target, 2, 3, 8);
        int cur_c2 = init_idx;
        int cur_cr = 187520 * 24; // Solved Cross * 24

        // 两级映射：先 conj_moves_flat，再 rot_map
        for (int m : alg) {
          int m_conj = conj_moves_flat[m][slot_k]; // 物理 → slot_k 视角
          int m_rot = mapper[m_conj];              // slot_k → 规范化
          cur_c2 = p_corner2_move_ptr[cur_c2 * 18 + m_rot];
          cur_cr = p_multi_move_ptr[cur_cr + m_rot];
        }

        out_aux[count].def = &it->second;
        out_aux[count].current_idx = cur_c2;
        out_aux[count].current_cross_scaled = cur_cr;
        out_aux[count].move_mapper = mapper;
        out_aux[count].slot_k = slot_k;
        count++;
      }
    }

    // 2. Edge2 表
    // 计算相对槽位 (相对于 slot_k)
    {
      int r1 = (slot2_arg - slot_k + 4) % 4; // 映射到 0-3 范围
      int r2 = (slot3_arg - slot_k + 4) % 4;
      int k1 = r1, k2 = r2;
      if (k1 > k2)
        std::swap(k1, k2);

      bool is_diag = (k2 - k1) == 2;
      std::vector<int> canon_key =
          is_diag ? std::vector<int>{0, 2} : std::vector<int>{0, 1};

      auto it = aux_registry.find(canon_key);
      if (it != aux_registry.end()) {
        int rot_idx = 0;
        std::vector<int> target;

        if (is_diag) {
          // 对角: {0,2} -> Id, {1,3} -> y
          rot_idx = (k1 == 0) ? 0 : 1;
          target = {0, 4}; // E0(0), E2(4)
        } else {
          // 邻接
          if (k1 == 0 && k2 == 1)
            rot_idx = 0;
          else if (k1 == 0 && k2 == 3)
            rot_idx = 1;
          else if (k1 == 2 && k2 == 3)
            rot_idx = 2;
          else if (k1 == 1 && k2 == 2)
            rot_idx = 3;
          target = {0, 2}; // E0(0), E1(2)
        }

        const int *mapper = rot_map[rot_idx];
        int init_idx = array_to_index(target, 2, 2, 12);
        int cur_e2 = init_idx;
        int cur_cr = 187520 * 24; // Solved Cross * 24

        // 两级映射：先 conj_moves_flat，再 rot_map
        for (int m : alg) {
          int m_conj = conj_moves_flat[m][slot_k]; // 物理 → slot_k 视角
          int m_rot = mapper[m_conj];              // slot_k → 规范化
          cur_e2 = p_edge2_move_ptr[cur_e2 * 18 + m_rot];
          cur_cr = p_multi_move_ptr[cur_cr + m_rot];
        }

        out_aux[count].def = &it->second;
        out_aux[count].current_idx = cur_e2;
        out_aux[count].current_cross_scaled = cur_cr;
        out_aux[count].move_mapper = mapper;
        out_aux[count].slot_k = slot_k;
        count++;
      }
    }

    return count;
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

  bool depth_limited_search_3(
      int arg_index1, int arg_index2, int arg_index4, int arg_index6,
      int arg_index7, int arg_index8, int arg_index9, int depth, int prev,
      const unsigned char *prune1, const unsigned char *prune2,
      const unsigned char *prune3, const unsigned char *edge_prune1,
      const unsigned char *prune_xc3, int num_aux, const AuxState *aux_states) {
    const int *moves = valid_moves_flat[prev];
    const int count_moves = valid_moves_count[prev];

    for (int k = 0; k < count_moves; ++k) {
      COUNT_NODE
      int i = moves[k];

      // 1. 首先计算 Cross 状态索引 (用于 Aux 剪枝)
      int index1_tmp = p_multi_move_ptr[arg_index1 + i];
      int cross_state_idx = index1_tmp / 24;

      // 2. Aux Pruning (最先! Corner2 → Edge2)
      bool aux_pruned = false;
      AuxState next_aux[MAX_AUX];
      for (int a = 0; a < num_aux; ++a) {
        const auto &cur = aux_states[a];
        if (!cur.def)
          continue;
        next_aux[a].def = cur.def;
        next_aux[a].move_mapper = cur.move_mapper;
        next_aux[a].slot_k = cur.slot_k;

        int lookup_cross_idx;
        if (cur.move_mapper) {
          // 两级映射：先 conj_moves_flat，再 rot_map
          int m_conj = conj_moves_flat[i][cur.slot_k]; // 物理 → slot_k 视角
          int m_rot = cur.move_mapper[m_conj];         // slot_k → 规范化
          next_aux[a].current_idx =
              cur.def->p_move[cur.current_idx * 18 + m_rot];
          next_aux[a].current_cross_scaled =
              p_multi_move_ptr[cur.current_cross_scaled + m_rot];
          lookup_cross_idx = next_aux[a].current_cross_scaled / 24;
        } else {
          // 标准逻辑 (无 mapper)
          next_aux[a].current_idx = cur.def->p_move[cur.current_idx * 18 + i];
          lookup_cross_idx = cross_state_idx;
        }

        long long idx_aux = (long long)lookup_cross_idx * cur.def->multiplier +
                            next_aux[a].current_idx;
        if (get_prune_ptr(cur.def->p_prune, idx_aux) >= depth) {
          aux_pruned = true;
          break;
        }
      }
      if (aux_pruned)
        continue;

      // 3. Base Pruning 继续原有逻辑
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
      } else if (depth_limited_search_3(
                     index1_tmp, index2_tmp * 18, index4_tmp * 18,
                     index6_tmp * 18, index7_tmp * 18, index8_tmp * 18,
                     index9_tmp * 18, depth - 1, i, prune1, prune2, prune3,
                     edge_prune1, prune_xc3, num_aux, next_aux))
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
      tasks.push_back({(int)r, std::max({h1, h2, h3, h4, h5})});
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
        // [新增] 设置 AuxState (Corner2 + Edge2)
        std::vector<int> rotated_alg = alg_rotation(base_alg, rotations[r]);
        AuxState aux_init[MAX_AUX];
        int num_aux = setup_aux_pruners_for_search3(
            pslot1, slot2, slot3, pslot2, pslot3, rotated_alg, aux_init);

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
                                     p_prune3, p_edge_prune1, p_prune_xc3,
                                     num_aux, aux_init)) {
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
                              const unsigned char *prune_xc4, int num_aux,
                              AuxState *aux_states) { // [重构] 使用 AuxState
    const int *moves = valid_moves_flat[prev];
    const int count_moves = valid_moves_count[prev];

    for (int k = 0; k < count_moves; ++k) {
      COUNT_NODE
      int i = moves[k];

      int index1_tmp = p_multi_move_ptr[arg_index1 + i];
      int cross_state_idx = index1_tmp / 24; // Cross State for AuxState pruning

      // [重构] AuxState 剪枝 (Corner3 + Edge3, 优先于其他剪枝!)
      bool aux_pruned = false;
      AuxState next_aux[MAX_AUX];
      for (int a = 0; a < num_aux; ++a) {
        const auto &cur = aux_states[a];
        if (!cur.def)
          continue;
        next_aux[a].def = cur.def;
        next_aux[a].current_idx = cur.def->p_move[cur.current_idx * 18 + i];
        long long idx_aux = (long long)cross_state_idx * cur.def->multiplier +
                            next_aux[a].current_idx;
        if (get_prune_ptr(cur.def->p_prune, idx_aux) >= depth) {
          aux_pruned = true;
          break;
        }
      }
      if (aux_pruned)
        continue;

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
                     prune_xc4, num_aux, next_aux)) // [重构] 传递 next_aux
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

        // [重构] 使用 AuxState 架构设置 Corner3 + Edge3 剪枝
        std::vector<int> rotated_alg = alg_rotation(base_alg, rotations[r]);
        AuxState aux_init[MAX_AUX];
        int num_aux = setup_aux_pruners_for_search4(
            slot2, slot3, slot4,    // Edge 槽位 (固定位)
            pslot2, pslot3, pslot4, // Corner 伪槽位
            rotated_alg, aux_init);

        int found = 999;
        int start_depth =
            std::max({prune1_tmp, prune2_tmp, prune3_tmp, prune4_tmp,
                      edge_prune1_tmp, prune_xc4_tmp});
        for (int d = start_depth; d <= max_length; d++) {
          if (depth_limited_search_4(
                  index1, index2, index4, index6, index8, index9, index10,
                  index11, index12, d, 18, p_prune1, p_prune2, p_prune3,
                  p_prune4, p_edge_prune1, p_prune_xc4, num_aux, aux_init)) {
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
const int *xcross_analyzer2::p_edge3_move_ptr = nullptr;
const int *xcross_analyzer2::p_corner3_move_ptr = nullptr; // [新增]

// Edge3 剪枝表定义
std::vector<unsigned char> xcross_analyzer2::prune_e0e1e2;
std::vector<unsigned char> xcross_analyzer2::prune_e1e2e3;
std::vector<unsigned char> xcross_analyzer2::prune_e0e2e3;
std::vector<unsigned char> xcross_analyzer2::prune_e0e1e3;

// [新增] Corner3 剪枝表定义
std::vector<unsigned char> xcross_analyzer2::prune_c4c5c6;
std::vector<unsigned char> xcross_analyzer2::prune_c4c5c7;
std::vector<unsigned char> xcross_analyzer2::prune_c4c6c7;
std::vector<unsigned char> xcross_analyzer2::prune_c5c6c7;

// [新增] Corner2/Edge2 移动表指针定义
const int *xcross_analyzer2::p_corner2_move_ptr = nullptr;
const int *xcross_analyzer2::p_edge2_move_ptr = nullptr;

// [新增] Corner2 剪枝表定义
std::vector<unsigned char> xcross_analyzer2::prune_c4c5;
std::vector<unsigned char> xcross_analyzer2::prune_c4c6;
std::vector<unsigned char> xcross_analyzer2::prune_c4c7;
std::vector<unsigned char> xcross_analyzer2::prune_c5c6;
std::vector<unsigned char> xcross_analyzer2::prune_c5c7;
std::vector<unsigned char> xcross_analyzer2::prune_c6c7;

// [新增] Edge2 剪枝表定义
std::vector<unsigned char> xcross_analyzer2::prune_e0e1;
std::vector<unsigned char> xcross_analyzer2::prune_e0e2;
std::vector<unsigned char> xcross_analyzer2::prune_e0e3;
std::vector<unsigned char> xcross_analyzer2::prune_e1e2;
std::vector<unsigned char> xcross_analyzer2::prune_e1e3;
std::vector<unsigned char> xcross_analyzer2::prune_e2e3;

// [新增] aux_registry 定义
std::map<std::vector<int>, xcross_analyzer2::AuxPrunerDef>
    xcross_analyzer2::aux_registry;

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
      oss << ",pseudo_cross_pseudo_pair" << s;
    for (const auto &s : suffixes)
      oss << ",pseudo_xcross_pseudo_pair" << s;
    for (const auto &s : suffixes)
      oss << ",pseudo_xxcross_pseudo_pair" << s;
    for (const auto &s : suffixes)
      oss << ",pseudo_xxxcross_pseudo_pair" << s;
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
