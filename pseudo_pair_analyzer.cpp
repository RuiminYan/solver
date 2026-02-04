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

// --- Search 3 剪枝统计 ---
std::atomic<long long> s3_aux_checked{0};
std::atomic<long long> s3_aux_pruned{0};
std::atomic<long long> s3_prune1_checked{0};
std::atomic<long long> s3_prune1_pruned{0};
std::atomic<long long> s3_edge_checked{0};
std::atomic<long long> s3_edge_pruned{0};
std::atomic<long long> s3_prune2_checked{0};
std::atomic<long long> s3_prune2_pruned{0};
std::atomic<long long> s3_prune3_checked{0};
std::atomic<long long> s3_prune3_pruned{0};
std::atomic<long long> s3_xc3_checked{0};
std::atomic<long long> s3_xc3_pruned{0};

// --- Search 4 剪枝统计 (已禁用以测试纯净版性能) ---
// std::atomic<long long> s4_total_calls{0};
// std::atomic<long long> s4_aux_checked{0};
// std::atomic<long long> s4_aux_pruned{0};
// std::atomic<long long> s4_prune1_checked{0};
// std::atomic<long long> s4_prune1_pruned{0};
// std::atomic<long long> s4_edge_checked{0};
// std::atomic<long long> s4_edge_pruned{0};
// std::atomic<long long> s4_prune2_checked{0};
// std::atomic<long long> s4_prune2_pruned{0};
// std::atomic<long long> s4_prune3_checked{0};
// std::atomic<long long> s4_prune3_pruned{0};
// std::atomic<long long> s4_prune4_checked{0};
// std::atomic<long long> s4_prune4_pruned{0};
// std::atomic<long long> s4_xc4_checked{0};
// std::atomic<long long> s4_xc4_pruned{0};

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
  // [重构] Edge3 剪枝表 (只保留规范化表)
  static std::vector<unsigned char> prune_e0e1e2;

  // [重构] Corner3 剪枝表 (只保留规范化表)
  static std::vector<unsigned char> prune_c4c5c6;

  // [重构] Corner2 剪枝表 (只保留规范化表)
  static std::vector<unsigned char> prune_c4c5; // 邻接
  static std::vector<unsigned char> prune_c4c6; // 对角

  // [重构] Edge2 剪枝表 (只保留规范化表)
  static std::vector<unsigned char> prune_e0e1; // 邻接
  static std::vector<unsigned char> prune_e0e2; // 对角

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

  // [Conj优化] XC 预计算状态 (用于 pseudo_base 表的 Conj 优化)
  // 从 pslot 视角追踪 Cross + Corner + 4个Edge 状态
  struct ConjStateXC {
    int cross;   // Cross 索引 (已 ×24 缩放)
    int corner;  // Corner 索引 (从 C4=12 开始)
    int edge[4]; // 4 个 Edge 相对索引 (从 {0,2,4,6} 开始)
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

    // 2. Load XCross Prune Tables
    xc_prune_tables.resize(16);
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
      }
    }

    // 3. Load Pair Prune Tables
    ec_prune_tables.resize(16);
    for (int e = 0; e < 4; ++e) {
      for (int c = 0; c < 4; ++c) {
        int idx = e * 4 + c;
        std::string fn_ec = "prune_table_pseudo_pair_C" +
                            std::to_string(c + 4) + "_E" + std::to_string(e) +
                            ".bin";
        if (!load_vector(ec_prune_tables[idx], fn_ec)) {
          std::cerr << "Error: Missing table " << fn_ec << std::endl;
          exit(1);
        }
      }
    }

    // 4. Load Pseudo Base Prune Tables (XCross Base)
    // [Conj优化] 只加载 4 个 C4 基准表 (diff=0,1,2,3 对应 E0,E1,E2,E3)
    pseudo_base_prune_tables.resize(4);
    for (int e = 0; e < 4; ++e) {
      std::string filename =
          "prune_table_pseudo_cross_C4_E" + std::to_string(e) + ".bin";
      if (!load_vector(pseudo_base_prune_tables[e], filename)) {
        std::cerr << "Error: Missing table " << filename << std::endl;
        exit(1);
      }
    }

    // 5. [重构] Load Edge3 Prune Tables (只加载规范化表)
    mtm.loadEdge3Table();
    p_edge3_move_ptr = mtm.getEdge3TablePtr();

    if (!load_vector(prune_e0e1e2, "prune_table_pseudo_cross_E0_E1_E2.bin")) {
      std::cerr << "Error: Missing prune_table_pseudo_cross_E0_E1_E2.bin"
                << std::endl;
      exit(1);
    }

    // 6. [重构] Load Corner3 Prune Tables (只加载规范化表)
    mtm.loadCorner3Table();
    p_corner3_move_ptr = mtm.getCorner3TablePtr();

    if (!load_vector(prune_c4c5c6, "prune_table_pseudo_cross_C4_C5_C6.bin")) {
      std::cerr << "Error: Missing prune_table_pseudo_cross_C4_C5_C6.bin"
                << std::endl;
      exit(1);
    }

    // 7. [重构] Load Corner2 Prune Tables (只加载规范化表)
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

    // 8. [重构] Load Edge2 Prune Tables (只加载规范化表)
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

    // 9. [重构] Register aux_registry (只注册规范化表)
    // Corner3 表 (multiplier = 9072) - 只注册规范化表
    aux_registry[{4, 5, 6}] = {prune_c4c5c6.data(), p_corner3_move_ptr, 9072};
    // Edge3 表 (multiplier = 10560) - 只注册规范化表
    aux_registry[{0, 1, 2}] = {prune_e0e1e2.data(), p_edge3_move_ptr, 10560};
    // Corner2 表 (multiplier = 504) - 只注册规范化表
    aux_registry[{4, 5}] = {prune_c4c5.data(), p_corner2_move_ptr, 504}; // 邻接
    aux_registry[{4, 6}] = {prune_c4c6.data(), p_corner2_move_ptr, 504}; // 对角
    // Edge2 表 (multiplier = 528) - 只注册规范化表
    aux_registry[{0, 1}] = {prune_e0e1.data(), p_edge2_move_ptr, 528}; // 邻接
    aux_registry[{0, 2}] = {prune_e0e2.data(), p_edge2_move_ptr, 528}; // 对角

    tables_initialized = true;
  }

  xcross_analyzer2() {
    initialize_tables();
    stage_results = StageResults();
  }

  // [Conj优化] 预计算 XC Conj 状态
  // 从 pslot 视角追踪 Cross + Corner + 4个Edge
  // 用于将物理索引转换为 C4 基准的 Conj 索引
  static void get_conj_state_xc(const std::vector<int> &alg, int pslot,
                                ConjStateXC &out) {
    int cur_mul = 187520 * 24;
    int cur_cn = 12; // 永远从 C4 开始
    int cur_e[] = {0, 2, 4, 6};

    for (int m : alg) {
      int mc = conj_moves_flat[m][pslot];
      cur_mul = p_multi_move_ptr[cur_mul + mc];
      cur_cn = p_corner_move_ptr[cur_cn * 18 + mc];
      for (int k = 0; k < 4; ++k)
        cur_e[k] = p_edge_move_ptr[cur_e[k] * 18 + mc];
    }

    out.cross = cur_mul;
    out.corner = cur_cn;
    for (int k = 0; k < 4; ++k)
      out.edge[k] = cur_e[k];
  }

  // [重构] 为 Search 4 设置 AuxState
  // 返回值: num_aux (已设置的辅助表数量)
  // 使用两级映射：conj_moves_flat[m][slot_k] + rot_map[rot_idx]
  int setup_aux_pruners_for_search4(int pslot1_arg, // 参考槽位 (slot_k)
                                    int slot2_arg, int slot3_arg,
                                    int slot4_arg, // Edge 槽位 (固定位)
                                    int pslot2_arg, int pslot3_arg,
                                    int pslot4_arg, // Corner 伪槽位
                                    const std::vector<int> &alg,
                                    AuxState *out_aux) {
    int count = 0;
    int slot_k = pslot1_arg; // 参考槽位

    // 1. Corner3 表 (优先剪枝)
    // 计算相对槽位 (相对于 slot_k)
    {
      int r1 = ((pslot2_arg)-slot_k + 4) % 4 + 4; // 映射到 4-7 范围
      int r2 = ((pslot3_arg)-slot_k + 4) % 4 + 4;
      int r3 = ((pslot4_arg)-slot_k + 4) % 4 + 4;
      std::vector<int> keys = {r1, r2, r3};
      std::sort(keys.begin(), keys.end());

      // 规范化表 key: {4, 5, 6}
      std::vector<int> canon_key = {4, 5, 6};
      auto it = aux_registry.find(canon_key);
      if (it != aux_registry.end()) {
        int rot_idx = 0;
        // 确定旋转：将 keys 映射到 {4, 5, 6}
        if (keys[0] == 4 && keys[1] == 5 && keys[2] == 6)
          rot_idx = 0; // Id
        else if (keys[0] == 4 && keys[1] == 5 && keys[2] == 7)
          rot_idx = 1; // y
        else if (keys[0] == 4 && keys[1] == 6 && keys[2] == 7)
          rot_idx = 2; // y2
        else if (keys[0] == 5 && keys[1] == 6 && keys[2] == 7)
          rot_idx = 3; // y'

        const int *mapper = rot_map[rot_idx];
        // Target Canonical Corner3: {4, 5, 6} -> Indices 12, 15, 18
        std::vector<int> target = {12, 15, 18};
        int init_idx = array_to_index(target, 3, 3, 8);
        int cur_c3 = init_idx;
        int cur_cr = 187520 * 24; // Solved Cross * 24

        // 两级映射：先 conj_moves_flat，再 rot_map
        for (int m : alg) {
          int m_conj = conj_moves_flat[m][slot_k]; // 物理 → slot_k 视角
          int m_rot = mapper[m_conj];              // slot_k → 规范化
          cur_c3 = p_corner3_move_ptr[cur_c3 * 18 + m_rot];
          cur_cr = p_multi_move_ptr[cur_cr + m_rot];
        }

        out_aux[count].def = &it->second;
        out_aux[count].current_idx = cur_c3;
        out_aux[count].current_cross_scaled = cur_cr;
        out_aux[count].move_mapper = mapper;
        out_aux[count].slot_k = slot_k;
        count++;
      }
    }

    // 2. Edge3 表
    // 计算相对槽位 (相对于 slot_k)
    {
      int r1 = (slot2_arg - slot_k + 4) % 4; // 映射到 0-3 范围
      int r2 = (slot3_arg - slot_k + 4) % 4;
      int r3 = (slot4_arg - slot_k + 4) % 4;
      std::vector<int> keys = {r1, r2, r3};
      std::sort(keys.begin(), keys.end());

      // 规范化表 key: {0, 1, 2}
      std::vector<int> canon_key = {0, 1, 2};
      auto it = aux_registry.find(canon_key);
      if (it != aux_registry.end()) {
        int rot_idx = 0;
        // 确定旋转：将 keys 映射到 {0, 1, 2}
        if (keys[0] == 0 && keys[1] == 1 && keys[2] == 2)
          rot_idx = 0; // Id
        else if (keys[0] == 0 && keys[1] == 1 && keys[2] == 3)
          rot_idx = 1; // y
        else if (keys[0] == 0 && keys[1] == 2 && keys[2] == 3)
          rot_idx = 2; // y2
        else if (keys[0] == 1 && keys[1] == 2 && keys[2] == 3)
          rot_idx = 3; // y'

        const int *mapper = rot_map[rot_idx];
        // Target Canonical Edge3: {0, 1, 2} -> Indices 0, 2, 4
        std::vector<int> target = {0, 2, 4};
        int init_idx = array_to_index(target, 3, 2, 12);
        int cur_e3 = init_idx;
        int cur_cr = 187520 * 24; // Solved Cross * 24

        // 两级映射：先 conj_moves_flat，再 rot_map
        for (int m : alg) {
          int m_conj = conj_moves_flat[m][slot_k]; // 物理 → slot_k 视角
          int m_rot = mapper[m_conj];              // slot_k → 规范化
          cur_e3 = p_edge3_move_ptr[cur_e3 * 18 + m_rot];
          cur_cr = p_multi_move_ptr[cur_cr + m_rot];
        }

        out_aux[count].def = &it->second;
        out_aux[count].current_idx = cur_e3;
        out_aux[count].current_cross_scaled = cur_cr;
        out_aux[count].move_mapper = mapper;
        out_aux[count].slot_k = slot_k;
        count++;
      }
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

  // [Conj优化] XC2 使用 Conj 状态追踪
  // 新增参数: xc2_cross, xc2_corner, xc2_e0-e3 (Conj 状态), diff2 (边选择)
  bool depth_limited_search_2(int arg_index1, int arg_index2, int arg_index4,
                              int arg_index5, int arg_index6, int depth,
                              int prev, const unsigned char *prune1,
                              const unsigned char *prune2,
                              const unsigned char *edge_prune1,
                              const unsigned char *prune_xc2, int xc2_cr,
                              int xc2_cn, int xc2_e0, int xc2_e1, int xc2_e2,
                              int xc2_e3, int diff2) {
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

      // [Conj] 用 Conj 移动更新 XC2 状态
      int mc = conj_moves_flat[i][pslot2];
      int xc2_cr_n = p_multi_move_ptr[xc2_cr + mc];
      int xc2_cn_n = p_corner_move_ptr[xc2_cn + mc];
      int xc2_e0_n = p_edge_move_ptr[xc2_e0 + mc];
      int xc2_e1_n = p_edge_move_ptr[xc2_e1 + mc];
      int xc2_e2_n = p_edge_move_ptr[xc2_e2 + mc];
      int xc2_e3_n = p_edge_move_ptr[xc2_e3 + mc];
      int xc2_e_sel = (diff2 == 0)   ? xc2_e0_n
                      : (diff2 == 1) ? xc2_e1_n
                      : (diff2 == 2) ? xc2_e2_n
                                     : xc2_e3_n;

      long long idx_xc2 = (long long)(xc2_cr_n + xc2_cn_n) * 24 + xc2_e_sel;
      int prune_xc2_tmp = get_prune_ptr(prune_xc2, idx_xc2);
      if (prune_xc2_tmp >= depth)
        continue;

      int index6_tmp = p_edge_move_ptr[arg_index6 + i];

      if (depth == 1) {
        if (prune1_tmp == 0 && prune2_tmp == 0 && edge_prune1_tmp == 0 &&
            prune_xc2_tmp == 0 && index6_tmp == edge_solved2) {
          return true;
        }
      } else if (depth_limited_search_2(
                     index1_tmp, index2_tmp * 18, index4_tmp * 18,
                     index5_tmp * 18, index6_tmp * 18, depth - 1, i, prune1,
                     prune2, edge_prune1, prune_xc2, xc2_cr_n, xc2_cn_n * 18,
                     xc2_e0_n * 18, xc2_e1_n * 18, xc2_e2_n * 18, xc2_e3_n * 18,
                     diff2))
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

    // [Conj优化] 使用 diff 选择 C4 基准表
    int diff2 = (slot2 - pslot2 + 4) & 3;
    std::vector<unsigned char> &prune_xc2 = pseudo_base_prune_tables[diff2];

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

      // [Conj优化] 使用 Conj 索引计算 h4
      std::vector<int> rotated_alg = alg_rotation(base_alg, rotations[r]);
      ConjStateXC st;
      get_conj_state_xc(rotated_alg, pslot2, st);
      long long conj_idx_xc2 =

          (long long)(st.cross + st.corner) * 24 + st.edge[diff2];
      int h4 = get_prune_ptr(p_prune_xc2, conj_idx_xc2);

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

      // [Conj优化] 计算 XC2 Conj 状态
      std::vector<int> rotated_alg = alg_rotation(base_alg, rotations[r]);
      ConjStateXC st;
      get_conj_state_xc(rotated_alg, pslot2, st);
      long long conj_idx_xc2 =
          (long long)(st.cross + st.corner) * 24 + st.edge[diff2];
      int prune_xc2_tmp = get_prune_ptr(p_prune_xc2, conj_idx_xc2);

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
                                     p_prune_xc2, st.cross, st.corner * 18,
                                     st.edge[0] * 18, st.edge[1] * 18,
                                     st.edge[2] * 18, st.edge[3] * 18, diff2)) {
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

  // [Conj优化] XC3 使用 Conj 状态追踪
  // 新增参数: xc3_cr/cn/e0-e3 (Conj 状态), diff3 (边选择)
  bool depth_limited_search_3(
      int arg_index1, int arg_index2, int arg_index4, int arg_index6,
      int arg_index7, int arg_index8, int arg_index9, int depth, int prev,
      const unsigned char *prune1, const unsigned char *prune2,
      const unsigned char *prune3, const unsigned char *edge_prune1,
      const unsigned char *prune_xc3, int num_aux, const AuxState *aux_states,
      int xc3_cr, int xc3_cn, int xc3_e0, int xc3_e1, int xc3_e2, int xc3_e3,
      int diff3) {
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
      ++s3_aux_checked;
      if (aux_pruned) {
        ++s3_aux_pruned;
        continue;
      }

      // 3. Base Pruning 继续原有逻辑
      int index2_tmp = p_corner_move_ptr[arg_index2 + i];
      int prune1_tmp = get_prune_ptr(prune1, index1_tmp + index2_tmp);
      ++s3_prune1_checked;
      if (prune1_tmp >= depth) {
        ++s3_prune1_pruned;
        continue;
      }

      int index7_tmp = p_edge_move_ptr[arg_index7 + i];
      int edge_prune1_tmp =
          get_prune_ptr(edge_prune1, index7_tmp * 24 + index2_tmp);
      ++s3_edge_checked;
      if (edge_prune1_tmp >= depth) {
        ++s3_edge_pruned;
        continue;
      }

      int index4_tmp = p_corner_move_ptr[arg_index4 + i];
      int prune2_tmp = get_prune_ptr(prune2, index1_tmp + index4_tmp);
      ++s3_prune2_checked;
      if (prune2_tmp >= depth) {
        ++s3_prune2_pruned;
        continue;
      }

      int index6_tmp = p_corner_move_ptr[arg_index6 + i];
      int prune3_tmp = get_prune_ptr(prune3, index1_tmp + index6_tmp);
      ++s3_prune3_checked;
      if (prune3_tmp >= depth) {
        ++s3_prune3_pruned;
        continue;
      }

      // [Conj] 用 Conj 移动更新 XC3 状态
      int mc = conj_moves_flat[i][pslot3];
      int xc3_cr_n = p_multi_move_ptr[xc3_cr + mc];
      int xc3_cn_n = p_corner_move_ptr[xc3_cn + mc];
      int xc3_e0_n = p_edge_move_ptr[xc3_e0 + mc];
      int xc3_e1_n = p_edge_move_ptr[xc3_e1 + mc];
      int xc3_e2_n = p_edge_move_ptr[xc3_e2 + mc];
      int xc3_e3_n = p_edge_move_ptr[xc3_e3 + mc];
      int xc3_e_sel = (diff3 == 0)   ? xc3_e0_n
                      : (diff3 == 1) ? xc3_e1_n
                      : (diff3 == 2) ? xc3_e2_n
                                     : xc3_e3_n;

      long long idx_xc3 = (long long)(xc3_cr_n + xc3_cn_n) * 24 + xc3_e_sel;
      int prune_xc3_tmp = get_prune_ptr(prune_xc3, idx_xc3);
      ++s3_xc3_checked;
      if (prune_xc3_tmp >= depth) {
        ++s3_xc3_pruned;
        continue;
      }

      int index8_tmp = p_edge_move_ptr[arg_index8 + i];
      int index9_tmp = p_edge_move_ptr[arg_index9 + i];

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
                     edge_prune1, prune_xc3, num_aux, next_aux, xc3_cr_n,
                     xc3_cn_n * 18, xc3_e0_n * 18, xc3_e1_n * 18, xc3_e2_n * 18,
                     xc3_e3_n * 18, diff3))
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

    // [Conj优化] 使用 diff 选择 C4 基准表
    int diff3 = (slot3 - pslot3 + 4) & 3;
    std::vector<unsigned char> &prune_xc3 = pseudo_base_prune_tables[diff3];

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

      // [Conj优化] 使用 Conj 索引计算 h5
      std::vector<int> rotated_alg = alg_rotation(base_alg, rotations[r]);
      ConjStateXC st;
      get_conj_state_xc(rotated_alg, pslot3, st);

      long long conj_idx_xc3 =
          (long long)(st.cross + st.corner) * 24 + st.edge[diff3];
      int h5 = get_prune_ptr(p_prune_xc3, conj_idx_xc3);

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

      // [Conj优化] 计算 XC3 Conj 状态
      std::vector<int> rotated_alg = alg_rotation(base_alg, rotations[r]);
      ConjStateXC st;
      get_conj_state_xc(rotated_alg, pslot3, st);
      long long conj_idx_xc3 =
          (long long)(st.cross + st.corner) * 24 + st.edge[diff3];
      int prune_xc3_tmp = get_prune_ptr(p_prune_xc3, conj_idx_xc3);

      if (prune1_tmp == 0 && prune2_tmp == 0 && prune3_tmp == 0 &&
          edge_prune1_tmp == 0 && prune_xc3_tmp == 0 &&
          index8 == edge_solved2 && index9 == edge_solved3) {
        results[r] = 0;
      } else {
        // [新增] 设置 AuxState (Corner2 + Edge2)
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
          if (depth_limited_search_3(
                  index1, index2, index4, index6, index7, index8, index9, d, 18,
                  p_prune1, p_prune2, p_prune3, p_edge_prune1, p_prune_xc3,
                  num_aux, aux_init, st.cross, st.corner * 18, st.edge[0] * 18,
                  st.edge[1] * 18, st.edge[2] * 18, st.edge[3] * 18, diff3)) {
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

  // [Conj优化] XC4 使用 Conj 状态追踪
  // 新增参数: xc4_cr/cn/e0-e3 (Conj 状态), diff4 (边选择)
  // Search 4
  bool depth_limited_search_4(
      int arg_index1, int arg_index2, int arg_index4, int arg_index6,
      int arg_index8, int arg_index9, int arg_index10, int arg_index11,
      int arg_index12, int depth, int prev, const unsigned char *prune1,
      const unsigned char *prune2, const unsigned char *prune3,
      const unsigned char *prune4, const unsigned char *edge_prune1,
      const unsigned char *prune_xc4, int num_aux, AuxState *aux_states,
      int xc4_cr, int xc4_cn, int xc4_e0, int xc4_e1, int xc4_e2, int xc4_e3,
      int diff4) {
    const int *moves = valid_moves_flat[prev];
    const int count_moves = valid_moves_count[prev];

    for (int k = 0; k < count_moves; ++k) {
      COUNT_NODE
      int i = moves[k];

      int index1_tmp = p_multi_move_ptr[arg_index1 + i];
      int cross_state_idx = index1_tmp / 24; // Cross State for AuxState pruning

      // [重构] AuxState 剪枝 (Corner3 + Edge3, 优先于其他剪枝!)
      // ++s4_aux_checked; // 统计已禁用
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
      if (aux_pruned) {
        // ++s4_aux_pruned; // 统计已禁用
        continue;
      }

      int index2_tmp = p_corner_move_ptr[arg_index2 + i];

      // [优化顺序] 先检查 prune_xc4 (Conj) - 6.10% 剪枝率
      int mc = conj_moves_flat[i][pslot4];
      int xc4_cr_n = p_multi_move_ptr[xc4_cr + mc];
      int xc4_cn_n = p_corner_move_ptr[xc4_cn + mc];
      int xc4_e0_n = p_edge_move_ptr[xc4_e0 + mc];
      int xc4_e1_n = p_edge_move_ptr[xc4_e1 + mc];
      int xc4_e2_n = p_edge_move_ptr[xc4_e2 + mc];
      int xc4_e3_n = p_edge_move_ptr[xc4_e3 + mc];
      int xc4_e_sel = (diff4 == 0)   ? xc4_e0_n
                      : (diff4 == 1) ? xc4_e1_n
                      : (diff4 == 2) ? xc4_e2_n
                                     : xc4_e3_n;

      long long idx_xc4 = (long long)(xc4_cr_n + xc4_cn_n) * 24 + xc4_e_sel;
      // ++s4_xc4_checked; // 统计已禁用
      int prune_xc4_tmp = get_prune_ptr(prune_xc4, idx_xc4);
      if (prune_xc4_tmp >= depth) {
        // ++s4_xc4_pruned; // 统计已禁用
        continue;
      }

      // NOTE: Search 4 不再使用 edge_prune1 (EC) 和 prune1 (XC slot1) 做剪枝
      // 但仍需计算它们用于 depth==1 时验证解决状态
      int index9_tmp = p_edge_move_ptr[arg_index9 + i];
      int edge_prune1_tmp =
          get_prune_ptr(edge_prune1, index9_tmp * 24 + index2_tmp);
      int prune1_tmp = get_prune_ptr(prune1, index1_tmp + index2_tmp);

      // NOTE: prune2/3/4 (Base 表) 已移除，因为被 AuxState (Corner3) 完全覆盖
      int index4_tmp = p_corner_move_ptr[arg_index4 + i];
      int index6_tmp = p_corner_move_ptr[arg_index6 + i];
      int index8_tmp = p_corner_move_ptr[arg_index8 + i];

      int index10_tmp = p_edge_move_ptr[arg_index10 + i];
      int index11_tmp = p_edge_move_ptr[arg_index11 + i];
      int index12_tmp = p_edge_move_ptr[arg_index12 + i];

      if (depth == 1) {
        // 验证解决状态：所有剪枝值必须为 0，且边块在正确位置
        if (prune1_tmp == 0 && edge_prune1_tmp == 0 && prune_xc4_tmp == 0 &&
            index10_tmp == edge_solved2 && index11_tmp == edge_solved3 &&
            index12_tmp == edge_solved4) {
          return true;
        }
      } else if (depth_limited_search_4(
                     index1_tmp, index2_tmp * 18, index4_tmp * 18,
                     index6_tmp * 18, index8_tmp * 18, index9_tmp * 18,
                     index10_tmp * 18, index11_tmp * 18, index12_tmp * 18,
                     depth - 1, i, prune1, prune2, prune3, prune4, edge_prune1,
                     prune_xc4, num_aux, next_aux, xc4_cr_n, xc4_cn_n * 18,
                     xc4_e0_n * 18, xc4_e1_n * 18, xc4_e2_n * 18, xc4_e3_n * 18,
                     diff4))
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

    // [Conj优化] 使用 diff 选择 C4 基准表
    int diff4 = (slot4 - pslot4 + 4) & 3;
    std::vector<unsigned char> &prune_xc4 = pseudo_base_prune_tables[diff4];

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

      // [Conj优化] 使用 Conj 索引计算 h6
      std::vector<int> rotated_alg = alg_rotation(base_alg, rotations[r]);
      ConjStateXC st;
      get_conj_state_xc(rotated_alg, pslot4, st);
      long long conj_idx_xc4 =
          (long long)(st.cross + st.corner) * 24 + st.edge[diff4];
      int h6 = get_prune_ptr(p_prune_xc4, conj_idx_xc4);

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

      // [Conj优化] 计算 XC4 Conj 状态
      std::vector<int> rotated_alg = alg_rotation(base_alg, rotations[r]);
      ConjStateXC st;
      get_conj_state_xc(rotated_alg, pslot4, st);
      long long conj_idx_xc4 =
          (long long)(st.cross + st.corner) * 24 + st.edge[diff4];
      int prune_xc4_tmp = get_prune_ptr(p_prune_xc4, conj_idx_xc4);

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
        AuxState aux_init[MAX_AUX];
        int num_aux = setup_aux_pruners_for_search4(
            pslot1,                 // 参考槽位 (slot_k)
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
                  p_prune4, p_edge_prune1, p_prune_xc4, num_aux, aux_init,
                  st.cross, st.corner * 18, st.edge[0] * 18, st.edge[1] * 18,
                  st.edge[2] * 18, st.edge[3] * 18, diff4)) {
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

// [重构] Edge3 剪枝表定义 (只保留规范化表)
std::vector<unsigned char> xcross_analyzer2::prune_e0e1e2;

// [重构] Corner3 剪枝表定义 (只保留规范化表)
std::vector<unsigned char> xcross_analyzer2::prune_c4c5c6;

// [新增] Corner2/Edge2 移动表指针定义
const int *xcross_analyzer2::p_corner2_move_ptr = nullptr;
const int *xcross_analyzer2::p_edge2_move_ptr = nullptr;

// [重构] Corner2 剪枝表定义 (只保留规范化表)
std::vector<unsigned char> xcross_analyzer2::prune_c4c5;
std::vector<unsigned char> xcross_analyzer2::prune_c4c6;

// [重构] Edge2 剪枝表定义 (只保留规范化表)
std::vector<unsigned char> xcross_analyzer2::prune_e0e1;
std::vector<unsigned char> xcross_analyzer2::prune_e0e2;

// [新增] aux_registry 定义
std::map<std::vector<int>, xcross_analyzer2::AuxPrunerDef>
    xcross_analyzer2::aux_registry;

// ============================================================
// [验证] Conj 映射可行性测试
// 目的：验证 xc_prune_tables[slot*4+pslot] 能否通过 Conj 映射到
// xc_prune_tables[diff*4+0]
// ============================================================
void verify_conj_mapping() {
  std::cout << "\n=== Conj Mapping Verification ===" << std::endl;
  std::cout
      << "Testing if xc_prune_tables can be reduced from 16 to 4 tables\n";

  xcross_analyzer2::initialize_tables();

  const int *p_multi = xcross_analyzer2::p_multi_move_ptr;
  const int *p_corner = xcross_analyzer2::p_corner_move_ptr;
  auto &xc_tables = xcross_analyzer2::xc_prune_tables;

  // 测试几个不同的 scramble
  std::vector<std::string> test_scrambles = {
      "R U R' U'",         "L U L' U' F' U F",      "R U R' U R U2 R'",
      "L' U' L U L' U' L", "R2 U R' U' R U R U R2", "F R U R' U' F'",
  };

  std::cout << "\nComparing [slot,pslot] table vs [diff,0] table with Conj:\n";
  std::cout << "--------------------------------------------\n";

  int total_tests = 0;
  int passed_tests = 0;

  for (const auto &scramble_str : test_scrambles) {
    std::vector<int> alg = string_to_alg(scramble_str);
    std::cout << "Scramble: " << scramble_str << std::endl;

    for (int slot = 0; slot < 4; ++slot) {
      for (int pslot = 0; pslot < 4; ++pslot) {
        total_tests++;
        int diff = (slot - pslot + 4) & 3;

        // --- 物理方法 (使用表 [slot*4+pslot]) ---
        int phy_cross = 187520 * 24;
        int phy_corner = (pslot + 4) * 3; // C4=12, C5=15, C6=18, C7=21

        for (int m : alg) {
          phy_cross = p_multi[phy_cross + m];
          phy_corner = p_corner[phy_corner * 18 + m];
        }

        int phy_table_idx = slot * 4 + pslot;
        int phy_prune_idx = phy_cross / 24 + phy_corner;
        int phy_prune_val =
            get_prune_ptr(xc_tables[phy_table_idx].data(), phy_prune_idx);

        // --- Conj 方法 (使用表 [diff*4+0]) ---
        // 从 C4 (12) 开始，用映射移动
        int conj_cross = 187520 * 24;
        int conj_corner = 12; // 永远从 C4 开始

        for (int m : alg) {
          int mc = conj_moves_flat[m][pslot]; // 映射到 pslot 视角
          conj_cross = p_multi[conj_cross + mc];
          conj_corner = p_corner[conj_corner * 18 + mc];
        }

        int conj_table_idx = diff * 4 + 0; // 使用 [diff,0] 表
        int conj_prune_idx = conj_cross / 24 + conj_corner;
        int conj_prune_val =
            get_prune_ptr(xc_tables[conj_table_idx].data(), conj_prune_idx);

        // 比较
        bool match = (phy_prune_val == conj_prune_val);
        if (match)
          passed_tests++;

        // 显示不匹配的情况
        if (!match) {
          std::cout << "  [" << slot << "," << pslot << "] diff=" << diff
                    << " Phy(t" << phy_table_idx << ",idx" << phy_prune_idx
                    << ")=" << phy_prune_val << " vs Conj(t" << conj_table_idx
                    << ",idx" << conj_prune_idx << ")=" << conj_prune_val
                    << " MISMATCH!" << std::endl;
        }
      }
    }
  }

  std::cout << "--------------------------------------------\n";
  std::cout << "Results: " << passed_tests << "/" << total_tests
            << " tests passed\n";

  if (passed_tests == total_tests) {
    std::cout << "====> SUCCESS: 16 tables CAN be reduced to 4! <====\n";
  } else {
    double rate = (double)passed_tests / total_tests * 100;
    std::cout << "Pass rate: " << rate << "%\n";
    if (passed_tests * 4 == total_tests) {
      std::cout << "NOTE: Only slot==pslot (diff=0) cases pass. Tables are NOT "
                   "equivalent.\n";
    }
    std::cout << "CONCLUSION: Table reduction via Conj is NOT feasible for "
                 "xc_prune_tables.\n";
  }
  std::cout << "=== End Verification ===\n\n";
}

// ============================================================
// [验证] pseudo_base_prune_tables Conj 可行性测试
// 目的：验证 16 张表 C{c}_E{e} 是否可以用 4 张 C4_E{diff} 表 + Conj 替代
// ============================================================
void verify_pseudo_base_conj() {
  std::cout << "\n=== Pseudo Base Conj Verification ===" << std::endl;
  std::cout << "Testing if 16 pseudo_base tables can be reduced to 4\n";

  xcross_analyzer2::initialize_tables();

  const int *p_multi = xcross_analyzer2::p_multi_move_ptr;
  const int *p_corner = xcross_analyzer2::p_corner_move_ptr;
  const int *p_edge = xcross_analyzer2::p_edge_move_ptr;
  auto &base_tables = xcross_analyzer2::pseudo_base_prune_tables;

  std::vector<std::string> test_scrambles = {
      "R U R' U'",         "L U L' U' F' U F",      "R U R' U R U2 R'",
      "L' U' L U L' U' L", "R2 U R' U' R U R U R2", "F R U R' U' F'",
  };

  std::cout << "\nComparing [pslot,slot] table vs [0,diff] table with Conj:\n";
  std::cout << "--------------------------------------------\n";

  int total_tests = 0;
  int passed_tests = 0;

  for (const auto &scramble_str : test_scrambles) {
    std::vector<int> alg = string_to_alg(scramble_str);
    std::cout << "Scramble: " << scramble_str << std::endl;

    for (int slot = 0; slot < 4; ++slot) {
      for (int pslot = 0; pslot < 4; ++pslot) {
        total_tests++;
        int diff = (slot - pslot + 4) & 3;

        // --- 物理方法 (使用表 [pslot*4+slot]) ---
        int phy_cross = 187520 * 24;
        int phy_corner = (pslot + 4) * 3; // C4=12, C5=15, C6=18, C7=21
        int phy_edge = slot * 2;          // E0=0, E1=2, E2=4, E3=6

        for (int m : alg) {
          phy_cross = p_multi[phy_cross + m];
          phy_corner = p_corner[phy_corner * 18 + m];
          phy_edge = p_edge[phy_edge * 18 + m];
        }

        int phy_table_idx = pslot * 4 + slot;
        long long phy_prune_idx =
            (long long)(phy_cross / 24 + phy_corner) * 24 + phy_edge;
        int phy_prune_val =
            get_prune_ptr(base_tables[phy_table_idx].data(), phy_prune_idx);

        // --- Conj 方法 (使用表 [0*4+diff] = diff) ---
        int conj_cross = 187520 * 24;
        int conj_corner = 12;     // 永远从 C4 开始
        int conj_edge = diff * 2; // 使用相对 Edge 位置

        for (int m : alg) {
          int mc = conj_moves_flat[m][pslot]; // 映射到 pslot 视角
          conj_cross = p_multi[conj_cross + mc];
          conj_corner = p_corner[conj_corner * 18 + mc];
          conj_edge = p_edge[conj_edge * 18 + mc];
        }

        int conj_table_idx = diff; // 只使用 C4 系列 (index 0-3)
        long long conj_prune_idx =
            (long long)(conj_cross / 24 + conj_corner) * 24 + conj_edge;
        int conj_prune_val =
            get_prune_ptr(base_tables[conj_table_idx].data(), conj_prune_idx);

        bool match = (phy_prune_val == conj_prune_val);
        if (match)
          passed_tests++;

        if (!match) {
          std::cout << "  [" << pslot << "," << slot << "] diff=" << diff
                    << " Phy(t" << phy_table_idx << ")=" << phy_prune_val
                    << " vs Conj(t" << conj_table_idx << ")=" << conj_prune_val
                    << " MISMATCH!" << std::endl;
        }
      }
    }
  }

  std::cout << "--------------------------------------------\n";
  std::cout << "Results: " << passed_tests << "/" << total_tests
            << " tests passed\n";

  if (passed_tests == total_tests) {
    std::cout
        << "====> SUCCESS: 16 pseudo_base tables CAN be reduced to 4! <====\n";
    std::cout << "Expected RAM savings: ~624 MB (12 x 52MB)\n";
  } else {
    double rate = (double)passed_tests / total_tests * 100;
    std::cout << "Pass rate: " << rate << "%\n";
    std::cout << "CONCLUSION: Table reduction is NOT feasible.\n";
  }
  std::cout << "=== End Pseudo Base Verification ===\n\n";
}

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
    xcross_analyzer2::initialize_tables();
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

  static void print_stats() {
    std::cerr << "\n=== Search 3 Pruning Statistics ===\n";
    auto print_line = [](const char *name, long long checked,
                         long long pruned) {
      double pct = (checked > 0) ? (100.0 * pruned / checked) : 0.0;
      std::cerr << std::setw(22) << std::right << name << ": " << std::setw(15)
                << std::right << checked << " checked, " << std::setw(15)
                << std::right << pruned << " pruned (" << std::fixed
                << std::setprecision(2) << std::setw(6) << pct << "%)\n";
    };
    print_line("AuxState(C2+E2)", s3_aux_checked.load(), s3_aux_pruned.load());
    print_line("prune1 (XC slot1)", s3_prune1_checked.load(),
               s3_prune1_pruned.load());
    print_line("edge_prune1 (EC)", s3_edge_checked.load(),
               s3_edge_pruned.load());
    print_line("prune2 (XC slot2)", s3_prune2_checked.load(),
               s3_prune2_pruned.load());
    print_line("prune3 (XC slot3)", s3_prune3_checked.load(),
               s3_prune3_pruned.load());
    print_line("prune_xc3 (Conj)", s3_xc3_checked.load(), s3_xc3_pruned.load());
  }
};

int main() {
  printCuberootLogo();

  // NOTE: verify_conj_mapping() 验证显示 xc_prune_tables 无法通过 Conj 减少
  // (62.5% 通过率) 原因：表索引 cross_idx * 24 + corner_idx 包含绝对 Corner

  // NOTE: verify_pseudo_base_conj() 验证显示 pseudo_base_prune_tables
  // 也无法减少 (43.75% 通过率) 原因：C5/C6/C7_E* 表是用不同 Corner 基准生成的
  // verify_pseudo_base_conj();

  run_analyzer_app<PseudoPairSolverWrapper>("_pseudo_pair");
  return 0;
}
