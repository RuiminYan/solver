/*
 * pseudo_analyzer.cpp - Pseudo X..Cross 分析器 (重构版 - 支持通用辅助剪枝)
 */

#include "cube_common.h"
#include "move_tables.h"
#include "prune_tables.h"
#include <map>
#include <array>

// --- 全局统计变量 ---
std::atomic<long long> global_nodes{0};
std::atomic<int> completed_tasks{0};
std::atomic<bool> is_solving{false};

// --- Profiling Counters ---
std::atomic<long long> cnt_search2_total{0};
std::atomic<long long> cnt_search3_total{0};
std::atomic<long long> cnt_huge_active{0};
std::atomic<long long> cnt_aux_pruned{0};
std::atomic<long long> cnt_huge_pruned{0};
std::atomic<long long> cnt_base_pruned{0};

#define COUNT_NODE \
    static thread_local int local_node_counter = 0; \
    local_node_counter++; \
    if (local_node_counter >= 1000) { \
        global_nodes.fetch_add(local_node_counter, std::memory_order_relaxed); \
        local_node_counter = 0; \
    }

struct SearchContext {
    std::vector<int> sol_len;
    int current_max_depth = 0;
    SearchContext() { sol_len.reserve(32); }
};

int trans_moves[4][4][18]; 

// --- 镜像映射表 (用于 Diff=3 -> Diff=1) ---
int sym_corner2[504];
int sym_edge6_pos[665280]; // 12P6
int sym_edge6_ori[64];     // 2^6

// 物理位置镜像映射 (L <-> R)
// Edges: 0:BL, 1:BR, 2:FR, 3:FL, 4:UB, 5:UR, 6:UF, 7:UL, 8:DB, 9:DR, 10:DF, 11:DL
const int mirror_edge_map[12] = {1, 0, 3, 2, 4, 7, 6, 5, 8, 11, 10, 9};
// Corners: 0:UBL, 1:UBR, 2:UFR, 3:UFL, 4:DBL, 5:DBR, 6:DFR, 7:DFL
const int mirror_corn_map[8]  = {1, 0, 3, 2, 5, 4, 7, 6};

void initMirrorTables() {
    std::cout << "[Init] Generating Mirror Tables..." << std::endl;
    
    // 1. Corner2 Mirror Table
    // Swap C4 <-> C5 roles
    std::vector<int> c_arr(2);
    for (int i = 0; i < 504; ++i) {
        index_to_array(c_arr, i, 2, 3, 8); // c_arr[0]=C4_state, c_arr[1]=C5_state
        // Decode state to physical pos & ori
        int p0 = c_arr[0] / 3; int o0 = c_arr[0] % 3;
        int p1 = c_arr[1] / 3; int o1 = c_arr[1] % 3;
        
        // Apply Mirror
        int np0 = mirror_corn_map[p0]; int no0 = (o0 == 0) ? 0 : (3 - o0);
        int np1 = mirror_corn_map[p1]; int no1 = (o1 == 0) ? 0 : (3 - o1);
        
        // Swap roles: New C4 state comes from old C5 transformed
        // New C5 state comes from old C4 transformed
        // Wait, logic check:
        // Diff 3 C4 is at DBR. Mirror -> DBL. This matches Diff 1 C5 Target.
        // So Old_C4_Transformed should act as New_C5.
        // Old_C5_Transformed should act as New_C4.
        
        int n_state_c4 = np1 * 3 + no1; // New C4 comes from Old C5
        int n_state_c5 = np0 * 3 + no0; // New C5 comes from Old C4
        
        std::vector<int> res = {n_state_c4, n_state_c5};
        sym_corner2[i] = array_to_index(res, 2, 3, 8);
    }
    
    // 2. Edge6 Position Mirror Table
    // Swap E0 <-> E1 roles
    std::vector<int> e_arr(6);
    for (int i = 0; i < 665280; ++i) {
        index_to_array(e_arr, i, 6, 1, 12); // Only position
        // e_arr[0]=E0, e_arr[1]=E1, ...
        
        std::vector<int> res(6);
        // Apply Mirror to all
        for(int k=0; k<6; ++k) res[k] = mirror_edge_map[e_arr[k]];
        
        // Swap roles: E0 <-> E1
        std::swap(res[0], res[1]);
        
        sym_edge6_pos[i] = array_to_index(res, 6, 1, 12);
    }
    
    // 3. Edge6 Orientation Mirror Table
    // Swap E0 <-> E1 roles
    for (int i = 0; i < 64; ++i) {
        int o[6];
        int temp = i;
        for(int k=5; k>=0; --k) { o[k] = temp % 2; temp /= 2; }
        
        // Mirror does not change Edge Ori (Standard definition L/R swap)
        // Just swap roles E0 <-> E1
        std::swap(o[0], o[1]);
        
        int res = 0;
        for(int k=0; k<6; ++k) res = res * 2 + o[k];
        sym_edge6_ori[i] = res;
    }
}

void init_pseudo_matrix() {
    for (int s1 = 0; s1 < 4; ++s1) {
        for (int s2 = 0; s2 < 4; ++s2) {
            for (int m_phys = 0; m_phys < 18; ++m_phys) {
                int m_s1 = conj_moves_flat[m_phys][s1];
                int m_s2 = conj_moves_flat[m_phys][s2];
                trans_moves[s1][s2][m_s1] = m_s2;
            }
        }
    }
}

// --- 通用辅助剪枝结构 ---
struct AuxPrunerDef {
    const unsigned char* p_prune; // 剪枝表指针
    const int* p_move;            // 移动表指针 (Edges2, Corners2, etc.)
    int multiplier;               // 状态乘数 (用于结合 Cross 状态)
};

struct AuxState {
    const AuxPrunerDef* def = nullptr;
    int current_idx = 0;
};

// 最大支持的辅助表数量 (每个搜索路径)
constexpr int MAX_AUX = 8;

struct CrossSolver {
    const int* p_multi;
    const unsigned char* p_prune;

    CrossSolver() {
        auto& mtm = MoveTableManager::getInstance();
        auto& ptm = PruneTableManager::getInstance();
        p_multi = mtm.getEdges2TablePtr();
        p_prune = ptm.getPseudoCrossPrunePtr();
    }
    
    bool search(SearchContext& ctx, int i1, int i2, int depth, int prev) {
        const int* moves = valid_moves_flat[prev]; const int count = valid_moves_count[prev];
        for (int k = 0; k < count; ++k) {
            int m = moves[k]; COUNT_NODE
            int n_i1 = p_multi[i1 + m]; int n_i2 = p_multi[i2 + m];
            long long idx = (long long)n_i1 * 528 + n_i2;
            if (get_prune_ptr(p_prune, idx) >= depth) continue;
            if (depth == 1) { ctx.sol_len.push_back(ctx.current_max_depth); return true; }
            if (search(ctx, n_i1 * 18, n_i2 * 18, depth - 1, m)) return true;
        }
        return false;
    }
    
    std::vector<int> get_stats(const std::vector<int>& base_alg, const std::vector<std::string>& rots) {
        std::vector<int> res(rots.size(), 0);
        for (size_t i = 0; i < rots.size(); ++i) {
            std::vector<int> alg = alg_rotation(base_alg, rots[i]);
            int i1 = 416, i2 = 520; for (int m : alg) { i1 = p_multi[i1 * 18 + m]; i2 = p_multi[i2 * 18 + m]; } 
            long long idx = (long long)i1 * 528 + i2;
            int d_min = get_prune_ptr(p_prune, idx); if (d_min == 0) continue;
            SearchContext ctx; 
            for (int d = d_min; d <= 8; ++d) { ctx.current_max_depth = d; if (search(ctx, i1 * 18, i2 * 18, d, 18)) { res[i] = d; break; } } 
        }
        return res;
    }
};

struct XCrossSolver {
    const int *p_multi, *p_corn, *p_edge, *p_edges2, *p_corners2, *p_corners3, *p_edge3;
    const int *p_edge6 = nullptr; // 增加 Edge6 引用
    const unsigned char* p_prune_base[4];
    const unsigned char* p_huge_neighbor = nullptr;
    const unsigned char* p_huge_neighbor_d1 = nullptr; // Diff=1 Neighbor Table
    // const unsigned char* p_huge_diagonal = nullptr; // Disabled for memory
    
    // 剪枝表注册表: Key = {PieceID1, PieceID2...} (有序)
    std::map<std::vector<int>, AuxPrunerDef> aux_registry;

    struct PseudoTask1 { int c_idx; int diff; int h; };
    
    struct PseudoTask2 {
        int c1, c2; int diff1, diff2; int h; 
        int num_aux; 
        AuxState aux_init[MAX_AUX];
        int i_e6, i_c2;
        const unsigned char* p_huge_table = nullptr; 
        bool mirror_huge = false;
        
        // Explicit constructor to avoid warnings
        PseudoTask2(int _c1, int _c2, int _d1, int _d2, int _h, int _n) 
            : c1(_c1), c2(_c2), diff1(_d1), diff2(_d2), h(_h), num_aux(_n), i_e6(0), i_c2(0) {
                for(int i=0; i<MAX_AUX; ++i) aux_init[i] = AuxState();
            }
    };
    
    struct PseudoTask3 {
        int c1, c2, c3; int diff1, diff2, diff3; int h; 
        int num_aux; 
        AuxState aux_init[MAX_AUX];

        PseudoTask3(int _c1, int _c2, int _c3, int _d1, int _d2, int _d3, int _h, int _n)
            : c1(_c1), c2(_c2), c3(_c3), diff1(_d1), diff2(_d2), diff3(_d3), h(_h), num_aux(_n) {
                for(int i=0; i<MAX_AUX; ++i) aux_init[i] = AuxState();
            }
    };
    
    struct ConjState { int im; int ic_b; int ie_rel[4]; };

    XCrossSolver() {
        auto& mtm = MoveTableManager::getInstance();
        auto& ptm = PruneTableManager::getInstance();
        p_multi = mtm.getCrossTablePtr();
        p_corn = mtm.getCornerTablePtr();
        p_edge = mtm.getEdgeTablePtr();
        p_edges2 = mtm.getEdges2TablePtr();
        p_corners2 = mtm.getCorner2TablePtr(); 
        p_corners3 = mtm.getCorner3TablePtr();
        p_edge3 = mtm.getEdge3TablePtr();
        p_edge6 = mtm.getEdge6TablePtr();
        
        // MMap loading for Huge Tables (Twin Titans Strategy)
        p_huge_neighbor = ptm.loadTableMMap("prune_table_cross_C4_E0_C5_E1.bin");
        p_huge_neighbor_d1 = ptm.loadTableMMap("prune_table_cross_C4_E0_C5_E1_diff1.bin");
        // p_huge_diagonal = ptm.getHugeDiagonalPrunePtr(); // Disabled
        
        for(int i=0; i<4; ++i) p_prune_base[i] = ptm.getPseudoCrossBasePrunePtr(i);
        
        // 注册 E0_E2 表 (IDs: 0, 2)
        if (ptm.hasPseudoCrossE0E2Prune()) {
            aux_registry[{0, 2}] = { ptm.getPseudoCrossE0E2PrunePtr(), p_edges2, 528 };
        }
        // 注册 E1_E3 表 (IDs: 1, 3)
        if (ptm.hasPseudoCrossE1E3Prune()) {
            aux_registry[{1, 3}] = { ptm.getPseudoCrossE1E3PrunePtr(), p_edges2, 528 };
        }
        // 注册邻棱表 (Neighboring Edges)
        if (ptm.hasPseudoCrossE0E1Prune()) {
            aux_registry[{0, 1}] = { ptm.getPseudoCrossE0E1PrunePtr(), p_edges2, 528 };
        }
        if (ptm.hasPseudoCrossE1E2Prune()) {
            aux_registry[{1, 2}] = { ptm.getPseudoCrossE1E2PrunePtr(), p_edges2, 528 };
        }
        if (ptm.hasPseudoCrossE2E3Prune()) {
            aux_registry[{2, 3}] = { ptm.getPseudoCrossE2E3PrunePtr(), p_edges2, 528 };
        }
        if (ptm.hasPseudoCrossE0E3Prune()) {
            aux_registry[{0, 3}] = { ptm.getPseudoCrossE0E3PrunePtr(), p_edges2, 528 };
        }
        
        // 注册角块对表 (Corner Pairs) - 使用逻辑 ID {4, 6} 等
        if (ptm.hasPseudoCrossC4C6Prune()) {
            aux_registry[{4, 6}] = { ptm.getPseudoCrossC4C6PrunePtr(), p_corners2, 504 };
        }
        if (ptm.hasPseudoCrossC5C7Prune()) {
            aux_registry[{5, 7}] = { ptm.getPseudoCrossC5C7PrunePtr(), p_corners2, 504 };
        }
        if (ptm.hasPseudoCrossC4C5Prune()) {
            aux_registry[{4, 5}] = { ptm.getPseudoCrossC4C5PrunePtr(), p_corners2, 504 };
        }
        if (ptm.hasPseudoCrossC5C6Prune()) {
            aux_registry[{5, 6}] = { ptm.getPseudoCrossC5C6PrunePtr(), p_corners2, 504 };
        }
        if (ptm.hasPseudoCrossC6C7Prune()) {
            aux_registry[{6, 7}] = { ptm.getPseudoCrossC6C7PrunePtr(), p_corners2, 504 };
        }
        if (ptm.hasPseudoCrossC4C7Prune()) {
            aux_registry[{4, 7}] = { ptm.getPseudoCrossC4C7PrunePtr(), p_corners2, 504 };
        }
        
        // Register Corner3 Tables (Triples)
        if (ptm.hasPseudoCrossC4C5C6Prune()) {
            aux_registry[{4, 5, 6}] = { ptm.getPseudoCrossC4C5C6PrunePtr(), p_corners3, 9072 };
        }
        if (ptm.hasPseudoCrossC5C6C7Prune()) {
            aux_registry[{5, 6, 7}] = { ptm.getPseudoCrossC5C6C7PrunePtr(), p_corners3, 9072 };
        }
        if (ptm.hasPseudoCrossC4C6C7Prune()) {
            aux_registry[{4, 6, 7}] = { ptm.getPseudoCrossC4C6C7PrunePtr(), p_corners3, 9072 };
        }
        if (ptm.hasPseudoCrossC4C5C7Prune()) {
            aux_registry[{4, 5, 7}] = { ptm.getPseudoCrossC4C5C7PrunePtr(), p_corners3, 9072 };
        }

        // Register Edge3 Tables (Triples)
        if (ptm.hasPseudoCrossE0E1E2Prune()) {
            aux_registry[{0, 1, 2}] = { ptm.getPseudoCrossE0E1E2PrunePtr(), p_edge3, 10560 };
        }
        if (ptm.hasPseudoCrossE0E1E3Prune()) {
            aux_registry[{0, 1, 3}] = { ptm.getPseudoCrossE0E1E3PrunePtr(), p_edge3, 10560 };
        }
        if (ptm.hasPseudoCrossE0E2E3Prune()) {
            aux_registry[{0, 2, 3}] = { ptm.getPseudoCrossE0E2E3PrunePtr(), p_edge3, 10560 };
        }
        if (ptm.hasPseudoCrossE1E2E3Prune()) {
            aux_registry[{1, 2, 3}] = { ptm.getPseudoCrossE1E2E3PrunePtr(), p_edge3, 10560 };
        }
    }

    inline int get_diff(int sc, int se) { return (se - sc + 4) & 3; }

    void get_conjugated_indices_all(const std::vector<int>& alg, int slot_k, ConjState& out) {
        int cur_mul = 187520 * 24, cur_cn = 12 * 18;      
        int cur_e[] = {0, 2, 4, 6};
        for (int m : alg) {
            int mc = conj_moves_flat[m][slot_k];
            cur_mul = p_multi[cur_mul + mc]; cur_cn  = p_corn[cur_cn + mc] * 18;
            for(int k=0;k<4;++k) cur_e[k] = p_edge[cur_e[k] * 18 + mc];
        }
        out.im = cur_mul; out.ic_b = cur_cn / 18;
        for(int k=0;k<4;++k) out.ie_rel[k] = cur_e[k];
    }

    int get_edges2_conjugated(const std::vector<int>& alg, int slot_k, int idx_init) {
        int cur = idx_init;
        for (int m : alg) {
            int mc = conj_moves_flat[m][slot_k];
            cur = p_edges2[cur * 18 + mc];
        }
        return cur;
    }

    int get_corner2_conjugated(const std::vector<int>& alg, int slot_k, int idx_init) {
        int cur = idx_init;
        for (int m : alg) {
            int mc = conj_moves_flat[m][slot_k];
            cur = p_corners2[cur * 18 + mc];
        }
        return cur;
    }

    int get_corner3_conjugated(const std::vector<int>& alg, int slot_k, int idx_init) {
        int cur = idx_init;
        for (int m : alg) {
            int mc = conj_moves_flat[m][slot_k];
            cur = p_corners3[cur * 18 + mc];
        }
        return cur;
    }

    int get_edge3_conjugated(const std::vector<int>& alg, int slot_k, int idx_init) {
        int cur = idx_init;
        for (int m : alg) {
            int mc = conj_moves_flat[m][slot_k];
            cur = p_edge3[cur * 18 + mc];
        }
        return cur;
    }
    
    // 更好的 setup 函数
    int setup_aux_pruners_struct(const std::vector<int>& target_pieces, const std::vector<int>& alg, int slot_k, AuxState* out_aux) {
        int count = 0;
        bool covered[8][8] = {false};

        // 1. 优先检查所有 3-subset (Triples) - Corner3 AND Edge3
        // Need to check all combinations of 3 pieces
        if (target_pieces.size() >= 3) {
            for (size_t i = 0; i < target_pieces.size(); ++i) {
                for (size_t j = i + 1; j < target_pieces.size(); ++j) {
                    for (size_t k = j + 1; k < target_pieces.size(); ++k) {
                        if (count >= MAX_AUX) break;
                        
                        int p1 = target_pieces[i];
                        int p2 = target_pieces[j];
                        int p3 = target_pieces[k];
                        
                        // Determine type: All Corners (>=4) or All Edges (<4)
                        bool is_corner3 = (p1 >= 4 && p2 >= 4 && p3 >= 4);
                        bool is_edge3 = (p1 < 4 && p2 < 4 && p3 < 4);
                        
                        if (!is_corner3 && !is_edge3) continue;

                        int init_idx = -1;
                        int conjugated_idx = -1;
                        const AuxPrunerDef* def_ptr = nullptr;

                        // Normalize keys
                        int r1, r2, r3;
                        if (is_corner3) {
                            r1 = ((p1 - 4) - slot_k + 4) % 4 + 4;
                            r2 = ((p2 - 4) - slot_k + 4) % 4 + 4;
                            r3 = ((p3 - 4) - slot_k + 4) % 4 + 4;
                        } else {
                            r1 = (p1 - slot_k + 4) % 4;
                            r2 = (p2 - slot_k + 4) % 4;
                            r3 = (p3 - slot_k + 4) % 4;
                        }
                        
                        std::vector<int> keys = {r1, r2, r3};
                        std::sort(keys.begin(), keys.end());
                        
                        auto it = aux_registry.find(keys);
                        if (it != aux_registry.end()) {
                            def_ptr = &it->second;
                            if (is_corner3) {
                                std::vector<int> target_arr = {(keys[0]-4)*3+12, (keys[1]-4)*3+12, (keys[2]-4)*3+12};
                                init_idx = array_to_index(target_arr, 3, 3, 8);
                                conjugated_idx = get_corner3_conjugated(alg, slot_k, init_idx);
                            } else {
                                std::vector<int> target_arr = {keys[0]*2, keys[1]*2, keys[2]*2};
                                init_idx = array_to_index(target_arr, 3, 2, 12);
                                conjugated_idx = get_edge3_conjugated(alg, slot_k, init_idx);
                            }
                            
                            out_aux[count].def = def_ptr;
                            out_aux[count].current_idx = conjugated_idx;
                            count++;

                            // Mark pairs as covered
                            if(p1<8 && p2<8) covered[p1][p2] = covered[p2][p1] = true;
                            if(p1<8 && p3<8) covered[p1][p3] = covered[p3][p1] = true;
                            if(p2<8 && p3<8) covered[p2][p3] = covered[p3][p2] = true;
                        }
                    }
                }
            }
        }

        // 2. 检查所有 2-subset
        for (size_t i = 0; i < target_pieces.size(); ++i) {
            for (size_t j = i + 1; j < target_pieces.size(); ++j) {
                int p1 = target_pieces[i];
                int p2 = target_pieces[j];
                
                // Skip if already covered by a triple
                if (p1 < 8 && p2 < 8 && covered[p1][p2]) continue;

                const AuxPrunerDef* def_ptr = nullptr;
                int conjugated_idx = -1;

                // Case 1: 两个都是棱块 (0-3)
                if (p1 < 4 && p2 < 4) { 
                    int r1 = (p1 - slot_k + 4) % 4;
                    int r2 = (p2 - slot_k + 4) % 4;
                    int key_a = r1, key_b = r2;
                    if (key_a > key_b) std::swap(key_a, key_b);

                    auto it = aux_registry.find({key_a, key_b});
                    if (it != aux_registry.end()) {
                        def_ptr = &it->second;
                        // 逻辑位置初始化: key * 2
                        std::vector<int> target = {key_a * 2, key_b * 2};
                        int init_idx = array_to_index(target, 2, 2, 12);
                        conjugated_idx = get_edges2_conjugated(alg, slot_k, init_idx);
                    }
                }
                // Case 2: 两个都是角块 (4-7)
                else if (p1 >= 4 && p2 >= 4) {
                    int r1 = ((p1 - 4) - slot_k + 4) % 4 + 4;
                    int r2 = ((p2 - 4) - slot_k + 4) % 4 + 4;
                    int key_a = r1, key_b = r2;
                    if (key_a > key_b) std::swap(key_a, key_b);
                    
                    auto it = aux_registry.find({key_a, key_b});
                    if (it != aux_registry.end()) {
                        def_ptr = &it->second;
                        // 逻辑位置初始化: (ID-4)*3 + 12
                        // 注意: key_a/b 已经是 4-7 的值了
                        std::vector<int> target = {(key_a - 4) * 3 + 12, (key_b - 4) * 3 + 12};
                        int init_idx = array_to_index(target, 2, 3, 8);
                        conjugated_idx = get_corner2_conjugated(alg, slot_k, init_idx);
                    }
                }

                if (def_ptr) {
                    if (count >= MAX_AUX) break;
                    out_aux[count].def = def_ptr;
                    out_aux[count].current_idx = conjugated_idx;
                    count++;
                }
            }
        }
        
        return count;
    }

    bool search_1(SearchContext& ctx, int i1, int i2, int i3, int depth, int prev, const unsigned char* p_prune) {
        const int* moves = valid_moves_flat[prev]; const int count = valid_moves_count[prev];
        for (int k = 0; k < count; ++k) {
            int m = moves[k]; COUNT_NODE
            int n_i1 = p_multi[i1 + m]; int n_i2 = p_corn[i2 + m];
            long long idx = (long long)(n_i1 + n_i2) * 24 + p_edge[i3 + m];
            if (get_prune_ptr(p_prune, idx) >= depth) continue;
            if (depth == 1) { ctx.sol_len.push_back(ctx.current_max_depth); return true; }
            if (search_1(ctx, n_i1, n_i2 * 18, p_edge[i3 + m] * 18, depth - 1, m, p_prune)) return true;
        }
        return false;
    }

    bool search_2(SearchContext& ctx, int i1a, int i2a, int i3a, const unsigned char* p1, 
                  int i1b, int i2b, int i3b, const int* tr_b, const unsigned char* p2, 
                  int i_e6, int i_c2, const unsigned char* p_huge_table, bool mirror_huge, // Added mirror_huge
                  int depth, int prev, 
                  int num_aux, const AuxState* aux_states) {
        
        cnt_search2_total.fetch_add(1, std::memory_order_relaxed);
        if (p_huge_table) cnt_huge_active.fetch_add(1, std::memory_order_relaxed);

        const int* moves = valid_moves_flat[prev]; const int count = valid_moves_count[prev];
        for (int k = 0; k < count; ++k) {
            int m = moves[k]; COUNT_NODE
            
            // 1. Aux Pruning (First!)
            int n_i1a = p_multi[i1a + m];
            int cross_state_idx = n_i1a / 24; 

            bool aux_pruned = false;
            AuxState next_aux[MAX_AUX];
            for (int i = 0; i < num_aux; ++i) {
                const auto& cur = aux_states[i];
                if (!cur.def) continue; 
                next_aux[i].def = cur.def;
                next_aux[i].current_idx = cur.def->p_move[cur.current_idx * 18 + m];
                
                long long idx_aux = (long long)cross_state_idx * cur.def->multiplier + next_aux[i].current_idx;
                if (get_prune_ptr(cur.def->p_prune, idx_aux) >= depth) {
                    aux_pruned = true;
                    break;
                }
            }
            if (aux_pruned) {
                cnt_aux_pruned.fetch_add(1, std::memory_order_relaxed);
                continue;
            }

            // 2. Huge Table Pruning
            int n_e6 = 0, n_c2 = 0;
            bool huge_pruned = false;
            if (p_huge_table) {
                n_e6 = p_edge6[i_e6 + m];
                n_c2 = p_corners2[i_c2 + m];
                
                long long huge_idx;
                if (mirror_huge) {
                    int e_pos = n_e6 / 64; int e_ori = n_e6 % 64;
                    int ne6 = sym_edge6_pos[e_pos] * 64 + sym_edge6_ori[e_ori];
                    int nc2 = sym_corner2[n_c2];
                    huge_idx = (long long)ne6 * 504 + nc2;
                } else {
                    huge_idx = (long long)n_e6 * 504 + n_c2;
                }
                
                if (get_prune_ptr(p_huge_table, huge_idx) >= depth) {
                    huge_pruned = true;
                }
            }

            if (huge_pruned) {
                cnt_huge_pruned.fetch_add(1, std::memory_order_relaxed);
                continue;
            }

            // 3. Base Pruning (Side A) - Skip if Huge Table was checked (Twin Titans dominance)
            // Compute Side A recursive states (needed regardless of pruning source)
            int n_i2a = p_corn[i2a + m];
            int n_i3a = p_edge[i3a + m];

            if (!p_huge_table) {
                long long idx1 = (long long)(n_i1a + n_i2a) * 24 + n_i3a;
                if (get_prune_ptr(p1, idx1) >= depth) {
                    cnt_base_pruned.fetch_add(1, std::memory_order_relaxed);
                    continue;
                }
            }
            
            // 4. Base Pruning (Side B)

            int m_b = tr_b[m]; int n_i1b = p_multi[i1b + m_b]; int n_i2b = p_corn[i2b + m_b]; 
            long long idx2 = (long long)(n_i1b + n_i2b) * 24 + p_edge[i3b + m_b];
            if (get_prune_ptr(p2, idx2) >= depth) continue;
            
            if (depth == 1) { return true; }
            if (search_2(ctx, n_i1a, n_i2a * 18, n_i3a * 18, p1, 
                         n_i1b, n_i2b * 18, p_edge[i3b + m_b] * 18, tr_b, p2, 
                         n_e6 * 18, n_c2 * 18, p_huge_table, mirror_huge,
                         depth - 1, m, num_aux, next_aux)) return true;
        }
        return false;
    }

    bool search_3(SearchContext& ctx, int i1a, int i2a, int i3a, const unsigned char* p1, 
                  int i1b, int i2b, int i3b, const int* tr_b, const unsigned char* p2, 
                  int i1c, int i2c, int i3c, const int* tr_c, const unsigned char* p3, 
                  int depth, int prev, 
                  int num_aux, const AuxState* aux_states) {
        
        cnt_search3_total.fetch_add(1, std::memory_order_relaxed);

        const int* moves = valid_moves_flat[prev]; const int count = valid_moves_count[prev];
        for (int k = 0; k < count; ++k) {
            int m = moves[k]; COUNT_NODE
            
            // 1. Aux Pruning (Moved to Front)
            int n_i1a = p_multi[i1a + m];
            int cross_state_idx = n_i1a / 24;

            bool aux_pruned = false;
            AuxState next_aux[MAX_AUX];
            for (int i = 0; i < num_aux; ++i) {
                const auto& cur = aux_states[i];
                if (!cur.def) continue;
                next_aux[i].def = cur.def;
                next_aux[i].current_idx = cur.def->p_move[cur.current_idx * 18 + m];
                long long idx_aux = (long long)cross_state_idx * cur.def->multiplier + next_aux[i].current_idx;
                if (get_prune_ptr(cur.def->p_prune, idx_aux) >= depth) {
                    aux_pruned = true;
                    break;
                }
            }
            if (aux_pruned) continue;

            // 2. Base Pruning
            int n_i2a = p_corn[i2a + m]; 
            long long idx1 = (long long)(n_i1a + n_i2a) * 24 + p_edge[i3a + m];
            if (get_prune_ptr(p1, idx1) >= depth) continue;
            
            int m_b = tr_b[m]; int n_i1b = p_multi[i1b + m_b]; int n_i2b = p_corn[i2b + m_b]; 
            long long idx2 = (long long)(n_i1b + n_i2b) * 24 + p_edge[i3b + m_b];
            if (get_prune_ptr(p2, idx2) >= depth) continue;
            
            int m_c = tr_c[m]; int n_i1c = p_multi[i1c + m_c]; int n_i2c = p_corn[i2c + m_c]; 
            long long idx3 = (long long)(n_i1c + n_i2c) * 24 + p_edge[i3c + m_c];
            if (get_prune_ptr(p3, idx3) >= depth) continue;
            
            if (depth == 1) { ctx.sol_len.push_back(ctx.current_max_depth); return true; }
            if (search_3(ctx, n_i1a, n_i2a * 18, p_edge[i3a + m] * 18, p1, 
                         n_i1b, n_i2b * 18, p_edge[i3b + m_b] * 18, tr_b, p2, 
                         n_i1c, n_i2c * 18, p_edge[i3c + m_c] * 18, tr_c, p3, 
                         depth - 1, m, num_aux, next_aux)) return true;
        }
        return false;
    }

    std::vector<int> get_stats(const std::vector<int>& base_alg, const std::vector<std::string>& rots, CrossSolver& ca) {
        std::vector<int> results; results.reserve(24);
        results = ca.get_stats(base_alg, rots);

        std::vector<std::vector<ConjState>> precomputed_states(rots.size(), std::vector<ConjState>(4));
        for(size_t r=0; r<rots.size(); ++r) {
            std::vector<int> alg = alg_rotation(base_alg, rots[r]);
            for(int k=0; k<4; ++k) get_conjugated_indices_all(alg, k, precomputed_states[r][k]);
        }
        auto get_h = [&](const ConjState& s, int diff_idx) {
             long long idx = (long long)(s.im + s.ic_b) * 24 + s.ie_rel[diff_idx];
             return get_prune_ptr(p_prune_base[diff_idx], idx);
        };
        
        {
            std::vector<int> stage_min(rots.size(), 99);
            for(size_t r=0; r<rots.size(); ++r) {
                std::vector<PseudoTask1> tasks;
                for (int c=0; c<4; ++c) { for (int e=0; e<4; ++e) { int diff = get_diff(c, e); tasks.push_back({c, diff, get_h(precomputed_states[r][c], diff)}); } } 
                std::sort(tasks.begin(), tasks.end(), [](const PseudoTask1& a, const PseudoTask1& b){ return a.h < b.h; });
                int current_best = 99;
                for(auto& t : tasks) {
                    if (t.h >= current_best) break;
                    int res = 99; if(t.h > 0) { SearchContext ctx; int max_search = std::min(16, current_best - 1); auto& st = precomputed_states[r][t.c_idx];
                        for(int d=t.h; d<=max_search; ++d) { ctx.current_max_depth=d; if(search_1(ctx, st.im, st.ic_b*18, st.ie_rel[t.diff]*18, d, 18, p_prune_base[t.diff])) { res = d; break; } } 
                    } else { res = 0; } if(res < current_best) current_best = res;
                }
                stage_min[r] = current_best;
            }
            results.insert(results.end(), stage_min.begin(), stage_min.end());
        }

        {
            std::vector<int> stage_min(rots.size(), 99); std::vector<std::pair<int,int>> pairs = {{0,1},{0,2},{0,3},{1,2},{1,3},{2,3}};
            for(size_t r=0; r<rots.size(); ++r) {
                std::vector<PseudoTask2> tasks;
                std::vector<int> alg = alg_rotation(base_alg, rots[r]); 
                
                for (auto& cp : pairs) { 
                    for (auto& ep : pairs) {
                        int d1 = get_diff(cp.first, ep.first); int d2 = get_diff(cp.second, ep.second); 
                        int h_base = std::max(get_h(precomputed_states[r][cp.first], d1), get_h(precomputed_states[r][cp.second], d2));
                        
                        // --- Huge Table Integration (Twin Titans) ---
                        const unsigned char* selected_huge_table = nullptr;
                        bool use_mirror = false;
                        int init_e6 = 0, init_c2 = 0;
                        
                        // Only for Neighbor Pairs (0,1) with matching Diffs
                        // Removed r == 0 restriction to support all rotations
                        if (cp.first == 0 && cp.second == 1 && d1 == d2) {
                            if (d1 == 0) selected_huge_table = p_huge_neighbor; // Diff 0
                            else if (d1 == 1) selected_huge_table = p_huge_neighbor_d1; // Diff 1
                            else if (d1 == 3) {
                                selected_huge_table = p_huge_neighbor_d1; // Reuse Diff 1 table
                                use_mirror = true;
                            }
                        }
                        
                        if (selected_huge_table) {
                                static int slv_e6_n = -1, slv_c2_n = -1;
                                if (slv_e6_n == -1) {
                                    std::vector<int> v1={0,2,16,18,20,22}; slv_e6_n = array_to_index(v1,6,2,12);
                                    std::vector<int> v2={12,15}; slv_c2_n = array_to_index(v2,2,3,8);
                                }
                                init_e6 = slv_e6_n;
                                init_c2 = slv_c2_n;
                                
                                // For mirror mode, we need to transform the INITIAL state too?
                                // Yes. The search loop transforms every state. The initial state is just State 0.
                                // But wait, 'init_e6' is computed based on 'alg'.
                                // So we just compute the standard 'init_e6' here.
                                // The 'search_2' function will apply mirror to it at depth 0 (first check).
                                
                                for (int m : alg) {
                                    init_e6 = p_edge6[init_e6 * 18 + m];
                                    init_c2 = p_corners2[init_c2 * 18 + m];
                                }
                                
                                long long huge_idx;
                                if (use_mirror) {
                                    int e_pos = init_e6 / 64;
                                    int e_ori = init_e6 % 64;
                                    int ne6 = sym_edge6_pos[e_pos] * 64 + sym_edge6_ori[e_ori];
                                    int nc2 = sym_corner2[init_c2];
                                    huge_idx = (long long)ne6 * 504 + nc2;
                                } else {
                                    huge_idx = (long long)init_e6 * 504 + init_c2;
                                }

                                int h_huge = get_prune_ptr(selected_huge_table, huge_idx);
                                if (h_huge > h_base) h_base = h_huge;
                        }
                        // ------------------------------------------------

                        AuxState aux[MAX_AUX];
                        int n_aux = 0;
                        if (true) { // Safety constraint removed
                            n_aux = setup_aux_pruners_struct({cp.first+4, cp.second+4, ep.first, ep.second}, alg, cp.first, aux);
                            int cross_idx_high = precomputed_states[r][cp.first].im / 24;
                            for(int k=0; k<n_aux; ++k) {
                                long long idx_aux = (long long)cross_idx_high * aux[k].def->multiplier + aux[k].current_idx;
                                h_base = std::max(h_base, get_prune_ptr(aux[k].def->p_prune, idx_aux));
                            }
                        }
                        
                        PseudoTask2 t(cp.first, cp.second, d1, d2, h_base, n_aux);
                        t.i_e6 = init_e6; t.i_c2 = init_c2; t.p_huge_table = selected_huge_table; t.mirror_huge = use_mirror;
                        for(int k=0; k<n_aux; ++k) t.aux_init[k] = aux[k];
                        tasks.push_back(t);
                        
                        int d1_s = get_diff(cp.first, ep.second); int d2_s = get_diff(cp.second, ep.first); 
                        h_base = std::max(get_h(precomputed_states[r][cp.first], d1_s), get_h(precomputed_states[r][cp.second], d2_s));
                        
                        n_aux = 0;
                        if (true) {
                            n_aux = setup_aux_pruners_struct({cp.first+4, cp.second+4, ep.second, ep.first}, alg, cp.first, aux);
                            int cross_idx_high = precomputed_states[r][cp.first].im / 24;
                            for(int k=0; k<n_aux; ++k) {
                                long long idx_aux = (long long)cross_idx_high * aux[k].def->multiplier + aux[k].current_idx;
                                h_base = std::max(h_base, get_prune_ptr(aux[k].def->p_prune, idx_aux));
                            }
                        }
                        
                        PseudoTask2 t2(cp.first, cp.second, d1_s, d2_s, h_base, n_aux);
                        for(int k=0; k<n_aux; ++k) t2.aux_init[k] = aux[k];
                        tasks.push_back(t2);
                } } 
                std::sort(tasks.begin(), tasks.end(), [](const PseudoTask2& a, const PseudoTask2& b){ return a.h < b.h; });
                int current_best = 99;
                for(auto& t : tasks) {
                    if (t.h >= current_best) break;
                    int res = 99; if(t.h > 0) { SearchContext ctx; int max_search = std::min(16, current_best - 1); auto& st1 = precomputed_states[r][t.c1]; auto& st2 = precomputed_states[r][t.c2];
                        for(int d=t.h; d<=max_search; ++d) { ctx.current_max_depth=d; 
                            if(search_2(ctx, st1.im, st1.ic_b*18, st1.ie_rel[t.diff1]*18, p_prune_base[t.diff1], 
                                            st2.im, st2.ic_b*18, st2.ie_rel[t.diff2]*18, trans_moves[t.c1][t.c2], p_prune_base[t.diff2], 
                                            t.i_e6*18, t.i_c2*18, t.p_huge_table, t.mirror_huge,
                                            d, 18, t.num_aux, t.aux_init)) { res = d; break; } } 
                    } else { res = 0; } if(res < current_best) current_best = res;
                }
                stage_min[r] = current_best;
            }
            results.insert(results.end(), stage_min.begin(), stage_min.end());
        }

        {
            std::vector<int> stage_min(rots.size(), 99); std::vector<std::vector<int>> triples = {{0,1,2},{0,1,3},{0,2,3},{1,2,3}};
            for(size_t r=0; r<rots.size(); ++r) {
                std::vector<PseudoTask3> tasks;
                std::vector<int> alg = alg_rotation(base_alg, rots[r]);

                for (auto& ct : triples) { 
                    for (auto& et : triples) {
                        std::vector<int> p = {0, 1, 2}; do { 
                            int d1 = get_diff(ct[0], et[p[0]]); int d2 = get_diff(ct[1], et[p[1]]); int d3 = get_diff(ct[2], et[p[2]]);
                            if (d1 == d2 && d2 == d3) {
                                int h_base = std::max({get_h(precomputed_states[r][ct[0]], d1), get_h(precomputed_states[r][ct[1]], d2), get_h(precomputed_states[r][ct[2]], d3)});
                                
                                AuxState aux[MAX_AUX];
                                int n_aux = 0;
                                n_aux = setup_aux_pruners_struct({ct[0]+4, ct[1]+4, ct[2]+4, et[p[0]], et[p[1]], et[p[2]]}, alg, ct[0], aux);
                                int cross_idx_high = precomputed_states[r][ct[0]].im / 24;
                                for(int k=0; k<n_aux; ++k) {
                                    long long idx_aux = (long long)cross_idx_high * aux[k].def->multiplier + aux[k].current_idx;
                                    h_base = std::max(h_base, get_prune_ptr(aux[k].def->p_prune, idx_aux));
                                }
                                
                                PseudoTask3 t(ct[0], ct[1], ct[2], d1, d2, d3, h_base, n_aux);
                                for(int k=0; k<n_aux; ++k) t.aux_init[k] = aux[k];
                                tasks.push_back(t);
                            }
                        } while(std::next_permutation(p.begin(), p.end()));
                } } 
                std::sort(tasks.begin(), tasks.end(), [](const PseudoTask3& a, const PseudoTask3& b){ return a.h < b.h; });
                int current_best = 99;
                for(auto& t : tasks) {
                    if (t.h >= current_best) break;
                    int res = 99; if(t.h > 0) { SearchContext ctx; int max_search = std::min(16, current_best - 1); auto& s1 = precomputed_states[r][t.c1]; auto& s2 = precomputed_states[r][t.c2]; auto& s3 = precomputed_states[r][t.c3];
                        for(int d=t.h; d<=max_search; ++d) { ctx.current_max_depth=d; 
                             if(search_3(ctx, s1.im, s1.ic_b*18, s1.ie_rel[t.diff1]*18, p_prune_base[t.diff1], 
                                            s2.im, s2.ic_b*18, s2.ie_rel[t.diff2]*18, trans_moves[t.c1][t.c2], p_prune_base[t.diff2], 
                                            s3.im, s3.ic_b*18, s3.ie_rel[t.diff3]*18, trans_moves[t.c1][t.c3], p_prune_base[t.diff3], 
                                            d, 18, t.num_aux, t.aux_init)) { res = d; break; } } 
                    } else { res = 0; } if(res < current_best) current_best = res;
                }
                stage_min[r] = current_best;
            }
            results.insert(results.end(), stage_min.begin(), stage_min.end());
        }        return results;    }
};

int main() {
    system("color 0A");
    init_matrix();
    init_pseudo_matrix();
    initMirrorTables();
    std::cout << "Initializing Pseudo Analyzer (Generic Pruning)..." << std::endl;
    
    auto& mtm = MoveTableManager::getInstance();
    auto& ptm = PruneTableManager::getInstance();
    
    std::cout << "[Init] Loading necessary tables..." << std::endl;
    bool ok = true;
    if (!mtm.loadCrossTable()) ok = false;
    if (!mtm.loadCornerTable()) ok = false;
    if (!mtm.loadEdgeTable()) ok = false;
    if (!mtm.loadEdges2Table()) ok = false;
    // Corner2 table 尚未用于剪枝生成，但在通用结构中可以预加载
    if (mtm.loadCorner2Table()) std::cout << "  Corner2 Table loaded." << std::endl;
    // 加载 Edge6 移动表
    if (mtm.loadEdge6Table()) std::cout << "  Edge6 Table loaded." << std::endl;
    // 加载 Corner3 移动表
    if (mtm.loadCorner3Table()) std::cout << "  Corner3 Table loaded." << std::endl;
    // 加载 Edge3 移动表
    if (mtm.loadEdge3Table()) std::cout << "  Edge3 Table loaded." << std::endl;

    if (!ok) { std::cerr << "Error: Move tables missing. Run table_generator.exe." << std::endl; return 1; }
    
    if (!ptm.loadPseudoTables()) { std::cerr << "Error: Pseudo prune tables missing. Run table_generator.exe." << std::endl; return 1; }
    
    CrossSolver ca;
    XCrossSolver xcs;

    std::vector<std::string> rots = {"", "z2", "z'", "z", "x'", "x"};

    while (true) {
        std::string in_fn; std::cout << "\nEnter file: "; if (!(std::cin >> in_fn)) break;
        std::vector<std::pair<std::string, std::vector<int>>> tasks; std::ifstream infile(in_fn); if(!infile) { std::cout << "File not found." << std::endl; continue; }
        std::string line; while (std::getline(infile, line)) { if (line.empty()) continue; size_t p = line.find(','); if (p != std::string::npos) tasks.push_back({line.substr(0, p), string_to_alg(line.substr(p+1))}); else tasks.push_back({std::to_string(tasks.size() + 1), string_to_alg(line)}); } infile.close();

        std::cout << "Loaded " << tasks.size() << " tasks." << std::endl;
        std::ofstream outfile(in_fn + ".pseudo.csv");
        outfile << "Scramble"; for(const auto& r : rots) outfile << ",pCross_" << r; for(const auto& r : rots) outfile << ",pXC_" << r; for(const auto& r : rots) outfile << ",pXXC_" << r; for(const auto& r : rots) outfile << ",pXXXC_" << r; outfile << "\n";
        auto start = std::chrono::high_resolution_clock::now(); int total = tasks.size();
        std::vector<std::string> result_buffer(total); std::vector<bool> result_ready(total, false); int next_write_idx = 0;
        global_nodes = 0; completed_tasks = 0; is_solving = true;

        std::thread monitor_thread([&]() {
            auto t0 = std::chrono::high_resolution_clock::now();
            while (is_solving) { std::this_thread::sleep_for(std::chrono::milliseconds(200)); auto t1 = std::chrono::high_resolution_clock::now(); double dt = std::chrono::duration<double>(t1 - t0).count(); long long n = global_nodes.load(std::memory_order_relaxed); int c = completed_tasks.load(std::memory_order_relaxed); double nps = (dt > 0.001) ? n / dt : 0; printf("Progress: %d/%d (%.1f%%) | NPS: %.2f M/s      \r", c, total, (float)c*100.0/total, nps / 1000000.0); fflush(stdout); } 
        });

        #pragma omp parallel for schedule(dynamic, 1)
        for (int i = 0; i < total; ++i) {
            std::vector<int> s_pseudo = xcs.get_stats(tasks[i].second, rots, ca);
            std::stringstream ss; ss << tasks[i].first; for (int v : s_pseudo) ss << "," << v; ss << "\n";
            result_buffer[i] = ss.str(); result_ready[i] = true; completed_tasks.fetch_add(1, std::memory_order_relaxed);
            #pragma omp critical
            { while (next_write_idx < total && result_ready[next_write_idx]) { outfile << result_buffer[next_write_idx]; next_write_idx++; } } 
        }
        is_solving = false; monitor_thread.join();
        auto end = std::chrono::high_resolution_clock::now(); double total_dt = std::chrono::duration<double>(end-start).count();
        std::cout << "\nDone. Total Time: " << total_dt << "s" << " | Avg NPS: " << (global_nodes / total_dt / 1000000.0) << " M/s" << std::endl;
        
        long long s2_tot = cnt_search2_total.load();
        long long s3_tot = cnt_search3_total.load();
        long long h_act = cnt_huge_active.load();
        long long aux_p = cnt_aux_pruned.load();
        long long huge_p = cnt_huge_pruned.load();
        long long base_p = cnt_base_pruned.load();
        
        printf("--- Profiling Stats ---\n");
        printf("Search2 Calls: %lld\n", s2_tot);
        printf("Search3 Calls: %lld\n", s3_tot);
        printf("Huge Active:   %lld (%.1f%%)\n", h_act, s2_tot > 0 ? (100.0 * h_act / s2_tot) : 0);
        printf("Aux Pruned:    %lld\n", aux_p);
        printf("Huge Pruned:   %lld\n", huge_p);
        printf("Base Pruned:   %lld\n", base_p);
        printf("-----------------------\n");
    }
    return 0;
}
