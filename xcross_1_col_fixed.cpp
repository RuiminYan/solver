#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <string>
#include <vector>
#include <omp.h>
#include <iomanip>
#include <cstring>

// --- 基础辅助与查找表 ---
std::vector<std::string> move_names = {"U", "U2", "U'", "D", "D2", "D'", "L", "L2", "L'", "R", "R2", "R'", "F", "F2", "F'", "B", "B2", "B'"};
std::vector<std::vector<int>> c_array = {{0},{1,1,1,1,1,1,1},{1,2,4,8,16,32,64},{1,3,9,27,81,243,729}};
std::vector<std::vector<int>> base_array = {{0},{0},{1,12,132,1320,11880,95040},{1,8,56,336,1680,6720}};
std::vector<std::vector<int>> base_array2 = {{0},{0},{12,11,10,9,8,7},{8,7,6,5,4,3}};
std::vector<int> sorted(6);

// --- 魔方基础逻辑 (用于生成移动表) ---
struct State {
    std::vector<int> cp, co, ep, eo;
    State(std::vector<int> c_p={0,1,2,3,4,5,6,7}, std::vector<int> c_o={0,0,0,0,0,0,0,0}, std::vector<int> e_p={0,1,2,3,4,5,6,7,8,9,10,11}, std::vector<int> e_o={0,0,0,0,0,0,0,0,0,0,0,0}) 
        : cp(c_p), co(c_o), ep(e_p), eo(e_o) {}

    State apply_move_edge(State m, int e) {
        std::vector<int> nep(12,-1), neo(12,-1);
        auto it = std::find(ep.begin(), ep.end(), e); int idx = std::distance(ep.begin(), it);
        it = std::find(m.ep.begin(), m.ep.end(), e); int idx_next = std::distance(m.ep.begin(), it);
        nep[idx_next] = e; neo[idx_next] = (eo[idx] + m.eo[idx_next]) % 2;
        return State(cp, co, nep, neo);
    }
    State apply_move_corner(State m, int c) {
        std::vector<int> ncp(8,-1), nco(8,-1);
        auto it = std::find(cp.begin(), cp.end(), c); int idx = std::distance(cp.begin(), it);
        it = std::find(m.cp.begin(), m.cp.end(), c); int idx_next = std::distance(m.cp.begin(), it);
        ncp[idx_next] = c; nco[idx_next] = (co[idx] + m.co[idx_next]) % 3;
        return State(ncp, nco, ep, eo);
    }
};

std::unordered_map<std::string, State> moves_map = {
    {"U", State({3,0,1,2,4,5,6,7}, {0,0,0,0,0,0,0,0}, {0,1,2,3,7,4,5,6,8,9,10,11}, {0,0,0,0,0,0,0,0,0,0,0,0})},
    {"U2", State({2,3,0,1,4,5,6,7}, {0,0,0,0,0,0,0,0}, {0,1,2,3,6,7,4,5,8,9,10,11}, {0,0,0,0,0,0,0,0,0,0,0,0})},
    {"U'", State({1,2,3,0,4,5,6,7}, {0,0,0,0,0,0,0,0}, {0,1,2,3,5,6,7,4,8,9,10,11}, {0,0,0,0,0,0,0,0,0,0,0,0})},
    {"D", State({0,1,2,3,5,6,7,4}, {0,0,0,0,0,0,0,0}, {0,1,2,3,4,5,6,7,9,10,11,8}, {0,0,0,0,0,0,0,0,0,0,0,0})},
    {"D2", State({0,1,2,3,6,7,4,5}, {0,0,0,0,0,0,0,0}, {0,1,2,3,4,5,6,7,10,11,8,9}, {0,0,0,0,0,0,0,0,0,0,0,0})},
    {"D'", State({0,1,2,3,7,4,5,6}, {0,0,0,0,0,0,0,0}, {0,1,2,3,4,5,6,7,11,8,9,10}, {0,0,0,0,0,0,0,0,0,0,0,0})},
    {"L", State({4,1,2,0,7,5,6,3}, {2,0,0,1,1,0,0,2}, {11,1,2,7,4,5,6,0,8,9,10,3}, {0,0,0,0,0,0,0,0,0,0,0,0})},
    {"L2", State({7,1,2,4,3,5,6,0}, {0,0,0,0,0,0,0,0}, {3,1,2,0,4,5,6,11,8,9,10,7}, {0,0,0,0,0,0,0,0,0,0,0,0})},
    {"L'", State({3,1,2,7,0,5,6,4}, {2,0,0,1,1,0,0,2}, {7,1,2,11,4,5,6,3,8,9,10,0}, {0,0,0,0,0,0,0,0,0,0,0,0})},
    {"R", State({0,2,6,3,4,1,5,7}, {0,1,2,0,0,2,1,0}, {0,5,9,3,4,2,6,7,8,1,10,11}, {0,0,0,0,0,0,0,0,0,0,0,0})},
    {"R2", State({0,6,5,3,4,2,1,7}, {0,0,0,0,0,0,0,0}, {0,2,1,3,4,9,6,7,8,5,10,11}, {0,0,0,0,0,0,0,0,0,0,0,0})},
    {"R'", State({0,5,1,3,4,6,2,7}, {0,1,2,0,0,2,1,0}, {0,9,5,3,4,1,6,7,8,2,10,11}, {0,0,0,0,0,0,0,0,0,0,0,0})},
    {"F", State({0,1,3,7,4,5,2,6}, {0,0,1,2,0,0,2,1}, {0,1,6,10,4,5,3,7,8,9,2,11}, {0,0,1,1,0,0,1,0,0,0,1,0})},
    {"F2", State({0,1,7,6,4,5,3,2}, {0,0,0,0,0,0,0,0}, {0,1,3,2,4,5,10,7,8,9,6,11}, {0,0,0,0,0,0,0,0,0,0,0,0})},
    {"F'", State({0,1,6,2,4,5,7,3}, {0,0,1,2,0,0,2,1}, {0,1,10,6,4,5,2,7,8,9,3,11}, {0,0,1,1,0,0,1,0,0,0,1,0})},
    {"B", State({1,5,2,3,0,4,6,7}, {1,2,0,0,2,1,0,0}, {4,8,2,3,1,5,6,7,0,9,10,11}, {1,1,0,0,1,0,0,0,1,0,0,0})},
    {"B2", State({5,4,2,3,1,0,6,7}, {0,0,0,0,0,0,0,0}, {1,0,2,3,8,5,6,7,4,9,10,11}, {0,0,0,0,0,0,0,0,0,0,0,0})},
    {"B'", State({4,0,2,3,5,1,6,7}, {1,2,0,0,2,1,0,0}, {8,4,2,3,0,5,6,7,1,9,10,11}, {1,1,0,0,1,0,0,0,1,0,0,0})}
};

// --- 坐标转换函数 ---
inline int array_to_index(std::vector<int> &a, int n, int c, int pn) {
    int idx_p=0, idx_o=0, tmp, tmp2=24/pn;
    for (int i=0; i<n; ++i) { idx_o += (a[i]%c) * c_array[c][n-i-1]; a[i]/=c; }
    for (int i=0; i<n; ++i) {
        tmp=0; for (int j=0; j<i; ++j) if (a[j]<a[i]) tmp++;
        idx_p += (a[i]-tmp) * base_array[tmp2][i];
    }
    return idx_p * c_array[c][n] + idx_o;
}
inline void index_to_array(std::vector<int> &p, int index, int n, int c, int pn) {
    int tmp2=24/pn, p_idx=index/c_array[c][n], o_idx=index%c_array[c][n];
    for (int i=0; i<n; ++i) {
        p[i] = p_idx % base_array2[tmp2][i]; p_idx /= base_array2[tmp2][i];
        std::sort(sorted.begin(), sorted.begin()+i);
        for (int j=0; j<i; ++j) if (sorted[j]<=p[i]) p[i]++;
        sorted[i] = p[i];
    }
    for (int i=0; i<n; ++i) { p[n-i-1] = 18 * (c*p[n-i-1] + o_idx%c); o_idx /= c; }
}

// --- 生成移动表函数 ---
std::vector<int> create_edge_move_table() {
    std::vector<int> mt(24*18, -1);
    for (int i=0; i<24; ++i) {
        State s; s.ep={0,1,2,3,4,5,6,7,8,9,10,11}; s.eo={0,0,0,0,0,0,0,0,0,0,0,0};
        s.ep[i/2]=i/2; s.eo[i/2]=i%2;
        for (int j=0; j<18; ++j) {
            State ns = s.apply_move_edge(moves_map[move_names[j]], i/2);
            auto it = std::find(ns.ep.begin(), ns.ep.end(), i/2);
            int idx = std::distance(ns.ep.begin(), it);
            mt[18*i+j] = 2*idx + ns.eo[idx];
        }
    }
    return mt;
}
std::vector<int> create_corner_move_table() {
    std::vector<int> mt(24*18, -1);
    for (int i=0; i<24; ++i) {
        State s; s.cp={0,1,2,3,4,5,6,7}; s.co={0,0,0,0,0,0,0,0};
        s.cp[i/3]=i/3; s.co[i/3]=i%3;
        for (int j=0; j<18; ++j) {
            State ns = s.apply_move_corner(moves_map[move_names[j]], i/3);
            auto it = std::find(ns.cp.begin(), ns.cp.end(), i/3);
            int idx = std::distance(ns.cp.begin(), it);
            mt[18*i+j] = 3*idx + ns.co[idx];
        }
    }
    return mt;
}
std::vector<int> create_multi_move_table2(int n, int c, int pn, int size, const std::vector<int> &basic_t) {
    std::vector<int> mt(size*24, -1);
    std::vector<int> a(n), b(n);
    std::vector<int> inv = {2,1,0,5,4,3,8,7,6,11,10,9,14,13,12,17,16,15};
    for (int i=0; i<size; ++i) {
        index_to_array(a, i, n, c, pn);
        int tmp_i = i*24;
        for (int j=0; j<18; ++j) {
            if (mt[tmp_i+j] == -1) {
                for (int k=0; k<n; ++k) b[k] = basic_t[a[k]+j];
                int tmp = 24 * array_to_index(b, n, c, pn);
                mt[tmp_i+j] = tmp;
                mt[tmp + inv[j]] = tmp_i;
            }
        }
    }
    return mt;
}

// --- 核心：BFS 生成分布并打印 ---
void generate_and_print_distribution(
    int idx_cr, int idx_cn, int idx_ed, 
    int sz_cr, int sz_cn, int sz_ed, 
    const std::vector<int> &t1, const std::vector<int> &t2, const std::vector<int> &t3) 
{
    long long total = (long long)sz_cr * sz_cn * sz_ed;
    std::cout << "State Space Size: " << total << std::endl;
    std::cout << "Allocating memory (approx " << (total / 1024 / 1024) << " MB)..." << std::endl;

    // 使用 255 表示未访问。深度通常不会超过 20，unsigned char 足够。
    std::vector<unsigned char> tmp;
    try { 
        tmp.resize(total, 255); 
    } catch(...) { 
        std::cerr << "Memory allocation failed.\n"; 
        exit(1); 
    }
    
    // 初始化起始状态 (复原态)
    long long cur_cr = idx_cr * 24LL;
    int cur_cn = idx_cn * 18;
    int cur_ed = idx_ed;            
    long long start_idx = (cur_cr + cur_cn/18) * 24 + cur_ed;
    if (start_idx < total) tmp[start_idx] = 0;
    
    std::vector<long long> dist;
    long long total_visited = 0;

    std::cout << "\n=== Computing Fixed Slot X-Cross Distribution ===\n" << std::endl;

    for (int d = 0; d < 20; ++d) { // 深度循环
        int nd = d + 1; 
        long long cnt = 0;

        // OpenMP 并行扫描
        #pragma omp parallel for reduction(+:cnt)
        for (long long i = 0; i < total; ++i) {
            if (tmp[i] == d) {
                cnt++;
                // 解码当前状态索引，以便查表
                long long comb = i / sz_ed;
                int cur_ed_idx = i % sz_ed;
                int cur_cr_idx = (comb / sz_cn) * 24;
                int cur_cn_idx = (comb % sz_cn) * 18;
                int idx3_base = cur_ed_idx * 18;

                // 尝试 18 种转动
                for (int j = 0; j < 18; ++j) {
                    long long n_cr = t1[cur_cr_idx + j];
                    int n_cn = t2[cur_cn_idx + j];
                    // 计算新状态索引
                    long long ni = (n_cr + n_cn) * 24 + t3[idx3_base + j];
                    // 如果未访问过，标记为下一层深度
                    if (tmp[ni] == 255) tmp[ni] = nd;
                }
            }
        }

        if (cnt > 0) {
            dist.push_back(cnt);
            total_visited += cnt;
            std::cout << "Depth " << std::setw(2) << d << ": " << std::setw(12) << cnt << std::endl;
        } else {
            break; // 没发现新节点，结束
        }
    }

    // --- 打印最终格式化表格 ---
    std::cout << "\n=============================================" << std::endl;
    std::cout << "       Fixed Slot X-Cross Distribution       " << std::endl;
    std::cout << "=============================================" << std::endl;
    std::cout << " Depth |    Count     |   Percent   | Cumul %" << std::endl;
    std::cout << "-------|--------------|-------------|--------" << std::endl;

    long long cumul = 0;
    long long sum_dist = 0;

    for (size_t d = 0; d < dist.size(); ++d) {
        cumul += dist[d];
        sum_dist += dist[d] * d;
        double pct = (double)dist[d] / total * 100.0;
        double cum_pct = (double)cumul / total * 100.0;
        
        std::cout << std::setw(6) << d << " | " 
                  << std::setw(12) << dist[d] << " | "
                  << std::fixed << std::setprecision(4) << std::setw(8) << pct << "% | "
                  << std::setw(6) << std::setprecision(2) << cum_pct << "%" << std::endl;
    }
    std::cout << "---------------------------------------------" << std::endl;
    std::cout << " Total : " << std::setw(12) << total_visited << " / " << total << std::endl;
    std::cout << " Avg Len: " << (double)sum_dist / total_visited << std::endl;
    std::cout << "=============================================" << std::endl;
}

int main() {
    // 1. 初始化移动表
    std::cout << "[Init] Generating Move Tables..." << std::endl;
    auto mt_edge = create_edge_move_table();
    auto mt_corn = create_corner_move_table();
    // Cross part uses Multi table (Permutation of 4 edges)
    auto mt_multi = create_multi_move_table2(4, 2, 12, 24*22*20*18, mt_edge);

    // 2. 定义目标状态参数
    // Cross Solved Index (4 edges correct): 187520
    // Corner Solved (DFR corner): 12
    // Edge Solved (FR edge): 0
    int cr_slv = 187520; 
    int base_c = 12; 
    int base_e = 0;

    // 3. 执行 BFS 统计
    generate_and_print_distribution(
        cr_slv, base_c, base_e, 
        24*22*20*18, // Size of Cross permutation space
        24,          // Size of Corner space (8 pos * 3 ori)
        24,          // Size of Edge space (8 pos * 2 ori + buffer? No, actually 12 edges * 2 = 24 states per slot edge)
        mt_multi, mt_corn, mt_edge
    );

    return 0;
}