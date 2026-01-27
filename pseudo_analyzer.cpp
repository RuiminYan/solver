#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <omp.h>
#include <chrono>
#include <mutex>
#include <atomic>
#include <cstring>
#include <cmath>
#include <thread>
#include <numeric>

// --- 全局统计变量 ---
std::atomic<long long> global_nodes{0};
std::atomic<int> completed_tasks{0};
std::atomic<bool> is_solving{false};

// NPS 计数宏
#define COUNT_NODE \
    static thread_local int local_node_counter = 0; \
    local_node_counter++; \
    if (local_node_counter >= 1000) { \
        global_nodes.fetch_add(local_node_counter, std::memory_order_relaxed); \
        local_node_counter = 0; \
    }

// --- 通用结构 ---
struct SearchContext {
    std::vector<int> sol_len;
    int current_max_depth = 0;
    SearchContext() {}
};

// 移动表定义
// 优化：静态扁平化移动表，避免 vector 访问开销
int valid_moves_flat[20][18];
int valid_moves_count[20];

// [优化 1] 扁平化共轭表：使用静态数组替代 vector<vector>
// 原代码: std::vector<std::vector<int>> conj_moves;
int conj_moves_flat[18][4]; 

int trans_moves[4][4][18]; 

// 优化：指针版内联剪枝读取
inline int get_prune_ptr(const unsigned char* t, long long i) { 
    return (t[i >> 1] >> ((i & 1) << 2)) & 0xF; 
}

std::vector<std::string> move_names = {"U", "U2", "U'", "D", "D2", "D'", "L", "L2", "L'", "R", "R2", "R'", "F", "F2", "F'", "B", "B2", "B'"};

// 查表辅助
std::vector<std::vector<int>> c_array = {{0},{1,1,1,1,1,1,1},{1,2,4,8,16,32,64},{1,3,9,27,81,243,729}};
std::vector<std::vector<int>> base_array = {{0},{0},{1,12,132,1320,11880,95040},{1,8,56,336,1680,6720}};
std::vector<std::vector<int>> base_array2 = {{0},{0},{12,11,10,9,8,7},{8,7,6,5,4,3}};
std::vector<int> sorted(6);

// --- 魔方基础逻辑 ---
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

// 移动定义
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

// [优化 2] 硬编码移动映射：使用 switch-case 替代 find
int get_move_id(const std::string& name) {
    if(name.length() < 1) return -1;
    char face = name[0];
    int base = -1;
    switch(face) {
        case 'U': base = 0; break;
        case 'D': base = 3; break;
        case 'L': base = 6; break;
        case 'R': base = 9; break;
        case 'F': base = 12; break;
        case 'B': base = 15; break;
        default: return -1;
    }
    if (name.length() > 1) {
        if (name[1] == '2') base += 1;
        else if (name[1] == '\'') base += 2;
    }
    return base;
}

void init_matrix() {
    for (int prev = 0; prev <= 18; ++prev) {
        int cnt = 0;
        for (int i = 0; i < 18; ++i) {
            bool bad = (prev < 18) && (i / 3 == prev / 3 || ((i / 3) / 2 == (prev / 3) / 2 && (prev / 3) % 2 > (i / 3) % 2));
            if (!bad) valid_moves_flat[prev][cnt++] = i;
        }
        valid_moves_count[prev] = cnt;
    }
    
    // [优化 1] 初始化静态数组 conj_moves_flat
    for(int i=0; i<18; ++i) {
        int m_type = i/3;
        int m_pow = i%3;
        conj_moves_flat[i][0] = i;
        int m1 = -1; if(m_type==0) m1=i; else if(m_type==1) m1=i; else if(m_type==2) m1=12+m_pow; else if(m_type==3) m1=15+m_pow; else if(m_type==4) m1=9+m_pow; else if(m_type==5) m1=6+m_pow;
        conj_moves_flat[i][1] = m1;
        int m2 = -1; if(m_type==0) m2=i; else if(m_type==1) m2=i; else if(m_type==2) m2=9+m_pow; else if(m_type==3) m2=6+m_pow; else if(m_type==4) m2=15+m_pow; else if(m_type==5) m2=12+m_pow;
        conj_moves_flat[i][2] = m2;
        int m3 = -1; if(m_type==0) m3=i; else if(m_type==1) m3=i; else if(m_type==2) m3=15+m_pow; else if(m_type==3) m3=12+m_pow; else if(m_type==4) m3=6+m_pow; else if(m_type==5) m3=9+m_pow;
        conj_moves_flat[i][3] = m3;
    }

    for (int s1 = 0; s1 < 4; ++s1) {
        for (int s2 = 0; s2 < 4; ++s2) {
            for (int m_phys = 0; m_phys < 18; ++m_phys) {
                int m_s1 = conj_moves_flat[m_phys][s1]; // 使用 conj_moves_flat
                int m_s2 = conj_moves_flat[m_phys][s2]; // 使用 conj_moves_flat
                trans_moves[s1][s2][m_s1] = m_s2;
            }
        }
    }
}

// --- 辅助函数 ---
// [优化 2] 使用 get_move_id 优化解析
std::vector<int> string_to_alg(std::string str) {
    std::vector<int> alg; std::istringstream iss(str); std::string name;
    while (iss >> name) {
        int id = get_move_id(name);
        if (id != -1) alg.emplace_back(id);
    }
    return alg;
}

std::vector<int> alg_convert_rotation(std::vector<int> alg, std::string rot) {
    if (rot.empty()) return alg;
    std::vector<int> f;
    if (rot == "x") f = {5,4,2,3,0,1}; else if (rot == "x2") f = {1,0,2,3,5,4}; else if (rot == "x'") f = {4,5,2,3,1,0};
    else if (rot == "y") f = {0,1,5,4,2,3}; else if (rot == "y2") f = {0,1,3,2,5,4}; else if (rot == "y'") f = {0,1,4,5,3,2};
    else if (rot == "z") f = {3,2,0,1,4,5}; else if (rot == "z2") f = {1,0,3,2,4,5}; else if (rot == "z'") f = {2,3,1,0,4,5};
    for (size_t i=0; i<alg.size(); ++i) alg[i] = 3 * f[alg[i]/3] + alg[i]%3;
    return alg;
}

std::vector<int> alg_rotation(std::vector<int> a, std::string r) {
    std::istringstream iss(r); std::string tmp; while (iss >> tmp) a = alg_convert_rotation(a, tmp); return a;
}

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

template <typename T>
bool load_vector(std::vector<T>& vec, const std::string& filename) {
    std::ifstream in(filename, std::ios::binary);
    if (!in) return false; 
    in.seekg(0, std::ios::end); size_t file_size = in.tellg(); in.seekg(0, std::ios::beg);
    size_t size; in.read(reinterpret_cast<char*>(&size), sizeof(size));
    if (!in) return false;
    size_t expected_data_size = size * sizeof(T);
    if (file_size != expected_data_size + sizeof(size_t)) return false; 
    vec.resize(size);
    in.read(reinterpret_cast<char*>(vec.data()), expected_data_size);
    return in.good();
}

template <typename T>
bool save_vector(const std::vector<T>& vec, const std::string& filename) {
    std::ofstream out(filename, std::ios::binary);
    size_t size = vec.size();
    out.write(reinterpret_cast<const char*>(&size), sizeof(size));
    out.write(reinterpret_cast<const char*>(vec.data()), size * sizeof(T));
    return out.good();
}

inline void set_prune(std::vector<unsigned char> &table, long long index, int value) {
    int shift = (index & 1) << 2;
    table[index >> 1] &= ~(0xF << shift);
    table[index >> 1] |= (value & 0xF) << shift;
}

// --- 表生成函数 ---
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
std::vector<int> create_multi_move_table(int n, int c, int pn, int size, const std::vector<int> &basic_t) {
    std::vector<int> mt(size*18, -1);
    std::vector<int> a(n), b(n);
    std::vector<int> inv = {2,1,0,5,4,3,8,7,6,11,10,9,14,13,12,17,16,15};
    for (int i=0; i<size; ++i) {
        index_to_array(a, i, n, c, pn);
        int tmp_i = i*18;
        for (int j=0; j<18; ++j) {
            if (mt[tmp_i+j] == -1) {
                for (int k=0; k<n; ++k) b[k] = basic_t[a[k]+j];
                int tmp = array_to_index(b, n, c, pn);
                mt[tmp_i+j] = tmp;
                mt[18*tmp + inv[j]] = i;
            }
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

// Pseudo Cross Table Generation
void create_prune_table_xcross_full(int idx_cr, int idx_cn, int idx_ed, 
                                    int sz_cr, int sz_cn, int sz_ed, int depth, 
                                    const std::vector<int> &t1, const std::vector<int> &t2, const std::vector<int> &t3,
                                    std::vector<unsigned char> &pt, bool is_pseudo) {
    long long total = (long long)sz_cr * sz_cn * sz_ed;
    std::vector<unsigned char> tmp;
    try { tmp.resize(total, 255); } catch(...) { std::cerr << "Alloc fail\n"; exit(1); }
    
    int d_moves[] = {-1, 3, 4, 5}; 
    int num_init = is_pseudo ? 4 : 1; 

    for (int m_idx = 0; m_idx < num_init; ++m_idx) {
        long long cur_cr; int cur_cn; int cur_ed = idx_ed;            
        if (m_idx > 0) {
            int move = d_moves[m_idx];
            cur_cr = t1[idx_cr * 24 + move]; 
            cur_cn = t2[idx_cn * 18 + move] * 18;
        } else {
             cur_cr = idx_cr * 24LL;
             cur_cn = idx_cn * 18;
        }
        long long start_idx = (cur_cr + cur_cn/18) * 24 + cur_ed;
        if (start_idx < total) tmp[start_idx] = 0;
    }
    
    for (int d=0; d<depth; ++d) {
        int nd = d+1; long long cnt=0;
        #pragma omp parallel for reduction(+:cnt)
        for (long long i=0; i<total; ++i) {
            if (tmp[i] == d) {
                cnt++;
                long long comb = i / sz_ed;
                int cur_ed = i % sz_ed;
                int cur_cr = (comb / sz_cn) * 24;
                int cur_cn = (comb % sz_cn) * 18;
                int idx3_base = cur_ed * 18;
                for (int j=0; j<18; ++j) {
                    long long n_cr = t1[cur_cr + j];
                    int n_cn = t2[cur_cn + j];
                    long long ni = (n_cr + n_cn) * 24 + t3[idx3_base + j];
                    if (tmp[ni] == 255) tmp[ni] = nd;
                }
            }
        }
        std::cout << "  [Gen Pseudo Diff " << (idx_ed/2) << "] Depth " << d << ": " << cnt << std::endl;
        if (cnt==0) break;
    }
    std::fill(pt.begin(), pt.end(), 0xFF);
    #pragma omp parallel for
    for (long long i=0; i<total; ++i) if (tmp[i]!=255) set_prune(pt, i, tmp[i]);
}

// =========================================================
// 类：CrossSolver (Pseudo Support)
// =========================================================
struct CrossSolver {
    std::vector<int> edge_move_table, multi_move_table;
    std::vector<unsigned char> prune_table;
    const int* p_multi;
    bool is_pseudo;
    std::string move_prefix;
    
    CrossSolver(bool pseudo) : is_pseudo(pseudo) {
        move_prefix = "move_table_cross_";
        std::string prune_fn = pseudo ? "prune_table_pseudo_cross.bin" : "prune_table_std_cross.bin";

        bool loaded = false;
        if (load_vector(edge_move_table, move_prefix + "edge.bin") &&
            load_vector(multi_move_table, move_prefix + "multi.bin") &&
            load_vector(prune_table, prune_fn)) loaded = true;

        if (!loaded) {
            std::cout << "[Init] Gen " << (pseudo?"Pseudo":"Std") << " Cross..." << std::endl;
            edge_move_table = create_edge_move_table();
            multi_move_table = create_multi_move_table(2, 2, 12, 24 * 22, edge_move_table);
            long long sz = 24LL*22*24*22;
            std::vector<unsigned char> tmp(sz, 255); 
            int d_moves[] = {-1, 3, 4, 5}; 
            int num_init = is_pseudo ? 4 : 1;
            for(int k=0; k<num_init; ++k) {
                int i1=416, i2=520;
                if(k>0) {
                    i1 = multi_move_table[i1*18 + d_moves[k]];
                    i2 = multi_move_table[i2*18 + d_moves[k]];
                }
                tmp[(long long)i1*528 + i2] = 0;
            }
            for(int d=0;d<10;++d) {
                long long cnt = 0;
                #pragma omp parallel for reduction(+:cnt)
                for(long long i=0; i<sz; ++i) {
                    if(tmp[i]==d) {
                        cnt++;
                        int idx1=(i/528)*18, idx2=(i%528)*18;
                        for(int j=0; j<18; ++j) {
                            long long ni = (long long)multi_move_table[idx1+j]*528 + multi_move_table[idx2+j];
                            if(tmp[ni]==255) tmp[ni]=d+1;
                        }
                    }
                }
            }
            int c_sz = (sz+1)/2; prune_table.assign(c_sz, 0xFF);
            for(long long i=0;i<sz;++i) set_prune(prune_table, i, tmp[i]);
            save_vector(edge_move_table, move_prefix+"edge.bin");
            save_vector(multi_move_table, move_prefix+"multi.bin");
            save_vector(prune_table, prune_fn);
        }
        
        p_multi = multi_move_table.data();
    }

    bool search(SearchContext& ctx, int i1, int i2, int depth, int prev) {
        // 优化：使用静态数组 valid_moves_flat
        const int* moves = valid_moves_flat[prev];
        const int count = valid_moves_count[prev];

        for (int k = 0; k < count; ++k) {
            int m = moves[k];
            COUNT_NODE 
            
            int n_i1 = p_multi[i1 + m];
            int n_i2 = p_multi[i2 + m];
            long long idx = (long long)n_i1 * 528 + n_i2;
            
            // 优化：内联读取
            int prune_val = (prune_table[idx >> 1] >> ((idx & 1) << 2)) & 0xF;
            
            if (prune_val >= depth) continue;

            if (depth == 1) { ctx.sol_len.push_back(ctx.current_max_depth); return true; }
            if (search(ctx, n_i1 * 18, n_i2 * 18, depth - 1, m)) return true;
        }
        return false;
    }

    std::vector<int> get_stats(const std::vector<int>& base_alg, const std::vector<std::string>& rots) {
        std::vector<int> res(rots.size(), 0);
        for (size_t i = 0; i < rots.size(); ++i) {
            std::vector<int> alg = alg_rotation(base_alg, rots[i]);
            int i1 = 416, i2 = 520;
            for (int m : alg) { i1 = p_multi[i1 * 18 + m]; i2 = p_multi[i2 * 18 + m]; }
            long long idx = (long long)i1 * 528 + i2;
            int d_min = (prune_table[idx >> 1] >> ((idx & 1) << 2)) & 0xF;
            if (d_min == 0) continue;
            SearchContext ctx; 
            for (int d = d_min; d <= 8; ++d) {
                ctx.current_max_depth = d;
                if (search(ctx, i1 * 18, i2 * 18, d, 18)) { res[i] = d; break; }
            }
        }
        return res;
    }
};

// =========================================================
// 类：XCrossSolver
// =========================================================
struct XCrossSolver {
    std::vector<int> mt_edge, mt_corn, mt_multi;
    std::vector<std::vector<unsigned char>> prune_tables_base;
    const int *p_multi, *p_corn, *p_edge;
    std::string move_prefix;
    std::string prune_prefix;

    struct PseudoTask1 { int c_idx; int diff; int h; };
    struct PseudoTask2 { int c1, c2; int diff1, diff2; int h; };
    struct PseudoTask3 { int c1, c2, c3; int diff1, diff2, diff3; int h; };

    struct ConjState { int im; int ic_b; int ie_rel[4]; };

    XCrossSolver(bool pseudo) {
        move_prefix = "move_table_xcross_"; 
        prune_prefix = pseudo ? "prune_table_pseudo_base_" : "prune_table_std_base_";
        bool loaded = false;
        if (load_vector(mt_edge, move_prefix+"edge.bin") &&
            load_vector(mt_corn, move_prefix+"corner.bin") &&
            load_vector(mt_multi, move_prefix+"multi.bin")) loaded = true;

        if (!loaded) {
            std::cout << "[Init] Gen " << (pseudo?"Pseudo":"Std") << " Moves..." << std::endl;
            mt_edge = create_edge_move_table();
            mt_corn = create_corner_move_table();
            mt_multi = create_multi_move_table2(4, 2, 12, 24*22*20*18, mt_edge);
            save_vector(mt_edge, move_prefix+"edge.bin");
            save_vector(mt_corn, move_prefix+"corner.bin");
            save_vector(mt_multi, move_prefix+"multi.bin");
        }
        p_multi = mt_multi.data(); p_corn = mt_corn.data(); p_edge = mt_edge.data();

        prune_tables_base.resize(4);
        std::vector<int> e_diffs = {0, 2, 4, 6}; 
        std::cout << "[Init] Checking Base Prune Tables (C4 Base)..." << std::endl;
        for (int i = 0; i < 4; ++i) {
            std::string fn;
            if (pseudo) {
                fn = "prune_table_pseudo_xcross_C4_E" + std::to_string(i) + ".bin";
            } else {
                fn = prune_prefix + "diff_" + std::to_string(i) + ".bin";
            }

            if (!load_vector(prune_tables_base[i], fn)) {
                std::cout << "  Generating " << fn << " ..." << std::endl;
                prune_tables_base[i].resize(((long long)24*22*20*18 * 24 * 24 + 1) / 2, 0xFF);
                create_prune_table_xcross_full(187520, 12, e_diffs[i], 24*22*20*18, 24, 24, 11,
                                               mt_multi, mt_corn, mt_edge, prune_tables_base[i], pseudo);
                save_vector(prune_tables_base[i], fn);
            }
        }
    }

    inline int get_diff(int sc, int se) { return (se - sc + 4) % 4; }

    void get_conjugated_indices_all(const std::vector<int>& alg, int slot_k, ConjState& out) {
        int cur_mul = 187520 * 24, cur_cn = 12 * 18;      
        int cur_e[] = {0, 2, 4, 6};
        for (int m : alg) {
            // [优化 1] 使用扁平化共轭表 conj_moves_flat 替代 conj_moves[m][slot_k]
            int mc = conj_moves_flat[m][slot_k];
            cur_mul = p_multi[cur_mul + mc];
            cur_cn  = p_corn[cur_cn + mc] * 18;
            for(int k=0;k<4;++k) cur_e[k] = p_edge[cur_e[k] * 18 + mc];
        }
        out.im = cur_mul; out.ic_b = cur_cn / 18;
        for(int k=0;k<4;++k) out.ie_rel[k] = cur_e[k];
    }

    // [优化] Search 1: pseudo_xcross
    // 引入静态数组遍历 + 内联剪枝
    bool search_1(SearchContext& ctx, int i1, int i2, int i3, int depth, int prev, const unsigned char* p_prune) {
        const int* moves = valid_moves_flat[prev];
        const int count = valid_moves_count[prev];

        for (int k = 0; k < count; ++k) {
            int m = moves[k];
            COUNT_NODE 
            
            int n_i1 = p_multi[i1 + m];
            int n_i2 = p_corn[i2 + m];
            long long idx = (long long)(n_i1 + n_i2) * 24 + p_edge[i3 + m];
            
            if (get_prune_ptr(p_prune, idx) >= depth) continue;

            if (depth == 1) { ctx.sol_len.push_back(ctx.current_max_depth); return true; }
            if (search_1(ctx, n_i1, n_i2 * 18, p_edge[i3 + m] * 18, depth - 1, m, p_prune)) return true;
        }
        return false;
    }

    // [优化] Search 2: pseudo_xxcross
    bool search_2(SearchContext& ctx, 
                  int i1a, int i2a, int i3a, const unsigned char* p1,
                  int i1b, int i2b, int i3b, const int* tr_b, const unsigned char* p2,
                  int depth, int prev) {
        
        const int* moves = valid_moves_flat[prev];
        const int count = valid_moves_count[prev];

        for (int k = 0; k < count; ++k) {
            int m = moves[k];
            COUNT_NODE 
            
            // 阶段 A
            int n_i1a = p_multi[i1a + m];
            int n_i2a = p_corn[i2a + m];
            long long idx1 = (long long)(n_i1a + n_i2a) * 24 + p_edge[i3a + m];
            if (get_prune_ptr(p1, idx1) >= depth) continue;
            
            // 阶段 B
            int m_b = tr_b[m];
            int n_i1b = p_multi[i1b + m_b];
            int n_i2b = p_corn[i2b + m_b];
            long long idx2 = (long long)(n_i1b + n_i2b) * 24 + p_edge[i3b + m_b];
            if (get_prune_ptr(p2, idx2) >= depth) continue;

            if (depth == 1) { ctx.sol_len.push_back(ctx.current_max_depth); return true; }
            if (search_2(ctx, n_i1a, n_i2a * 18, p_edge[i3a + m] * 18, p1,
                              n_i1b, n_i2b * 18, p_edge[i3b + m_b] * 18, tr_b, p2, depth - 1, m)) return true;
        }
        return false;
    }

    // [优化] Search 3: pseudo_xxxcross
    bool search_3(SearchContext& ctx, 
                  int i1a, int i2a, int i3a, const unsigned char* p1,
                  int i1b, int i2b, int i3b, const int* tr_b, const unsigned char* p2,
                  int i1c, int i2c, int i3c, const int* tr_c, const unsigned char* p3,
                  int depth, int prev) {
        
        const int* moves = valid_moves_flat[prev];
        const int count = valid_moves_count[prev];

        for (int k = 0; k < count; ++k) {
            int m = moves[k];
            COUNT_NODE 
            
            // 阶段 A
            int n_i1a = p_multi[i1a + m];
            int n_i2a = p_corn[i2a + m];
            long long idx1 = (long long)(n_i1a + n_i2a) * 24 + p_edge[i3a + m];
            if (get_prune_ptr(p1, idx1) >= depth) continue;

            // 阶段 B
            int m_b = tr_b[m];
            int n_i1b = p_multi[i1b + m_b];
            int n_i2b = p_corn[i2b + m_b];
            long long idx2 = (long long)(n_i1b + n_i2b) * 24 + p_edge[i3b + m_b];
            if (get_prune_ptr(p2, idx2) >= depth) continue;

            // 阶段 C
            int m_c = tr_c[m];
            int n_i1c = p_multi[i1c + m_c];
            int n_i2c = p_corn[i2c + m_c];
            long long idx3 = (long long)(n_i1c + n_i2c) * 24 + p_edge[i3c + m_c];
            if (get_prune_ptr(p3, idx3) >= depth) continue;

            if (depth == 1) { ctx.sol_len.push_back(ctx.current_max_depth); return true; }
            if (search_3(ctx, n_i1a, n_i2a * 18, p_edge[i3a + m] * 18, p1,
                              n_i1b, n_i2b * 18, p_edge[i3b + m_b] * 18, tr_b, p2,
                              n_i1c, n_i2c * 18, p_edge[i3c + m_c] * 18, tr_c, p3, depth - 1, m)) return true;
        }
        return false;
    }

    std::vector<int> get_pseudo_stats(const std::vector<int>& base_alg, const std::vector<std::string>& rots, CrossSolver& ca) {
        std::vector<int> results; results.reserve(24);
        
        {
            auto cross_stats = ca.get_stats(base_alg, rots);
            results.insert(results.end(), cross_stats.begin(), cross_stats.end());
        }

        std::vector<std::vector<ConjState>> precomputed_states(rots.size(), std::vector<ConjState>(4));
        for(size_t r=0; r<rots.size(); ++r) {
            std::vector<int> alg = alg_rotation(base_alg, rots[r]);
            for(int k=0; k<4; ++k) get_conjugated_indices_all(alg, k, precomputed_states[r][k]);
        }
        auto get_h = [&](const ConjState& s, int diff_idx) {
             long long idx = (long long)(s.im + s.ic_b) * 24 + s.ie_rel[diff_idx];
             return (prune_tables_base[diff_idx][idx>>1]>>((idx&1)<<2))&0xF;
        };

        // pseudo_xcross
        {
            std::vector<int> stage_min(rots.size(), 99);
            for(size_t r=0; r<rots.size(); ++r) {
                std::vector<PseudoTask1> tasks;
                for (int c=0; c<4; ++c) {
                    for (int e=0; e<4; ++e) {
                        int diff = get_diff(c, e);
                        int h = get_h(precomputed_states[r][c], diff);
                        tasks.push_back({c, diff, h});
                    }
                }
                std::sort(tasks.begin(), tasks.end(), [](const PseudoTask1& a, const PseudoTask1& b){ return a.h < b.h; });

                int current_best = 99;
                for(auto& t : tasks) {
                    if (t.h >= current_best) break;
                    int res = 99;
                    if(t.h > 0) {
                        SearchContext ctx;
                        int max_search = std::min(16, current_best - 1);
                        auto& st = precomputed_states[r][t.c_idx];
                        for(int d=t.h; d<=max_search; ++d) { 
                            ctx.current_max_depth=d; 
                            if(search_1(ctx, st.im, st.ic_b*18, st.ie_rel[t.diff]*18, d, 18, 
                                        prune_tables_base[t.diff].data())) { res = d; break; }
                        }
                    } else { res = 0; }
                    if(res < current_best) current_best = res;
                }
                stage_min[r] = current_best;
            }
            results.insert(results.end(), stage_min.begin(), stage_min.end());
        }

        // pseudo_xxcross
        {
            std::vector<int> stage_min(rots.size(), 99);
            std::vector<std::pair<int,int>> pairs = {{0,1},{0,2},{0,3},{1,2},{1,3},{2,3}};
            for(size_t r=0; r<rots.size(); ++r) {
                std::vector<PseudoTask2> tasks;
                for (auto& cp : pairs) { 
                    for (auto& ep : pairs) { 
                        // Permutation 1: Identity
                        int d1 = get_diff(cp.first, ep.first);
                        int d2 = get_diff(cp.second, ep.second);
                        {
                            int h1 = get_h(precomputed_states[r][cp.first], d1);
                            int h2 = get_h(precomputed_states[r][cp.second], d2);
                            tasks.push_back({cp.first, cp.second, d1, d2, std::max(h1,h2)});
                        }
                        
                        // Permutation 2: Swap
                        int d1_s = get_diff(cp.first, ep.second);
                        int d2_s = get_diff(cp.second, ep.first);
                        {
                            int h1_s = get_h(precomputed_states[r][cp.first], d1_s);
                            int h2_s = get_h(precomputed_states[r][cp.second], d2_s);
                            tasks.push_back({cp.first, cp.second, d1_s, d2_s, std::max(h1_s, h2_s)});
                        }
                    }
                }
                std::sort(tasks.begin(), tasks.end(), [](const PseudoTask2& a, const PseudoTask2& b){ return a.h < b.h; });

                int current_best = 99;
                for(auto& t : tasks) {
                    if (t.h >= current_best) break;
                    int res = 99;
                    if(t.h > 0) {
                        SearchContext ctx;
                        int max_search = std::min(16, current_best - 1);
                        auto& st1 = precomputed_states[r][t.c1];
                        auto& st2 = precomputed_states[r][t.c2];
                        for(int d=t.h; d<=max_search; ++d) { 
                            ctx.current_max_depth=d; 
                            if(search_2(ctx, st1.im, st1.ic_b*18, st1.ie_rel[t.diff1]*18, prune_tables_base[t.diff1].data(),
                                             st2.im, st2.ic_b*18, st2.ie_rel[t.diff2]*18, trans_moves[t.c1][t.c2], prune_tables_base[t.diff2].data(),
                                             d, 18)) { res = d; break; }
                        }
                    } else { res = 0; }
                    if(res < current_best) current_best = res;
                }
                stage_min[r] = current_best;
            }
            results.insert(results.end(), stage_min.begin(), stage_min.end());
        }

        // pseudo_xxxcross
        {
            std::vector<int> stage_min(rots.size(), 99);
            std::vector<std::vector<int>> triples = {{0,1,2},{0,1,3},{0,2,3},{1,2,3}};
            for(size_t r=0; r<rots.size(); ++r) {
                std::vector<PseudoTask3> tasks;
                for (auto& ct : triples) {
                     for (auto& et : triples) {
                        std::vector<int> p = {0, 1, 2}; 
                        do {
                            int e0 = et[p[0]];
                            int e1 = et[p[1]];
                            int e2 = et[p[2]];
                            
                            int d1 = get_diff(ct[0], e0);
                            int d2 = get_diff(ct[1], e1);
                            
                            if (d1 == d2) {
                                int d3 = get_diff(ct[2], e2);
                                if (d2 == d3) {
                                    int h = std::max({
                                        get_h(precomputed_states[r][ct[0]], d1), 
                                        get_h(precomputed_states[r][ct[1]], d2), 
                                        get_h(precomputed_states[r][ct[2]], d3)
                                    });
                                    tasks.push_back({ct[0], ct[1], ct[2], d1, d2, d3, h});
                                }
                            }
                        } while(std::next_permutation(p.begin(), p.end()));
                     }
                }
                std::sort(tasks.begin(), tasks.end(), [](const PseudoTask3& a, const PseudoTask3& b){ return a.h < b.h; });

                int current_best = 99;
                for(auto& t : tasks) {
                    if (t.h >= current_best) break;
                    int res = 99;
                    if(t.h > 0) {
                        SearchContext ctx;
                        int max_search = std::min(16, current_best - 1);
                        auto& s1 = precomputed_states[r][t.c1];
                        auto& s2 = precomputed_states[r][t.c2];
                        auto& s3 = precomputed_states[r][t.c3];
                        for(int d=t.h; d<=max_search; ++d) { 
                            ctx.current_max_depth=d; 
                            if(search_3(ctx, s1.im, s1.ic_b*18, s1.ie_rel[t.diff1]*18, prune_tables_base[t.diff1].data(),
                                             s2.im, s2.ic_b*18, s2.ie_rel[t.diff2]*18, trans_moves[t.c1][t.c2], prune_tables_base[t.diff2].data(),
                                             s3.im, s3.ic_b*18, s3.ie_rel[t.diff3]*18, trans_moves[t.c1][t.c3], prune_tables_base[t.diff3].data(),
                                             d, 18)) { res = d; break; }
                        }
                    } else { res = 0; }
                    if(res < current_best) current_best = res;
                }
                stage_min[r] = current_best;
            }
            results.insert(results.end(), stage_min.begin(), stage_min.end());
        }

        return results;
    }
};

int main() {
	system("color 0A");
    init_matrix();
    std::cout << "Initializing Optimized Pseudo System (Flat Array + Hardcoded MoveID)..." << std::endl;
    
    CrossSolver ca_pseudo(true);
    XCrossSolver xcs_pseudo(true);

    std::vector<std::string> rots = {"", "z2", "z'", "z", "x'", "x"};

    while (true) {
        std::string in_fn;
        std::cout << "\nEnter file: ";
        if (!(std::cin >> in_fn)) break;

        std::vector<std::pair<std::string, std::vector<int>>> tasks;
        std::ifstream infile(in_fn);
        if(!infile) { std::cout << "File not found." << std::endl; continue; }
        std::string line;
        while (std::getline(infile, line)) {
            if (line.empty()) continue; // 跳过空行
            size_t p = line.find(',');
            if (p != std::string::npos) {
                // 格式 1：序号,打乱
                tasks.push_back({line.substr(0, p), string_to_alg(line.substr(p+1))});
            } else {
                // 格式 2：纯打乱（自动生成 1, 2, 3... 作为序号）
                tasks.push_back({std::to_string(tasks.size() + 1), string_to_alg(line)});
            }
        }
        infile.close();

        std::cout << "Loaded " << tasks.size() << " tasks." << std::endl;
        std::ofstream outfile(in_fn + ".csv");
        auto start = std::chrono::high_resolution_clock::now();
        int total = tasks.size();

        std::vector<std::string> result_buffer(total);
        std::vector<bool> result_ready(total, false);
        int next_write_idx = 0;

        global_nodes = 0;
        completed_tasks = 0;
        is_solving = true;

        std::thread monitor_thread([&]() {
            auto t0 = std::chrono::high_resolution_clock::now();
            while (is_solving) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                auto t1 = std::chrono::high_resolution_clock::now();
                double dt = std::chrono::duration<double>(t1 - t0).count();
                long long n = global_nodes.load(std::memory_order_relaxed);
                int c = completed_tasks.load(std::memory_order_relaxed);
                double nps = (dt > 0.001) ? n / dt : 0;
                printf("Progress: %d/%d (%.1f%%) | NPS: %.2f M/s      \r", 
                       c, total, (float)c*100.0/total, nps / 1000000.0);
                fflush(stdout);
            }
        });

        #pragma omp parallel for schedule(dynamic, 1)
        for (int i = 0; i < total; ++i) {
            std::vector<int> s_pseudo = xcs_pseudo.get_pseudo_stats(tasks[i].second, rots, ca_pseudo);
            std::stringstream ss;
            ss << tasks[i].first;
            for (int v : s_pseudo) ss << "," << v;
            ss << "\n";
            result_buffer[i] = ss.str();
            result_ready[i] = true;
            completed_tasks.fetch_add(1, std::memory_order_relaxed);

            #pragma omp critical
            {
                while (next_write_idx < total && result_ready[next_write_idx]) {
                    outfile << result_buffer[next_write_idx];
                    next_write_idx++;
                }
            }
        }
        
        is_solving = false;
        monitor_thread.join();

        auto end = std::chrono::high_resolution_clock::now();
        double total_dt = std::chrono::duration<double>(end-start).count();
        std::cout << "\nDone. Total Time: " << total_dt << "s" 
                  << " | Avg NPS: " << (global_nodes / total_dt / 1000000.0) << " M/s" << std::endl;
    }
    return 0;
}