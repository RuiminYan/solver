/*
 * prune_tables.h - 剪枝表相关功能
 */

#ifndef PRUNE_TABLES_H
#define PRUNE_TABLES_H

#include "cube_common.h"

// --- 剪枝表操作 ---
inline void set_prune(std::vector<unsigned char> &table, long long index, int value) {
    int shift = (index & 1) << 2;
    table[index >> 1] &= ~(0xF << shift);
    table[index >> 1] |= (value & 0xF) << shift;
}

inline int get_prune_4bit(const unsigned char* table, long long index) {
    return (table[index >> 1] >> ((index & 1) << 2)) & 0xF;
}

inline int get_prune(const std::vector<unsigned char>& table, long long index) {
    return (table[index >> 1] >> ((index & 1) << 2)) & 0xF;
}

inline int get_prune_ptr(const unsigned char* table, long long index) {
    return (table[index >> 1] >> ((index & 1) << 2)) & 0xF;
}

// --- 文件读写辅助 ---
template <typename T>
bool load_vector_chunked(std::vector<T>& vec, const std::string& filename) {
    std::ifstream in(filename, std::ios::binary); 
    if (!in) return false;
    in.seekg(0, std::ios::end); size_t file_size = in.tellg(); in.seekg(0, std::ios::beg);
    size_t size; in.read(reinterpret_cast<char*>(&size), sizeof(size));
    if (!in) return false;
    size_t expected = size * sizeof(T); 
    if (file_size != expected + sizeof(size_t)) return false;
    try { vec.resize(size); } catch(...) { return false; }
    char* ptr = reinterpret_cast<char*>(vec.data()); size_t remain = expected;
    while(remain > 0) { 
        size_t to_read = std::min(remain, (size_t)64*1024*1024); // 64MB chunks
        in.read(ptr, to_read); 
        if(!in) return false; 
        ptr += to_read; remain -= to_read; 
    }
    return true;
}

template <typename T>
bool load_vector(std::vector<T>& vec, const std::string& filename) {
    return load_vector_chunked(vec, filename);
}

template <typename T>
bool save_vector(const std::vector<T>& vec, const std::string& filename) {
    std::ofstream out(filename, std::ios::binary);
    size_t size = vec.size();
    out.write(reinterpret_cast<const char*>(&size), sizeof(size));
    out.write(reinterpret_cast<const char*>(vec.data()), size * sizeof(T));
    return out.good();
}

template <typename T>
bool save_vector_chunked(const std::vector<T>& vec, const std::string& filename) {
    std::ofstream out(filename, std::ios::binary); 
    size_t size = vec.size();
    out.write(reinterpret_cast<const char*>(&size), sizeof(size));
    const char* ptr = reinterpret_cast<const char*>(vec.data()); 
    size_t remain = size * sizeof(T);
    while(remain > 0) { 
        size_t to_write = std::min(remain, (size_t)64*1024*1024); 
        out.write(ptr, to_write); 
        ptr += to_write; remain -= to_write; 
    }
    return out.good();
}

// --- 剪枝表管理器 ---
class PruneTableManager {
private:
    // Cross相关剪枝表
    std::vector<unsigned char> cross_prune;           // Cross基础剪枝表
    std::vector<unsigned char> cross_c4_prune;        // Cross + C4剪枝表
    std::vector<unsigned char> pair_c4_e0_prune;      // Pair C4+E0剪枝表
    std::vector<unsigned char> xcross_c4_e0_prune;    // XCross C4+E0剪枝表
    
    // 巨型剪枝表
    std::vector<unsigned char> huge_neighbor_prune;   // 相邻槽剪枝表
    std::vector<unsigned char> huge_diagonal_prune;   // 对角槽剪枝表
    
    // 单例模式
    static PruneTableManager* instance;
    PruneTableManager() = default;

public:
    static PruneTableManager& getInstance();
    
    // 初始化所有剪枝表
    void initialize();
    
    // 获取剪枝表的只读访问
    const std::vector<unsigned char>& getCrossPrune() const { return cross_prune; }
    const std::vector<unsigned char>& getCrossC4Prune() const { return cross_c4_prune; }
    const std::vector<unsigned char>& getPairC4E0Prune() const { return pair_c4_e0_prune; }
    const std::vector<unsigned char>& getXCrossC4E0Prune() const { return xcross_c4_e0_prune; }
    const std::vector<unsigned char>& getHugeNeighborPrune() const { return huge_neighbor_prune; }
    const std::vector<unsigned char>& getHugeDiagonalPrune() const { return huge_diagonal_prune; }
    
    // 获取指针（用于性能关键的代码）
    const unsigned char* getCrossPrunePtr() const { return cross_prune.data(); }
    const unsigned char* getCrossC4PrunePtr() const { return cross_c4_prune.data(); }
    const unsigned char* getPairC4E0PrunePtr() const { return pair_c4_e0_prune.data(); }
    const unsigned char* getXCrossC4E0PrunePtr() const { return xcross_c4_e0_prune.data(); }
    const unsigned char* getHugeNeighborPrunePtr() const { return huge_neighbor_prune.data(); }
    const unsigned char* getHugeDiagonalPrunePtr() const { return huge_diagonal_prune.data(); }

private:
    // 生成函数
    void generateCrossPrune();
    void generateCrossC4Prune();
    void generatePairC4E0Prune();
    void generateXCrossC4E0Prune();
    void generateHugeNeighborPrune();
    void generateHugeDiagonalPrune();
    
    // 文件操作
    bool loadTable(std::vector<unsigned char>& table, const std::string& filename);
    void saveTable(const std::vector<unsigned char>& table, const std::string& filename);
};

// --- 剪枝表生成函数 ---
void create_prune_table_cross_c4(int idx1, int idx2, int sz1, int sz2, int depth,
                                 const std::vector<int>& t1, const std::vector<int>& t2,
                                 std::vector<unsigned char>& pt);

void create_prune_table_pair_base(int idx_e, int idx_c, int sz_e, int sz_c, int depth,
                                  const std::vector<int>& t_edge, const std::vector<int>& t_corn,
                                  std::vector<unsigned char>& pt);

void create_prune_table_xcross_base(int idx_cr, int idx_cn, int idx_ex, 
                                    int sz_cr, int sz_cn, int sz_ex, int depth, 
                                    const std::vector<int> &t1, const std::vector<int> &t2, const std::vector<int> &t3,
                                    std::vector<unsigned char> &pt);

void create_prune_table_xcross_full(int idx_cr, int idx_cn, int idx_ed, int sz_cr, int sz_cn, int sz_ed, int depth, 
                                    const std::vector<int> &t1, const std::vector<int> &t2, const std::vector<int> &t3, 
                                    std::vector<unsigned char> &pt);

void create_prune_table_huge(int sz_e6, int sz_c2, int depth, const std::vector<int>& target_e_ids, 
                            const std::vector<int>& target_c_ids, const std::vector<int>& mt_e6, 
                            const std::vector<int>& mt_c2, std::vector<unsigned char>& pt);

#endif // PRUNE_TABLES_H