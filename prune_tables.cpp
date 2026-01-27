/*
 * prune_tables.cpp - 剪枝表实现
 */

#include "prune_tables.h"
#include "move_tables.h"

PruneTableManager* PruneTableManager::instance = nullptr;

PruneTableManager& PruneTableManager::getInstance() {
    if (instance == nullptr) {
        instance = new PruneTableManager();
    }
    return *instance;
}

void PruneTableManager::initialize() {
    std::cout << "[PruneTable] Initializing prune tables..." << std::endl;
    
    generateCrossPrune();
    generateCrossC4Prune();
    generatePairC4E0Prune();
    generateXCrossC4E0Prune();
    generateHugeNeighborPrune();
    generateHugeDiagonalPrune();
    
    std::cout << "[PruneTable] All prune tables initialized." << std::endl;
}

bool PruneTableManager::loadTable(std::vector<unsigned char>& table, const std::string& filename) {
    return load_vector_chunked(table, filename);
}

void PruneTableManager::saveTable(const std::vector<unsigned char>& table, const std::string& filename) {
    save_vector_chunked(table, filename);
}

void PruneTableManager::generateCrossPrune() {
    if (loadTable(cross_prune, "prune_table_cross.bin")) {
        std::cout << "[PruneTable] Loaded cross prune table from file." << std::endl;
        return;
    }
    
    std::cout << "[PruneTable] Generating cross prune table..." << std::endl;
    auto& mtm = MoveTableManager::getInstance();
    const auto& edges_2_table = mtm.getEdges2Table();
    
    long long sz = 24LL*22*24*22; 
    std::vector<unsigned char> tmp(sz, 255); 
    int i1=416, i2=520; tmp[(long long)i1*528 + i2] = 0;
    
    for(int d=0;d<10;++d) {
        long long cnt = 0;
        #pragma omp parallel for reduction(+:cnt)
        for(long long i=0; i<sz; ++i) {
            if(tmp[i]==d) {
                cnt++; int idx1=(i/528)*18, idx2=(i%528)*18;
                for(int j=0; j<18; ++j) { 
                    long long ni = (long long)edges_2_table[idx1+j]*528 + edges_2_table[idx2+j]; 
                    if(tmp[ni]==255) tmp[ni]=d+1; 
                }
            }
        }
    }
    int c_sz = (sz+1)/2; 
    cross_prune.assign(c_sz, 0xFF); 
    for(long long i=0;i<sz;++i) set_prune(cross_prune, i, tmp[i]);
    
    saveTable(cross_prune, "prune_table_cross.bin");
}

void PruneTableManager::generateCrossC4Prune() {
    if (loadTable(cross_c4_prune, "prune_table_cross_c4.bin")) {
        std::cout << "[PruneTable] Loaded cross+c4 prune table from file." << std::endl;
        return;
    }
    
    std::cout << "[PruneTable] Generating cross+c4 prune table..." << std::endl;
    auto& mtm = MoveTableManager::getInstance();
    
    const int IDX_MULTI_BASE = 187520; // Cross 已还原
    const int IDX_C4 = 12; // 基准角块 (DBL)
    
    cross_c4_prune.resize((long long)24*22*20*18 * 24, 255);
    create_prune_table_cross_c4(IDX_MULTI_BASE, IDX_C4, 24*22*20*18, 24, 10, 
                                mtm.getCrossTable(), mtm.getCornerTable(), cross_c4_prune);
    saveTable(cross_c4_prune, "prune_table_cross_c4.bin");
}

void PruneTableManager::generatePairC4E0Prune() {
    if (loadTable(pair_c4_e0_prune, "prune_table_pair_c4_e0.bin")) {
        std::cout << "[PruneTable] Loaded pair c4+e0 prune table from file." << std::endl;
        return;
    }
    
    std::cout << "[PruneTable] Generating pair c4+e0 prune table..." << std::endl;
    auto& mtm = MoveTableManager::getInstance();
    
    const int IDX_E0 = 0;  // 基准棱块 (BL)
    const int IDX_C4 = 12; // 基准角块 (DBL)
    
    pair_c4_e0_prune.resize(24*24, 255);
    create_prune_table_pair_base(IDX_E0, IDX_C4, 24, 24, 8, 
                                mtm.getEdgeTable(), mtm.getCornerTable(), pair_c4_e0_prune);
    saveTable(pair_c4_e0_prune, "prune_table_pair_c4_e0.bin");
}

void PruneTableManager::generateXCrossC4E0Prune() {
    if (loadTable(xcross_c4_e0_prune, "prune_table_xcross_c4_e0.bin")) {
        std::cout << "[PruneTable] Loaded xcross c4+e0 prune table from file." << std::endl;
        return;
    }
    
    std::cout << "[PruneTable] Generating xcross c4+e0 prune table..." << std::endl;
    auto& mtm = MoveTableManager::getInstance();
    
    const int IDX_MULTI_BASE = 187520; // Cross 已还原
    const int IDX_C4 = 12; // 基准角块 (DBL)
    const int IDX_E0 = 0;  // 基准棱块 (BL)
    
    long long c_sz = ((long long)24*22*20*18 * 24 * 24 + 1) / 2;
    xcross_c4_e0_prune.resize(c_sz, 0xFF);
    create_prune_table_xcross_full(IDX_MULTI_BASE, IDX_C4, IDX_E0, 24*22*20*18, 24, 24, 11, 
                                  mtm.getCrossTable(), mtm.getCornerTable(), mtm.getEdgeTable(), xcross_c4_e0_prune);
    saveTable(xcross_c4_e0_prune, "prune_table_xcross_c4_e0.bin");
}

void PruneTableManager::generateHugeNeighborPrune() {
    if (loadTable(huge_neighbor_prune, "prune_table_huge_neighbor.bin")) {
        std::cout << "[PruneTable] Loaded huge neighbor prune table from file." << std::endl;
        return;
    }
    
    std::cout << "[PruneTable] Generating huge neighbor prune table..." << std::endl;
    auto& mtm = MoveTableManager::getInstance();
    
    create_prune_table_huge(42577920, 504, 15, {0, 2, 16, 18, 20, 22}, {12, 15}, 
                           mtm.getEdge6Table(), mtm.getCorner2Table(), huge_neighbor_prune);
    saveTable(huge_neighbor_prune, "prune_table_huge_neighbor.bin");
}

void PruneTableManager::generateHugeDiagonalPrune() {
    if (!ENABLE_DIAGONAL_TABLE) {
        std::cout << "[PruneTable] Diagonal table disabled." << std::endl;
        return;
    }
    
    if (loadTable(huge_diagonal_prune, "prune_table_huge_diagonal.bin")) {
        std::cout << "[PruneTable] Loaded huge diagonal prune table from file." << std::endl;
        return;
    }
    
    std::cout << "[PruneTable] Generating huge diagonal prune table..." << std::endl;
    auto& mtm = MoveTableManager::getInstance();
    
    create_prune_table_huge(42577920, 504, 15, {0, 4, 16, 18, 20, 22}, {12, 18}, 
                           mtm.getEdge6Table(), mtm.getCorner2Table(), huge_diagonal_prune);
    saveTable(huge_diagonal_prune, "prune_table_huge_diagonal.bin");
}

// 1. Cross + C4 (Base)
void create_prune_table_cross_c4(int idx1, int idx2, int sz1, int sz2, int depth,
                                 const std::vector<int>& t1, const std::vector<int>& t2,
                                 std::vector<unsigned char>& pt) {
    long long total = (long long)sz1 * sz2;
    std::fill(pt.begin(), pt.end(), 255);
    std::vector<std::string> am = {"L U L'", "L U' L'", "B' U B", "B' U' B"};
    pt[(long long)idx1 * sz2 + idx2] = 0;
    for(const auto& s : am) {
        int i1 = idx1*24, i2 = idx2;
        for(int m : string_to_alg(s)) { i1=t1[i1+m]; i2=t2[i2*18+m]; }
        pt[(long long)i1/24 * sz2 + i2] = 0; 
        int base1 = i1, base2 = i2*18;
        pt[t1[base1] + t2[base2]] = 0;
        pt[t1[base1+1] + t2[base2+1]] = 0;
        pt[t1[base1+2] + t2[base2+2]] = 0;
    }
    for(int d=0; d<depth; ++d) {
        int nd = d+1; long long cnt=0;
        #pragma omp parallel for reduction(+:cnt)
        for(long long i=0; i<total; ++i) {
            if(pt[i] == d) {
                cnt++;
                int i1 = (i / sz2) * 24;
                int i2 = (i % sz2) * 18;
                for(int j=0; j<18; ++j) {
                    long long ni = (long long)t1[i1+j] + t2[i2+j];
                    if(pt[ni] == 255) pt[ni] = nd;
                }
            }
        }
        std::cout << "  [Base Cross+C4] Depth " << d << ": " << cnt << std::endl;
        if(cnt==0) break;
    }
}

// 2. Pair C4 + E0 (Base)
void create_prune_table_pair_base(int idx_e, int idx_c, int sz_e, int sz_c, int depth,
                                  const std::vector<int>& t_edge, const std::vector<int>& t_corn,
                                  std::vector<unsigned char>& pt) {
    long long total = (long long)sz_e * sz_c;
    std::fill(pt.begin(), pt.end(), 255);
    std::vector<std::string> am = {"L U L'", "L U' L'", "B' U B", "B' U' B"};
    pt[idx_e * sz_c + idx_c] = 0;
    for(const auto& s : am) {
        int c1 = idx_e, c2 = idx_c;
        for(int m : string_to_alg(s)) { c1 = t_edge[c1*18 + m]; c2 = t_corn[c2*18 + m]; }
        pt[c1*sz_c + c2] = 0;
        for(int k=0; k<3; ++k) {
             int n1 = t_edge[c1*18 + k]; int n2 = t_corn[c2*18 + k];
             pt[n1*sz_c + n2] = 0;
        }
    }
    for(int d=0; d<depth; ++d) {
        int nd = d+1; long long cnt=0;
        #pragma omp parallel for reduction(+:cnt)
        for(long long i=0; i<total; ++i) {
            if(pt[i] == d) {
                cnt++;
                int i1 = (i / sz_c) * 18;
                int i2 = (i % sz_c) * 18;
                for(int j=0; j<18; ++j) {
                    long long ni = (long long)t_edge[i1+j] * sz_c + t_corn[i2+j];
                    if(pt[ni] == 255) pt[ni] = nd;
                }
            }
        }
        std::cout << "  [Base Pair C4+E0] Depth " << d << ": " << cnt << std::endl;
        if(cnt==0) break;
    }
}

// 3. XCross Base Generator
void create_prune_table_xcross_base(int idx_cr, int idx_cn, int idx_ex, 
                                    int sz_cr, int sz_cn, int sz_ex, int depth, 
                                    const std::vector<int> &t1, const std::vector<int> &t2, const std::vector<int> &t3,
                                    std::vector<unsigned char> &pt) {
    long long total = (long long)sz_cr * sz_cn * sz_ex;
    std::vector<unsigned char> tmp;
    try { tmp.resize(total, 255); } catch(...) { std::cerr << "Alloc fail XCross\n"; exit(1); }
    long long cur_cr = (long long)idx_cr * 24; 
    long long start_idx = (cur_cr + idx_cn) * 24 + idx_ex;
    if (start_idx < total) tmp[start_idx] = 0;
    
    for (int d=0; d<depth; ++d) {
        int nd = d+1; long long cnt=0;
        #pragma omp parallel for reduction(+:cnt)
        for (long long i=0; i<total; ++i) {
            if (tmp[i] == d) {
                cnt++;
                long long comb = i / sz_ex;
                int cur_ex = i % sz_ex;
                int cur_cr = (comb / sz_cn) * 24;
                int cur_cn = (comb % sz_cn) * 18;
                int idx3_base = cur_ex * 18; 
                for (int j=0; j<18; ++j) {
                    long long n_cr = t1[cur_cr + j];
                    int n_cn = t2[cur_cn + j];
                    long long ni = (n_cr + n_cn) * 24 + t3[idx3_base + j];
                    if (tmp[ni] == 255) tmp[ni] = nd;
                }
            }
        }
        std::cout << "  [XCross Gen] Depth " << d << ": " << cnt << std::endl;
        if (cnt==0) break;
    }
    pt.assign((total + 1) / 2, 0xFF);
    #pragma omp parallel for
    for (long long i=0; i<total; ++i) if (tmp[i]!=255) set_prune(pt, i, tmp[i]);
}

void create_prune_table_xcross_full(int idx_cr, int idx_cn, int idx_ed, int sz_cr, int sz_cn, int sz_ed, int depth, 
                                    const std::vector<int> &t1, const std::vector<int> &t2, const std::vector<int> &t3, 
                                    std::vector<unsigned char> &pt) {
    long long total = (long long)sz_cr * sz_cn * sz_ed;
    std::vector<unsigned char> tmp; 
    try { tmp.resize(total, 255); } catch(...) { std::cerr << "Alloc fail\n"; exit(1); }
    long long cur_cr = idx_cr * 24LL; int cur_cn = idx_cn * 18; int cur_ed = idx_ed;            
    long long start_idx = (cur_cr + cur_cn/18) * 24 + cur_ed; 
    if (start_idx < total) tmp[start_idx] = 0;
    
    for (int d=0; d<depth; ++d) {
        int nd = d+1; long long cnt=0;
        #pragma omp parallel for reduction(+:cnt)
        for (long long i=0; i<total; ++i) {
            if (tmp[i] == d) {
                cnt++;
                long long comb = i / sz_ed; int cur_ed = i % sz_ed; 
                int cur_cr = (comb / sz_cn) * 24; int cur_cn = (comb % sz_cn) * 18; 
                int idx3_base = cur_ed * 18;
                for (int j=0; j<18; ++j) {
                    long long n_cr = t1[cur_cr + j]; int n_cn = t2[cur_cn + j];
                    long long ni = (n_cr + n_cn) * 24 + t3[idx3_base + j];
                    if (tmp[ni] == 255) tmp[ni] = nd;
                }
            }
        }
        std::cout << "  [Gen XCross] Depth " << d << ": " << cnt << std::endl;
        if (cnt==0) break;
    }
    std::fill(pt.begin(), pt.end(), 0xFF);
    #pragma omp parallel for
    for (long long i=0; i<total; ++i) if (tmp[i]!=255) set_prune(pt, i, tmp[i]);
}

void create_prune_table_huge(int sz_e6, int sz_c2, int depth, const std::vector<int>& target_e_ids, 
                            const std::vector<int>& target_c_ids, const std::vector<int>& mt_e6, 
                            const std::vector<int>& mt_c2, std::vector<unsigned char>& pt) {
    long long total = (long long)sz_e6 * sz_c2;
    std::cout << "  Allocating " << (total/2/1024/1024) << " MB for Huge Table..." << std::endl;
    std::vector<unsigned char> tmp; 
    try { tmp.resize(total, 255); } catch(...) { std::cerr << "Alloc fail (Huge). Need ~20GB RAM.\n"; exit(1); }
    int idx_e6_solved = array_to_index(target_e_ids, 6, 2, 12);
    int idx_c2_solved = array_to_index(target_c_ids, 2, 3, 8);
    long long start_idx = (long long)idx_e6_solved * sz_c2 + idx_c2_solved; 
    if (start_idx < total) tmp[start_idx] = 0;
    
    for (int d=0; d<depth; ++d) {
        int nd = d+1; long long cnt=0;
        #pragma omp parallel for reduction(+:cnt)
        for (long long i=0; i<total; ++i) {
            if (tmp[i] == d) {
                cnt++;
                int cur_c2 = i % sz_c2; int cur_e6 = i / sz_c2;
                int base_e6 = cur_e6 * 18; int base_c2 = cur_c2 * 18;
                for (int j=0; j<18; ++j) {
                    int n_e6 = mt_e6[base_e6 + j]; int n_c2 = mt_c2[base_c2 + j];
                    long long ni = (long long)n_e6 * sz_c2 + n_c2;
                    if (tmp[ni] == 255) tmp[ni] = nd;
                }
            }
        }
        std::cout << "  [Gen Huge] Depth " << d << ": " << cnt << std::endl;
        if (cnt==0) break;
    }
    long long c_sz = (total + 1) / 2; pt.assign(c_sz, 0xFF);
    #pragma omp parallel for
    for (long long i=0; i<total; ++i) if (tmp[i]!=255) set_prune(pt, i, tmp[i]);
    std::vector<unsigned char>().swap(tmp); 
}