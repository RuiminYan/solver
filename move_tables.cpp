/*
 * move_tables.cpp - 移动表实现
 */

#include "move_tables.h"

MoveTableManager* MoveTableManager::instance = nullptr;

MoveTableManager& MoveTableManager::getInstance() {
    if (instance == nullptr) {
        instance = new MoveTableManager();
    }
    return *instance;
}

void MoveTableManager::initialize() {
    std::cout << "[MoveTable] Initializing move tables..." << std::endl;
    
    generateEdgeTable();
    generateCornerTable();
    generateCrossTable();
    generateEdges2Table();
    generateEdge6Table();
    generateCorner2Table();
    
    std::cout << "[MoveTable] All move tables initialized." << std::endl;
}

bool MoveTableManager::loadTable(std::vector<int>& table, const std::string& filename) {
    return load_vector_chunked(table, filename);
}

void MoveTableManager::saveTable(const std::vector<int>& table, const std::string& filename) {
    save_vector_chunked(table, filename);
}

void MoveTableManager::generateEdgeTable() {
    if (loadTable(edge_table, "move_table_edge.bin")) {
        std::cout << "[MoveTable] Loaded edge table from file." << std::endl;
        return;
    }
    
    std::cout << "[MoveTable] Generating edge table..." << std::endl;
    edge_table = create_edge_move_table();
    saveTable(edge_table, "move_table_edge.bin");
}

void MoveTableManager::generateCornerTable() {
    if (loadTable(corner_table, "move_table_corner.bin")) {
        std::cout << "[MoveTable] Loaded corner table from file." << std::endl;
        return;
    }
    
    std::cout << "[MoveTable] Generating corner table..." << std::endl;
    corner_table = create_corner_move_table();
    saveTable(corner_table, "move_table_corner.bin");
}

void MoveTableManager::generateCrossTable() {
    if (loadTable(cross_table, "move_table_cross.bin")) {
        std::cout << "[MoveTable] Loaded cross table from file." << std::endl;
        return;
    }
    
    std::cout << "[MoveTable] Generating cross table..." << std::endl;
    // Cross表：4个棱块的组合 (24*22*20*18 states)
    cross_table = create_multi_move_table2(4, 2, 12, 24*22*20*18, edge_table);
    saveTable(cross_table, "move_table_cross.bin");
}

void MoveTableManager::generateEdges2Table() {
    if (loadTable(edges_2_table, "move_table_edges_2.bin")) {
        std::cout << "[MoveTable] Loaded edges_2 table from file." << std::endl;
        return;
    }
    
    std::cout << "[MoveTable] Generating edges_2 table..." << std::endl;
    // 2个棱块的组合 (24*22 states)
    edges_2_table = create_multi_move_table(2, 2, 12, 24 * 22, edge_table);
    saveTable(edges_2_table, "move_table_edges_2.bin");
}

void MoveTableManager::generateEdge6Table() {
    if (loadTable(edge6_table, "move_table_edges_6.bin")) {
        std::cout << "[MoveTable] Loaded edge6 table from file." << std::endl;
        return;
    }
    
    std::cout << "[MoveTable] Generating edge6 table..." << std::endl;
    // 6个棱块的组合 (用于巨型剪枝表)
    edge6_table = create_multi_move_table(6, 2, 12, 42577920, edge_table);
    saveTable(edge6_table, "move_table_edges_6.bin");
}

void MoveTableManager::generateCorner2Table() {
    if (loadTable(corner2_table, "move_table_corners_2.bin")) {
        std::cout << "[MoveTable] Loaded corner2 table from file." << std::endl;
        return;
    }
    
    std::cout << "[MoveTable] Generating corner2 table..." << std::endl;
    // 2个角块的组合 (504 states)
    corner2_table = create_multi_move_table(2, 3, 8, 504, corner_table);
    saveTable(corner2_table, "move_table_corners_2.bin");
}

// --- 基础移动表生成函数 ---
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