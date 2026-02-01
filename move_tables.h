/*
 * move_tables.h - 移动表管理
 */

#ifndef MOVE_TABLES_H
#define MOVE_TABLES_H

#include "cube_common.h"

// --- 移动表管理器 ---
class MoveTableManager {
private:
    // 基础移动表
    std::vector<int> edge_table;
    std::vector<int> corner_table;
    
    // 复合移动表
    std::vector<int> cross_table;        // 4个棱块的Cross表
    std::vector<int> edges_2_table;      // 2个棱块组合表
    std::vector<int> edge6_table;        // 6个棱块组合表 (E0+E1)
    std::vector<int> corner2_table;      // 2个角块组合表 (C4+C5)
    std::vector<int> corner3_table;      // 3个角块组合表 (C4+C5+C6)
    std::vector<int> edge3_table;        // 3个棱块组合表 (E0+E1+E2)
    
    // 单例模式
    static MoveTableManager* instance;
    MoveTableManager() = default;

public:
    static MoveTableManager& getInstance();
    
    // 初始化所有移动表
    void initialize();

    // 尝试加载所有移动表（如果不存则返回 false）
    bool loadAll();
    
    // 顺序生成所有表（生成后释放，仅保留文件，用于节省内存）
    void generateAllSequentially();

    // 细粒度资源管理（供 PruneTableManager 生成时使用）
    bool loadEdgeTable() { return loadTable(edge_table, "move_table_edge.bin"); }
    void releaseEdgeTable() { std::vector<int>().swap(edge_table); }
    
    bool loadCornerTable() { return loadTable(corner_table, "move_table_corner.bin"); }
    void releaseCornerTable() { std::vector<int>().swap(corner_table); }
    
    bool loadCrossTable() { return loadTable(cross_table, "move_table_cross.bin"); }
    void releaseCrossTable() { std::vector<int>().swap(cross_table); }
    
    bool loadEdges2Table() { return loadTable(edges_2_table, "move_table_edges_2.bin"); }
    void releaseEdges2Table() { std::vector<int>().swap(edges_2_table); }

    bool loadEdge3Table() { return loadTable(edge3_table, "move_table_edges_3.bin"); }
    void releaseEdge3Table() { std::vector<int>().swap(edge3_table); }
    
    bool loadEdge6Table() { return loadTable(edge6_table, "move_table_edges_6.bin"); }
    void releaseEdge6Table() { std::vector<int>().swap(edge6_table); }
    
    bool loadCorner2Table() { return loadTable(corner2_table, "move_table_corners_2.bin"); }
    void releaseCorner2Table() { std::vector<int>().swap(corner2_table); }

    bool loadCorner3Table() { return loadTable(corner3_table, "move_table_corners_3.bin"); }
    void releaseCorner3Table() { std::vector<int>().swap(corner3_table); }

    // 获取移动表的只读访问
    const std::vector<int>& getEdgeTable() const { return edge_table; }
    const std::vector<int>& getCornerTable() const { return corner_table; }
    const std::vector<int>& getCrossTable() const { return cross_table; }
    const std::vector<int>& getEdges2Table() const { return edges_2_table; }
    const std::vector<int>& getEdge3Table() const { return edge3_table; }
    const std::vector<int>& getEdge6Table() const { return edge6_table; }
    const std::vector<int>& getCorner2Table() const { return corner2_table; }
    const std::vector<int>& getCorner3Table() const { return corner3_table; }
    
    // 获取指针（用于性能关键的代码）
    const int* getEdgeTablePtr() const { return edge_table.data(); }
    const int* getCornerTablePtr() const { return corner_table.data(); }
    const int* getCrossTablePtr() const { return cross_table.data(); }
    const int* getEdges2TablePtr() const { return edges_2_table.data(); }
    const int* getEdge3TablePtr() const { return edge3_table.data(); }
    const int* getEdge6TablePtr() const { return edge6_table.data(); }
    const int* getCorner2TablePtr() const { return corner2_table.data(); }
    const int* getCorner3TablePtr() const { return corner3_table.data(); }

private:
    // 生成函数
    void generateEdgeTable();
    void generateCornerTable();
    void generateCrossTable();
    void generateEdges2Table();
    void generateEdge3Table();
    void generateEdge6Table();
    void generateCorner2Table();
    void generateCorner3Table();
    
    // 文件操作
    bool loadTable(std::vector<int>& table, const std::string& filename);
    void saveTable(const std::vector<int>& table, const std::string& filename);
};

// --- 基础移动表生成函数 ---
std::vector<int> create_edge_move_table();
std::vector<int> create_corner_move_table();
std::vector<int> create_ep_move_table();
std::vector<int> create_eo_move_table();
std::vector<int> create_eo_move_table2();

#endif // MOVE_TABLES_H