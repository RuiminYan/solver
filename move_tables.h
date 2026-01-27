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
    
    // 单例模式
    static MoveTableManager* instance;
    MoveTableManager() = default;

public:
    static MoveTableManager& getInstance();
    
    // 初始化所有移动表
    void initialize();
    
    // 获取移动表的只读访问
    const std::vector<int>& getEdgeTable() const { return edge_table; }
    const std::vector<int>& getCornerTable() const { return corner_table; }
    const std::vector<int>& getCrossTable() const { return cross_table; }
    const std::vector<int>& getEdges2Table() const { return edges_2_table; }
    const std::vector<int>& getEdge6Table() const { return edge6_table; }
    const std::vector<int>& getCorner2Table() const { return corner2_table; }
    
    // 获取指针（用于性能关键的代码）
    const int* getEdgeTablePtr() const { return edge_table.data(); }
    const int* getCornerTablePtr() const { return corner_table.data(); }
    const int* getCrossTablePtr() const { return cross_table.data(); }
    const int* getEdges2TablePtr() const { return edges_2_table.data(); }
    const int* getEdge6TablePtr() const { return edge6_table.data(); }
    const int* getCorner2TablePtr() const { return corner2_table.data(); }

private:
    // 生成函数
    void generateEdgeTable();
    void generateCornerTable();
    void generateCrossTable();
    void generateEdges2Table();
    void generateEdge6Table();
    void generateCorner2Table();
    
    // 文件操作
    bool loadTable(std::vector<int>& table, const std::string& filename);
    void saveTable(const std::vector<int>& table, const std::string& filename);
};

// --- 基础移动表生成函数 ---
std::vector<int> create_edge_move_table();
std::vector<int> create_corner_move_table();

#endif // MOVE_TABLES_H