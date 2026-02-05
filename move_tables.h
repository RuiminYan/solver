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
  std::vector<int> cross_table;   // 4个棱块的Cross表
  std::vector<int> edges_2_table; // 2个棱块组合表
  std::vector<int> edge6_table;   // 6个棱块组合表 (E0+E1)
  std::vector<int> corner2_table; // 2个角块组合表 (C4+C5)
  std::vector<int> corner3_table; // 3个角块组合表 (C4+C5+C6)
  std::vector<int> edge3_table;   // 3个棱块组合表 (E0+E1+E2)

  // EOCross 专用移动表
  std::vector<int> eo_cross_ep4_mt; // EP4移动表 (move_table_ep_4.bin)
  std::vector<int>
      eo_cross_eo_alt_mt;  // EO Alt移动表 (move_table_eo_12_alt.bin)
  std::vector<int> eo_mt;  // EO移动表 (move_table_eo_12.bin)
  std::vector<int> ep1_mt; // EP1移动表 (move_table_ep_1.bin)

  // 单例模式
  static MoveTableManager *instance;
  MoveTableManager() = default;

public:
  static MoveTableManager &getInstance();

  // 初始化所有移动表
  void initialize();

  // 尝试加载所有移动表（如果不存则返回 false）
  bool loadAll();

  // 顺序生成所有表（生成后释放，仅保留文件，用于节省内存）
  void generateAllSequentially();

  // 细粒度资源管理（供 PruneTableManager 生成时使用）
  // NOTE: 小表添加"已加载"检查，避免重复从磁盘加载
  bool loadEdgeTable() {
    if (!edge_table.empty())
      return true;
    return loadTable(edge_table, "move_table_edge.bin");
  }
  void releaseEdgeTable() { std::vector<int>().swap(edge_table); }

  bool loadCornerTable() {
    if (!corner_table.empty())
      return true;
    return loadTable(corner_table, "move_table_corner.bin");
  }
  void releaseCornerTable() { std::vector<int>().swap(corner_table); }

  bool loadCrossTable() {
    if (!cross_table.empty())
      return true;
    return loadTable(cross_table, "move_table_cross.bin");
  }
  void releaseCrossTable() { std::vector<int>().swap(cross_table); }

  bool loadEdges2Table() {
    if (!edges_2_table.empty())
      return true;
    return loadTable(edges_2_table, "move_table_edges_2.bin");
  }
  void releaseEdges2Table() { std::vector<int>().swap(edges_2_table); }

  bool loadEdge3Table() {
    if (!edge3_table.empty())
      return true;
    return loadTable(edge3_table, "move_table_edges_3.bin");
  }
  void releaseEdge3Table() { std::vector<int>().swap(edge3_table); }

  // NOTE: Edge6 保持原有行为，按需加载+用完释放，因为它占用 ~3GB 内存
  bool loadEdge6Table() {
    return loadTable(edge6_table, "move_table_edges_6.bin");
  }
  void releaseEdge6Table() { std::vector<int>().swap(edge6_table); }

  bool loadCorner2Table() {
    if (!corner2_table.empty())
      return true;
    return loadTable(corner2_table, "move_table_corners_2.bin");
  }
  void releaseCorner2Table() { std::vector<int>().swap(corner2_table); }

  bool loadCorner3Table() {
    if (!corner3_table.empty())
      return true;
    return loadTable(corner3_table, "move_table_corners_3.bin");
  }
  void releaseCorner3Table() { std::vector<int>().swap(corner3_table); }

  // 加载 EOCross 专用移动表
  bool loadEOCrossMoveTables();
  bool loadEOTable();  // 加载 EO 移动表 (move_table_eo_12.bin)
  bool loadEP1Table(); // 加载 EP1 移动表 (move_table_ep_1.bin)

  // 获取移动表的只读访问
  const std::vector<int> &getEdgeTable() const { return edge_table; }
  const std::vector<int> &getCornerTable() const { return corner_table; }
  const std::vector<int> &getCrossTable() const { return cross_table; }
  const std::vector<int> &getEdges2Table() const { return edges_2_table; }
  const std::vector<int> &getEdge3Table() const { return edge3_table; }
  const std::vector<int> &getEdge6Table() const { return edge6_table; }
  const std::vector<int> &getCorner2Table() const { return corner2_table; }
  const std::vector<int> &getCorner3Table() const { return corner3_table; }

  // 获取指针（用于性能关键的代码）
  const int *getEdgeTablePtr() const { return edge_table.data(); }
  const int *getCornerTablePtr() const { return corner_table.data(); }
  const int *getCrossTablePtr() const { return cross_table.data(); }
  const int *getEdges2TablePtr() const { return edges_2_table.data(); }
  const int *getEdge3TablePtr() const { return edge3_table.data(); }
  const int *getEdge6TablePtr() const { return edge6_table.data(); }
  const int *getCorner2TablePtr() const { return corner2_table.data(); }
  const int *getCorner3TablePtr() const { return corner3_table.data(); }

  // EOCross 专用移动表 Getter
  const int *getEOCrossEP4Ptr() const { return eo_cross_ep4_mt.data(); }
  const int *getEOCrossEOAltPtr() const { return eo_cross_eo_alt_mt.data(); }
  const int *getEOTablePtr() const { return eo_mt.data(); }
  const int *getEP1TablePtr() const { return ep1_mt.data(); }
  // EOCross 专用移动表 Vector Getter (用于剪枝表生成)
  const std::vector<int> &getEOCrossEP4Table() const { return eo_cross_ep4_mt; }
  const std::vector<int> &getEOCrossEOAltTable() const {
    return eo_cross_eo_alt_mt;
  }

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
  bool loadTable(std::vector<int> &table, const std::string &filename);
  void saveTable(const std::vector<int> &table, const std::string &filename);
};

// --- 基础移动表生成函数 ---
std::vector<int> create_edge_move_table();
std::vector<int> create_corner_move_table();
std::vector<int> create_ep_move_table();
std::vector<int> create_eo_move_table();
std::vector<int> create_eo_move_table2();

#endif // MOVE_TABLES_H