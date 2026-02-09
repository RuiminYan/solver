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
  std::vector<int> mt_edge;
  std::vector<int> mt_corn;

  // 复合移动表
  std::vector<int> mt_cross; // 4个棱块的Cross表
  std::vector<int> mt_edge2; // 2个棱块组合表
  std::vector<int> mt_edge6; // 6个棱块组合表 (E0+E1)
  std::vector<int> mt_corn2; // 2个角块组合表 (C4+C5)
  std::vector<int> mt_corn3; // 3个角块组合表 (C4+C5+C6)
  std::vector<int> mt_edge3; // 3个棱块组合表 (E0+E1+E2)

  // EOCross 专用移动表
  std::vector<int> mt_ep4;    // EP4移动表 (mt_ep4.bin)
  std::vector<int> mt_eo_alt; // EO Alt移动表 (mt_eo_alt.bin)
  std::vector<int> mt_eo;     // EO移动表 (mt_eo.bin)
  std::vector<int> mt_ep1;    // EP1移动表 (mt_ep1.bin)

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
  bool loadEdgeMT() {
    if (!mt_edge.empty())
      return true;
    return loadTable(mt_edge, "move_table_edge.bin");
  }
  void releaseEdgeMT() { std::vector<int>().swap(mt_edge); }

  bool loadCornMT() {
    if (!mt_corn.empty())
      return true;
    return loadTable(mt_corn, "move_table_corner.bin");
  }
  void releaseCornMT() { std::vector<int>().swap(mt_corn); }

  bool loadCrossMT() {
    if (!mt_cross.empty())
      return true;
    return loadTable(mt_cross, "move_table_cross.bin");
  }
  void releaseCrossMT() { std::vector<int>().swap(mt_cross); }

  bool loadEdge2MT() {
    if (!mt_edge2.empty())
      return true;
    return loadTable(mt_edge2, "move_table_edges_2.bin");
  }
  void releaseEdge2MT() { std::vector<int>().swap(mt_edge2); }

  bool loadEdge3MT() {
    if (!mt_edge3.empty())
      return true;
    return loadTable(mt_edge3, "move_table_edges_3.bin");
  }
  void releaseEdge3MT() { std::vector<int>().swap(mt_edge3); }

  // NOTE: Edge6 保持原有行为，按需加载+用完释放，因为它占用 ~3GB 内存
  bool loadEdge6MT() { return loadTable(mt_edge6, "move_table_edges_6.bin"); }
  void releaseEdge6MT() { std::vector<int>().swap(mt_edge6); }

  bool loadCorn2MT() {
    if (!mt_corn2.empty())
      return true;
    return loadTable(mt_corn2, "move_table_corners_2.bin");
  }
  void releaseCorn2MT() { std::vector<int>().swap(mt_corn2); }

  bool loadCorn3MT() {
    if (!mt_corn3.empty())
      return true;
    return loadTable(mt_corn3, "move_table_corners_3.bin");
  }
  void releaseCorn3MT() { std::vector<int>().swap(mt_corn3); }

  // 加载 EOCross 专用移动表
  bool loadEOCrossMTs();
  bool loadEOMT();  // 加载 EO 移动表 (mt_eo.bin)
  bool loadEP1MT(); // 加载 EP1 移动表 (mt_ep1.bin)

  // 获取移动表的只读访问
  const std::vector<int> &getEdgeMT() const { return mt_edge; }
  const std::vector<int> &getCornMT() const { return mt_corn; }
  const std::vector<int> &getCrossMT() const { return mt_cross; }
  const std::vector<int> &getEdge2MT() const { return mt_edge2; }
  const std::vector<int> &getEdge3MT() const { return mt_edge3; }
  const std::vector<int> &getEdge6MT() const { return mt_edge6; }
  const std::vector<int> &getCorn2MT() const { return mt_corn2; }
  const std::vector<int> &getCorn3MT() const { return mt_corn3; }

  // 获取指针（用于性能关键的代码）
  const int *getEdgeMTPtr() const { return mt_edge.data(); }
  const int *getCornMTPtr() const { return mt_corn.data(); }
  const int *getCrossMTPtr() const { return mt_cross.data(); }
  const int *getEdge2MTPtr() const { return mt_edge2.data(); }
  const int *getEdge3MTPtr() const { return mt_edge3.data(); }
  const int *getEdge6MTPtr() const { return mt_edge6.data(); }
  const int *getCorn2MTPtr() const { return mt_corn2.data(); }
  const int *getCorn3MTPtr() const { return mt_corn3.data(); }

  // EOCross 专用移动表 Getter
  const int *getEP4MTPtr() const { return mt_ep4.data(); }
  const int *getEOAltMTPtr() const { return mt_eo_alt.data(); }
  const int *getEOMTPtr() const { return mt_eo.data(); }
  const int *getEP1MTPtr() const { return mt_ep1.data(); }
  // EOCross 专用移动表 Vector Getter (用于剪枝表生成)
  const std::vector<int> &getEP4MT() const { return mt_ep4; }
  const std::vector<int> &getEOAltMT() const { return mt_eo_alt; }

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