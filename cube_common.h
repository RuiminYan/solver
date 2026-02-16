/*
 * cube_common.h - 魔方公共定义和工具函数
 */

#ifndef CUBE_COMMON_H
#define CUBE_COMMON_H

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <omp.h>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

// --- 配置 ---
// Diagonal 表配置（每个 analyzer 独立控制，节省 ~10 GB 内存）
#define ENABLE_DIAGONAL_STD 1      // Std Analyzer 的 Diagonal 表
#define ENABLE_DIAGONAL_PAIR 1     // Pair Analyzer 的 Diagonal 表
#define ENABLE_DIAGONAL_EO_CROSS 0 // EO Cross Analyzer 的 Diagonal 表
#define ENABLE_EO_SEARCH_4 1 // 启用 EO Cross Analyzer 的 XXXXCross+EO 计算

// --- ANSI 颜色定义 ---
#define ANSI_RESET "\033[0m"
#define ANSI_CYAN "\033[36m"
#define ANSI_GREEN "\033[32m"
#define ANSI_YELLOW "\033[33m"
#define ANSI_MAGENTA "\033[35m"
#define ANSI_RED "\033[31m"
#define ANSI_BLUE "\033[34m"

// --- 日志标签统一颜色（蓝色）---
#define TAG_COLOR "\033[34m"

// 表文件存储目录前缀
// NOTE: 所有 .bin 文件统一存放在此目录下，避免与源码混合
constexpr const char *TABLE_DIR = "tables/";

// --- Logo ---
#include "logo.h"

void printTableInfo(const std::string &category, const std::string &filename,
                    size_t sizeBytes);
bool fileExists(const std::string &filename);
std::string formatFileSize(size_t bytes);

// 表生成计时器 - 在生成开始前构造，saveTable 之后调用 printElapsed
struct GenerationTimer {
  std::chrono::steady_clock::time_point start;

  GenerationTimer() : start(std::chrono::steady_clock::now()) {}

  void printElapsed(const std::string & /*filename*/) const {
    auto end = std::chrono::steady_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
                  .count();
    int min = static_cast<int>(ms / 60000);
    int sec = static_cast<int>((ms % 60000) / 1000);
    if (min > 0) {
      std::cout << "  Done in " << min << "min " << sec << "s." << std::endl;
    } else {
      std::cout << "  Done in " << sec << "s." << std::endl;
    }
  }
};

// --- 全局变量 ---
extern int valid_moves_flat[20][18];
extern int valid_moves_count[20];
extern int conj_moves_flat[18][4];
extern int rot_map[4][18]; // Y-rotation mapping: 0:Id, 1:y, 2:y2, 3:y'
extern int sym_moves_flat[18][12];
extern std::vector<std::string> move_names;

// 查表辅助结构
extern std::vector<std::vector<int>> c_array;
extern std::vector<std::vector<int>> c_array2;
extern std::vector<std::vector<int>> base_array;
extern std::vector<std::vector<int>> base_array2;
extern thread_local std::vector<int> sorted_buffer;

// 已加载表的总大小（字节）
// NOTE: 用于显示实际加载的移动表和剪枝表的内存占用
inline std::atomic<size_t> g_loadedTableBytes{0};

// --- 魔方状态定义 ---
struct State {
  std::vector<int> cp, co, ep, eo;
  State(std::vector<int> c_p = {0, 1, 2, 3, 4, 5, 6, 7},
        std::vector<int> c_o = {0, 0, 0, 0, 0, 0, 0, 0},
        std::vector<int> e_p = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11},
        std::vector<int> e_o = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0})
      : cp(c_p), co(c_o), ep(e_p), eo(e_o) {}

  State apply_move(const State &m) const;
  State apply_move_edge(State m, int e);
  State apply_move_corner(State m, int c);
};

extern std::unordered_map<std::string, State> moves_map;

// --- 工具函数声明 ---
void init_matrix();
std::vector<int> string_to_alg(std::string str);
std::vector<int> alg_convert_rotation(std::vector<int> alg, std::string rot);
std::vector<int> alg_rotation(std::vector<int> a, std::string r);

// 索引转换函数
int array_to_index(const std::vector<int> &a, int n, int c, int pn);
void index_to_array(std::vector<int> &p, int index, int n, int c, int pn);
int o_to_index(const std::vector<int> &o, int c, int pn);
void index_to_o(std::vector<int> &o, int idx, int c, int pn);

// 移动表创建函数
std::vector<int> createMultiMoveTable(int n, int c, int pn, int size,
                                      const std::vector<int> &basic_table);
std::vector<int> createMultiMoveTable2(int n, int c, int pn, int size,
                                       const std::vector<int> &basic_t);

// --- 文件读写辅助 (Template Implementation) ---
// 移动到此处，使所有模块均可使用，消除模块间不必要的依赖

// NOTE: 大文件加载阈值 (1GB)，超过此阈值时显示进度条
constexpr size_t LARGE_FILE_THRESHOLD = 1ULL * 1024 * 1024 * 1024;

template <typename T>
bool load_vector_chunked(std::vector<T> &vec, const std::string &filename,
                         bool enable_progress = true) {
  std::ifstream in(filename, std::ios::binary);
  if (!in)
    return false;
  in.seekg(0, std::ios::end);
  size_t file_size = in.tellg();
  in.seekg(0, std::ios::beg);

  size_t size;
  in.read(reinterpret_cast<char *>(&size), sizeof(size));
  if (!in)
    return false;

  size_t expected = size * sizeof(T);
  if (file_size != expected + sizeof(size_t))
    return false;

  try {
    vec.resize(size);
  } catch (...) {
    return false;
  }

  char *ptr = reinterpret_cast<char *>(vec.data());
  size_t remain = expected;
  size_t total_bytes = expected;

  // NOTE: 仅当enable_progress为true且文件超过1GB时才显示进度条
  bool show_progress = enable_progress && (file_size > LARGE_FILE_THRESHOLD);

  // 提取文件名（不含路径）用于显示
  std::string display_name = filename;
  size_t last_slash = filename.find_last_of("/\\");
  if (last_slash != std::string::npos) {
    display_name = filename.substr(last_slash + 1);
  }

  while (remain > 0) {
    size_t to_read = std::min(remain, (size_t)64 * 1024 * 1024); // 64MB chunks
    in.read(ptr, to_read);
    if (!in)
      return false;
    ptr += to_read;
    remain -= to_read;

    if (show_progress) {
      double progress = (double)(total_bytes - remain) / total_bytes * 100.0;
      printf("\033[34m[LOAD]\033[0m (%.2f GB) %s: %.1f%%\r",
             (double)file_size / (1024.0 * 1024.0 * 1024.0),
             display_name.c_str(), progress);
      fflush(stdout);
    }
  }

  // 加载完成后清除进度条行，由 load_vector 打印最终信息
  if (show_progress) {
    printf("\r\033[K"); // 回到行首并清除整行
    fflush(stdout);
  }

  // 累加已加载表的大小
  g_loadedTableBytes.fetch_add(expected, std::memory_order_relaxed);

  return true;
}

template <typename T>
bool load_vector(std::vector<T> &vec, const std::string &filename) {
  if (load_vector_chunked(vec, filename)) {
    // 计算并打印表大小
    size_t size_bytes = vec.size() * sizeof(T);
    // 提取文件名（不含路径）用于显示
    std::string display_name = filename;
    size_t last_slash = filename.find_last_of("/\\");
    if (last_slash != std::string::npos) {
      display_name = filename.substr(last_slash + 1);
    }
    std::cout << TAG_COLOR << "[LOAD]" << ANSI_RESET << " ("
              << formatFileSize(size_bytes) << ") " << display_name
              << std::endl;
    return true;
  }
  return false;
}

template <typename T>
bool save_vector(const std::vector<T> &vec, const std::string &filename) {
  std::ofstream out(filename, std::ios::binary);
  size_t size = vec.size();
  out.write(reinterpret_cast<const char *>(&size), sizeof(size));
  out.write(reinterpret_cast<const char *>(vec.data()), size * sizeof(T));
  return out.good();
}

template <typename T>
bool save_vector_chunked(const std::vector<T> &vec,
                         const std::string &filename) {
  std::ofstream out(filename, std::ios::binary);
  size_t size = vec.size();
  out.write(reinterpret_cast<const char *>(&size), sizeof(size));
  const char *ptr = reinterpret_cast<const char *>(vec.data());
  size_t remain = size * sizeof(T);
  while (remain > 0) {
    size_t to_write = std::min(remain, (size_t)64 * 1024 * 1024);
    out.write(ptr, to_write);
    ptr += to_write;
    remain -= to_write;
  }
  return out.good();
}

// NOTE: 各子空间的状态数量，源于排列/组合计算
// 用作剪枝表维度参数和索引计算，消除硬编码魔数
namespace StateSpace {
constexpr int EDGE = 24;        // 单棱: P(12,1) * 2 = 24
constexpr int CORNER = 24;      // 单角: P(8,1) * 3  = 24
constexpr int EDGE2 = 528;      // 2棱: 24 * 22
constexpr int EDGE3 = 10560;    // 3棱: 24 * 22 * 20
constexpr int CORNER2 = 504;    // 2角: 24 * 21
constexpr int CORNER3 = 9072;   // 3角: 24 * 21 * 18
constexpr int CROSS = 190080;   // 4棱(Cross): 24 * 22 * 20 * 18
constexpr int EDGE6 = 42577920; // 6棱: P(12,6) * 2^6 = 665280 * 64
constexpr int EP4 = 11880;      // EP4: P(12,4) = 12 * 11 * 10 * 9
constexpr int EO12 = 2048;      // EO12: 2^11

// --- Solved State Indices ---
// Cross 4 棱 {8,9,10,11} 还原态编码 = Lehmer({8,9,10,11}) * 2^4
// = 8*(1+12+132+1320) * 16 = 11720 * 16
constexpr int CROSS_SOLVED = 187520;
// Edge2 两个分量：Cross 4 棱拆成 {8,9} 和 {10,11} 两组
// 416 = Lehmer({8,9}) * 2^2 = 104 * 4
// 520 = Lehmer({10,11}) * 2^2 = 130 * 4
constexpr int EDGE2_A_SOLVED = 416;
constexpr int EDGE2_B_SOLVED = 520;
// EP4 中 {8,9,10,11} 的 Lehmer 秩（不含朝向）= 8*(1+12+132+1320)
constexpr int EP4_SOLVED = 11720;
// Edge6 仅位置空间 = P(12,6) = 12!/6!（不含朝向，区别于 EDGE6）
constexpr int EDGE6_POS = 665280;
} // namespace StateSpace

// --- 槽位关系判断工具函数 ---
// NOTE: F2L 4个槽位 (0-3) 按环形排列，以下函数判断两个槽位的空间关系
// 用于选择正确的 Conj 表（Neighbor/Diagonal）和 Plus 表索引

// 判断两个 slot 是否相邻，返回 Conj 基准 slot; -1 表示非相邻
inline int getNeighborView(int s1, int s2) {
  if ((s2 - s1 + 4) % 4 == 1)
    return s1;
  if ((s1 - s2 + 4) % 4 == 1)
    return s2;
  return -1;
}

// 判断两个 slot 是否对角，返回 Conj 基准 slot; -1 表示非对角
inline int getDiagonalView(int s1, int s2) {
  int mn = std::min(s1, s2);
  int mx = std::max(s1, s2);
  if (mn == 0 && mx == 2)
    return 0;
  if (mn == 1 && mx == 3)
    return 1;
  return -1;
}

// slot 差值映射到 Plus 表索引: 0=Right, 1=Diag, 2=Left; -1 表示无效
inline int getPlusTableIdx(int s_base, int s_target) {
  int diff = (s_target - s_base + 4) % 4;
  if (diff == 1)
    return 0;
  if (diff == 2)
    return 1;
  if (diff == 3)
    return 2;
  return -1;
}

// --- 通用辅助剪枝结构 ---
// NOTE: 用于 Pseudo Analyzer 和 PseudoPair Analyzer 的 Aux 表架构
// 每个 AuxPrunerDef 定义一张辅助剪枝表的查询参数
struct AuxPrunerDef {
  const unsigned char *p_prune; // 剪枝表指针
  const int *p_move; // 移动表指针(Edge2, Corner2, Edge3, Corner3)
  int multiplier;    // 状态乘数(用于结合 Cross 状态)
};

// 搜索过程中追踪的辅助表状态
struct AuxState {
  const AuxPrunerDef *def = nullptr;
  int current_idx = 0;
  int current_cross_scaled = 0;     // Cross state * 24 (for virtual cross)
  const int *move_mapper = nullptr; // Map move m -> m'
  int slot_k = 0;                   // 共轭参考槽位(PseudoPair 使用)
};

// 每个搜索路径最大支持的辅助表数量
constexpr int MAX_AUX = 8;

// --- Aux 表索引类型判断 ---
// Edge2: 返回 0=邻接, 1=对角
inline int getE2Type(int e1, int e2) {
  int diff = (e2 - e1 + 4) & 3;
  return (diff == 2) ? 1 : 0;
}

// Corner2: 返回 0=邻接, 1=对角
inline int getC2Type(int c1, int c2) {
  int diff = (c2 - c1 + 4) & 3;
  return (diff == 2) ? 1 : 0;
}

#endif // CUBE_COMMON_H