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
// NOTE: 所有 [INIT], [MOVE], [PRUNE], [LOAD] 等标签统一使用此颜色
#define TAG_COLOR "\033[34m"

// --- Logo 和打印函数 ---
void printCuberootLogo();
void printTableInfo(const std::string &category, const std::string &filename,
                    size_t sizeBytes);
bool fileExists(const std::string &filename);
std::string formatFileSize(size_t bytes);

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
std::vector<int> create_multi_move_table(int n, int c, int pn, int size,
                                         const std::vector<int> &basic_t);
std::vector<int> create_multi_move_table2(int n, int c, int pn, int size,
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
      // 计算进度百分比
      double progress = (double)(total_bytes - remain) / total_bytes * 100.0;
      int bar_width = 30;
      int filled = (int)(progress / 100.0 * bar_width);

      // 构建进度条字符串
      std::string bar(filled, '#');
      bar += std::string(bar_width - filled, '-');

      // ANSI 蓝色输出: \033[34m ... \033[0m
      printf("\033[34m[LOAD]\033[0m (%.2f GB) %s: [%s] %.1f%%\r",
             (double)file_size / (1024.0 * 1024.0 * 1024.0),
             display_name.c_str(), bar.c_str(), progress);
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

#endif // CUBE_COMMON_H