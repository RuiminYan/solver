/*
 * cube_common.h - 魔方公共定义和工具函数
 */

#ifndef CUBE_COMMON_H
#define CUBE_COMMON_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <string>
#include <sstream>
#include <fstream>
#include <omp.h>
#include <chrono>
#include <mutex>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <atomic>
#include <thread>

// --- 配置 ---
#define ENABLE_DIAGONAL_TABLE 1

// --- 全局变量 ---
extern int valid_moves_flat[20][18];
extern int valid_moves_count[20];
extern int conj_moves_flat[18][4];
extern std::vector<std::string> move_names;

// 查表辅助结构
extern std::vector<std::vector<int>> c_array;
extern std::vector<std::vector<int>> base_array;
extern std::vector<std::vector<int>> base_array2;
extern thread_local std::vector<int> sorted_buffer;

// --- 魔方状态定义 ---
struct State {
    std::vector<int> cp, co, ep, eo;
    State(std::vector<int> c_p={0,1,2,3,4,5,6,7}, std::vector<int> c_o={0,0,0,0,0,0,0,0}, 
          std::vector<int> e_p={0,1,2,3,4,5,6,7,8,9,10,11}, std::vector<int> e_o={0,0,0,0,0,0,0,0,0,0,0,0}) 
        : cp(c_p), co(c_o), ep(e_p), eo(e_o) {}

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

// 移动表创建函数
std::vector<int> create_multi_move_table(int n, int c, int pn, int size, const std::vector<int> &basic_t);
std::vector<int> create_multi_move_table2(int n, int c, int pn, int size, const std::vector<int> &basic_t);

#endif // CUBE_COMMON_H