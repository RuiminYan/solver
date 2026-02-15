/*
 * cube_common.cpp - 魔方公共实现
 */

#include "cube_common.h"

// --- Logo 和打印函数实现 ---

// 检查文件是否存在
// NOTE: 自动拼接 TABLE_DIR 前缀，调用方只需传裸文件名
bool fileExists(const std::string &filename) {
  std::string path = std::string(TABLE_DIR) + filename;
  std::ifstream f(path);
  return f.good();
}

// 格式化文件大小
std::string formatFileSize(size_t bytes) {
  std::ostringstream oss;
  if (bytes >= 1024 * 1024 * 1024) {
    oss << std::fixed << std::setprecision(2)
        << (bytes / (1024.0 * 1024.0 * 1024.0)) << " GB";
  } else if (bytes >= 1024 * 1024) {
    oss << std::fixed << std::setprecision(2) << (bytes / (1024.0 * 1024.0))
        << " MB";
  } else if (bytes >= 1024) {
    oss << std::fixed << std::setprecision(2) << (bytes / 1024.0) << " KB";
  } else {
    oss << bytes << " B";
  }
  return oss.str();
}

// 打印表信息（统一格式）
void printTableInfo(const std::string &category, const std::string &filename,
                    size_t sizeBytes) {
  std::cout << ANSI_BLUE << "[" << category << "]" << ANSI_RESET
            << " Loaded: " << filename << " (" << formatFileSize(sizeBytes)
            << ")" << std::endl;
}

// --- 全局变量定义 ---
int valid_moves_flat[20][18];
int valid_moves_count[20];
int conj_moves_flat[18][4];
int rot_map[4][18];
int sym_moves_flat[18][12];
std::vector<std::string> move_names = {"U", "U2", "U'", "D", "D2", "D'",
                                       "L", "L2", "L'", "R", "R2", "R'",
                                       "F", "F2", "F'", "B", "B2", "B'"};

// 查表辅助结构
std::vector<std::vector<int>> c_array = {{0},
                                         {1, 1, 1, 1, 1, 1, 1},
                                         {1, 2, 4, 8, 16, 32, 64},
                                         {1, 3, 9, 27, 81, 243, 729}};
std::vector<std::vector<int>> c_array2 = {
    {0},
    {0},
    {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048},
    {1, 3, 9, 27, 81, 243, 729, 2187}};
std::vector<std::vector<int>> base_array = {
    {0}, {0}, {1, 12, 132, 1320, 11880, 95040}, {1, 8, 56, 336, 1680, 6720}};
std::vector<std::vector<int>> base_array2 = {
    {0}, {0}, {12, 11, 10, 9, 8, 7}, {8, 7, 6, 5, 4, 3}};
thread_local std::vector<int> sorted_buffer(12);

// 魔方状态定义
std::unordered_map<std::string, State> moves_map = {
    {"U", State({3, 0, 1, 2, 4, 5, 6, 7}, {0, 0, 0, 0, 0, 0, 0, 0},
                {0, 1, 2, 3, 7, 4, 5, 6, 8, 9, 10, 11},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0})},
    {"U2", State({2, 3, 0, 1, 4, 5, 6, 7}, {0, 0, 0, 0, 0, 0, 0, 0},
                 {0, 1, 2, 3, 6, 7, 4, 5, 8, 9, 10, 11},
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0})},
    {"U'", State({1, 2, 3, 0, 4, 5, 6, 7}, {0, 0, 0, 0, 0, 0, 0, 0},
                 {0, 1, 2, 3, 5, 6, 7, 4, 8, 9, 10, 11},
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0})},
    {"D", State({0, 1, 2, 3, 5, 6, 7, 4}, {0, 0, 0, 0, 0, 0, 0, 0},
                {0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 8},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0})},
    {"D2", State({0, 1, 2, 3, 6, 7, 4, 5}, {0, 0, 0, 0, 0, 0, 0, 0},
                 {0, 1, 2, 3, 4, 5, 6, 7, 10, 11, 8, 9},
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0})},
    {"D'", State({0, 1, 2, 3, 7, 4, 5, 6}, {0, 0, 0, 0, 0, 0, 0, 0},
                 {0, 1, 2, 3, 4, 5, 6, 7, 11, 8, 9, 10},
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0})},
    {"L", State({4, 1, 2, 0, 7, 5, 6, 3}, {2, 0, 0, 1, 1, 0, 0, 2},
                {11, 1, 2, 7, 4, 5, 6, 0, 8, 9, 10, 3},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0})},
    {"L2", State({7, 1, 2, 4, 3, 5, 6, 0}, {0, 0, 0, 0, 0, 0, 0, 0},
                 {3, 1, 2, 0, 4, 5, 6, 11, 8, 9, 10, 7},
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0})},
    {"L'", State({3, 1, 2, 7, 0, 5, 6, 4}, {2, 0, 0, 1, 1, 0, 0, 2},
                 {7, 1, 2, 11, 4, 5, 6, 3, 8, 9, 10, 0},
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0})},
    {"R", State({0, 2, 6, 3, 4, 1, 5, 7}, {0, 1, 2, 0, 0, 2, 1, 0},
                {0, 5, 9, 3, 4, 2, 6, 7, 8, 1, 10, 11},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0})},
    {"R2", State({0, 6, 5, 3, 4, 2, 1, 7}, {0, 0, 0, 0, 0, 0, 0, 0},
                 {0, 2, 1, 3, 4, 9, 6, 7, 8, 5, 10, 11},
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0})},
    {"R'", State({0, 5, 1, 3, 4, 6, 2, 7}, {0, 1, 2, 0, 0, 2, 1, 0},
                 {0, 9, 5, 3, 4, 1, 6, 7, 8, 2, 10, 11},
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0})},
    {"F", State({0, 1, 3, 7, 4, 5, 2, 6}, {0, 0, 1, 2, 0, 0, 2, 1},
                {0, 1, 6, 10, 4, 5, 3, 7, 8, 9, 2, 11},
                {0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0})},
    {"F2", State({0, 1, 7, 6, 4, 5, 3, 2}, {0, 0, 0, 0, 0, 0, 0, 0},
                 {0, 1, 3, 2, 4, 5, 10, 7, 8, 9, 6, 11},
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0})},
    {"F'", State({0, 1, 6, 2, 4, 5, 7, 3}, {0, 0, 1, 2, 0, 0, 2, 1},
                 {0, 1, 10, 6, 4, 5, 2, 7, 8, 9, 3, 11},
                 {0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0})},
    {"B", State({1, 5, 2, 3, 0, 4, 6, 7}, {1, 2, 0, 0, 2, 1, 0, 0},
                {4, 8, 2, 3, 1, 5, 6, 7, 0, 9, 10, 11},
                {1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0})},
    {"B2", State({5, 4, 2, 3, 1, 0, 6, 7}, {0, 0, 0, 0, 0, 0, 0, 0},
                 {1, 0, 2, 3, 8, 5, 6, 7, 4, 9, 10, 11},
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0})},
    {"B'", State({4, 0, 2, 3, 5, 1, 6, 7}, {1, 2, 0, 0, 2, 1, 0, 0},
                 {8, 4, 2, 3, 0, 5, 6, 7, 1, 9, 10, 11},
                 {1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0})}};

// --- State 方法实现 ---
State State::apply_move_edge(State m, int e) {
  std::vector<int> nep(12, -1), neo(12, -1);
  auto it = std::find(ep.begin(), ep.end(), e);
  int idx = std::distance(ep.begin(), it);
  it = std::find(m.ep.begin(), m.ep.end(), e);
  int idx_next = std::distance(m.ep.begin(), it);
  nep[idx_next] = e;
  neo[idx_next] = (eo[idx] + m.eo[idx_next]) % 2;
  return State(cp, co, nep, neo);
}

State State::apply_move_corner(State m, int c) {
  std::vector<int> ncp(8, -1), nco(8, -1);
  auto it = std::find(cp.begin(), cp.end(), c);
  int idx = std::distance(cp.begin(), it);
  it = std::find(m.cp.begin(), m.cp.end(), c);
  int idx_next = std::distance(m.cp.begin(), it);
  ncp[idx_next] = c;
  nco[idx_next] = (co[idx] + nco[idx_next]) % 3;
  ncp[idx_next] = c;
  nco[idx_next] = (co[idx] + m.co[idx_next]) % 3;
  return State(ncp, nco, ep, eo);
}

State State::apply_move(const State &m) const {
  std::vector<int> ncp, nco, nep, neo;
  ncp.reserve(8);
  nco.reserve(8);
  nep.reserve(12);
  neo.reserve(12);
  for (int i = 0; i < 8; ++i) {
    ncp.push_back(cp[m.cp[i]]);
    nco.push_back((co[m.cp[i]] + m.co[i]) % 3);
  }
  for (int i = 0; i < 12; ++i) {
    nep.push_back(ep[m.ep[i]]);
    neo.push_back((eo[m.ep[i]] + m.eo[i]) % 2);
  }
  return State(ncp, nco, nep, neo);
}

// --- 索引转换工具 ---
int o_to_index(const std::vector<int> &o, int c, int pn) {
  int idx = 0;
  for (int i = 0; i < pn - 1; ++i)
    idx += o[i] * c_array2[c][pn - i - 2];
  return idx;
}

void index_to_o(std::vector<int> &o, int idx, int c, int pn) {
  int cnt = 0;
  for (int i = 0; i < pn - 1; ++i) {
    o[pn - i - 2] = idx % c;
    cnt += o[pn - i - 2];
    idx /= c;
  }
  o[pn - 1] = (c - cnt % c) % c;
}

// --- Y-rotation mapping initialization ---
static void init_rot_maps() {
  // Generate y-rotation permutations
  // y (U CW): F->L, L->B, B->R, R->F, U->U, D->D
  // Base: 0:U, 1:D, 2:L, 3:R, 4:F, 5:B (Type)
  int y_type_map[6] = {0, 1, 5, 4, 2, 3}; // U->U, D->D, L->B, R->F, F->L, B->R

  for (int k = 0; k < 4; ++k) {
    for (int m = 0; m < 18; ++m) {
      if (k == 0) {
        rot_map[k][m] = m;
      } else {
        int prev_map = rot_map[k - 1][m];
        int prev_type = prev_map / 3;
        int prev_pow = prev_map % 3;
        int new_type = y_type_map[prev_type];
        rot_map[k][m] = new_type * 3 + prev_pow;
      }
    }
  }
}

// --- 初始化矩阵 ---
static void init_symmetry_matrix() {
  std::vector<std::string> rot_names = {"",  "y",   "z2", "z2 y", "z'", "z' y",
                                        "z", "z y", "x'", "x' y", "x",  "x y"};
  for (int m = 0; m < 18; ++m) {
    for (int s = 0; s < 12; ++s) {
      std::vector<int> rotated = alg_rotation({m}, rot_names[s]);
      sym_moves_flat[m][s] = rotated[0];
    }
  }
}

void init_matrix() {
  for (int prev = 0; prev <= 18; ++prev) {
    int cnt = 0;
    for (int i = 0; i < 18; ++i) {
      bool bad =
          (prev < 18) && (i / 3 == prev / 3 || ((i / 3) / 2 == (prev / 3) / 2 &&
                                                (prev / 3) % 2 > (i / 3) % 2));
      if (!bad)
        valid_moves_flat[prev][cnt++] = i;
    }
    valid_moves_count[prev] = cnt;
  }
  for (int i = 0; i < 18; ++i) {
    int m_type = i / 3;
    int m_pow = i % 3;
    conj_moves_flat[i][0] = i;
    int m1 = -1;
    if (m_type == 0)
      m1 = i;
    else if (m_type == 1)
      m1 = i;
    else if (m_type == 2)
      m1 = 12 + m_pow;
    else if (m_type == 3)
      m1 = 15 + m_pow;
    else if (m_type == 4)
      m1 = 9 + m_pow;
    else if (m_type == 5)
      m1 = 6 + m_pow;
    conj_moves_flat[i][1] = m1;
    int m2 = -1;
    if (m_type == 0)
      m2 = i;
    else if (m_type == 1)
      m2 = i;
    else if (m_type == 2)
      m2 = 9 + m_pow;
    else if (m_type == 3)
      m2 = 6 + m_pow;
    else if (m_type == 4)
      m2 = 15 + m_pow;
    else if (m_type == 5)
      m2 = 12 + m_pow;
    conj_moves_flat[i][2] = m2;
    int m3 = -1;
    if (m_type == 0)
      m3 = i;
    else if (m_type == 1)
      m3 = i;
    else if (m_type == 2)
      m3 = 15 + m_pow;
    else if (m_type == 3)
      m3 = 12 + m_pow;
    else if (m_type == 4)
      m3 = 6 + m_pow;
    else if (m_type == 5)
      m3 = 9 + m_pow;
    conj_moves_flat[i][3] = m3;
  }

  init_symmetry_matrix();
  init_rot_maps();
}

// --- 实用函数 ---
std::vector<int> string_to_alg(std::string str) {
  std::vector<int> alg;
  std::istringstream iss(str);
  std::string name;
  while (iss >> name) {
    auto it = std::find(move_names.begin(), move_names.end(), name);
    if (it != move_names.end())
      alg.emplace_back(std::distance(move_names.begin(), it));
  }
  return alg;
}

std::vector<int> alg_convert_rotation(std::vector<int> alg, std::string rot) {
  if (rot.empty())
    return alg;
  std::vector<int> f;
  if (rot == "x")
    f = {5, 4, 2, 3, 0, 1};
  else if (rot == "x2")
    f = {1, 0, 2, 3, 5, 4};
  else if (rot == "x'")
    f = {4, 5, 2, 3, 1, 0};
  else if (rot == "y")
    f = {0, 1, 5, 4, 2, 3};
  else if (rot == "y2")
    f = {0, 1, 3, 2, 5, 4};
  else if (rot == "y'")
    f = {0, 1, 4, 5, 3, 2};
  else if (rot == "z")
    f = {3, 2, 0, 1, 4, 5};
  else if (rot == "z2")
    f = {1, 0, 3, 2, 4, 5};
  else if (rot == "z'")
    f = {2, 3, 1, 0, 4, 5};
  for (size_t i = 0; i < alg.size(); ++i)
    alg[i] = 3 * f[alg[i] / 3] + alg[i] % 3;
  return alg;
}

std::vector<int> alg_rotation(std::vector<int> a, std::string r) {
  std::istringstream iss(r);
  std::string tmp;
  while (iss >> tmp)
    a = alg_convert_rotation(a, tmp);
  return a;
}

int array_to_index(const std::vector<int> &a, int n, int c, int pn) {
  int idx_p = 0, idx_o = 0, tmp, tmp2 = 24 / pn;
  for (int i = 0; i < n; ++i) {
    idx_o += (a[i] % c) * c_array[c][n - i - 1];
  }
  std::vector<int> pa = a;
  for (int i = 0; i < n; ++i)
    pa[i] /= c;
  for (int i = 0; i < n; ++i) {
    tmp = 0;
    for (int j = 0; j < i; ++j)
      if (pa[j] < pa[i])
        tmp++;
    idx_p += (pa[i] - tmp) * base_array[tmp2][i];
  }
  return idx_p * c_array[c][n] + idx_o;
}

void index_to_array(std::vector<int> &p, int index, int n, int c, int pn) {
  int tmp2 = 24 / pn, p_idx = index / c_array[c][n],
      o_idx = index % c_array[c][n];
  if (sorted_buffer.size() < (size_t)n)
    sorted_buffer.resize(n);
  for (int i = 0; i < n; ++i) {
    p[i] = p_idx % base_array2[tmp2][i];
    p_idx /= base_array2[tmp2][i];
    std::sort(sorted_buffer.begin(), sorted_buffer.begin() + i);
    for (int j = 0; j < i; ++j)
      if (sorted_buffer[j] <= p[i])
        p[i]++;
    sorted_buffer[i] = p[i];
  }
  for (int i = 0; i < n; ++i) {
    p[n - i - 1] = 18 * (c * p[n - i - 1] + o_idx % c);
    o_idx /= c;
  }
}

// 构建多块移动表（标准版，步长18）
// 对状态空间中的每个状态，预计算 18 种操作后的目标状态索引。
// 返回值语义：mt[index * 18 + move] = 目标状态的原始索引
// 调用方查表时需要：next_index = mt[cur_index * 18 + move]
//
// 参数:
//   n       - 参与编码的块数量
//   c       - 方向基数（edge=2, corner=3）
//   pn      - 块总数（edge=12, corner=8）
//   size    - 状态空间大小
//   basic_t - 单块基础移动表（由 genMTEdge/genMTCorn 等生成）
//
// NOTE: 利用逆操作对称性（inv[j]）双向填充，将计算量减半。
//       被 mt_edge2, mt_edge3, mt_edge6, mt_corn2, mt_corn3, mt_ep4 使用。
std::vector<int> createMultiMoveTable(int n, int c, int pn, int size,
                                      const std::vector<int> &basic_t) {
  std::vector<int> mt(size * 18, -1);
  std::vector<int> a(n), b(n);
  std::vector<int> inv = {2,  1,  0, 5,  4,  3,  8,  7,  6,
                          11, 10, 9, 14, 13, 12, 17, 16, 15};
  // NOTE: 仅对大表显示进度，小表瞬间完成无需显示
  bool showProgress = (size > 100000);
  int lastPct = -1;
  for (int i = 0; i < size; ++i) {
    if (showProgress) {
      int pct = (int)((long long)i * 100 / size);
      if (pct != lastPct) {
        lastPct = pct;
        std::cout << "\r  Progress: " << pct << "%" << std::flush;
      }
    }
    index_to_array(a, i, n, c, pn);
    int tmp_i = i * 18;
    for (int j = 0; j < 18; ++j) {
      if (mt[tmp_i + j] == -1) {
        for (int k = 0; k < n; ++k)
          b[k] = basic_t[a[k] + j];
        int tmp = array_to_index(b, n, c, pn);
        mt[tmp_i + j] = tmp;
        mt[18 * tmp + inv[j]] = i;
      }
    }
  }
  if (showProgress)
    std::cout << "\r\033[2K" << std::flush;
  return mt;
}

// 构建多块移动表（预乘偏移版，步长24）
// 与标准版的核心区别：表中存储的不是原始索引，而是预乘了步长的数组偏移量。
// 返回值语义：mt[offset + move] = 目标状态的预乘偏移量（= index * 24）
// 调用方查表时无需乘法：next_offset = mt[cur_offset + move]
//
// 为什么需要这个版本：
//   mt_edge4 是搜索热路径上访问最频繁的表，在 IDA* 内层循环中被调用数十亿次。
//   标准版每次查表后需要 result * 18 才能进行下一次查表（18 非 2
//   的幂，无法位移优化）。 本版本通过预乘偏移量，将链式查表简化为 mt[offset +
//   move]，省去内层循环的乘法开销。 代价是每个状态占 24 个槽位而非 18 个（额外
//   ~4.3MB），属于空间换时间。
//
// 仅被 mt_edge4 使用。
std::vector<int> createMultiMoveTable2(int n, int c, int pn, int size,
                                       const std::vector<int> &basic_t) {
  std::vector<int> mt(size * 24, -1);
  std::vector<int> a(n), b(n);
  std::vector<int> inv = {2,  1,  0, 5,  4,  3,  8,  7,  6,
                          11, 10, 9, 14, 13, 12, 17, 16, 15};
  // NOTE: 仅对大表显示进度，小表瞬间完成无需显示
  bool showProgress = (size > 100000);
  int lastPct = -1;
  for (int i = 0; i < size; ++i) {
    if (showProgress) {
      int pct = (int)((long long)i * 100 / size);
      if (pct != lastPct) {
        lastPct = pct;
        std::cout << "\r  Progress: " << pct << "%" << std::flush;
      }
    }
    index_to_array(a, i, n, c, pn);
    int tmp_i = i * 24;
    for (int j = 0; j < 18; ++j) {
      if (mt[tmp_i + j] == -1) {
        for (int k = 0; k < n; ++k)
          b[k] = basic_t[a[k] + j];
        int tmp = 24 * array_to_index(b, n, c, pn);
        mt[tmp_i + j] = tmp;
        mt[tmp + inv[j]] = tmp_i;
      }
    }
  }
  if (showProgress)
    std::cout << "\r\033[2K" << std::flush;
  return mt;
}