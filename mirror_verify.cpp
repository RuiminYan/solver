/*
 * mirror_verify.cpp - 验证 pt_cross_C4E0E1.bin (CEE[0]) 和
 *                     pt_cross_C4E0E3.bin (CEE[2]) 之间是否存在
 *                     可利用的对称关系，从而只保留一张表
 *
 * 编译: g++ -std=c++17 -O3 -fopenmp -Wall -Wextra mirror_verify.cpp
 * cube_common.o move_tables.o prune_tables.o -o mirror_verify.exe -lpsapi
 *
 * ============================================================
 * 研究结论: 不可行
 * ============================================================
 *
 * CEE[0] 和 CEE[2] 使用完全相同的 4 个移动表和 18 个 canonical moves，
 * 唯一差异是 BFS 起点的第 4 维度 (Extra Edge):
 *   CEE[0]: Extra = E1 (pos=1, BR, solved=2)
 *   CEE[2]: Extra = E3 (pos=3, FL, solved=6)
 *
 * 要通过 CEE[0] 推导 CEE[2], 需要一个对称操作 T 满足:
 *   1. T 是所有 4 个移动表 (mt_edge4, mt_corn, mt_edge) 的自同构
 *   2. T 将 CEE[2] 的 solved 映射到 CEE[0] 的 solved:
 *      T(Cross_S, C4_S, E0_S, E3_S) = (Cross_S, C4_S, E0_S, E1_S)
 *      即 T 必须同时:
 *        - 保持 Cross solved (187520) 不动
 *        - 保持 C4 solved (12, DBL) 不动
 *        - 保持 E0 solved (0, BL) 不动
 *        - 把 E3 solved (6, FL) 映射到 E1 solved (2, BR)
 *
 * 测试了两类对称操作:
 *
 * 1. BL-FR 对角反射 (x,y,z)→(z,y,x): R↔F', L↔B', U↔U', D↔D'
 *    - 物理上把 BR↔FL, 且保持 BL/FR/DBL 不动
 *    - 但反射不是魔方群的内部对称 —— 它改变手性
 *    - BFS 验证: mt_corn 和 mt_edge 的所有 24 个候选全部碰撞
 *    - 碰撞示例: Corner 从两条路径到 state 18,
 *      一条给 mirror[18]=16, 另一条给 mirror[18]=12
 *    - 结论: 反射变换不是单角/单棱移动表的自同构
 *
 * 2. y2 旋转 (180° 绕 y 轴): R↔L, F↔B, U/D 不变
 *    - 是 mt_corn 和 mt_edge 的合法自同构 (BFS 验证一致)
 *    - 但 y2 同时移动了其他 3 个维度的 solved 状态:
 *      y2(Cross_S) = 93760 ≠ 187520
 *      y2(C4_S=12) = 9 ≠ 12
 *      y2(E0_S=0)  = 6 ≠ 0
 *    - BFS 从不同起点出发, 距离分布不同
 *    - 12 种候选组合的最佳匹配率: 80.74%
 *
 * 根本矛盾:
 *   不存在一个魔方对称操作能同时满足:
 *   (a) 保持 Cross/C4(DBL)/E0(BL) 的 solved 不动
 *   (b) 把 E1(BR) 映射到 E3(FL)
 *   因为 BL 不在任何连接 BR→FL 的对称轴上。
 *
 * 建议: 保留两张独立表, 或考虑 2-bit 压缩等替代方案减小存储。
 */

#include "cube_common.h"
#include "move_tables.h"
#include "prune_tables.h"
#include <cassert>
#include <iomanip>
#include <queue>

// y2 rotation move mapping: R↔L, F↔B, U/D 不变 (旋转不反转方向)
static const int Y2_MOVE[18] = {
    3,  4,  5,  // R→L, R'→L', R2→L2
    0,  1,  2,  // L→R, L'→R', L2→R2
    6,  7,  8,  // U→U, U'→U', U2→U2
    9,  10, 11, // D→D, D'→D', D2→D2
    15, 16, 17, // F→B, F'→B', F2→B2
    12, 13, 14  // B→F, B'→F', B2→F2
};

// BFS 自动发现映射 (小表)
std::vector<int> bfsMirrorSmall(const int *mt, int solved, int cand, int sz,
                                int step) {
  std::vector<int> mir(sz, -1);
  mir[solved] = cand;
  std::queue<int> q;
  q.push(solved);
  bool ok = true;
  while (!q.empty() && ok) {
    int st = q.front();
    q.pop();
    int st_m = mir[st];
    for (int m = 0; m < 18 && ok; ++m) {
      int next = mt[st * step + m];
      int next_m = mt[st_m * step + Y2_MOVE[m]];
      if (mir[next] == -1) {
        mir[next] = next_m;
        q.push(next);
      } else if (mir[next] != next_m)
        ok = false;
    }
  }
  return ok ? mir : std::vector<int>{};
}

// BFS 自动发现映射 (Edge4, 预乘表)
std::vector<int> bfsMirrorE4(const int *mt, int solved, int cand, int sz) {
  std::vector<int> mir(sz, -1);
  mir[solved] = cand;
  std::queue<int> q;
  q.push(solved);
  bool ok = true;
  while (!q.empty() && ok) {
    int st = q.front();
    q.pop();
    int st_m = mir[st];
    for (int m = 0; m < 18 && ok; ++m) {
      int next = mt[st * 24 + m] / 24;
      int next_m = mt[st_m * 24 + Y2_MOVE[m]] / 24;
      if (mir[next] == -1) {
        mir[next] = next_m;
        q.push(next);
      } else if (mir[next] != next_m)
        ok = false;
    }
  }
  if (!ok)
    return {};
  // 检查覆盖
  int mapped = 0;
  for (int i = 0; i < sz; ++i)
    if (mir[i] != -1)
      mapped++;
  std::cout << "    Edge4: mapped " << mapped << "/" << sz << std::endl;
  return mir;
}

int main() {
  std::cout << "=== Mirror Symmetry Verification (y2 Rotation) ==="
            << std::endl;
  std::cout << "y2: R<->L, F<->B, U/D unchanged" << std::endl << std::endl;

  init_matrix();

  std::cout << "[1/5] Loading tables..." << std::endl;
  auto &mm = MoveTableManager::getInstance();
  mm.loadMTEdge();
  mm.loadMTCorn();
  mm.loadMTEdge4();

  const int *mt_e4 = mm.getMTEdge4Ptr();
  const int *mt_cn = mm.getMTCornPtr();
  const int *mt_ed = mm.getMTEdgePtr();

  std::cout << "\n[2/5] Loading prune tables..." << std::endl;
  auto &pm = PruneTableManager::getInstance();
  pm.genPTCrossCEE(0);
  pm.genPTCrossCEE(2);
  const unsigned char *cee0 = pm.getCrossCEEPTPtr(0);
  const unsigned char *cee2 = pm.getCrossCEEPTPtr(2);
  if (!cee0 || !cee2) {
    std::cerr << "ERROR: load failed" << std::endl;
    return 1;
  }

  int sz_e4 = StateSpace::CROSS;
  int sz_cn = StateSpace::CORNER;
  int sz_ed = StateSpace::EDGE;

  // Solved: C4=12, E0=0, E1=2, E3=6, Cross=187520
  int solved_cn = 12, solved_ed = 0, solved_e1 = 2, solved_e4 = 187520;

  // ============================================================
  // Step 3: BFS 发现 y2 映射
  // ============================================================
  std::cout << "\n[3/5] BFS discovery with y2 rotation..." << std::endl;

  // Corner: 找所有一致候选
  std::cout << "  Corner candidates:" << std::endl;
  std::vector<std::vector<int>> cn_candidates;
  for (int c = 0; c < sz_cn; ++c) {
    auto r = bfsMirrorSmall(mt_cn, solved_cn, c, sz_cn, 18);
    if (!r.empty()) {
      std::cout << "    cand=" << c << " (pos=" << c / 3 << " co=" << c % 3
                << ")" << std::endl;
      cn_candidates.push_back(r);
    }
  }

  // Edge0: E0 solved=0(BL)
  std::cout << "  Edge0 candidates:" << std::endl;
  std::vector<std::vector<int>> ed_candidates;
  for (int c = 0; c < sz_ed; ++c) {
    auto r = bfsMirrorSmall(mt_ed, solved_ed, c, sz_ed, 18);
    if (!r.empty()) {
      std::cout << "    cand=" << c << " (pos=" << c / 2 << " eo=" << c % 2
                << ")" << std::endl;
      ed_candidates.push_back(r);
    }
  }

  // Extra: E1 solved=2(BR)
  std::cout << "  Extra(E1) candidates:" << std::endl;
  std::vector<std::vector<int>> ex_candidates;
  for (int c = 0; c < sz_ed; ++c) {
    auto r = bfsMirrorSmall(mt_ed, solved_e1, c, sz_ed, 18);
    if (!r.empty()) {
      std::cout << "    cand=" << c << " (pos=" << c / 2 << " eo=" << c % 2
                << ")" << std::endl;
      ex_candidates.push_back(r);
    }
  }

  // Edge4: 尝试 solved 本身和附近候选
  std::cout << "  Edge4:" << std::endl;
  std::vector<int> mirror_e4;
  // 先试 solved 本身
  mirror_e4 = bfsMirrorE4(mt_e4, solved_e4, solved_e4, sz_e4);
  if (mirror_e4.empty()) {
    std::cout << "    solved itself failed, trying brute force..." << std::endl;
    for (int c = 0; c < sz_e4 && mirror_e4.empty(); ++c) {
      mirror_e4 = bfsMirrorE4(mt_e4, solved_e4, c, sz_e4);
      if (!mirror_e4.empty())
        std::cout << "    FOUND: cand=" << c << std::endl;
    }
  } else {
    std::cout << "    FOUND: cand=" << solved_e4 << " (self)" << std::endl;
  }

  if (mirror_e4.empty() || cn_candidates.empty() || ed_candidates.empty() ||
      ex_candidates.empty()) {
    std::cout << "ERROR: Missing mappings!" << std::endl;
    return 1;
  }

  // ============================================================
  // Step 4: 尝试所有候选组合
  // ============================================================
  std::cout << "\n[4/5] Testing " << cn_candidates.size() << " x "
            << ed_candidates.size() << " x " << ex_candidates.size()
            << " combinations (sampling)..." << std::endl;

  long long total = (long long)sz_e4 * sz_cn * sz_ed * sz_ed;
  long long sampleSize = std::min(total, (long long)100000000);
  long long step = total / sampleSize;
  if (step < 1)
    step = 1;

  int bestCn = -1, bestEd = -1, bestEx = -1;
  double bestRatio = 0;

  for (int ci = 0; ci < (int)cn_candidates.size(); ++ci) {
    for (int ei = 0; ei < (int)ed_candidates.size(); ++ei) {
      for (int xi = 0; xi < (int)ex_candidates.size(); ++xi) {
        const auto &mirCn = cn_candidates[ci];
        const auto &mirEd = ed_candidates[ei];
        const auto &mirEx = ex_candidates[xi];

        long long match = 0, mismatch = 0;
#pragma omp parallel for reduction(+ : match, mismatch)
        for (long long s = 0; s < sampleSize; ++s) {
          long long i = s * step;
          if (i >= total)
            continue;
          long long rem = i;
          int c_ex = rem % 24;
          rem /= 24;
          int c_ed = rem % 24;
          rem /= 24;
          int c_cn = rem % 24;
          rem /= 24;
          int c_mul = (int)rem;

          int m_mul = mirror_e4[c_mul];
          int m_cn = mirCn[c_cn];
          int m_ed = mirEd[c_ed];
          int m_ex = mirEx[c_ex];

          long long idx0 = ((long long)m_mul * sz_cn + m_cn) * sz_ed * sz_ed +
                           (long long)m_ed * sz_ed + m_ex;

          if (get_prune(cee2, i) == get_prune(cee0, idx0))
            match++;
          else
            mismatch++;
        }

        double ratio = (double)match / (match + mismatch) * 100.0;
        std::cout << "  cn[" << ci << "] ed[" << ei << "] ex[" << xi
                  << "]: " << std::fixed << std::setprecision(4) << ratio << "%"
                  << std::endl;

        if (ratio > bestRatio) {
          bestRatio = ratio;
          bestCn = ci;
          bestEd = ei;
          bestEx = xi;
        }
      }
    }
  }

  if (bestRatio < 99.99) {
    std::cout << "\n*** NO combination found with 100% match. ***" << std::endl;
    std::cout << "Best: " << std::fixed << std::setprecision(4) << bestRatio
              << "%" << std::endl;
    return 1;
  }

  // ============================================================
  // Step 5: 全量验证最佳组合
  // ============================================================
  std::cout << "\n[5/5] Full verification with best combination..."
            << std::endl;
  const auto &mirCn = cn_candidates[bestCn];
  const auto &mirEd = ed_candidates[bestEd];
  const auto &mirEx = ex_candidates[bestEx];

  long long fullMatch = 0, fullMismatch = 0;
#pragma omp parallel for reduction(+ : fullMatch, fullMismatch)                \
    schedule(dynamic, 1000)
  for (long long c_mul = 0; c_mul < sz_e4; ++c_mul) {
    int m_mul = mirror_e4[c_mul];
    for (int c_cn = 0; c_cn < sz_cn; ++c_cn) {
      int m_cn = mirCn[c_cn];
      for (int c_ed = 0; c_ed < sz_ed; ++c_ed) {
        int m_ed = mirEd[c_ed];
        for (int c_ex = 0; c_ex < sz_ed; ++c_ex) {
          int m_ex = mirEx[c_ex];
          long long idx2 =
              ((c_mul * (long long)sz_cn + c_cn) * sz_ed + c_ed) * sz_ed + c_ex;
          long long idx0 =
              ((m_mul * (long long)sz_cn + m_cn) * sz_ed + m_ed) * sz_ed + m_ex;
          if (get_prune(cee2, idx2) == get_prune(cee0, idx0))
            fullMatch++;
          else
            fullMismatch++;
        }
      }
    }
  }

  std::cout << "\n=== Results ===" << std::endl;
  std::cout << "Match:    " << fullMatch << std::endl;
  std::cout << "Mismatch: " << fullMismatch << std::endl;
  if (fullMismatch == 0) {
    std::cout << "\n*** PERFECT MATCH! y2 symmetry CONFIRMED! ***" << std::endl;
    std::cout << "CEE[2] can be derived from CEE[0] via y2 mirror mapping."
              << std::endl;
  }

  return 0;
}
