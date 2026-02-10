/*
 * prune_create.cpp - BFS
 */

#include "prune_tables.h"
#include <iostream>
#include <omp.h>
void createPTPsCrossCorn2(int idx_cr, int idx_c2, int sz_cr, int sz_c2,
                          int depth, const std::vector<int> &t_cr,
                          const std::vector<int> &t_c2,
                          std::vector<unsigned char> &pt) {
  long long total = (long long)sz_cr * sz_c2;
  std::vector<unsigned char> tmp;
  try {
    tmp.resize(total, 255);
  } catch (...) {
    std::cerr << "Alloc fail (Corner2)\n";
    exit(1);
  }

  int d_moves[] = {-1, 3, 4, 5};
  for (int m_idx = 0; m_idx < 4; ++m_idx) {
    int cur_cr = idx_cr;
    int cur_c2 = idx_c2;
    if (m_idx > 0) {
      int move = d_moves[m_idx];
      cur_cr = t_cr[idx_cr * 24 + move] / 24;
      cur_c2 = t_c2[idx_c2 * 18 + move];
    }
    long long start_idx = (long long)cur_cr * sz_c2 + cur_c2;
    if (start_idx < total)
      tmp[start_idx] = 0;
  }

  DistributionPrinter dp(total);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      dp.progress(i, total, d);
      if (tmp[i] == d) {
        cnt++;
        int cur_c2 = i % sz_c2;
        int cur_cr = i / sz_c2;
        int base_cr = cur_cr * 24;
        int base_c2 = cur_c2 * 18;
        for (int j = 0; j < 18; ++j) {
          int n_cr = t_cr[base_cr + j] / 24;
          int n_c2 = t_c2[base_c2 + j];
          long long ni = (long long)n_cr * sz_c2 + n_c2;
          // NOTE: 使用 CAS 避免竞态条件
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&tmp[ni], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  std::fill(pt.begin(), pt.end(), 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

void createPTPsCrossCorn3(int idx_cr, int idx_c3, int sz_cr, int sz_c3,
                          int depth, const std::vector<int> &t_cr,
                          const std::vector<int> &t_c3,
                          std::vector<unsigned char> &pt) {
  long long total = (long long)sz_cr * sz_c3;
  std::cout << "Allocating Corner3 Prune Table: " << total / 1024 / 1024
            << " MB" << std::endl;
  std::vector<unsigned char> tmp;
  try {
    tmp.resize(total, 255);
  } catch (...) {
    std::cerr << "Alloc fail (Corner3)\n";
    exit(1);
  }

  int d_moves[] = {-1, 3, 4, 5};
  for (int m_idx = 0; m_idx < 4; ++m_idx) {
    int cur_cr = idx_cr;
    int cur_c3 = idx_c3;
    if (m_idx > 0) {
      int move = d_moves[m_idx];
      cur_cr = t_cr[idx_cr * 24 + move] / 24;
      cur_c3 = t_c3[idx_c3 * 18 + move];
    }
    long long start_idx = (long long)cur_cr * sz_c3 + cur_c3;
    if (start_idx < total)
      tmp[start_idx] = 0;
  }

  DistributionPrinter dp(total);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      dp.progress(i, total, d);
      if (tmp[i] == d) {
        cnt++;
        int cur_c3 = i % sz_c3;
        int cur_cr = i / sz_c3;
        int base_cr = cur_cr * 24;
        int base_c3 = cur_c3 * 18;
        for (int j = 0; j < 18; ++j) {
          int n_cr = t_cr[base_cr + j] / 24;
          int n_c3 = t_c3[base_c3 + j];
          long long ni = (long long)n_cr * sz_c3 + n_c3;
          // NOTE: 使用 CAS 避免竞态条件
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&tmp[ni], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  std::fill(pt.begin(), pt.end(), 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

void createPTPsCrossEdge3(int idx_cr, int idx_e3, int sz_cr, int sz_e3,
                          int depth, const std::vector<int> &t_cr,
                          const std::vector<int> &t_e3,
                          std::vector<unsigned char> &pt) {
  long long total = (long long)sz_cr * sz_e3;
  std::cout << "Allocating Edge3 Prune Table: " << total / 1024 / 1024 << " MB"
            << std::endl;
  std::vector<unsigned char> tmp;
  try {
    tmp.resize(total, 255);
  } catch (...) {
    std::cerr << "Alloc fail (Edge3)\n";
    exit(1);
  }

  int d_moves[] = {-1, 3, 4, 5};
  for (int m_idx = 0; m_idx < 4; ++m_idx) {
    int cur_cr = idx_cr;
    int cur_e3 = idx_e3;
    if (m_idx > 0) {
      int move = d_moves[m_idx];
      cur_cr = t_cr[idx_cr * 24 + move] / 24;
      cur_e3 = t_e3[idx_e3 * 18 + move];
    }
    long long start_idx = (long long)cur_cr * sz_e3 + cur_e3;
    if (start_idx < total)
      tmp[start_idx] = 0;
  }

  DistributionPrinter dp(total);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      dp.progress(i, total, d);
      if (tmp[i] == d) {
        cnt++;
        int cur_e3 = i % sz_e3;
        int cur_cr = i / sz_e3;
        int base_cr = cur_cr * 24;
        int base_e3 = cur_e3 * 18;
        for (int j = 0; j < 18; ++j) {
          int n_cr = t_cr[base_cr + j] / 24;
          int n_e3 = t_e3[base_e3 + j];
          long long ni = (long long)n_cr * sz_e3 + n_e3;
          // NOTE: 使用 CAS 避免竞态条件
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&tmp[ni], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  std::fill(pt.begin(), pt.end(), 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

// 1. Cross + C4 Insertion (Base)
void createPTCrossInsC(int idx_cr, int idx_cn, int sz_cr, int sz_cn, int depth,
                       const std::vector<int> &t_cr,
                       const std::vector<int> &t_cn,
                       std::vector<unsigned char> &pt) {
  long long total = (long long)sz_cr * sz_cn;
  std::vector<unsigned char> tmp(total, 255);
  std::vector<std::string> am = {"L U L'", "L U' L'", "B' U B", "B' U' B"};
  tmp[(long long)idx_cr * sz_cn + idx_cn] = 0;
  for (const auto &s : am) {
    int i_cr = idx_cr * 24, i_cn = idx_cn;
    for (int m : string_to_alg(s)) {
      i_cr = t_cr[i_cr + m];
      i_cn = t_cn[i_cn * 18 + m];
    }
    tmp[(long long)i_cr / 24 * sz_cn + i_cn] = 0;
    int base_cr = i_cr, base_cn = i_cn * 18;
    tmp[t_cr[base_cr] + t_cn[base_cn]] = 0;
    tmp[t_cr[base_cr + 1] + t_cn[base_cn + 1]] = 0;
    tmp[t_cr[base_cr + 2] + t_cn[base_cn + 2]] = 0;
  }
  DistributionPrinter dp(total);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      dp.progress(i, total, d);
      if (tmp[i] == d) {
        cnt++;
        int i_cr = (i / sz_cn) * 24;
        int i_cn = (i % sz_cn) * 18;
        for (int j = 0; j < 18; ++j) {
          long long ni = (long long)t_cr[i_cr + j] + t_cn[i_cn + j];
          // NOTE: 使用 CAS 避免竞态条件
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&tmp[ni], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  std::fill(pt.begin(), pt.end(), 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

// 2. Pair C4 + E0 (Base)
void createPTPair(int idx_ed, int idx_cn, int sz_ed, int sz_cn, int depth,
                  const std::vector<int> &t_ed, const std::vector<int> &t_cn,
                  std::vector<unsigned char> &pt) {
  long long total = (long long)sz_ed * sz_cn;
  std::vector<unsigned char> tmp(total, 255);
  std::vector<std::string> am = {"L U L'", "L U' L'", "B' U B", "B' U' B"};
  tmp[idx_ed * sz_cn + idx_cn] = 0;
  for (const auto &s : am) {
    int c_ed = idx_ed, c_cn = idx_cn;
    for (int m : string_to_alg(s)) {
      c_ed = t_ed[c_ed * 18 + m];
      c_cn = t_cn[c_cn * 18 + m];
    }
    tmp[c_ed * sz_cn + c_cn] = 0;
    for (int k = 0; k < 3; ++k) {
      int n_ed = t_ed[c_ed * 18 + k];
      int n_cn = t_cn[c_cn * 18 + k];
      tmp[n_ed * sz_cn + n_cn] = 0;
    }
  }
  DistributionPrinter dp(total);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      dp.progress(i, total, d);
      if (tmp[i] == d) {
        cnt++;
        int i_ed = (i / sz_cn) * 18;
        int i_cn = (i % sz_cn) * 18;
        for (int j = 0; j < 18; ++j) {
          long long ni = (long long)t_ed[i_ed + j] * sz_cn + t_cn[i_cn + j];
          // NOTE: 使用 CAS 避免竞态条件
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&tmp[ni], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  std::fill(pt.begin(), pt.end(), 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

void createPTCrossCE(int idx_cr, int idx_cn, int idx_ed, int sz_cr, int sz_cn,
                     int sz_ed, int depth, const std::vector<int> &t1,
                     const std::vector<int> &t2, const std::vector<int> &t3,
                     std::vector<unsigned char> &pt, bool is_pseudo) {
  long long total = (long long)sz_cr * sz_cn * sz_ed;
  std::vector<unsigned char> tmp;
  try {
    tmp.resize(total, 255);
  } catch (...) {
    std::cerr << "Alloc fail\n";
    exit(1);
  }

  int d_moves[] = {-1, 3, 4, 5};
  int num_init = is_pseudo ? 4 : 1;

  for (int m_idx = 0; m_idx < num_init; ++m_idx) {
    long long cur_cr;
    int cur_cn;
    int cur_ed = idx_ed;
    if (m_idx > 0) {
      int move = d_moves[m_idx];
      cur_cr = t1[idx_cr * 24 + move];
      cur_cn = t2[idx_cn * 18 + move] * 18;
    } else {
      cur_cr = idx_cr * 24LL;
      cur_cn = idx_cn * 18;
    }
    long long start_idx = (cur_cr + cur_cn / 18) * 24 + cur_ed;
    if (start_idx < total)
      tmp[start_idx] = 0;
  }

  DistributionPrinter dp(total);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      dp.progress(i, total, d);
      if (tmp[i] == d) {
        cnt++;
        long long comb = i / sz_ed;
        int cur_ed = i % sz_ed;
        int cur_cr = (comb / sz_cn) * 24;
        int cur_cn = (comb % sz_cn) * 18;
        int idx3_base = cur_ed * 18;
        for (int j = 0; j < 18; ++j) {
          long long n_cr = t1[cur_cr + j];
          int n_cn = t2[cur_cn + j];
          long long ni = (n_cr + n_cn) * 24 + t3[idx3_base + j];
          // NOTE: 使用 CAS 避免竞态条件
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&tmp[ni], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  std::fill(pt.begin(), pt.end(), 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

void createPTEdge6Corn2(int sz_e6, int sz_c2, int depth,
                        const std::vector<int> &target_e_ids,
                        const std::vector<int> &target_c_ids,
                        const std::vector<int> &mt_e6,
                        const std::vector<int> &mt_c2,
                        std::vector<unsigned char> &pt) {
  long long total = (long long)sz_e6 * sz_c2;
  std::cout << "  Allocating " << (total / 2 / 1024 / 1024)
            << " MB for Huge Table..." << std::endl;
  std::vector<unsigned char> tmp;
  try {
    tmp.resize(total, 255);
  } catch (...) {
    std::cerr << "Alloc fail (Huge). Need ~20GB RAM.\n";
    exit(1);
  }
  int idx_e6_solved = array_to_index(target_e_ids, 6, 2, 12);
  int idx_c2_solved = array_to_index(target_c_ids, 2, 3, 8);
  long long start_idx = (long long)idx_e6_solved * sz_c2 + idx_c2_solved;
  if (start_idx < total)
    tmp[start_idx] = 0;

  DistributionPrinter dp(total);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      dp.progress(i, total, d);
      if (tmp[i] == d) {
        cnt++;
        int cur_c2 = i % sz_c2;
        int cur_e6 = i / sz_c2;
        int base_e6 = cur_e6 * 18;
        int base_c2 = cur_c2 * 18;
        for (int j = 0; j < 18; ++j) {
          int n_e6 = mt_e6[base_e6 + j];
          int n_c2 = mt_c2[base_c2 + j];
          long long ni = (long long)n_e6 * sz_c2 + n_c2;
          // NOTE: 使用 CAS 避免竞态条件
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&tmp[ni], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  long long c_sz = (total + 1) / 2;
  pt.assign(c_sz, 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
  std::vector<unsigned char>().swap(tmp);
}

void createPTPsCrossEdge2(int idx_cr, int idx_e2, int sz_cr, int sz_e2,
                          int depth, const std::vector<int> &t_cr,
                          const std::vector<int> &t_e2,
                          std::vector<unsigned char> &pt) {
  long long total = (long long)sz_cr * sz_e2;
  std::vector<unsigned char> tmp;
  try {
    tmp.resize(total, 255);
  } catch (...) {
    std::cerr << "Alloc fail (E0E2)\n";
    exit(1);
  }

  int d_moves[] = {-1, 3, 4, 5};
  for (int m_idx = 0; m_idx < 4; ++m_idx) {
    int cur_cr = idx_cr;
    int cur_e2 = idx_e2;
    if (m_idx > 0) {
      int move = d_moves[m_idx];
      cur_cr = t_cr[idx_cr * 24 + move] / 24;
      cur_e2 = t_e2[idx_e2 * 18 + move];
    }
    long long start_idx = (long long)cur_cr * sz_e2 + cur_e2;
    if (start_idx < total)
      tmp[start_idx] = 0;
  }

  DistributionPrinter dp(total);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      dp.progress(i, total, d);
      if (tmp[i] == d) {
        cnt++;
        int cur_e2 = i % sz_e2;
        int cur_cr = i / sz_e2;
        int base_cr = cur_cr * 24;
        int base_e2 = cur_e2 * 18;
        for (int j = 0; j < 18; ++j) {
          int n_cr = t_cr[base_cr + j] / 24;
          int n_e2 = t_e2[base_e2 + j];
          long long ni = (long long)n_cr * sz_e2 + n_e2;
          // NOTE: 使用 CAS 避免竞态条件
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&tmp[ni], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  std::fill(pt.begin(), pt.end(), 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

// --- 剪枝表生成函数实现 (from eo_cross_analyzer) ---

void createPTDim2(int idx1, int idx2, int sz1, int sz2, int depth,
                  const std::vector<int> &t1, const std::vector<int> &t2,
                  std::vector<unsigned char> &pt) {
  long long total = (long long)sz1 * sz2;
  std::vector<unsigned char> tmp(total, 255);
  tmp[(long long)idx1 * sz2 + idx2] = 0;

  DistributionPrinter dp(total);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      dp.progress(i, total, d);
      if (tmp[i] == d) {
        cnt++;
        int i1 = (i / sz2) * 18, i2 = (i % sz2) * 18;
        for (int j = 0; j < 18; ++j) {
          long long ni = (long long)t1[i1 + j] * sz2 + t2[i2 + j];
          // NOTE: 使用 CAS 避免竞态条件
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&tmp[ni], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  pt.assign((total + 1) / 2, 0xFF);
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

void createPTCrossCEX(int idx_cr, int idx_cn, int idx_ed, int idx_extra,
                      int sz_cr, int sz_cn, int sz_ed, int sz_ex, int depth,
                      const std::vector<int> &t1, const std::vector<int> &t2,
                      const std::vector<int> &t3, const std::vector<int> &t4,
                      std::vector<unsigned char> &pt) {
  long long total = (long long)sz_cr * sz_cn * sz_ed * sz_ex;
  std::cout << "  Allocating " << (total / 1024 / 1024)
            << " MB for Plus Table..." << std::endl;
  std::vector<unsigned char> tmp;
  try {
    tmp.resize(total, 255);
  } catch (...) {
    std::cerr << "Alloc fail (Plus Table).\n";
    exit(1);
  }

  long long cur_cr = idx_cr * 24LL;
  int cur_cn = idx_cn * 18;
  int cur_ed = idx_ed;
  int cur_ex = idx_extra;

  long long start_idx = ((cur_cr + cur_cn / 18) * 24 + cur_ed) * 24 + cur_ex;
  if (start_idx < total)
    tmp[start_idx] = 0;

  DistributionPrinter dp(total);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      dp.progress(i, total, d);
      if (tmp[i] == d) {
        cnt++;
        long long rem = i;
        int c_ex = rem % sz_ex;
        rem /= sz_ex;
        int c_ed = rem % sz_ed;
        rem /= sz_ed;
        int c_cn = rem % sz_cn;
        rem /= sz_cn;
        long long c_mul = rem;

        int idx1_base = c_mul * 24;
        int idx2_base = c_cn * 18;
        int idx3_base = c_ed * 18;
        int idx4_base = c_ex * 18;

        for (int j = 0; j < 18; ++j) {
          long long n_cr = t1[idx1_base + j];
          int n_cn = t2[idx2_base + j];
          int n_ed = t3[idx3_base + j];
          int n_ex = t4[idx4_base + j];

          long long ni = ((n_cr + n_cn) * 24 + n_ed) * 24 + n_ex;
          // NOTE: 使用 CAS 避免竞态条件
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&tmp[ni], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  long long c_sz = (total + 1) / 2;
  pt.assign(c_sz, 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

void createPTCrossCCC(int idx_cr, int idx_cn1, int idx_cn2, int idx_cn3,
                      int sz_cr, int sz_cn1, int sz_cn2, int sz_cn3, int depth,
                      const std::vector<int> &t_cr,
                      const std::vector<int> &t_cn1,
                      const std::vector<int> &t_cn2,
                      const std::vector<int> &t_cn3,
                      std::vector<unsigned char> &pt) {
  long long total = (long long)sz_cr * sz_cn1 * sz_cn2 * sz_cn3;
  std::cout << "  Allocating " << (total / 1024 / 1024)
            << " MB for 3-Corner Table..." << std::endl;
  std::vector<unsigned char> tmp;
  try {
    tmp.resize(total, 255);
  } catch (...) {
    std::cerr << "Alloc fail (3-Corner Table).\n";
    exit(1);
  }

  long long cur_cr = idx_cr * 24LL;
  int cur_cn1 = idx_cn1 * 18;

  long long start_idx =
      ((cur_cr + cur_cn1 / 18) * sz_cn2 + idx_cn2) * sz_cn3 + idx_cn3;
  if (start_idx < total)
    tmp[start_idx] = 0;

  DistributionPrinter dp(total);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < total; ++i) {
      dp.progress(i, total, d);
      if (tmp[i] == d) {
        cnt++;
        long long rem = i;
        int c_cn3 = rem % sz_cn3;
        rem /= sz_cn3;
        int c_cn2 = rem % sz_cn2;
        rem /= sz_cn2;
        int c_cn1 = rem % sz_cn1;
        rem /= sz_cn1;
        long long c_mul = rem;

        int idx_cr_base = c_mul * 24;
        int idx_cn1_base = c_cn1 * 18;
        int idx_cn2_base = c_cn2 * 18;
        int idx_cn3_base = c_cn3 * 18;

        for (int j = 0; j < 18; ++j) {
          long long n_cr = t_cr[idx_cr_base + j];
          int n_cn1 = t_cn1[idx_cn1_base + j];
          int n_cn2 = t_cn2[idx_cn2_base + j];
          int n_cn3 = t_cn3[idx_cn3_base + j];

          long long ni = ((n_cr + n_cn1) * sz_cn2 + n_cn2) * sz_cn3 + n_cn3;
          // NOTE: 使用 CAS 避免竞态条件
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&tmp[ni], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  long long c_sz = (total + 1) / 2;
  pt.assign(c_sz, 0xFF);
#pragma omp parallel for
  for (long long i = 0; i < total; ++i)
    if (tmp[i] != 255)
      set_prune(pt, i, tmp[i]);
}

// --- Pseudo Cross/XCross/Pair 变体表生成函数 ---
// NOTE: 以下函数用于生成 C{4-7} 变体的剪枝表

// 生成 Pseudo Cross + Corner 表 (例如: prune_table_pseudo_cross_C4.bin)
// index2: corner 初始索引 (如 12=C4, 15=C5, 18=C6, 21=C7)
void createPTPsCrossCorn(int index2, int depth, const std::vector<int> &table1,
                         const std::vector<int> &table2,
                         std::vector<unsigned char> &prune_table) {
  long long size1 = StateSpace::CROSS, size2 = StateSpace::CORNER,
            size = size1 * size2;
  std::vector<unsigned char> temp_table(size, 255);
  std::vector<int> a = {16, 18, 20, 22};
  int index1 = array_to_index(a, 4, 2, 12);
  temp_table[index1 * size2 + index2] = 0;
  temp_table[table1[index1 * 24 + 3] + table2[index2 * 18 + 3]] = 0;
  temp_table[table1[index1 * 24 + 4] + table2[index2 * 18 + 4]] = 0;
  temp_table[table1[index1 * 24 + 5] + table2[index2 * 18 + 5]] = 0;

  DistributionPrinter dp(size);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < size; ++i) {
      dp.progress(i, size, d);
      if (temp_table[i] == d) {
        cnt++;
        int index1_tmp = (i / size2) * 24;
        int index2_tmp = (i % size2) * 18;
        for (int j = 0; j < 18; ++j) {
          int next_i = table1[index1_tmp + j] + table2[index2_tmp + j];
          // NOTE: 使用 CAS 避免竞态条件
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&temp_table[next_i], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  prune_table.resize((size + 1) / 2);
  std::fill(prune_table.begin(), prune_table.end(), 0xFF);
  for (long long i = 0; i < size; ++i)
    if (temp_table[i] != 255)
      set_prune(prune_table, i, temp_table[i]);
}

// 生成 Pseudo XCross 表(例如: pt_pscross_ins_C4_diff0.bin)
// index3: edge 初始索引 (0, 2, 4, 6 for E0-E3)
// index2: corner 初始索引 (12, 15, 18, 21 for C4-C7)
void createPTPsCrossInsC(int index3, int index2, int depth,
                         const std::vector<int> &table1,
                         const std::vector<int> &table2,
                         std::vector<unsigned char> &prune_table) {
  long long size1 = StateSpace::CROSS, size2 = StateSpace::CORNER,
            size = size1 * size2;
  std::vector<unsigned char> temp_table(size, 255);

  // 根据边块位置选择初始化序列
  std::vector<std::string> appl_moves;
  std::vector<int> tmp_moves;
  if (index3 == 0) {
    appl_moves = {"L U L'", "L U' L'", "B' U B", "B' U' B"};
    tmp_moves = {0, 3, 4, 5};
  }
  if (index3 == 2) {
    appl_moves = {"R' U R", "R' U' R", "B U B'", "B U' B'"};
    tmp_moves = {5, 0, 3, 4};
  }
  if (index3 == 4) {
    appl_moves = {"R U R'", "R U' R'", "F' U F", "F' U' F"};
    tmp_moves = {4, 5, 0, 3};
  }
  if (index3 == 6) {
    appl_moves = {"L' U L", "L' U' L", "F U F'", "F U' F'"};
    tmp_moves = {3, 4, 5, 0};
  }

  std::vector<int> a = {16, 18, 20, 22};
  int index1 = array_to_index(a, 4, 2, 12);
  temp_table[index1 * size2 + index2] = 0;
  temp_table[table1[index1 * 24 + 3] + table2[index2 * 18 + 3]] = 0;
  temp_table[table1[index1 * 24 + 4] + table2[index2 * 18 + 4]] = 0;
  temp_table[table1[index1 * 24 + 5] + table2[index2 * 18 + 5]] = 0;

  // 初始化所有pseudo 等效状态
  for (int i = 0; i < 4; i++) {
    int index1_tmp_2 = index1 * 24, index2_tmp_2 = index2;
    index1_tmp_2 = table1[index1_tmp_2 + tmp_moves[index2 / 3 - 4]];
    index2_tmp_2 = table2[index2_tmp_2 * 18 + tmp_moves[index2 / 3 - 4]];
    for (int m : string_to_alg(appl_moves[i])) {
      index1_tmp_2 = table1[index1_tmp_2 + m];
      index2_tmp_2 = table2[index2_tmp_2 * 18 + m];
    }
    temp_table[index1_tmp_2 + index2_tmp_2] = 0;
    temp_table[table1[index1_tmp_2 + 3] + table2[index2_tmp_2 * 18 + 3]] = 0;
    temp_table[table1[index1_tmp_2 + 4] + table2[index2_tmp_2 * 18 + 4]] = 0;
    temp_table[table1[index1_tmp_2 + 5] + table2[index2_tmp_2 * 18 + 5]] = 0;
    temp_table[table1[table1[index1_tmp_2] + 3] +
               table2[table2[index2_tmp_2 * 18] * 18 + 3]] = 0;
    temp_table[table1[table1[index1_tmp_2] + 4] +
               table2[table2[index2_tmp_2 * 18] * 18 + 4]] = 0;
    temp_table[table1[table1[index1_tmp_2] + 5] +
               table2[table2[index2_tmp_2 * 18] * 18 + 5]] = 0;
    temp_table[table1[index1_tmp_2 + 1] + table2[index2_tmp_2 * 18 + 1]] = 0;
    temp_table[table1[table1[index1_tmp_2 + 1] + 3] +
               table2[table2[index2_tmp_2 * 18 + 1] * 18 + 3]] = 0;
    temp_table[table1[table1[index1_tmp_2 + 1] + 4] +
               table2[table2[index2_tmp_2 * 18 + 1] * 18 + 4]] = 0;
    temp_table[table1[table1[index1_tmp_2 + 1] + 5] +
               table2[table2[index2_tmp_2 * 18 + 1] * 18 + 5]] = 0;
    temp_table[table1[index1_tmp_2 + 2] + table2[index2_tmp_2 * 18 + 2]] = 0;
    temp_table[table1[table1[index1_tmp_2 + 2] + 3] +
               table2[table2[index2_tmp_2 * 18 + 2] * 18 + 3]] = 0;
    temp_table[table1[table1[index1_tmp_2 + 2] + 4] +
               table2[table2[index2_tmp_2 * 18 + 2] * 18 + 4]] = 0;
    temp_table[table1[table1[index1_tmp_2 + 2] + 5] +
               table2[table2[index2_tmp_2 * 18 + 2] * 18 + 5]] = 0;
  }

  DistributionPrinter dp(size);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < size; ++i) {
      dp.progress(i, size, d);
      if (temp_table[i] == d) {
        cnt++;
        int index1_tmp = (i / size2) * 24;
        int index2_tmp = (i % size2) * 18;
        for (int j = 0; j < 18; ++j) {
          int next_i = table1[index1_tmp + j] + table2[index2_tmp + j];
          // NOTE: 使用 CAS 避免竞态条件
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&temp_table[next_i], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  prune_table.resize((size + 1) / 2);
  std::fill(prune_table.begin(), prune_table.end(), 0xFF);
  for (long long i = 0; i < size; ++i)
    if (temp_table[i] != 255)
      set_prune(prune_table, i, temp_table[i]);
}

// 生成 Pseudo Pair 表(例如: prune_table_pseudo_pair_C4_E0.bin)
// index1: edge 初始索引 (0, 2, 4, 6)
// index2: corner 初始索引 (12, 15, 18, 21)
void createPTPsPair(int index1, int index2, int size1, int size2, int depth,
                    const std::vector<int> &table1,
                    const std::vector<int> &table2,
                    std::vector<unsigned char> &prune_table) {
  long long size = (long long)size1 * size2;
  std::vector<unsigned char> temp_table(size, 255);
  long long start = (long long)index1 * size2 + index2;
  temp_table[start] = 0;
  temp_table[table1[index1 * 18 + 3] * size2 + table2[index2 * 18 + 3]] = 0;
  temp_table[table1[index1 * 18 + 4] * size2 + table2[index2 * 18 + 4]] = 0;
  temp_table[table1[index1 * 18 + 5] * size2 + table2[index2 * 18 + 5]] = 0;

  std::vector<std::string> appl_moves;
  std::vector<int> tmp_moves;
  if (index1 == 0) {
    appl_moves = {"L U L'", "L U' L'", "B' U B", "B' U' B"};
    tmp_moves = {0, 3, 4, 5};
  }
  if (index1 == 2) {
    appl_moves = {"R' U R", "R' U' R", "B U B'", "B U' B'"};
    tmp_moves = {5, 0, 3, 4};
  }
  if (index1 == 4) {
    appl_moves = {"R U R'", "R U' R'", "F' U F", "F' U' F"};
    tmp_moves = {4, 5, 0, 3};
  }
  if (index1 == 6) {
    appl_moves = {"L' U L", "L' U' L", "F U F'", "F U' F'"};
    tmp_moves = {3, 4, 5, 0};
  }

  for (int i = 0; i < 4; i++) {
    int index1_tmp_2 = index1, index2_tmp_2 = index2;
    index1_tmp_2 = table1[index1_tmp_2 * 18 + tmp_moves[index2 / 3 - 4]];
    index2_tmp_2 = table2[index2_tmp_2 * 18 + tmp_moves[index2 / 3 - 4]];
    for (int m : string_to_alg(appl_moves[i])) {
      index1_tmp_2 = table1[index1_tmp_2 * 18 + m];
      index2_tmp_2 = table2[index2_tmp_2 * 18 + m];
    }
    temp_table[index1_tmp_2 * size2 + index2_tmp_2] = 0;
    temp_table[table1[index1_tmp_2 * 18 + 3] * size2 +
               table2[index2_tmp_2 * 18 + 3]] = 0;
    temp_table[table1[index1_tmp_2 * 18 + 4] * size2 +
               table2[index2_tmp_2 * 18 + 4]] = 0;
    temp_table[table1[index1_tmp_2 * 18 + 5] * size2 +
               table2[index2_tmp_2 * 18 + 5]] = 0;
    temp_table[table1[index1_tmp_2 * 18] * size2 + table2[index2_tmp_2 * 18]] =
        0;
    temp_table[table1[table1[index1_tmp_2 * 18] * 18 + 3] * size2 +
               table2[table2[index2_tmp_2 * 18] * 18 + 3]] = 0;
    temp_table[table1[table1[index1_tmp_2 * 18] * 18 + 4] * size2 +
               table2[table2[index2_tmp_2 * 18] * 18 + 4]] = 0;
    temp_table[table1[table1[index1_tmp_2 * 18] * 18 + 5] * size2 +
               table2[table2[index2_tmp_2 * 18] * 18 + 5]] = 0;
    temp_table[table1[index1_tmp_2 * 18 + 1] * size2 +
               table2[index2_tmp_2 * 18 + 1]] = 0;
    temp_table[table1[table1[index1_tmp_2 * 18 + 1] * 18 + 3] * size2 +
               table2[table2[index2_tmp_2 * 18 + 1] * 18 + 3]] = 0;
    temp_table[table1[table1[index1_tmp_2 * 18 + 1] * 18 + 4] * size2 +
               table2[table2[index2_tmp_2 * 18 + 1] * 18 + 4]] = 0;
    temp_table[table1[table1[index1_tmp_2 * 18 + 1] * 18 + 5] * size2 +
               table2[table2[index2_tmp_2 * 18 + 1] * 18 + 5]] = 0;
    temp_table[table1[index1_tmp_2 * 18 + 2] * size2 +
               table2[index2_tmp_2 * 18 + 2]] = 0;
    temp_table[table1[table1[index1_tmp_2 * 18 + 2] * 18 + 3] * size2 +
               table2[table2[index2_tmp_2 * 18 + 2] * 18 + 3]] = 0;
    temp_table[table1[table1[index1_tmp_2 * 18 + 2] * 18 + 4] * size2 +
               table2[table2[index2_tmp_2 * 18 + 2] * 18 + 4]] = 0;
    temp_table[table1[table1[index1_tmp_2 * 18 + 2] * 18 + 5] * size2 +
               table2[table2[index2_tmp_2 * 18 + 2] * 18 + 5]] = 0;
  }

  DistributionPrinter dp(size);
  for (int d = 0; d < depth; ++d) {
    int nd = d + 1;
    long long cnt = 0;
#pragma omp parallel for reduction(+ : cnt)
    for (long long i = 0; i < size; ++i) {
      dp.progress(i, size, d);
      if (temp_table[i] == d) {
        cnt++;
        int index1_tmp = (i / size2) * 18;
        int index2_tmp = (i % size2) * 18;
        for (int j = 0; j < 18; ++j) {
          int next_i = table1[index1_tmp + j] * size2 + table2[index2_tmp + j];
          // NOTE: 使用 CAS 避免竞态条件
          unsigned char expected = 255;
          __sync_val_compare_and_swap(&temp_table[next_i], expected, nd);
        }
      }
    }
    dp.clearProgress();
    dp.print(d, cnt);
    if (cnt == 0)
      break;
  }
  dp.done();
  prune_table.resize((size + 1) / 2);
  std::fill(prune_table.begin(), prune_table.end(), 0xFF);
  for (long long i = 0; i < size; ++i)
    if (temp_table[i] != 255)
      set_prune(prune_table, i, temp_table[i]);
}
