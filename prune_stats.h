#pragma once
#include <atomic>
#include <cstdio>

// ========== 开关配置 ==========
#define ENABLE_PRUNE_STATS 0 // 总开关 (1=开, 0=关)

#define ENABLE_STATS_S1 0 // S1: cross 89.9%, pair 24.2%
#define ENABLE_STATS_S2 0 // S2: xcross 89.9% → cross 15.9% → pair 11.5%
#define ENABLE_STATS_S3 0 // S3: huge 91.0% → cross 9.3% → pair 7.6%
#define ENABLE_STATS_S4                                                        \
  0 // S4: huge1 75% → huge2 53.4% → huge3 34.8% → pair 1% → cross 0.9%

// ========== 命名规范 ==========
// _c = checked (检查次数)
// _p = pruned  (剪枝次数)
// 剪枝率 = _p / _c * 100%

// ========== 宏定义 ==========
#if ENABLE_PRUNE_STATS

// 声明统计变量对: xxx_c (检查次数), xxx_p (剪枝次数)
#define STAT_DECL(name)                                                        \
  std::atomic<long long> name##_c{0}, name##_p { 0 }

// 内部辅助宏: 条件编译执行
#define _STAT_IF(cond, code)                                                   \
  do {                                                                         \
    if constexpr (cond) {                                                      \
      code;                                                                    \
    }                                                                          \
  } while (0)

// 各阶段统计宏
#define S1_CHECK(n) _STAT_IF(ENABLE_STATS_S1, ++n##_c) // S1: 记录检查
#define S1_HIT(n) _STAT_IF(ENABLE_STATS_S1, ++n##_p)   // S1: 记录剪枝
#define S2_CHECK(n) _STAT_IF(ENABLE_STATS_S2, ++n##_c)
#define S2_HIT(n) _STAT_IF(ENABLE_STATS_S2, ++n##_p)
#define S3_CHECK(n) _STAT_IF(ENABLE_STATS_S3, ++n##_c)
#define S3_HIT(n) _STAT_IF(ENABLE_STATS_S3, ++n##_p)
#define S4_CHECK(n) _STAT_IF(ENABLE_STATS_S4, ++n##_c)
#define S4_HIT(n) _STAT_IF(ENABLE_STATS_S4, ++n##_p)

// 输出统计结果: "名称: 剪枝数/检查数 (剪枝率%)"
#define PRINT_STAT(n)                                                          \
  printf(#n ": %lld/%lld (%.1f%%)\n", n##_p.load(), n##_c.load(),              \
         n##_c ? 100.0 * n##_p / n##_c : 0)

#else
// 关闭时所有宏展开为空操作
#define STAT_DECL(name)
#define S1_CHECK(n)
#define S1_HIT(n)
#define S2_CHECK(n)
#define S2_HIT(n)
#define S3_CHECK(n)
#define S3_HIT(n)
#define S4_CHECK(n)
#define S4_HIT(n)
#define PRINT_STAT(n)
#endif
