# Analyzer 通用优化 TODO

适用于所有 analyzer (`std_analyzer`, `pseudo_analyzer`, `pseudo_pair_analyzer`, `pair_analyzer`, `eo_cross_analyzer`)。

## 已验证有效的优化模式

### 1. ~~Early Exit（跨 slot 组合的搜索深度上界）~~ ✅ 已完成

- **原理**: `*_analyze` 函数对多种 (slot, pslot) 组合调用 `start_search_*`。用 `stage_results` 已知最优解限制后续搜索深度 (`min(固定上界, best-1)`)，heuristic ≥ best 时直接跳过
- **实施要点**:
  1. 移除 `results[]` 中间数组，直接更新 `stage_results`
  2. 搜索前读取 `stage_results.min_*[r]` 作为动态上界
  3. 排序后的 tasks 循环中加 `if (heuristic >= cur_best) continue`
- **已验证**: `pseudo_pair_analyzer` 实测 **10.5x 加速**（40亿节点→3亿）

### ~~2. 删除无用搜索参数~~ ✅ 已完成

- **原理**: 部分 `search_*` 函数接收的 prune table 参数实际剪枝率为 0%（被更强的剪枝完全覆盖）
- **实施要点**: 通过 `prune_stats.h` 统计各剪枝表命中率，移除 0% 的参数
- **风险**: 低，纯代码清理

### ~~3. 分支选择优化（三元条件→数组索引）~~ ❌ 不执行

- **原理**: `search_*` 内部的 `diff == 0 ? e0 : diff == 1 ? e1 : ...` 改为 `e_arr[diff]`
- **适用范围**: 所有使用 Conj 状态追踪的 search 函数
- **不执行原因**: 改动量大（3 函数 × 4 处）、递归调用需额外构建 `*18` 数组反而更复杂、风险回报比差


### ~~5. 预计算互补槽位~~ ✅ 已完成

- **原理**: `*_analyze` 中用 `static constexpr` 查找表替代动态计算的互补集合
- **适用范围**: `xxxcross_analyze`, `xxxxcross_analyze` 的 slot 组合枚举
- **风险**: 低

### ~~8. 跨阶段 Early Exit（级联搜索深度下界）~~ ✅ 已完成

- **原理**: 级联搜索中，更多约束只增不减步数（如 XXCross ≥ XCross）。用前一阶段的 best 做后一阶段的搜索起始深度下界
- **适用范围**: 任何具有级联搜索阶段的 analyzer（如 XCross → XXCross → XXXCross → XXXXCross）
- **已验证**: `eo_cross_analyzer` 实测效果极小（heuristic 通常已 ≥ 前阶段 best）

### ~~9. 配对内 best 共享（对称输出剪枝）~~ ✅ 已完成

- **原理**: 若最终输出取 `min(res[2c], res[2c+1])`，则偶数 sym 算完后，奇数 sym 只需搜索比它更优的解，可用偶数 sym 结果做搜索上界
- **适用范围**: 任何对 rotation 配对取 min 输出的 analyzer
- **已验证**: `eo_cross_analyzer` 实测 13s → 8.6s，节点数减少 ~30%
- **教训**: 不能跨所有 sym 共享 best（各 sym 独立输出），只能在配对内单向共享

## 未验证/高风险方向

### ~~6. 并行化（多线程搜索）~~ ❌ 不执行

- **原理**: 多线程搜索不同 rotation/slot 组合
- **预期收益**: 接近线性加速（N核≈Nx）
- **不执行原因**: 批量分析场景下 `analyzer_executor.h` 外层 OpenMP 已达 100% CPU 利用率，内层并行无收益且引入线程竞争。仅单打乱交互式求解有价值，但非主要使用场景

### 7. 更强的剪枝表

- **原理**: 增加新 pruning table 提高 search 内部剪枝率
- **参考**: `std_analyzer` 使用 Huge 表 (`C4C5E0E1`, `C4C6E0E2`)，剪枝效果远强于 Base 表
- **实施要点**:
  - 用 `prune_stats.h` 统计现有表剪枝率，确定瓶颈
  - 分析变体下可构造的状态组合
  - 权衡表大小与内存占用
- **风险**: 高（需要理论分析）
