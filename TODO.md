# Analyzer 通用优化 TODO

适用于所有 analyzer (`std_analyzer`, `pseudo_analyzer`, `pseudo_pair_analyzer`, `pair_analyzer`, `eo_cross_analyzer`)。

## 已验证有效的优化模式

### 1. Early Exit（跨 slot 组合的搜索深度上界）⭐ 收益最大

- **原理**: `*_analyze` 函数对多种 (slot, pslot) 组合调用 `start_search_*`。用 `stage_results` 已知最优解限制后续搜索深度 (`min(固定上界, best-1)`)，heuristic ≥ best 时直接跳过
- **实施要点**:
  1. 移除 `results[]` 中间数组，直接更新 `stage_results`
  2. 搜索前读取 `stage_results.min_*[r]` 作为动态上界
  3. 排序后的 tasks 循环中加 `if (heuristic >= cur_best) continue`
- **已验证**: `pseudo_pair_analyzer` 实测 **10.5x 加速**（40亿节点→3亿）

### 2. 删除无用搜索参数

- **原理**: 部分 `search_*` 函数接收的 prune table 参数实际剪枝率为 0%（被更强的剪枝完全覆盖）
- **实施要点**: 通过 `prune_stats.h` 统计各剪枝表命中率，移除 0% 的参数
- **风险**: 低，纯代码清理

### 3. 分支选择优化（三元条件→数组索引）

- **原理**: `search_*` 内部的 `diff == 0 ? e0 : diff == 1 ? e1 : ...` 改为 `e_arr[diff]`
- **适用范围**: 所有使用 Conj 状态追踪的 search 函数
- **风险**: 低

### 4. 容器降级（vector→C array）

- **原理**: 热路径中固定长度的小 vector 改为栈上 C array，避免堆分配
- **适用范围**: `setup_aux_pruners_*` 中的 keys 等
- **风险**: 低

### 5. 预计算互补槽位

- **原理**: `*_analyze` 中用 `static constexpr` 查找表替代动态计算的互补集合
- **适用范围**: `xxxcross_analyze`, `xxxxcross_analyze` 的 slot 组合枚举
- **风险**: 低

## 未验证/高风险方向

### 6. 并行化（多线程搜索）

- **原理**: 多线程搜索不同 rotation/slot 组合
- **预期收益**: 接近线性加速（N核≈Nx）
- **实施要点**:
  - `stage_results` 需原子操作或加锁
  - `search_*` 使用的成员变量需改为局部变量，或每线程一个 solver 实例
  - Early exit 的 `stage_results` 读取需线程安全
  - `COUNT_NODE` 全局计数器需原子操作
- **风险**: 高（改动大，数据竞争）
- **⚠️ 适用场景限制**:
  - **批量分析（上千打乱）**: `analyzer_executor.h` 的外层 OpenMP per-task 并行已达 100% CPU 利用率，**内层并行无收益**（反而引入线程竞争）
  - **单打乱交互式求解**: 仅此场景下内层并行有价值（1 task 时外层并行无法利用多核）
  - **结论**: 当前主要使用场景（批量分析）下此项优化**不适用**，可标记为低优先级

### 7. 更强的剪枝表

- **原理**: 增加新 pruning table 提高 search 内部剪枝率
- **参考**: `std_analyzer` 使用 Huge 表 (`C4C5E0E1`, `C4C6E0E2`)，剪枝效果远强于 Base 表
- **实施要点**:
  - 用 `prune_stats.h` 统计现有表剪枝率，确定瓶颈
  - 分析变体下可构造的状态组合
  - 权衡表大小与内存占用
- **风险**: 高（需要理论分析）
