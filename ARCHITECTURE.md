# 项目架构

三阶魔方 Cross/F2L 阶段最优解分析器框架，使用 IDA* 搜索 + 预计算表实现。

## 代码分层

```
┌───────────────────────────────────────────────────────────────┐
│                      应用层 (各 Analyzer)                      │
│  std_analyzer  pair_analyzer  pseudo_analyzer  eo_cross_...   │
│  pseudo_pair_analyzer        table_generator   mirror_verify  │
├───────────────────────────────────────────────────────────────┤
│                  执行框架层 (Executor)                          │
│  analyzer_executor.h   统一的输入/并行/输出/统计框架             │
├───────────────────────────────────────────────────────────────┤
│                     数据层 (Manager)                           │
│  MoveTableManager          PruneTableManager                  │
│  (move_tables.h/cpp)       (prune_tables.h/cpp)               │
├───────────────────────────────────────────────────────────────┤
│                     基础层 (Common)                            │
│  cube_common.h/cpp                                            │
│  State定义 · 索引转换 · 移动表引擎 · 文件I/O · 状态空间常量     │
└───────────────────────────────────────────────────────────────┘
```

> [!TIP]
> 阅读顺序建议：从底层往上读，先理解 `cube_common`，再看两个 Manager，最后看 Analyzer。

---

## 文件说明

### 基础层

| 文件 | 职责 |
|------|------|
| `cube_common.h` | 全局类型与接口：`State` 结构体、18种操作的定义 (`moves_map`)、排列/朝向的索引转换函数、移动表引擎 (`createMultiMoveTable`/`createMultiMoveTable2`)、文件 I/O 模板、`StateSpace` 常量命名空间 |
| `cube_common.cpp` | 上述声明的实现：操作矩阵、索引编解码、旋转映射 (`rot_map`, `conj_moves_flat`, `sym_moves_flat`) |

### 数据层

| 文件 | 职责 |
|------|------|
| `move_tables.h/cpp` | `MoveTableManager` 单例：管理 12 种移动表 (`.bin`) 的生成、加载、释放。提供细粒度资源控制（按需加载/用完释放），`Edge6` 约 3GB，需特殊处理 |
| `prune_tables.h/cpp` | `PruneTableManager` 单例：管理所有剪枝表的生成和加载。包含 15+ 种 `create*` 引擎函数和 60+ 种 `gen*` 生成函数。使用 4-bit 压缩存储，BFS 填充 |
| `prune_stats.h` | 剪枝效率统计宏（可开关），用于调优各阶段剪枝策略的命中率 |
| `cross_solver.h` | Cross 搜索器（Std/Pseudo 共享），通过 `isPseudo` 参数区分标准/伪 Cross 剪枝表。EO 版因维度差异独立定义 |

### 执行框架层

| 文件 | 职责 |
|------|------|
| `analyzer_executor.h` | 模板函数 `run_analyzer_app<SolverT>()` 封装公共执行逻辑：stdin 交互式输入、打乱序列解析、OpenMP 并行求解、CSV 输出、实时进度和汇总统计。所有 Analyzer 的 `main()` 只需一行调用 |

### 应用层 — 分析器

每个 Analyzer 是一个独立的 `.cpp` + `main()` 入口，编译为独立可执行文件。

| 文件 | 编译产物 | 功能 |
|------|----------|------|
| `std_analyzer.cpp` | `std_analyzer.exe` | Cross, XCross, XXCross, XXXCross, XXXXCross 最优步数 |
| `pair_analyzer.cpp` | `pair_analyzer.exe` | Cross+Pair, XCross+Pair, XXCross+Pair, XXXCross+Pair |
| `pseudo_analyzer.cpp` | `pseudo_analyzer.exe` | Pseudo Cross/XCross/XXCross/XXXCross（允许 D 层偏移） |
| `pseudo_pair_analyzer.cpp` | `pseudo_pair_analyzer.exe` | Pseudo Cross+Pseudo Pair 系列 |
| `eo_cross_analyzer.cpp` | `eo_cross_analyzer.exe` | EO+Cross/XCross/XXCross/XXXCross/XXXXCross（含棱块色向约束） |

### 应用层 — 工具

| 文件 | 编译产物 | 功能 |
|------|----------|------|
| `table_generator.cpp` | `table_generator.exe` | 离线预生成所有移动表和剪枝表 |
| `mirror_verify.cpp` | `mirror_verify.exe` | 验证剪枝表间的对称关系（用于探索表压缩） |

---

## 编译产物与依赖

所有可执行文件共享相同的三个公共 `.o`：

```
cube_common.o ──┐
move_tables.o ──┼──> xxx_analyzer.exe
prune_tables.o ─┘
```

编译命令统一为：
```
g++ -std=c++17 -O3 -fopenmp -Wall -Wextra
```

构建脚本：`build.ps1`（编译全部）、`clean.ps1`（清理编译产物）

---

## 核心数据流

```
打乱字符串
    │
    ▼
string_to_alg() 解析为 move 序列
    │
    ▼
State + 旋转变换 → 计算初始索引
    │
    ▼
┌─────────────────────────────────┐
│  IDA* 外层循环 (迭代加深)        │
│                                 │
│  for maxDepth = 0, 1, 2, ...    │
│    │                            │
│    ▼                            │
│  ┌────────────────────────┐     │
│  │ DFS 搜索 (深度 ≤ max)  │     │
│  │                        │     │
│  │ 每步:                  │     │
│  │  1. 查移动表更新索引   │     │
│  │  2. 查剪枝表得到下界   │     │
│  │  3. 下界>剩余深度→剪枝 │     │
│  └──────────┬─────────────┘     │
│             │                   │
│        找到解? ──否──► maxDepth++│
│             │             (回到 │
│            是              外层)│
│             │                   │
└─────────────┼───────────────────┘
              ▼
        输出最优步数 → CSV 文件
```

**关键概念：**

- **移动表 (Move Table, MT)**：预计算状态转移。给定状态索引和操作编号，O(1) 查出新状态索引
- **剪枝表 (Prune Table, PT)**：预计算启发值。给定状态索引，O(1) 查出到目标的最少步数下界
- **IDA\***：迭代加深 A\* 搜索，每轮设定深度上限，利用剪枝表提供的 admissible heuristic 剪枝

---

## Solver 内部结构模式

每个 Analyzer 内部的 Solver 类遵循相同模式：

```
struct XXXSolver {
    // 1. 移动表/剪枝表指针（从 Manager 获取）
    const int* p_mt_xxx;
    const unsigned char* p_pt_xxx;

    // 2. 构造函数：从 Manager 获取指针
    XXXSolver() { ... }

    // 3. 分阶段搜索函数
    search_1(...)    // 1-slot: Cross + 1 F2L pair
    search_2(...)    // 2-slot: 增加 Huge 表 + Edge6/Corn2 剪枝
    search_3(...)    // 3-slot: 多视角剪枝
    search_4(...)    // 4-slot: 全槽位

    // 4. 入口函数：遍历旋转方向，启动 IDA*
    get_stats(alg, rotations) → vector<int>
};

// 5. Wrapper 适配 run_analyzer_app 模板接口
struct XXXSolverWrapper {
    static void global_init();    // 加载所需的表
    string get_csv_header();      // CSV 表头
    string solve(alg, id);        // 求解单个打乱
};

int main() { run_analyzer_app<XXXSolverWrapper>("_suffix"); }
```

---

## 共轭优化 (Conjugation)

项目的核心优化策略——只存储一份"规范表"，其他对称等价的表通过共轭映射复用：

```
物理操作 m
    │
    ▼
conj_moves_flat[m][slot_k]    ← 将操作 m 变换到 slot_k 的视角
    │
    ▼
查询 规范表 (Canon)            ← 只存 C4/E0 基准的表
    │
    ▼
得到 slot_k 视角的剪枝值
```

目录中可见的三类表文件：
- **Canon**：实际加载到内存的规范表
- **Conj**：不加载，运行时通过 `rot_map` / `conj_moves_flat` 映射到 Canon 表
- **Zombie**：已生成但不使用，仅用于调试验证

---

## 补充文档索引

| 文档 | 内容 |
|------|------|
| `README.md` | 数学定义（块/槽位/各分析器的搜索目标）、编译与测试方法、正确结果 |
| `table_naming.csv` | 所有表的完整注册表：文件名、变量名、generator、各 Analyzer 的引用、加载状态、大小 |
| `create_functions.csv` | 所有 `create*` 引擎函数文档：状态空间、维度公式、移动表依赖、调用者 |
