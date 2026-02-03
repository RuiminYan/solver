/*
 * analyzer_executor.h - 统一分析器执行框架
 *
 * NOTE: 本模块封装了所有分析器的公共执行逻辑，包括：
 *       - 全局初始化
 *       - 交互式文件输入
 *       - OpenMP 并行求解
 *       - ANSI 彩色输出与进度条
 *       - CSV 输出（含表头）
 *       - 数据预览与汇总表格
 */

#ifndef ANALYZER_EXECUTOR_H
#define ANALYZER_EXECUTOR_H

#include "cube_common.h"
#include <atomic>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// --- ANSI 颜色代码 ---
// NOTE: 用于 Windows 10+ 和支持 ANSI 的终端
#define ANSI_RESET "\033[0m"
#define ANSI_CYAN "\033[36m"
#define ANSI_GREEN "\033[32m"
#define ANSI_YELLOW "\033[33m"
#define ANSI_MAGENTA "\033[35m"
#define ANSI_RED "\033[31m"

// --- 全局统计变量 ---
// NOTE: 使用原子变量保证多线程安全
namespace AnalyzerStats {
inline std::atomic<long long> globalNodes{0};
inline std::atomic<int> completedTasks{0};
inline std::atomic<bool> isSolving{false};
} // namespace AnalyzerStats

// --- 节点计数宏 ---
// NOTE: 用于搜索函数中统计节点数，使用线程局部变量减少原子操作开销
// 使用方法: 在搜索循环开头调用 COUNT_NODE
#define COUNT_NODE                                                             \
  static thread_local int local_node_counter = 0;                              \
  local_node_counter++;                                                        \
  if (local_node_counter >= 1000) {                                            \
    AnalyzerStats::globalNodes.fetch_add(local_node_counter,                   \
                                         std::memory_order_relaxed);           \
    local_node_counter = 0;                                                    \
  }

// --- 辅助函数：格式化数字（千位分隔符）---
inline std::string formatWithCommas(long long value) {
  std::string result = std::to_string(value);
  int insertPosition = result.length() - 3;
  while (insertPosition > 0) {
    result.insert(insertPosition, ",");
    insertPosition -= 3;
  }
  return result;
}

// --- 辅助函数：智能时间格式化 ---
inline std::string formatDuration(double seconds) {
  if (seconds < 60) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << seconds << "s";
    return oss.str();
  }
  int totalSeconds = static_cast<int>(seconds);
  int hours = totalSeconds / 3600;
  int minutes = (totalSeconds % 3600) / 60;
  int secs = totalSeconds % 60;

  std::ostringstream oss;
  if (hours > 0) {
    oss << hours << "h " << minutes << "m " << secs << "s";
  } else {
    oss << minutes << "m " << secs << "s";
  }
  return oss.str();
}

// --- 辅助函数：光标控制 ---
#ifdef _WIN32
#include <windows.h>

// 辅助函数：设置光标可见性
inline void setCursorVisibility(bool visible) {
  HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
  CONSOLE_CURSOR_INFO cursorInfo;
  GetConsoleCursorInfo(hConsole, &cursorInfo);
  cursorInfo.bVisible = visible;
  SetConsoleCursorInfo(hConsole, &cursorInfo);
}
#else
inline void setCursorVisibility(bool visible) { (void)visible; }
#endif

// --- 辅助函数：格式化内存大小 ---
inline std::string formatMemory(size_t bytes) {
  double gb = bytes / (1024.0 * 1024.0 * 1024.0);
  double mb = bytes / (1024.0 * 1024.0);
  std::ostringstream oss;
  if (gb >= 1.0) {
    oss << std::fixed << std::setprecision(2) << gb << " GB";
  } else {
    oss << std::fixed << std::setprecision(0) << mb << " MB";
  }
  return oss.str();
}

// --- 辅助函数：打印数据预览（前 N 行）---
inline void printDataPreview(const std::string &filename, int lines = 6) {
  std::ifstream file(filename);
  if (!file)
    return;

  std::cout << ANSI_BLUE << "[DATA] Preview:" << ANSI_RESET << std::endl;
  std::string line;
  int count = 0;
  while (count < lines && std::getline(file, line)) {
    // 截断过长的行
    if (line.length() > 80) {
      line = line.substr(0, 77) + "...";
    }
    std::cout << "  " << line << std::endl;
    count++;
  }
}

// --- 辅助函数：打印汇总表格 ---
inline void printSummaryTable(int totalTasks, const std::string &outputFile,
                              long long totalNodes, size_t ramUsage,
                              double avgNps, double totalDuration) {
  std::cout << std::endl;
  std::cout << "+----------------------------------------------------------+"
            << std::endl;
  std::cout << "|                   ANALYSIS SUMMARY                       |"
            << std::endl;
  std::cout << "+----------------------------------------------------------+"
            << std::endl;
  std::cout << "| Total Tasks      : " << std::left << std::setw(37)
            << totalTasks << "|" << std::endl;
  std::cout << "| Output File      : " << std::left << std::setw(37)
            << outputFile << "|" << std::endl;
  std::cout << "| Total Nodes      : " << std::left << std::setw(37)
            << formatWithCommas(totalNodes) << "|" << std::endl;
  std::cout << "| Ram Usage        : " << std::left << std::setw(37)
            << formatMemory(ramUsage) << "|" << std::endl;

  std::ostringstream npsStr;
  npsStr << std::fixed << std::setprecision(2) << avgNps << " M/s";
  std::cout << "| " << ANSI_MAGENTA << "Avg Performance  : " << std::left
            << std::setw(37) << npsStr.str() << ANSI_RESET << "|" << std::endl;
  std::cout << "| " << ANSI_GREEN << "Total Duration   : " << std::left
            << std::setw(37) << formatDuration(totalDuration) << ANSI_RESET
            << "|" << std::endl;
  std::cout << "+----------------------------------------------------------+"
            << std::endl;
}

/*
 * run_analyzer_app - 统一分析器执行模板
 *
 * 模板参数 SolverT 必须满足以下隐式接口：
 *   - static void global_init();          // 全局初始化（加载表等）
 *   - static std::string get_csv_header(); // 返回 CSV 表头（不含换行）
 *   - static void print_stats();           // 可选，打印额外统计
 *   - SolverT();                           // 线程局部构造
 *   - std::string solve(const std::vector<int>& alg, const std::string& id);
 *                                          // 求解单个任务，返回 CSV
 * 行（不含换行）
 *
 * 参数:
 *   suffix - 输出文件后缀，如 "_std", "_pair"
 */
template <typename SolverT> void run_analyzer_app(const std::string &suffix) {
  // 1. 全局初始化
  SolverT::global_init();

  int numThreads = 1;
#pragma omp parallel
  {
#pragma omp single
    numThreads = omp_get_num_threads();
  }
  // NOTE: 显示已加载表的总大小，而不是系统内存
  std::cout << "       RAM: " << formatMemory(g_loadedTableBytes.load())
            << " | Threads: " << numThreads << " | Done." << std::endl;

  // 2. 主循环
  while (true) {
    std::string inputFilename;
    std::cout << std::endl << "Enter file (or exit): ";
    if (!(std::cin >> inputFilename) || inputFilename == "exit") {
      break;
    }

    // 读取任务文件
    std::vector<std::pair<std::string, std::vector<int>>> tasks;
    std::ifstream infile(inputFilename);
    if (!infile) {
      std::cout << ANSI_RED << "[ERROR] File '" << inputFilename
                << "' not found!" << ANSI_RESET << std::endl;
      continue;
    }

    std::string line;
    while (std::getline(infile, line)) {
      if (line.empty())
        continue;
      if (line.back() == '\r')
        line.pop_back();

      size_t p = line.find(',');
      if (p != std::string::npos) {
        tasks.push_back({line.substr(0, p), string_to_alg(line.substr(p + 1))});
      } else {
        tasks.push_back(
            {std::to_string(tasks.size() + 1), string_to_alg(line)});
      }
    }
    infile.close();

    if (tasks.empty()) {
      std::cout << ANSI_YELLOW << "[WARN] No tasks found in file." << ANSI_RESET
                << std::endl;
      continue;
    }

    // 生成输出文件名
    std::string basename = inputFilename;
    size_t dotPos = basename.rfind('.');
    if (dotPos != std::string::npos) {
      basename = basename.substr(0, dotPos);
    }
    std::string outputFilename = basename + suffix + ".csv";

    std::cout << "Output file: " << ANSI_YELLOW << outputFilename << ANSI_RESET
              << std::endl;
    std::cout << "Loaded " << tasks.size() << " tasks. Solving..." << std::endl;

    // 准备输出
    std::ofstream outfile(outputFilename);
    outfile << SolverT::get_csv_header() << "\n";

    // 初始化统计
    auto startTime = std::chrono::high_resolution_clock::now();
    int total = static_cast<int>(tasks.size());
    std::vector<std::string> resultBuffer(total);
    std::vector<bool> resultReady(total, false);
    int nextWriteIdx = 0;

    AnalyzerStats::globalNodes = 0;
    AnalyzerStats::completedTasks = 0;
    AnalyzerStats::isSolving = true;

    // 隐藏光标以获得更好的视觉体验
    setCursorVisibility(false);

    // 启动监视线程（进度条 + 性能）
    std::thread monitorThread([&]() {
      auto t0 = std::chrono::high_resolution_clock::now();
      while (AnalyzerStats::isSolving.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        auto t1 = std::chrono::high_resolution_clock::now();
        double dt = std::chrono::duration<double>(t1 - t0).count();
        long long nodes =
            AnalyzerStats::globalNodes.load(std::memory_order_relaxed);
        int completed =
            AnalyzerStats::completedTasks.load(std::memory_order_relaxed);
        double nps = (dt > 0.001) ? nodes / dt / 1000000.0 : 0;
        double progress = (total > 0) ? (double)completed * 100.0 / total : 0;

        // 构建进度条
        int barWidth = 30;
        int filled = static_cast<int>(progress / 100.0 * barWidth);
        std::string bar(filled, '#');
        bar += std::string(barWidth - filled, '-');

        // ANSI 输出
        // NOTE: 完全使用 C 风格字符数组，避免任何 std::string 内存分配问题
        char etaBuf[64];
        if (nps > 0 && completed > 0 && completed < total) {
          double etaSec = dt * (total - completed) / completed;
          if (etaSec < 60) {
            snprintf(etaBuf, sizeof(etaBuf), "%.1fs", etaSec);
          } else {
            int totalSec = static_cast<int>(etaSec);
            int h = totalSec / 3600;
            int m = (totalSec % 3600) / 60;
            int s = totalSec % 60;
            if (h > 0) {
              snprintf(etaBuf, sizeof(etaBuf), "%dh %dm %ds", h, m, s);
            } else {
              snprintf(etaBuf, sizeof(etaBuf), "%dm %ds", m, s);
            }
          }
        } else {
          snprintf(etaBuf, sizeof(etaBuf), "...");
        }

        // ANSI 清除行 + 输出
        // NOTE: \033[2K 清除整行，避免旧内容残留
        std::cout << "\033[2K" << ANSI_YELLOW << "[PROG] [" << bar << "] "
                  << std::fixed << std::setprecision(1) << progress << "% ("
                  << completed << "/" << total << ")" << ANSI_RESET << "\n";
        std::cout << "\033[2K" << ANSI_MAGENTA
                  << "       Performance: " << std::fixed
                  << std::setprecision(2) << nps << " M/s | ETA: " << etaBuf
                  << ANSI_RESET << "\r\033[A" << std::flush;
      }
    });

// 3. 并行求解
#pragma omp parallel
    {
      SolverT solver; // 线程局部初始化

#pragma omp for schedule(dynamic, 1)
      for (int i = 0; i < total; ++i) {
        std::string result = solver.solve(tasks[i].second, tasks[i].first);
        AnalyzerStats::completedTasks.fetch_add(1, std::memory_order_relaxed);

// 顺序写入结果 - 所有对 resultBuffer/resultReady 的访问都在 critical section 中
#pragma omp critical
        {
          resultBuffer[i] = result;
          resultReady[i] = true;
          while (nextWriteIdx < total && resultReady[nextWriteIdx]) {
            outfile << resultBuffer[nextWriteIdx] << "\n";
            nextWriteIdx++;
          }
        }
      }
    }

    // 停止监视线程
    AnalyzerStats::isSolving = false;
    monitorThread.join();

    // 恢复光标
    setCursorVisibility(true);

    // 清除进度条残留
    printf("\033[2K\033[A\033[2K");

    auto endTime = std::chrono::high_resolution_clock::now();
    double totalDuration =
        std::chrono::duration<double>(endTime - startTime).count();
    long long totalNodes = AnalyzerStats::globalNodes.load();
    double avgNps =
        (totalDuration > 0.001) ? totalNodes / totalDuration / 1000000.0 : 0;
    // NOTE: 显示已加载表的总大小
    size_t ramUsage = g_loadedTableBytes.load();

    // 输出完成信息
    std::cout << TAG_COLOR << "[SUCCESS]" << ANSI_RESET
              << " Processing complete!" << ANSI_RESET << std::endl;

    outfile.close();

    // 数据预览
    printDataPreview(outputFilename, 6);

    // 汇总表格
    printSummaryTable(total, outputFilename, totalNodes, ramUsage, avgNps,
                      totalDuration);
  }

  // 4. 最终统计（可选）
  SolverT::print_stats();
}

#endif // ANALYZER_EXECUTOR_H
