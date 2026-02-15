/*
 * logo.h - CUBEROOT Logo 渲染
 *
 * NOTE: 像素艺术字体 + ANSI 渐变色输出，header-only 实现
 */

#ifndef LOGO_H
#define LOGO_H

#include <iostream>

// 打印CUBEROOT Logo（像素艺术+渐变色，类似GEMINI风格）
inline void printCuberootLogo() {
  // 渐变色：从蓝紫到粉红（类似GEMINI风格）
  const char *gradients[] = {
      "\033[38;5;63m",  // 蓝紫色
      "\033[38;5;99m",  // 紫色
      "\033[38;5;135m", // 淡紫色
      "\033[38;5;171m", // 粉紫色
      "\033[38;5;207m", // 粉色
      "\033[38;5;213m"  // 淡粉色
  };

  // CUBEROOT LOGO 精美像素艺术字体（使用ASCII字符@，兼容所有终端）
  const char *lines[] = {
      " @@@@   @    @  @@@@@   @@@@  @@@@@    @@@@    @@@@   @@@@@@",
      "@@  @@  @    @  @    @  @     @    @  @@  @@  @@  @@    @@  ",
      "@@      @    @  @@@@@   @@@@  @@@@@   @    @  @    @    @@  ",
      "@@      @    @  @    @  @     @  @    @    @  @    @    @@  ",
      "@@  @@  @    @  @    @  @     @   @   @@  @@  @@  @@    @@  ",
      " @@@@    @@@@   @@@@@   @@@@  @    @   @@@@    @@@@     @@  "};

  std::cout << std::endl;
  for (int i = 0; i < 6; ++i) {
    std::cout << gradients[i] << lines[i] << "\033[0m" << std::endl;
  }
  std::cout << std::endl;
}

// 打印CUBEROOT Logo（方块像素风格，类似Claude Code风格）
// NOTE: 使用 Unicode █ 字符填充，暖橙/赤陶色配色
inline void printCuberootLogoBlock() {
  // 暖橙色（与 Claude Code 赤陶/橙色风格一致）
  const char *color = "\033[38;5;208m";
  const char *reset = "\033[0m";

  // CUBE — 第一行（每个字母 6 列宽 × 6 行高，间距 2 列）
  const char *cubeLine[] = {
      "  █████   █    █  ██████  ██████", " █        █    █  █    █  █     ",
      " █        █    █  █████   ████  ", " █        █    █  █    █  █     ",
      " █        █    █  █    █  █     ", "  █████   ██████  ██████  ██████"};

  // ROOT — 第二行
  const char *rootLine[] = {
      " ██████   █████    █████   ██████", " █    █  █    █   █    █    ██   ",
      " █████   █    █   █    █    ██   ", " █  █    █    █   █    █    ██   ",
      " █   █   █    █   █    █    ██   ", " █    █   █████    █████    ██   "};

  std::cout << std::endl;
  for (int i = 0; i < 6; ++i) {
    std::cout << color << cubeLine[i] << reset << std::endl;
  }
  std::cout << std::endl;
  for (int i = 0; i < 6; ++i) {
    std::cout << color << rootLine[i] << reset << std::endl;
  }
  std::cout << std::endl;
}

#endif // LOGO_H
