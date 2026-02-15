/*
 * logo.h - CUBEROOT Logo 渲染
 *
 * NOTE: 像素艺术字体 + ANSI 渐变色输出，header-only 实现
 */

#ifndef LOGO_H
#define LOGO_H

#include <iostream>

#ifdef _WIN32
#include <windows.h>
#endif

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
// NOTE: 使用 Unicode █ 字符 + 双线阴影 (Draw Box Chars) 实现，24-bit 赤陶色
inline void printCuberootLogoBlock() {
  // NOTE: Windows 默认 codepage 无法显示 Unicode █/║/╗，强制切换为 UTF-8
#ifdef _WIN32
  UINT origCp = GetConsoleOutputCP();
  SetConsoleOutputCP(CP_UTF8);
#endif

  // Claude Code 赤陶配色（24-bit true color: #D97757）
  const char *color = "\033[38;2;217;119;87m";
  const char *reset = "\033[0m";

  // CUBE
  const char *cubeLine[] = {" ██████╗ ██╗   ██╗ ██████╗ ██████╗",
                            "██╔════╝ ██║   ██║ ██╔══██╗██╔════╝",
                            "██║      ██║   ██║ ██████╔╝█████╗  ",
                            "██║      ██║   ██║ ██╔══██╗██╔══╝  ",
                            "╚██████╗ ╚██████╔╝ ██████╔╝██████╗ ",
                            " ╚═════╝  ╚═════╝  ╚═════╝ ╚═════╝ "};

  // ROOT
  const char *rootLine[] = {"██████╗  ██████╗  ██████╗ ████████╗",
                            "██╔══██╗██╔═══██╗██╔═══██╗╚══██╔══╝",
                            "██████╔╝██║   ██║██║   ██║   ██║   ",
                            "██╔══██╗██║   ██║██║   ██║   ██║   ",
                            "██║  ██║╚██████╔╝╚██████╔╝   ██║   ",
                            "╚═╝  ╚═╝ ╚═════╝  ╚═════╝    ╚═╝   "};

  std::cout << std::endl;
  for (const char *line : cubeLine) {
    std::cout << color << line << reset << std::endl;
  }
  std::cout << std::endl;
  for (const char *line : rootLine) {
    std::cout << color << line << reset << std::endl;
  }
  std::cout << std::endl;

#ifdef _WIN32
  SetConsoleOutputCP(origCp);
#endif
}

#endif // LOGO_H
