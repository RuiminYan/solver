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

#endif // LOGO_H
