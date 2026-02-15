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
// NOTE: 使用 Unicode █ 字符 + 投影阴影实现立体感，24-bit 赤陶色精确匹配
inline void printCuberootLogoBlock() {
  // NOTE: Windows 默认 codepage 无法显示 Unicode █，强制切换为 UTF-8
#ifdef _WIN32
  UINT origCp = GetConsoleOutputCP();
  SetConsoleOutputCP(CP_UTF8);
#endif

  // Claude Code 赤陶配色（24-bit true color 精确匹配）
  const char *fg = "\033[38;2;204;136;102m";   // 赤陶色主体
  const char *shadow = "\033[38;2;120;78;55m"; // 深赤陶阴影
  const char *rst = "\033[0m";

  // 字母像素映射：'#'=填充, 其它=空
  // 每个字母 5 列宽，字母间距 2 列，共 26 列
  const char *cubeMap[] = {
      ".####  #...#  ####.  #####", "#....  #...#  #...#  #....",
      "#....  #...#  ####.  ####.", "#....  #...#  #...#  #....",
      "#....  #...#  #...#  #....", ".####  .###.  ####.  #####"};

  const char *rootMap[] = {
      "####.  .###.  .###.  #####", "#...#  #...#  #...#  ..#..",
      "####.  #...#  #...#  ..#..", "#.#..  #...#  #...#  ..#..",
      "#..#.  #...#  #...#  ..#..", "#...#  .###.  .###.  ..#.."};

  const int H = 6, W = 26;

  // NOTE: 合成缓冲区实现投影阴影 — 阴影层偏移(+1,+1)后被主体层覆盖，
  // 使右下边缘露出深色阴影，产生立体浮雕效果
  auto renderWithShadow = [&](const char *map[]) {
    // 0=空白, 1=主体, 2=阴影
    int buf[8][28] = {};

    // 第一遍：阴影层（向右下偏移 1 像素）
    for (int r = 0; r < H; r++)
      for (int c = 0; c < W; c++)
        if (map[r][c] == '#')
          buf[r + 1][c + 1] = 2;

    // 第二遍：主体层（覆盖阴影）
    for (int r = 0; r < H; r++)
      for (int c = 0; c < W; c++)
        if (map[r][c] == '#')
          buf[r][c] = 1;

    // 渲染输出（每像素 = 2 字符宽 ██）
    for (int r = 0; r <= H; r++) {
      std::cout << "  "; // 左边距
      for (int c = 0; c <= W; c++) {
        if (buf[r][c] == 1)
          std::cout << fg << "██";
        else if (buf[r][c] == 2)
          std::cout << shadow << "██";
        else
          std::cout << "  ";
      }
      std::cout << rst << "\n";
    }
  };

  std::cout << "\n";
  renderWithShadow(cubeMap);
  std::cout << "\n";
  renderWithShadow(rootMap);
  std::cout << "\n";

#ifdef _WIN32
  SetConsoleOutputCP(origCp);
#endif
}

#endif // LOGO_H
