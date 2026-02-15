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
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

// NOTE: printCuberootLogo 的辅助类型和函数，隔离在 detail 命名空间避免污染全局
// GEMINI 3 PRO DEEP THINK https://gemini.google.com/app/7a5b425b88288ce3
namespace logo_detail {

struct Color {
  int r, g, b;
};

// 官方配色插值: Hot Pink -> Light Purple -> Pale Blue -> Cyan
inline Color getColor(float t) {
  Color colors[] = {
      {255, 113, 204}, // 热力粉 (Hot Pink)
      {195, 153, 242}, // 浅紫 (Light Purple)
      {153, 186, 242}, // 淡蓝 (Pale Blue)
      {85, 225, 255}   // 青天蓝 (Cyan)
  };
  int n = 3;
  float scaledT = t * n;
  int i = (int)scaledT;
  if (i >= n)
    return colors[n];
  if (i < 0)
    return colors[0];
  float f = scaledT - i;
  Color c1 = colors[i];
  Color c2 = colors[i + 1];
  // 线性平滑插值计算
  return {(int)(c1.r + (c2.r - c1.r) * f), (int)(c1.g + (c2.g - c1.g) * f),
          (int)(c1.b + (c2.b - c1.b) * f)};
}

// 纯手工打磨的 Block 像素字模 (10x10)
inline std::vector<std::string> getLetter(char c) {
  if (c == 'C')
    return {"##########", "##########", "####      ", "####      ",
            "####      ", "####      ", "####      ", "####      ",
            "##########", "##########"};
  if (c == 'U')
    return {"####  ####", "####  ####", "####  ####", "####  ####",
            "####  ####", "####  ####", "####  ####", "####  ####",
            "##########", "##########"};
  if (c == 'B')
    return {"########  ", "##########", "####  ####", "####  ####",
            "########  ", "########  ", "####  ####", "####  ####",
            "##########", "########  "};
  if (c == 'E')
    return {"##########", "##########", "####      ", "####      ",
            "########  ", "########  ", "####      ", "####      ",
            "##########", "##########"};
  if (c == 'R')
    return {"########  ", "##########", "####  ####", "####  ####",
            "##########", "########  ", "####  ####", "####  ####",
            "####  ####", "####  ####"};
  if (c == 'O')
    return {"##########", "##########", "####  ####", "####  ####",
            "####  ####", "####  ####", "####  ####", "####  ####",
            "##########", "##########"};
  if (c == 'T')
    return {"##########", "##########", "   ####   ", "   ####   ",
            "   ####   ", "   ####   ", "   ####   ", "   ####   ",
            "   ####   ", "   ####   "};
  return std::vector<std::string>(10, "          ");
}

// 核心渲染器：光栅投射与渐变着色
inline void printWord(const std::string &word) {
  int numLetters = word.length();
  int charW = 10;
  int gap = 2; // 完美匹配原图紧凑的视觉间距
  int width = numLetters * charW + (numLetters - 1) * gap;

  // 阴影空间位移: 严格向左2列，向下1行
  int dx = -2;
  int dy = 1;

  int gridW = width + std::abs(dx);
  int gridH = 10 + std::abs(dy);

  // 前景字符缓冲
  std::vector<std::vector<int>> fg(10, std::vector<int>(width, 0));

  for (int i = 0; i < numLetters; ++i) {
    std::vector<std::string> letter = getLetter(word[i]);
    int startX = i * (charW + gap);
    for (int y = 0; y < 10; ++y) {
      for (int x = 0; x < charW; ++x) {
        if (letter[y][x] == '#')
          fg[y][startX + x] = 1;
      }
    }
  }

  // 最终合成画布: 0 = 空白, 1 = 底层阴影, 2 = 前景实体
  std::vector<std::vector<int>> canvas(gridH, std::vector<int>(gridW, 0));

  // Pass 1: 首先绘制底层阴影，使其永远垫底
  for (int y = 0; y < 10; ++y) {
    for (int x = 0; x < width; ++x) {
      if (fg[y][x]) {
        canvas[y + dy][x + std::abs(dx) + dx] = 1;
      }
    }
  }

  // Pass 2: 将前景实体覆盖上去
  for (int y = 0; y < 10; ++y) {
    for (int x = 0; x < width; ++x) {
      if (fg[y][x]) {
        canvas[y][x + std::abs(dx)] = 2;
      }
    }
  }

  // Pass 3: 应用 TrueColor 线性着色并直接推送到终端
  for (int y = 0; y < gridH; ++y) {
    std::cout << "  "; // 适当留白，保持优美的呼吸感
    for (int x = 0; x < gridW; ++x) {
      int val = canvas[y][x];
      if (val == 0) {
        std::cout << " ";
      } else {
        // 利用绝对 X 坐标进行色彩插值，使阴影和主体颜色无缝映射
        float t = (float)x / (gridW - 1);
        Color c = getColor(t);

        // 输出 ANSI 24-bit TrueColor 转义代码
        std::cout << "\033[38;2;" << c.r << ";" << c.g << ";" << c.b << "m";

        if (val == 2) {
          std::cout << "\xE2\x96\x88"; // █ (Solid Block)
        } else {
          std::cout << "\xE2\x96\x91"; // ░ (Light Shade / 半调阴影)
        }
      }
    }
    std::cout << "\033[0m\n"; // 重置终端色彩
  }
}

} // namespace logo_detail

// 打印CUBEROOT Logo（Gemini CLI 风格：Block 像素字体 + 粉紫蓝青渐变 +
// 半调阴影）
inline void printCuberootLogo() {
#ifdef _WIN32
  UINT origCp = GetConsoleOutputCP();
  SetConsoleOutputCP(CP_UTF8);
#endif

  std::cout << "\n";
  logo_detail::printWord("CUBE");
  std::cout << "\n";
  logo_detail::printWord("ROOT");
  std::cout << "\n";

#ifdef _WIN32
  SetConsoleOutputCP(origCp);
#endif
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

// 打印 CubeRoot 图标 Logo（立方根号 ³√ + 3×3 魔方网格）
// NOTE: 还原 cuberoot.png 中的视觉元素，使用 ANSI 24-bit TrueColor
inline void printCuberootLogoIcon() {
#ifdef _WIN32
  UINT origCp = GetConsoleOutputCP();
  SetConsoleOutputCP(CP_UTF8);
#endif

  // 颜色定义，提取自 cuberoot.png
  const char *G = "\033[38;2;85;85;85m";   // 灰色 (根号符号 + 数字3)
  const char *R = "\033[38;2;229;57;53m";  // 红色 (魔方块)
  const char *B = "\033[38;2;30;136;229m"; // 蓝色 (魔方块)
  const char *X = "\033[0m";               // 重置

  // NOTE: 所有行的魔方方块固定从第 24 列开始，确保 3×3 网格严格对齐
  // 根号 "/" 逐行左移 1 列（col 18→9），"\" 小勾从行 7 开始汇聚至 V 底
  std::cout << "\n";

  // 横线（根号顶部水平延伸线）
  std::cout << G << "                   ____________________________" << X
            << "\n";

  // 魔方第 1 行 (R R R) + 根号斜线
  std::cout << G << "                  /     " << R << "████" << G << "  " << R
            << "████" << G << "  " << R << "████" << X << "\n";
  std::cout << G << "                 /      " << R << "████" << G << "  " << R
            << "████" << G << "  " << R << "████" << X << "\n";

  // 根号斜线（无方块行）
  std::cout << G << "                /" << X << "\n";

  // 魔方第 2 行 (B R B) + 上标 ³
  std::cout << G << "    \xC2\xB3          /        " << B << "████" << G
            << "  " << R << "████" << G << "  " << B << "████" << X << "\n";
  std::cout << G << "              /         " << B << "████" << G << "  " << R
            << "████" << G << "  " << B << "████" << X << "\n";

  // 根号斜线（无方块行）
  std::cout << G << "             /" << X << "\n";

  // 魔方第 3 行 (B R B) + 根号左侧小勾 "\"
  std::cout << G << "     \\      /           " << B << "████" << G << "  " << R
            << "████" << G << "  " << B << "████" << X << "\n";
  std::cout << G << "      \\    /            " << B << "████" << G << "  " << R
            << "████" << G << "  " << B << "████" << X << "\n";

  // 根号 V 底收束
  std::cout << G << "       \\  /" << X << "\n";
  std::cout << G << "        \\/" << X << "\n";

  std::cout << "\n";

#ifdef _WIN32
  SetConsoleOutputCP(origCp);
#endif
}

#endif // LOGO_H
