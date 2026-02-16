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

// 打印 CubeRoot 图标 Logo V4（³√ 工业根号骨架 + 3×3 魔方发光矩阵 +
// 像素抖动渐变投影） NOTE: 基于 Box-Drawing 工程风格，使用 Unicode Block/Shade
// 字符 + ANSI 24-bit TrueColor
inline void printCuberootLogoV4() {
#ifdef _WIN32
  UINT origCp = GetConsoleOutputCP();
  SetConsoleOutputCP(CP_UTF8);

  HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
  DWORD dwMode = 0;
  if (hOut != INVALID_HANDLE_VALUE && GetConsoleMode(hOut, &dwMode)) {
    dwMode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
    SetConsoleMode(hOut, dwMode);
  }
#endif

  // [像素掩膜设计图] : 将三维结构展开为绝对坐标的编译期常量
  // 工业根号骨架 : 1, 2, 3, L, J
  // 抖动渐变投影 : S(▓), M(▒), W(░)
  // 魔方发光矩阵 : A(Red), B(Blue), a/m/w(Red Shade), b/n/q(Blue Shade)
  // 工程连接总线 : c(╗), d(╠), e(╚), f(╦), g(╝), h(═), i(║)
  const char *layout[] = {
      "         1333331         333333333333333333333333333333333333333333",
      "        22   333        33SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS",
      "           13332       33SSMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM",
      "        11   3331     33SSMM                                       ",
      "         2333332     33SSMM       AAAAAAc  AAAAAAc  AAAAAAc        ",
      "                    33SSMM        AAAAAAdhhAAAAAAdhhAAAAAAi        ",
      "                   33SSMM         aaammwi  aaammwi  aaammwi        ",
      "                  33SSMM          ehhfhhg  ehhfhhg  ehhfhhg        ",
      "                 33SSMM              i        i        i           ",
      "                33SSMM            BBBBBBc  AAAAAAc  BBBBBBc        ",
      "               33SSMM             BBBBBBdhhAAAAAAdhhBBBBBBi        ",
      "              33SSMM              bbbnnqi  aaammwi  bbbnnqi        ",
      "             33SSMM               ehhfhhg  ehhfhhg  ehhfhhg        ",
      "   1331     33SSMM                   i        i        i           ",
      " 12 2331   33SSMM                 BBBBBBc  AAAAAAc  BBBBBBc        ",
      "     2331333SSMM                  BBBBBBdhhAAAAAAdhhBBBBBBi        ",
      "      23333SSMM                   bbbnnqi  aaammwi  bbbnnqi        ",
      "       233SSMM                    ehhhhhg  ehhhhhg  ehhhhhg        "};

  std::cout << "\n\n";

  for (int y = 0; y < 18; ++y) {
    std::cout << "    "; // 左侧视觉留白

    for (int x = 0; layout[y][x] != '\0'; ++x) {
      char c = layout[y][x];
      if (c == ' ') {
        std::cout << " ";
        continue;
      }

      int r = 255, g = 255, b = 255;
      const char *utf8Str = " ";

      switch (c) {
      // 1. 钛灰质感线条 (Radical 巨型根号骨架与数字3)
      case '1':
        r = 160;
        g = 165;
        b = 170;
        utf8Str = "\xE2\x96\x84"; // ▄ (Lower half block)
        break;
      case '2':
        r = 160;
        g = 165;
        b = 170;
        utf8Str = "\xE2\x96\x80"; // ▀ (Upper half block)
        break;
      case '3':
        r = 160;
        g = 165;
        b = 170;
        utf8Str = "\xE2\x96\x88"; // █ (Full block)
        break;
      // 2. Gemini 像素抖动渐变投影 (计算动态 X轴 归一化渐变)
      case 'S':
      case 'M':
      case 'W': {
        float t = (float)x / 66.0f; // 光谱平滑步进
        if (t < 0.333f) {
          float f = t / 0.333f;
          r = (int)(255 + (195 - 255) * f);
          g = (int)(113 + (153 - 113) * f);
          b = (int)(204 + (242 - 204) * f);
        } else if (t < 0.666f) {
          float f = (t - 0.333f) / 0.333f;
          r = (int)(195 + (153 - 195) * f);
          g = (int)(153 + (186 - 153) * f);
          b = (int)(242 + (242 - 242) * f);
        } else {
          float f = (t - 0.666f) / 0.334f;
          r = (int)(153 + (85 - 153) * f);
          g = (int)(186 + (225 - 186) * f);
          b = (int)(242 + (255 - 242) * f);
        }
        if (c == 'S')
          utf8Str = "\xE2\x96\x93"; // ▓ (Dark Shade)
        else if (c == 'M')
          utf8Str = "\xE2\x96\x92"; // ▒ (Medium Shade)
        else if (c == 'W')
          utf8Str = "\xE2\x96\x91"; // ░ (Light Shade)
        break;
      }

      // 3. Matrix R/B 发光体与物理衰减阴影
      case 'A':
        r = 229;
        g = 57;
        b = 53;
        utf8Str = "\xE2\x96\x88"; // █ (Red Matrix)
        break;
      case 'a':
        r = 150;
        g = 20;
        b = 20;
        utf8Str = "\xE2\x96\x93"; // ▓ (Red Shade)
        break;
      case 'm':
        r = 110;
        g = 15;
        b = 15;
        utf8Str = "\xE2\x96\x92"; // ▒ (Red Shade)
        break;
      case 'w':
        r = 70;
        g = 10;
        b = 10;
        utf8Str = "\xE2\x96\x91"; // ░ (Red Shade)
        break;

      case 'B':
        r = 30;
        g = 136;
        b = 229;
        utf8Str = "\xE2\x96\x88"; // █ (Blue Matrix)
        break;
      case 'b':
        r = 20;
        g = 100;
        b = 180;
        utf8Str = "\xE2\x96\x93"; // ▓ (Blue Shade)
        break;
      case 'n':
        r = 15;
        g = 70;
        b = 130;
        utf8Str = "\xE2\x96\x92"; // ▒ (Blue Shade)
        break;
      case 'q':
        r = 10;
        g = 40;
        b = 80;
        utf8Str = "\xE2\x96\x91"; // ░ (Blue Shade)
        break;

      // 4. 模块化重金属架构与端子排线
      case 'c':
        r = 100;
        g = 105;
        b = 110;
        utf8Str = "\xE2\x95\x97"; // ╗ (U+2557)
        break;
      case 'd':
        r = 100;
        g = 105;
        b = 110;
        utf8Str = "\xE2\x95\xA0"; // ╠ (U+2560)
        break;
      case 'e':
        r = 100;
        g = 105;
        b = 110;
        utf8Str = "\xE2\x95\x9A"; // ╚ (U+255A)
        break;
      case 'f':
        r = 100;
        g = 105;
        b = 110;
        utf8Str = "\xE2\x95\xA6"; // ╦ (U+2566)
        break;
      case 'g':
        r = 100;
        g = 105;
        b = 110;
        utf8Str = "\xE2\x95\x9D"; // ╝ (U+255D)
        break;
      case 'h':
        r = 100;
        g = 105;
        b = 110;
        utf8Str = "\xE2\x95\x90"; // ═ (U+2550)
        break;
      case 'i':
        r = 100;
        g = 105;
        b = 110;
        utf8Str = "\xE2\x95\x91"; // ║ (U+2551)
        break;
      }

      // ANSI TrueColor 直写
      std::cout << "\033[38;2;" << r << ";" << g << ";" << b << "m" << utf8Str;
    }
    std::cout << "\033[0m\n";
  }
  std::cout << "\n\n";

#ifdef _WIN32
  SetConsoleOutputCP(origCp);
#endif
}

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
// 打印 CubeRoot 图标 Logo（立方根号 ³√ + 3×3 魔方网格）-- V3 高精度 Block Pixel
// 版 NOTE: 使用 Unicode Block 字符 (▀ ▄ █ ◢ ◣) 进行像素级绘制，实现粗体线条
inline void printCuberootLogoIcon() {
#ifdef _WIN32
  UINT origCp = GetConsoleOutputCP();
  SetConsoleOutputCP(CP_UTF8);
#endif

  // 颜色定义
  const char *G = "\033[38;2;85;85;85m";   // 灰色 (根号符号 + 数字3)
  const char *R = "\033[38;2;229;57;53m";  // 红色 (魔方块)
  const char *B = "\033[38;2;30;136;229m"; // 蓝色 (魔方块)
  const char *X = "\033[0m";               // 重置

  // 符号定义 (Unicode Block Elements)
  // █ (U+2588) Full block
  // ▀ (U+2580) Upper half block
  // ▄ (U+2584) Lower half block
  // ◢ (U+25E2) Black lower right triangle
  // ◣ (U+25E3) Black lower left triangle
  // ◥ (U+25E5) Black upper right triangle
  // ◤ (U+25E4) Black upper left triangle

  std::cout << "\n";

  // Row 1: 根号横线
  // 37列 = 23 (横线) + 14 (左侧偏移)
  std::cout << G << "                       ▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄▄" << X
            << "\n";

  // Row 2: 根号斜线 (Top) + 魔方第 1 行 (R R R)
  // 21空格 + ◢◤(2) + 6空格 = 29列 (方块起始)
  // 修正: 21 + 2 + 6 = 29
  std::cout << G << "                     ◢◤      " << R << "████" << G << "  "
            << R << "████" << G << "  " << R << "████" << X << "\n";
  std::cout << G << "                   ◢◤        " << R << "████" << G << "  "
            << R << "████" << G << "  " << R << "████" << X << "\n";

  // Row 3: 数字 '3' (Top) + 根号斜线 (Mid)
  std::cout << G << "      ▄▄▄▄       ◢◤" << X << "\n";

  // Row 4: 数字 '3' (Mid) + 根号斜线 (Mid) + 魔方第 2 行 (B R B)
  // 9空格 + █ + 5空格 + ◢◤(2) + 12空格 = 29列 (方块起始)
  // 9 + 1 + 5 + 2 + 12 = 29
  std::cout << G << "         █     ◢◤            " << B << "████" << G << "  "
            << R << "████" << G << "  " << B << "████" << X << "\n";
  // 6空格 + ▀▀▀█(4) + 3空格 + ◢◤(2) + 14空格 = 29列? No
  // 6 + 4 + 3 + 2 + 14 = 29
  std::cout << G << "      ▀▀▀█   ◢◤              " << B << "████" << G << "  "
            << R << "████" << G << "  " << B << "████" << X << "\n";

  // Row 5: 数字 '3' (Bot) + 根号斜线 (Bot)
  std::cout << G << "         █ ◢◤" << X << "\n";

  // Row 6: 数字 '3' (Bottom) + V 底收束 + 魔方第 3 行 (B R B)
  // 6空格 + ▀▀▀█◤(5) + 18空格 = 29列 (方块起始)
  // 修正: 6 + 5 + 18 = 29
  std::cout << G << "      ▀▀▀█◤                  " << B << "████" << G << "  "
            << R << "████" << G << "  " << B << "████" << X << "\n";
  // 9空格 + ◤(1) + 19空格 = 29列
  // 修正: 9 + 1 + 19 = 29
  std::cout << G << "         ◤                   " << B << "████" << G << "  "
            << R << "████" << G << "  " << B << "████" << X << "\n";

  std::cout << "\n";

#ifdef _WIN32
  SetConsoleOutputCP(origCp);
#endif
}

#endif // LOGO_H
