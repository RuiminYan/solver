@echo off
echo Building Cube Analyzers...

echo Compiling common modules...
g++ -std=c++17 -O3 -fopenmp -Wall -Wextra -c cube_common.cpp -o cube_common.o
if %errorlevel% neq 0 (
    echo Error compiling cube_common.cpp
    exit /b 1
)

g++ -std=c++17 -O3 -fopenmp -Wall -Wextra -c move_tables.cpp -o move_tables.o
if %errorlevel% neq 0 (
    echo Error compiling move_tables.cpp
    exit /b 1
)

g++ -std=c++17 -O3 -fopenmp -Wall -Wextra -c prune_tables.cpp -o prune_tables.o
if %errorlevel% neq 0 (
    echo Error compiling prune_tables.cpp
    exit /b 1
)

echo Compiling pair analyzer...
g++ -std=c++17 -O3 -fopenmp -Wall -Wextra -c pair_analyzer.cpp -o pair_analyzer.o
if %errorlevel% neq 0 (
    echo Error compiling pair_analyzer.cpp
    exit /b 1
)

echo Linking pair analyzer...
g++ -fopenmp -o pair_analyzer.exe cube_common.o move_tables.o prune_tables.o pair_analyzer.o -lpsapi
if %errorlevel% neq 0 (
    echo Error linking pair analyzer
    exit /b 1
)

echo Compiling std analyzer...
g++ -std=c++17 -O3 -fopenmp -Wall -Wextra -c std_analyzer.cpp -o std_analyzer.o
if %errorlevel% neq 0 (
    echo Error compiling std_analyzer.cpp
    exit /b 1
)

echo Linking std analyzer...
g++ -fopenmp -o std_analyzer.exe cube_common.o move_tables.o prune_tables.o std_analyzer.o -lpsapi
if %errorlevel% neq 0 (
    echo Error linking std analyzer
    exit /b 1
)

echo Compiling table generator...
g++ -std=c++17 -O3 -fopenmp -Wall -Wextra -c table_generator.cpp -o table_generator.o
if %errorlevel% neq 0 (
    echo Error compiling table_generator.cpp
    exit /b 1
)

echo Linking table generator...
g++ -fopenmp -o table_generator.exe cube_common.o move_tables.o prune_tables.o table_generator.o -lpsapi
if %errorlevel% neq 0 (
    echo Error linking table generator
    exit /b 1
)

echo Compiling pseudo analyzer...
g++ -std=c++17 -O3 -fopenmp -Wall -Wextra -c pseudo_analyzer.cpp -o pseudo_analyzer.o
if %errorlevel% neq 0 (
    echo Error compiling pseudo_analyzer.cpp
    exit /b 1
)

echo Linking pseudo analyzer...
g++ -fopenmp -o pseudo_analyzer.exe cube_common.o move_tables.o prune_tables.o pseudo_analyzer.o -lpsapi
if %errorlevel% neq 0 (
    echo Error linking pseudo analyzer
    exit /b 1
)

echo Compiling eo_cross_analyzer...
g++ -std=c++17 -O3 -fopenmp -Wall -Wextra eo_cross_analyzer.cpp cube_common.o move_tables.o prune_tables.o -o eo_cross_analyzer.exe -lpsapi
if %errorlevel% neq 0 (
    echo Error compiling eo_cross_analyzer
    exit /b 1
)

echo Compiling pseudo_pair_analyzer...
g++ -std=c++17 -O3 -fopenmp -Wall -Wextra -c pseudo_pair_analyzer.cpp -o pseudo_pair_analyzer.o
if %errorlevel% neq 0 (
    echo Error compiling pseudo_pair_analyzer.cpp
    exit /b 1
)

echo Linking pseudo_pair_analyzer...
g++ -fopenmp -o pseudo_pair_analyzer.exe cube_common.o move_tables.o prune_tables.o pseudo_pair_analyzer.o -lpsapi
if %errorlevel% neq 0 (
    echo Error linking pseudo_pair_analyzer
    exit /b 1
)

echo Build completed successfully!
echo Generated files:
echo - pair_analyzer.exe
echo - std_analyzer.exe
echo - table_generator.exe
echo - pseudo_analyzer.exe
echo - pseudo_pair_analyzer.exe
echo - eo_cross_analyzer.exe