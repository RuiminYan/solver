/* table_generator.cpp */
#include "cube_common.h"
#include "move_tables.h"
#include "prune_tables.h"

int main() {
    init_matrix();
    std::cout << "=== Cube Table Generator ===" << std::endl;
    
    // 强制检查并生成所有表
    MoveTableManager::getInstance().initialize();
    PruneTableManager::getInstance().initialize();
    
    std::cout << "=== All tables are ready ===" << std::endl;
    return 0;
}