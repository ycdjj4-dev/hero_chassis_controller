// gtest 测试入口。
#include <cstdlib>
#include <ctime>

#include <gtest/gtest.h>

// 运行所有已声明的 TEST() 测试。
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    return RUN_ALL_TESTS();
}
