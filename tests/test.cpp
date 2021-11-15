#include "gtest/gtest.h"
#include <iostream>

TEST(simple_test, test2){
    EXPECT_EQ(1, 1);
}

#ifdef TESTING
int main(int argc, char* argv[]){
    std::cout << "-----------------------------" << std::endl;
    std::cout << "Running tests..." << std::endl;
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
#endif