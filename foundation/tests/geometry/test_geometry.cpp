#include <gtest/gtest.h>
#include <geometry/point2.hpp>
#include <geometry/segment2.hpp>
#include <thread>

TEST(Geometry, Geometry_Slope_Test) {
    std::this_thread::sleep_for(std::chrono::seconds(20));

    Point2 p1(1, 1);
    Point2 p2(2, 2);
    Segment2 s(p1, p2);
    ASSERT_FLOAT_EQ(1, s.slope());
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
