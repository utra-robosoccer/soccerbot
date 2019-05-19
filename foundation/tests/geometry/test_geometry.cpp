#include <gtest/gtest.h>
#include <geometry/point2.hpp>
#include <geometry/point3.hpp>
#include <geometry/segment2.hpp>
#include <geometry/segment3.hpp>

TEST(Geometry, Segment2Slope) {
    Point2 p1(1, 1);
    Point2 p2(2, 2);
    Segment2 s(p1, p2);
    ASSERT_FLOAT_EQ(1, s.slope());
}

TEST(Geometry, Segment2Length) {
    Point2 p1(1, 1);
    Point2 p2(2, 2);
    Segment2 s(p1, p2);
    ASSERT_FLOAT_EQ(1.4142135, s.length());
}

TEST(Geometry, Point2Distance) {
    Point2 p1(1, 1);
    Point2 p2(2, 2);
    ASSERT_FLOAT_EQ(1.4142135, p1.distance(p1, p2));
}

TEST(Geometry, Point2Norm) {
    Point2 p1(3, 4);
    ASSERT_FLOAT_EQ(5.0, p1.norm());
}

TEST(Geometry, Segment3Length) {
    Point3 p1(1, 1, 1);
    Point3 p2(2, 2, 2);
    Segment3 s(p1, p2);
    ASSERT_FLOAT_EQ(1.732050808, s.length());
}

TEST(Geometry, Point3Distance) {
    Point3 p1(1, 1, 1);
    Point3 p2(2, 2, 2);
    ASSERT_FLOAT_EQ(1.732050808, p1.distance(p1, p2));
}

TEST(Geometry, Point3Norm) {
    Point3 p1(3, 4, 4);
    ASSERT_FLOAT_EQ(6.403124237, p1.norm());
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
