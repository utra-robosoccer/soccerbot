#include <gtest/gtest.h>
#include <geometry/point2.hpp>
#include <geometry/point3.hpp>
#include <geometry/segment2.hpp>

TEST(Geometry, GeometrySlopeTest) {
    Point2 p1(1, 1);
    Point2 p2(2, 2);
    Segment2 s(p1, p2);
    ASSERT_FLOAT_EQ(1, s.slope());
}

TEST(Geometry, Geometry_GeometryPoint2Distance_Test) {
    Point2 p1(1, 1);
    Point2 p2(2, 2);
    ASSERT_FLOAT_EQ(1.4142135, p1.distance(p1, p2));
}

TEST(Geometry, Geometry_GeometryPoint2Norm_Test) {
    Point2 p1(3, 4);
    ASSERT_FLOAT_EQ(5.0, p1.norm());
}

TEST(Geometry, Geometry_GeometryPoint3Distance_Test) {
    Point3 p1(1, 1, 1);
    Point3 p2(2, 2, 2);
    ASSERT_FLOAT_EQ(1.732050808, p1.distance(p1, p2));
}

TEST(Geometry, Geometry_GeometryPoint3Norm_Test) {
    Point3 p1(3, 4, 4);
    ASSERT_FLOAT_EQ(6.403124237, p1.norm());
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
