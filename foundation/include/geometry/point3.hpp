#pragma once

#include <geometry/point2.hpp>

struct Point3 : public Point2 {
    float z = 0;

    Point3(float x, float y, float z) : Point2(x, y), z(z) {}

    float norm() const override;

    static float distance(const Point3 &p1, const Point3 &p2);
};