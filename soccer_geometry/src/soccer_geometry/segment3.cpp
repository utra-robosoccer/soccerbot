#include <soccer_geometry/segment3.hpp>
#include <cmath>
#include <iostream>
#include <cassert>

float Segment3::length() const {
    return Point3::distance(p1, p2);
}

std::vector<Point3> Segment3::getSpacedPoints(float spacing) const {
    std::vector<Point3> points;

    assert(std::isnormal(spacing));
    assert(spacing > 0);

    int count = static_cast<int>(std::floor(length() / spacing));

    // If too small keep first and last point
    if (count == 0) {
        points.push_back(p1);
        points.push_back(p2);
    }

    float dx = (p2.x - p1.x) / count;
    float dy = (p2.y - p1.y) / count;
    float dz = (p2.z - p1.z) / count;

    for (int i = 0; i < count + 1; ++i) {
        points.emplace_back(Point3(p1.x + i * dx, p1.y + i * dy, p1.z + i * dz));
    }

    return points;
}
