#pragma once

#include <soccer_geometry/point3.hpp>
#include <memory>
#include <vector>

struct Segment3 {
    Point3 p1;
    Point3 p2;

    // Move copies obj into struct, similar to passing by reference, saves memory
    Segment3(Point3 p1, Point3 p2) : p1(std::move(p1)), p2(std::move(p2)) {}

    float length() const;

    std::vector<Point3> getSpacedPoints(float spacing) const;
};