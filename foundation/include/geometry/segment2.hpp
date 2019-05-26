#pragma once

#include <geometry/point2.hpp>
#include <memory>

struct Segment2 {
    Point2 p1;
    Point2 p2;

    // Move copies obj into struct, similar to passing by reference, saves memory
    Segment2(Point2 p1, Point2 p2) : p1(std::move(p1)), p2(std::move(p2)) {}

    float slope() const;

    float length() const;
};