#include <geometry/segment2.hpp>
#include <cmath>

float Segment2::slope() const {
    if (p2.x == p1.x) {
        return INFINITY;
    }
    return (p2.y - p1.y) / (p2.x - p1.x);
}

float Segment2::length() const {
    return Point2::distance(p1, p2);
}
