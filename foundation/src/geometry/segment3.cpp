#include <geometry/segment3.hpp>
#include <cmath>

float Segment3::length() const {
    return p1.distance(p1, p2);
}
