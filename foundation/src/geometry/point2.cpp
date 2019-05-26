#include <geometry/point2.hpp>
#include <cmath>

float Point2::norm() const {
    return sqrtf(x * x + y * y);
}

float Point2::distance(const Point2 &p1, const Point2 &p2) {
    return sqrtf(powf(p1.x - p2.x, 2) + powf(p1.y - p2.y, 2));
}
