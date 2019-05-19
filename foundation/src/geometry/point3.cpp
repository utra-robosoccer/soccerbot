#include <geometry/point3.hpp>
#include <cmath>

float Point3::norm() const {
    return sqrtf(x * x + y * y + z * z);
}

float Point3::distance(const Point3 &p1, const Point3 &p2) {
    return sqrtf(powf(p1.x - p2.x, 2) + powf(p1.y - p2.y, 2) + powf(p1.z - p2.z, 2));
}
