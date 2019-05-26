#pragma once

struct Point2 {
    float x = 0;
    float y = 0;

    Point2(float x, float y) : x(x), y(y) {}

    virtual float norm() const;

    static float distance(const Point2 &p1, const Point2 &p2);
};