#pragma once

#include <vector>
#include <geometry/segment2.hpp>
#include <geometry/line2.hpp>

class Image {
    int height = 240;   // Y
    int width = 360;    // X

    std::vector<Segment2> segments;

public:
    Image(int height, int width);

    void TrimLines(const std::vector<Line2> &lines);
};