#pragma once

#include <stdio.h>
#include <math.h>
#include <vector>

typedef std::vector<std::vector<double>> grid;

class Map
{
private:
    float m_InflationRadius;
    float m_Resolution;
    float m_StepSize;
    int m_nNodes;
    int m_Height;
    int m_Width;

    int m_Rows;
    int m_Cols;
    grid m_OccupancyMap;

public:
    Map();
    ~Map();
    void UpdateOccupancyMap();
    void InflateOccupancyMap();
    void InflatePoint(int row, int col, grid &OccupancyMap);
};