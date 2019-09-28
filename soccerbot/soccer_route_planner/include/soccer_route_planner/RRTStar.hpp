#pragma once

#include <stdio.h>

class RRTStar {
public:
    RRTStar();
    void m_RRTStar(int r, int n);
    void m_nearestNeighbour();
    void m_nearVertices();
    void m_steering();
    void m_collisionTest();
    void m_sample();
};