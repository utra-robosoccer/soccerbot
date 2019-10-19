//
// Created by manx52 on 2019-09-28.
//
#pragma once

class transform{



public:

    transform(float pos []);

    transform(float pos [], float orient []);

    float * rotm2quat(float array [4][4]);

    float ** quat2tform(float orient []);

    float orientation [4] = {1,0,0,0};
    float position [3] = {0,0,0};


    float **H(transform c);

   // transform ApplyTransformation(float *a, float *b);

    transform ApplyTransformation(transform a, transform b);
};