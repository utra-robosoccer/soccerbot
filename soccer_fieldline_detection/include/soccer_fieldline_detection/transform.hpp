//
// Created by manx52 on 2019-09-28.
//
#pragma once

class transform{



public:

    transform(double pos []);

    transform(double pos [], double orient []);

    double * rotm2quat(double array [4][4]);

    double ** quat2tform(double orient []);

    double orientation [4] = {1,0,0,0};
    double position [3] = {0,0,0};


    double **H(transform c);

   // transform ApplyTransformation(double *a, double *b);

    transform ApplyTransformation(transform a, transform b);


};