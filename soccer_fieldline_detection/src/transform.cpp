//
// Created by manx52 on 2019-09-28.
//

#include "../include/soccer_fieldline_detection/transform.hpp"
#include <cmath>
#include <stdio.h>
transform::transform(float pos[]) {
    transform::position[0] = pos[0];
    transform::position[1] = pos[1];
    transform::position[2] = pos[2];

    transform::orientation[0] = 1;
    transform::orientation[1] = 0;
    transform::orientation[2] = 0;
    transform::orientation[3] = 0;

}

transform::transform(float pos [], float orient[]) {
    transform::position[0] = pos[0];
    transform::position[1] = pos[1];
    transform::position[2] = pos[2];


    transform::orientation[0] = orient[0];
    transform::orientation[1] = orient[1];
    transform::orientation[2] = orient[2];
    transform::orientation[3] = orient[3];

}



float * transform::rotm2quat(float array [4][4]){ // Convert rot matrix to quaternion
    static float quat [4] = {0,0,0,0};// [w,x,y,z]
    float tr = array[0][0] + array[1][1] + array[2][2];

    if (tr > 0) {
        float s = sqrt(tr+1.0) * 2;
        quat[0] = 0.25*s;
        quat[1] = (array[2][1] - array[1][2]) / s;
        quat[2] = (array[0][2] - array[2][0]) / s;
        quat[3] = (array[1][0] - array[0][1]) / s;

    }
    else if ((array[0][0] > array[1][1])  && (array[0][0] > array[2][2] )){
        float s = sqrt(1.0 + array[0][0] - array[1][1] - array[2][2]) * 2;
        quat[0] = (array[2][1] - array[1][2]) / s;
        quat[1] = 0.25*s;
        quat[2] = (array[1][0] + array[0][1]) / s;
        quat[3] = (array[0][2] + array[2][0]) / s;
    }
    else if ((array[1][1] > array[2][2] )){
        float s = sqrt(1.0 - array[0][0] + array[1][1] - array[2][2]) * 2;
        quat[0] = (array[0][2] - array[2][0]) / s;
        quat[1] = (array[1][0] + array[0][1]) / s;
        quat[2] = 0.25*s;
        quat[3] = (array[2][1] + array[1][2]) / s;
    }
    else {
        float s = sqrt(1.0 - array[0][0] + array[1][1] - array[2][2]) * 2;
        quat[0] = (array[1][0] - array[0][1]) / s;
        quat[1] = (array[0][2] + array[2][0]) / s;
        quat[2] = (array[2][1] + array[1][2]) / s;
        quat[3] = 0.25*s;
    }

    return quat;

}


float ** transform::quat2tform(float orient []){// Convert quaternion to homogeneous transformation matrix

    float ** matrix = 0;
    matrix = new float*[4];

    for (int i = 0; i < 4; i++){
       matrix[i] = new float[4];
        for (int j = 0; j < 4; j++) {

            matrix[i][j] = 0;
        }
    }
    float sqw = orient[0] * orient[0];
    float sqx = orient[1] * orient[1];
    float sqy = orient[2] * orient[2];
    float sqz = orient[3] * orient[3];

    //If quaternion is not already normalised

    float invs = 1.0 / (sqw + sqx+ sqy + sqz);
    matrix [0][0] = (sqx - sqy -sqz +sqw)*invs;
    matrix [1][1] = (-sqx + sqy -sqz +sqw)*invs;
    matrix [2][2] = (-sqx - sqy +sqz +sqw)*invs;

    float tmp1 = orient[1] * orient[2];
    float tmp2 = orient[3] * orient[0];

    matrix[1][0] = 2.0 * (tmp1 + tmp2)*invs;
    matrix[0][1] = 2.0 * (tmp1 - tmp2)*invs;

    tmp1 = orient[1] * orient[3];
    tmp2 = orient[2] * orient[0];

    matrix[2][0] = 2.0 * (tmp1 - tmp2)*invs;
    matrix[0][2] = 2.0 * (tmp1 + tmp2)*invs;

    tmp1 = orient[2] * orient[3];
    tmp2 = orient[1] * orient[0];

    matrix[2][1] = 2.0 * (tmp1 + tmp2)*invs;
    matrix[1][2] = 2.0 * (tmp1 - tmp2)*invs;

    return matrix;
}




float ** transform::H (transform c) {

    float ** matrix = 0;
    matrix = new float*[4];

    for (int i = 0; i < 4; i++){
        matrix[i] = new float[4];
        for (int j = 0; j < 4; j++) {

            matrix[i][j] = 0;
        }
    }

    float ** tmp = transform::quat2tform(c.orientation);

    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++) {
            matrix[i][j] = tmp[i][j];

        }
    }

    matrix[0][3] = c.position[0];
    matrix[1][3] = c.position[1];
    matrix[2][3] = c.position[2];
    matrix[3][3] = 1;
    return matrix;
}

transform transform::ApplyTransformation(transform a, transform b){
    float matrix [4][4]  ={ { 0 } };

    float ** H1 = H(b);
    float ** H2 = H(a);

    //Matrix multiplication
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k< 4; k++){
                matrix[i][j] +=  H1[i][k] * H2[k][j];
            }

        }
    }

    float *orient ;
    orient  =  transform::rotm2quat(matrix);//{0,0,0,0};
    float pos [3] = {matrix[0][3],matrix[1][3],matrix[2][3]};


    transform result = transform(pos,orient);
    return result;
}