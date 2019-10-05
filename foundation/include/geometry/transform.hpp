//
// Created by manx52 on 2019-09-28.
//

#ifndef SRC_TRANSFORM_HPP
#define SRC_TRANSFORM_HPP

#endif //SRC_TRANSFORM_HPP

class transform{



public:

    transform(float pos []);

    transform(float pos [], float orient []);

    float * rotm2quat(float (&array) [4][4]);

    float ** quat2tform(float orient []);

    float orientation [4] = {1,0,0,0};
    float position [3] = {0,0,0};

    transform ApplyTransformation(transform a, transform b);

    float **H(transform c);
};