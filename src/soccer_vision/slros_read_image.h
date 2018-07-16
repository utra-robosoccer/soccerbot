/* Copyright 2017 The MathWorks, Inc. */

#ifndef _SLROS_READCOMPRESSIMAGE_H_
#define _SLROS_READCOMPRESSIMAGE_H_

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CompressedImage.h>
//********************************************
extern ros::NodeHandle * SLROSNodePtr;  ///< The global node handle that is used by all ROS entities in the model

/**
* Class for converting ROS image messages to arrays
*
* This class is used by code generated from the Simulink ROS
* Read Image blocks.
*/

template <int NumChannels, typename T>
void decompressImage(const uint8_T* data, int dataSize,
        T* image, int* width, int* height)
{
    typedef cv::Vec<T,NumChannels> Vec;
    cv::Mat compressedData(1, dataSize, CV_8UC1);
    compressedData.data = const_cast<uchar*>(data);
    cv::InputArray cvInput(compressedData);
    cv::Mat cvImage = cv::imdecode(cvInput, cv::IMREAD_ANYCOLOR);
    ROS_ASSERT(cvImage.channels() == NumChannels);
    // Convert BGR to RGB
    if (NumChannels == 3) {
        cv::cvtColor(cvImage, cvImage, CV_BGR2RGB);
    }
    // Convert BGRA to RGBA
    if (NumChannels == 4) {
        cv::cvtColor(cvImage, cvImage, CV_BGRA2RGBA);
    }
    int maxWidth = *width;
    int maxHeight = *height;
    *height = cvImage.size().height;
    *width = cvImage.size().width;
    ROS_ASSERT(*width <= maxWidth);
    ROS_ASSERT(*height <= maxHeight);
    int channelStep = maxWidth*maxHeight;
    for (int i = 0; i < *height; ++i)
    {
        for (int j = 0; j < *width; ++j)
        {
            const Vec& elem = cvImage.at<Vec>(i, j);
            // Only take RGB, even if alpha is available, to match MATLAB. The
            // alpha channel has been initialized to zeros in ReadImage.
            for (int k = 0 ; k < 3; ++k)
            {
                image[i + j*maxHeight + k*channelStep] = elem[k];
            }
        }
    }
}
#endif
