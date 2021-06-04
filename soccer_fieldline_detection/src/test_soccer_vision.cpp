#include <ros/ros.h>
#include <soccer_fieldline_detection/test_soccer_vision.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <soccer_geometry/segment2.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <iostream>
#include <string>

TestSoccerVision::TestSoccerVision() : tfListener(tfBuffer){
    image_transport::ImageTransport it(nh);
    image_subscriber = it.subscribe("camera/image_raw", 1, &TestSoccerVision::imageCallback, this);
    point_cloud_publisher = TestSoccerVision::nh.advertise<sensor_msgs::PointCloud2> ("field_point_cloud",1);

    image_publisher = it.advertise("camera/line_image",1);
    image_publisher2 = it.advertise("camera/base",1);
    image_publisher3 = it.advertise("camera/canny",1);
    image_publisher4 = it.advertise("camera/binary",1);
    // Parameters
    nh.getParam("soccer_fieldline_detector/cannythreshold1", cannythreshold1);
    nh.getParam("soccer_fieldline_detector/cannythreshold2", cannythreshold2);

    nh.getParam("soccer_fieldline_detector/houghRho", rho);
    nh.getParam("soccer_fieldline_detector/houghTheta", theta);
    nh.getParam("soccer_fieldline_detector/houghThreshold", threshold);
    nh.getParam("soccer_fieldline_detector/houghMinLineLength", minLineLength);
    nh.getParam("soccer_fieldline_detector/houghMaxLineGap", maxLineGap);



}

void TestSoccerVision::imageCallback(const sensor_msgs::ImageConstPtr &msg) {

    std::vector<cv::Vec4i> lines;
    std::vector<Point2> pts;

    try {
        const cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Detect Field Lines (Copy from simulink)
        cv::Mat dst, cdst;
        cv::Mat hsv, mask,out,mask2 , out2;
        cvtColor(image, hsv , cv::COLOR_BGR2HSV);

        cv::inRange(hsv,cv::Scalar (0,0,200), cv::Scalar(179, 77, 255), mask);
        cv::inRange(hsv,cv::Scalar (45, 45,45), cv::Scalar(70, 255,255), mask2);

        std::vector<std::vector<cv::Point> > contours;
        cv::findContours( mask2, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
        std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
        std::vector<cv::Rect> boundRect( contours.size() );
        std::vector<cv::Point2f>centers( contours.size() );
        std::vector<float>radius( contours.size() );
        std::vector<int> index;
        double max = 0;
        int count = 0;
        cv::Mat drawing = cv::Mat::zeros( mask2.size(), CV_8UC3 );
        cv::RNG rng(12345);
        for( size_t i = 0; i < contours.size(); i++ )
        {

            if (cv::contourArea(contours[i]) > 200 and cv::contourArea(contours[i]) < 500000) {
                cv::approxPolyDP(contours[i], contours_poly[i], 3, true);
                boundRect[count] = boundingRect(contours_poly[i]);


                cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
                cv::drawContours(drawing, contours_poly, (int) i, color);
//                cv::rectangle(drawing, boundRect[count].tl(), boundRect[count].br(), color, 2);
                count += 1;
//                cv::rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2);
            }
        }
        // Merge all bounding boxes
        cv::Rect final = boundRect[0];
        for(const auto& r : boundRect) { final |= r; }
        cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
        cv::rectangle(image, final.tl(), final.br(), color, 2);
        // Top Black rectangle
        cv::rectangle(image, cv::Point(0,0), cv::Point(final.br().x,final.tl().y), cv::Scalar(0, 0, 0), -1, 8);
        // Bottom Black rectangle
        cv::rectangle(image, cv::Point(final.tl().x,final.br().y), cv::Point(1920,1080), cv::Scalar(0, 0, 0), -1, 8);

//        cv::Mat image1,image2;
//        cv::String imageName( "/home/manx52/catkin_ws/src/soccerbot/soccer_fieldline_detection/media/pictures/2.jpeg" ); // by default
//
//        std::cout << imageName << std::endl;
//
//        image1=cv::imread(imageName, cv::IMREAD_COLOR);
//        cv::bitwise_and(image,image,out,out2);
//        cv::bitwise_and(image,image,out,mask2);
        cv::imshow("Car",image);
        int k = cv::waitKey(0); // Wait for a keystroke in the window

        cv::bitwise_and(image,image,out,mask);
        sensor_msgs::ImagePtr message1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", out).toImageMsg();
        //image_publisher2.publish(message1);
        cvtColor(out, cdst, CV_BGR2GRAY);

        cv::threshold(cdst,dst, 127, 255,cv::THRESH_BINARY);
        //cv::adaptiveThreshold(cdst,dst, 255,cv::ADAPTIVE_THRESH_GAUSSIAN_C,cv::THRESH_BINARY,11,2);
        sensor_msgs::ImagePtr message4 = cv_bridge::CvImage(std_msgs::Header(), "mono8", dst).toImageMsg();
        //image_publisher4.publish(message4);

        cv::Canny(dst, cdst,50,150);


        sensor_msgs::ImagePtr message2 = cv_bridge::CvImage(std_msgs::Header(), "mono8", cdst).toImageMsg();
        //image_publisher3.publish(message2);
        // Detect Field Lines (Copy from simulink)
//        cv::Mat dst, cdst;
//        cv::Canny(image, dst,cannythreshold1,cannythreshold2);
//        cvtColor(dst, cdst, CV_GRAY2BGR);
        //HoughLinesP(dst, lines, rho, theta,threshold,minLineLength,maxLineGap);
        // Cover Horizon
//        double roll, pitch, yaw;
//        tf2::Quaternion q(camera->getPose().orientation.x,camera->getPose().orientation.y,camera->getPose().orientation.z,camera->getPose().orientation.w);
//        tf2::Matrix3x3 m(q);
//        m.getRPY(roll, pitch, yaw);
        //Draw black box on screen based on the pitch of the camera
        //cv::rectangle(dst, cv::Point(0,(camera->getResolutionY()/2) - 350*pitch), cv::Point( camera->getResolutionX(),0), cv::Scalar(0, 0, 0), -1, 8);
        //cv::rectangle(cdst, cv::Point(0,(camera->getResolutionY()/2) - 350*pitch), cv::Point( camera->getResolutionX(),0), cv::Scalar(0, 0, 0), -1, 8);

        // Rectangle only for simulation
        //cv::rectangle(dst, cv::Point(0,360), cv::Point( camera->getResolutionX(),camera->getResolutionY()), cv::Scalar(0, 0, 0), -1, 8);
        //cv::rectangle(cdst, cv::Point(0,360), cv::Point( camera->getResolutionX(),camera->getResolutionY()), cv::Scalar(0, 0, 0), -1, 8);



        HoughLinesP(cdst, lines, rho, theta,threshold,minLineLength,maxLineGap);
        cvtColor(cdst, dst, CV_GRAY2BGR);
        for (const auto& l : lines) {
            cv::line(dst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);

            Point2 pt1(l[0],l[1]);
            Point2 pt2(l[2],l[3]);
            Segment2 s(pt1,pt2);
            float b = l[1] - s.slope()*l[0];

            for (size_t i = l[0]; i < l[2];i += 15) {
                float y = s.slope()*i + b;
                Point2 pt(i,y);
                pts.push_back(pt);
            }
        }

        sensor_msgs::ImagePtr message = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
        //if(image_publisher.getNumSubscribers() > 1){

        image_publisher.publish(message);

    } catch (const cv_bridge::Exception &e) {
        ROS_ERROR_STREAM("CV Exception" << e.what());
    }



}



int main(int argc, char **argv) {
    ros::init(argc, argv, "test_soccer_vision");

    TestSoccerVision detector;

    ros::spin();

    return 0;
}
