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
#include <chrono>


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
        auto t_start = std::chrono::high_resolution_clock::now();

        const cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Detect Field Lines (Copy from simulink)
        cv::Mat dst, cdst;
        cv::Mat hsv, mask,out,mask2 , out2;
        cvtColor(image, hsv , cv::COLOR_BGR2HSV);
        // Single image
//        cv::Mat image1,image2;
//        cv::String imageName( "/home/manx52/catkin_ws/src/soccerbot/soccer_fieldline_detection/media/field/field7.png" ); // by default
//
//        std::cout << imageName << std::endl;
//
//        image1=cv::imread(imageName, cv::IMREAD_COLOR);
//        cv::cvtColor(image1, hsv , cv::COLOR_BGR2HSV);


        // cv::inRange(hsv,cv::Scalar (0,0,255 - 15) , cv::Scalar(255, 15, 255), mask); Goal post detection
        // cv::Scalar (0,0,200), cv::Scalar(179, 77, 255) old
        // cv::inRange(hsv,cv::Scalar (0,0,255 - 110) , cv::Scalar(255, 110, 255), mask); pretty good, but field boundary needs to be better

        cv::inRange(hsv,cv::Scalar (45, 115,45), cv::Scalar(70, 255,255), mask2); // old


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

        std::vector< std::vector<cv::Point> > hull(contours.size());

        // create a blank image (black image)

        cv::Mat drawin = cv::Mat::zeros(mask2.size(), CV_8UC3);
        double area = 0;

        count =0;
        for(int i = 0; i < contours.size(); i++) {
//            cv::Scalar color_contours = cv::Scalar(0, 255, 0); // green - color for contours
//            std::cout << cv::contourArea(contours[i])  << "         " << area << std::endl;
            if (cv::contourArea(contours[i])  >  1000 ) {
                convexHull(cv::Mat(contours[i]), hull[i], false);
                area += 1; //cv::contourArea(contours[i]);
                count += 1;
//                std::cout << cv::contourArea(contours[i])  << "         " << area << std::endl;
//                cv::Scalar color_contours = cv::Scalar(0, 255, 0);
//                drawContours(drawin, contours, i, color_contours, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
//                cv::Scalar color = cv::Scalar(255, 0, 0); // blue - color for convex hull
//                drawContours(drawin, hull, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());

                boundRect[count] = boundingRect(hull[i]);
            }
    }
        // Merge biggest contours
        if (count > 0) {
            cv::Rect final = boundRect[0];
            for (const auto &r : boundRect) { final |= r; }
            cv::Scalar color = cv::Scalar(0, 0, 255); // blue - color for convex hull
            cv::rectangle(image, final.tl(), final.br(), color, 2);

            // Top Black rectangle
            cv::rectangle(image, cv::Point(0, 0), cv::Point(final.br().x, final.tl().y), cv::Scalar(0, 0, 0), -1, 8);
            // Bottom Black rectangle
            cv::rectangle(image, cv::Point(final.tl().x, final.br().y), cv::Point(1920, 1080), cv::Scalar(0, 0, 0), -1,
                          8);
        }


        cvtColor(image, hsv , cv::COLOR_BGR2HSV);
        cv::inRange(hsv,cv::Scalar (0,0,255 - 65) , cv::Scalar(255, 65, 255), mask);

        cv::bitwise_and(image,image,out,mask);

        cvtColor(out, cdst, CV_BGR2GRAY);

        cv::threshold(cdst,dst, 127, 255,cv::THRESH_BINARY);

        //cv::adaptiveThreshold(cdst,dst, 255,cv::ADAPTIVE_THRESH_GAUSSIAN_C,cv::THRESH_BINARY,11,2);

        cv::Canny(dst, cdst,50,150);
        cv::imshow("Car",cdst);
        int k = cv::waitKey(0); // Wait for a keystroke in the window
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
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
        std::cout << elapsed_time_ms << std::endl;

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
