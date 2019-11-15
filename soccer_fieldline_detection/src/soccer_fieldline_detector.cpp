#include <ros/ros.h>
#include <soccer_fieldline_detection/test_soccer_fieldline_detector.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry/segment2.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/transform_listener.h>

SoccerFieldlineDetector::SoccerFieldlineDetector() : tfListener(tfBuffer){
    image_transport::ImageTransport it(nh);
    image_subscriber = it.subscribe("front_camera/image_raw", 1, &SoccerFieldlineDetector::imageCallback, this);
    point_cloud_publisher = SoccerFieldlineDetector::nh.advertise<sensor_msgs::PointCloud2> ("/field_point_cloud",1);

    // Parameters
    nh.getParam("soccer_fieldline_detector/cannythreshold1", cannythreshold1);
    nh.getParam("soccer_fieldline_detector/cannythreshold2", cannythreshold2);

    nh.getParam("soccer_fieldline_detector/houghRho", rho);
    nh.getParam("soccer_fieldline_detector/houghTheta", theta);
    nh.getParam("soccer_fieldline_detector/houghThreshold", threshold);
    nh.getParam("soccer_fieldline_detector/houghMinLineLength", minLineLength);
    nh.getParam("soccer_fieldline_detector/houghMaxLineGap", maxLineGap);

    // Initialize Camera
    geometry_msgs::TransformStamped camera_pose;

    while(ros::ok()) {
        try{
            camera_pose = tfBuffer.lookupTransform("camera", "base_link",
                                                   ros::Time(0), ros::Duration(1.0));
            break;
        }
        catch (tf2::TransformException &ex) {
            std::string s = ex.what();
        }
    }

    // Initalize Camera
    Pose3 camera_position;
    camera_position.position.x = camera_pose.transform.translation.x;
    camera_position.position.y = camera_pose.transform.translation.y;
    camera_position.position.z = camera_pose.transform.translation.z;
    camera_position.orientation.w = camera_pose.transform.rotation.w;
    camera_position.orientation.x = camera_pose.transform.rotation.x;
    camera_position.orientation.y = camera_pose.transform.rotation.y;
    camera_position.orientation.z = camera_pose.transform.rotation.z;
    camera = std::make_unique<Camera>(camera_position,240,360);
}

void SoccerFieldlineDetector::imageCallback(const sensor_msgs::ImageConstPtr &msg) {

    std::vector<cv::Vec4i> lines;
    std::vector<Point2> pts;

    try {
        const cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Detect Field Lines (Copy from simulink)
        cv::Mat dst, cdst;
        cv::Canny(image, dst,cannythreshold1,cannythreshold2);
        cvtColor(dst, cdst, CV_GRAY2BGR);

        // Cover Horizon (Ignore for now)
        HoughLinesP(dst, lines, rho, theta,threshold,minLineLength,maxLineGap);

        for (const auto& l : lines) {
            cv::line(cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, CV_AA);

            Point2 pt1(l[0],l[1]);
            Point2 pt2(l[2],l[3]);
            Segment2 s(pt1,pt2);
            float b = l[1] - s.slope()*l[0];

            for (size_t i = l[0]; i < l[2];i++) {
                float y = s.slope()*i + b;
                Point2 pt(i,y);
                pts.push_back(pt);
            }
        }

    } catch (const cv_bridge::Exception &e) {
        ROS_ERROR_STREAM("CV Exception" << e.what());
    }

    std::vector<Point3> points3d;
    for (const auto& p : pts) {
        Point3 p2 = camera->FindFloorCoordinate(p.x, p.y);
        points3d.push_back(p2);
    }

    // Publish fieldlines in a LaserScan data format
    sensor_msgs::PointCloud2 point_cloud_msg;

    point_cloud_msg.header.stamp = ros::Time::now();
    point_cloud_msg.height = 1;
    point_cloud_msg.width = points3d.size();
    point_cloud_msg.is_bigendian = false;
    point_cloud_msg.is_dense = false;
    point_cloud_msg.point_step = 12;
    point_cloud_msg.row_step = 12 * points3d.size();

    sensor_msgs::PointCloud2Modifier modifier(point_cloud_msg);
    modifier.setPointCloud2Fields(1,"xyz");
    modifier.resize(points3d.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg, "z");

    for (const auto& p : points3d){
        *iter_x = p.x;
        *iter_y = p.y;
        *iter_z = p.z;
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    point_cloud_publisher.publish(msg);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "soccer_fieldline_detector");

    SoccerFieldlineDetector detector;

    ros::spin();

    return 0;
}