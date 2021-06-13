#include <ros/ros.h>
#include <soccer_fieldline_detection/soccer_fieldline_detector.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include<opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <soccer_geometry/segment2.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

SoccerFieldlineDetector::SoccerFieldlineDetector() : tfListener(tfBuffer){
    image_transport::ImageTransport it(nh);
    image_subscriber = it.subscribe("camera/image_raw", 1, &SoccerFieldlineDetector::imageCallback, this);
    point_cloud_publisher = SoccerFieldlineDetector::nh.advertise<sensor_msgs::PointCloud2> ("field_point_cloud",1);
    fallen_sub = nh.subscribe("trajectory_complete", 1, &SoccerFieldlineDetector::fallenCallBack, this);
    image_publisher = it.advertise("camera/line_image",1);

    // Parameters
    nh.getParam("soccer_fieldline_detector/cannythreshold1", cannythreshold1);
    nh.getParam("soccer_fieldline_detector/cannythreshold2", cannythreshold2);

    nh.getParam("soccer_fieldline_detector/houghRho", rho);
    nh.getParam("soccer_fieldline_detector/houghTheta", theta);
    nh.getParam("soccer_fieldline_detector/houghThreshold", threshold);
    nh.getParam("soccer_fieldline_detector/houghMinLineLength", minLineLength);
    nh.getParam("soccer_fieldline_detector/houghMaxLineGap", maxLineGap);
    robotName = ros::this_node::getNamespace();
    robotName.erase(0, 1);
    // Initialize Camera
    geometry_msgs::TransformStamped camera_pose;

    while(ros::ok()) {
        try{
            camera_pose = tfBuffer.lookupTransform(robotName + "/camera", robotName + "/base_footprint",
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
    camera = std::make_unique<Camera>(camera_position);
}

void SoccerFieldlineDetector::imageCallback(const sensor_msgs::ImageConstPtr &msg) {

    std::vector<cv::Vec4i> lines;
    std::vector<Point2> pts;

    if (!camera->ready()) {
        return;
    }

    // Get transformation
    geometry_msgs::TransformStamped camera_pose;
    try {

        camera_pose = tfBuffer.lookupTransform(robotName + "/base_footprint", robotName + "/camera",
                                               ros::Time(0), ros::Duration(0.1));

        Pose3 camera_position;
        camera_position.position.x = camera_pose.transform.translation.x;
        camera_position.position.y = camera_pose.transform.translation.y;
        camera_position.position.z = camera_pose.transform.translation.z;

        tf2::Quaternion q(camera_pose.transform.rotation.x,
                          camera_pose.transform.rotation.y,
                          camera_pose.transform.rotation.z,
                          camera_pose.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double r, p, y;
        m.getRPY(r, p, y);
        q.setRPY(r, p, 0);
        q.normalize();
        camera_position.orientation.x = q[0];
        camera_position.orientation.y = q[1];
        camera_position.orientation.z = q[2];
        camera_position.orientation.w = q[3];
        camera->setPose(camera_position);
    }
    catch (tf2::TransformException &ex) {
        return;
    }
    if (fallen) {
        try {
            const cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

            // Field Boundary Detector
            cv::Mat dst, cdst;
            cv::Mat hsv, mask, mask2, out;

            cvtColor(image, hsv , cv::COLOR_BGR2HSV);

            // cv::inRange(hsv,cv::Scalar (0,0,255 - 15) , cv::Scalar(255, 15, 255), mask); Goal post detection
            // cv::Scalar (0,0,200), cv::Scalar(179, 77, 255) old
            // cv::inRange(hsv,cv::Scalar (0,0,255 - 110) , cv::Scalar(255, 110, 255), mask); pretty good

            cv::inRange(hsv,cv::Scalar (45, 115,45), cv::Scalar(70, 255,255), mask2); // old


            std::vector<std::vector<cv::Point> > contours;
            cv::findContours( mask2, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
            std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
            std::vector<cv::Rect> boundRect( contours.size() );
            std::vector<cv::Point2f>centers( contours.size() );
            std::vector<float>radius( contours.size() );
            int count = 0;
            cv::RNG rng(12345);
            std::vector< std::vector<cv::Point> > hull(contours.size());
            count =0;
            for(int i = 0; i < contours.size(); i++) {
                if (cv::contourArea(contours[i])  >  1000 ) {
                    convexHull(cv::Mat(contours[i]), hull[i], false);

                    count += 1;
                    boundRect[count] = boundingRect(hull[i]);

                }

            }
            // Merge largest contours
            if (count > 0) {
                cv::Rect final = boundRect[0];
                for (const auto &r : boundRect) { final |= r; }
                cv::Scalar color = cv::Scalar(0, 0, 255); // blue - color for convex hull
                cv::rectangle(image, final.tl(), final.br(), color, 2);

                // Top Black rectangle
                cv::rectangle(image, cv::Point(0,0), cv::Point(final.br().x,final.tl().y), cv::Scalar(0, 0, 0), -1, 8);
                // Bottom Black rectangle
                // TODO Hardcoded second point needs to take camera info
                cv::rectangle(image, cv::Point(final.tl().x,final.br().y), cv::Point(640,480), cv::Scalar(0, 0, 0), -1, 8);

            }

            // Field Line Detection
            cvtColor(image, hsv , cv::COLOR_BGR2HSV);
            cv::inRange(hsv,cv::Scalar (0,0,255 - 65) , cv::Scalar(255, 65, 255), mask);

            cv::bitwise_and(image,image,out,mask);


            cvtColor(out, cdst, CV_BGR2GRAY);

            cv::threshold(cdst,dst, 127, 255,cv::THRESH_BINARY);

            //cv::adaptiveThreshold(cdst,dst, 255,cv::ADAPTIVE_THRESH_GAUSSIAN_C,cv::THRESH_BINARY,11,2);

            cv::Canny(dst, cdst,50,150);

            HoughLinesP(cdst, lines, rho, theta, threshold, minLineLength, maxLineGap);
            cvtColor(cdst, dst, CV_GRAY2BGR);
            for (const auto &l : lines) {
                cv::line(dst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);

                Point2 pt1(l[0], l[1]);
                Point2 pt2(l[2], l[3]);
                Segment2 s(pt1, pt2);
                float b = l[1] - s.slope() * l[0];

                for (size_t i = l[0]; i < l[2]; i += 15) {
                    float y = s.slope() * i + b;
                    Point2 pt(i, y);
                    pts.push_back(pt);
                }
            }

            sensor_msgs::ImagePtr message = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();

            image_publisher.publish(message);

        } catch (const cv_bridge::Exception &e) {
            ROS_ERROR_STREAM("CV Exception" << e.what());
        }

        std::vector<Point3> points3d;
        for (const auto &p : pts) {
            Point3 p2 = camera->FindFloorCoordinate(p.x, p.y);
            points3d.push_back(p2);
        }

        // Publish fieldlines in a LaserScan data format
        sensor_msgs::PointCloud2 point_cloud_msg;
        //Setting up PointCloud2 msg
        point_cloud_msg.header.stamp = msg->header.stamp;
        point_cloud_msg.header.frame_id = robotName + "/base_camera";
        point_cloud_msg.height = 1;
        point_cloud_msg.width = points3d.size();
        point_cloud_msg.is_bigendian = false;
        point_cloud_msg.is_dense = false;
        point_cloud_msg.point_step = 12;
        point_cloud_msg.row_step = 12 * points3d.size();

        //Adding points to the PointCloud2 msg
        sensor_msgs::PointCloud2Modifier modifier(point_cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(points3d.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg, "z");

        for (const auto &p : points3d) {
            *iter_x = p.x;
            *iter_y = p.y;
            *iter_z = p.z;
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }
        if (point_cloud_publisher.getNumSubscribers() > 0) {
            point_cloud_publisher.publish(point_cloud_msg);

        }
        pts.clear();
        points3d.clear();

        geometry_msgs::TransformStamped camera_footprint = camera_pose;
        camera_footprint.header.frame_id = robotName + "/base_footprint";
        camera_footprint.child_frame_id = robotName + "/base_camera";
        camera_footprint.header.stamp = msg->header.stamp;
        camera_footprint.header.seq = msg->header.seq;

        tf2::Quaternion q(camera_footprint.transform.rotation.x,
                          camera_footprint.transform.rotation.y,
                          camera_footprint.transform.rotation.z,
                          camera_footprint.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double r, p, y;
        m.getRPY(r, p, y);
        q.setRPY(0, 0, y);
        q.normalize();
        camera_footprint.transform.rotation.x = q[0];
        camera_footprint.transform.rotation.y = q[1];
        camera_footprint.transform.rotation.z = q[2];
        camera_footprint.transform.rotation.w = q[3];
        camera_footprint.transform.translation.z = 0;

        br.sendTransform(camera_footprint);
    }
}

void SoccerFieldlineDetector::fallenCallBack(const std_msgs::Bool &msg){
    fallen = msg.data;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "soccer_fieldline_detector");

    SoccerFieldlineDetector detector;

    ros::spin();

    return 0;
}
