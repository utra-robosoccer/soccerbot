#include <ros/ros.h>
#include <soccer_fieldline_detection/test_soccer_fieldline_detector.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry/segment2.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
SoccerFieldlineDetector::SoccerFieldlineDetector() {
    image_transport::ImageTransport it(nh);
    image_subscriber = it.subscribe("front_camera/image_raw", 1, &SoccerFieldlineDetector::imageCallback, this);
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0;

    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
}
void SoccerFieldlineDetector::callback(const sensor_msgs::PointCloud2ConstPtr& msg) {

}
void SoccerFieldlineDetector::pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    pose.position.x = msg->pose.pose.position.x;
    pose.position.y = msg->pose.pose.position.y;
    pose.position.z = msg->pose.pose.position.z;

    pose.orientation.x = msg->pose.pose.orientation.x;
    pose.orientation.y = msg->pose.pose.orientation.y;
    pose.orientation.z = msg->pose.pose.orientation.z;
    pose.orientation.w = msg->pose.pose.orientation.w;
}

void SoccerFieldlineDetector::initPose(double x, double y, double theta) {

    ros::Publisher pub = SoccerFieldlineDetector::nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("/initialpose",1);

    geometry_msgs::PoseWithCovarianceStamped tmpPose;
    tmpPose.header.frame_id = "map";
    tmpPose.header.stamp = ros::Time::now();

    tmpPose.pose.pose.position.x = x;
    tmpPose.pose.pose.position.y = y;
    tmpPose.pose.pose.position.z = 0.0;

    tf::Quaternion q;
    q.setRPY(0.0,0.0,theta);
    tf::quaternionStampedTFToMsg(q,tmpPose.pose.pose.orientation);

    tmpPose.pose.covariance[6] = 0.5 * 0.5;
    tmpPose.pose.covariance[7] = 0.5 * 0.5;
    tmpPose.pose.covariance[6*5 + 5] = (M_PI/12.0 )*(M_PI/12.0 );

    pub.publish((tmpPose));


}

void SoccerFieldlineDetector::imageCallback(const sensor_msgs::ImageConstPtr &msg) {

    try {
        const cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        cv::Point start = cv::Point(300, 300);
        cv::Point end = cv::Point(500, 500);
        cv::line(image, start, end, cv::Scalar(0, 0, 0), 3);

        // Detect Field Lines (Copy from simulink)
        cv::Mat dst, cdst;
        int cannythreshold1 = 200;
        int cannythreshold2 = 400;
        nh.getParam("soccer_fieldline_detector/cannythreshold1", cannythreshold1);
        nh.getParam("soccer_fieldline_detector/cannythreshold2", cannythreshold2);

        cv::Canny(
                image,
                dst,
                cannythreshold1,
                cannythreshold2);

        cvtColor(dst, cdst, CV_GRAY2BGR);

        // Cover Horizon (Ignore for now)

        // Organize Fieldlines
        std::vector<cv::Vec4i> lines;
        double rho = 1;
        double theta = CV_PI / 180;
        int threshold = 50;
        int minLineLength = 50;
        int maxLineGap = 50;
        nh.getParam("soccer_fieldline_detector/houghRho", rho);
        nh.getParam("soccer_fieldline_detector/houghTheta", theta);
        nh.getParam("soccer_fieldline_detector/houghThreshold", threshold);
        nh.getParam("soccer_fieldline_detector/houghMinLineLength", minLineLength);
        nh.getParam("soccer_fieldline_detector/houghMaxLineGap", maxLineGap);

        ROS_INFO_STREAM(theta);

        HoughLinesP(
                dst,
                lines,
                rho,
                theta,
                threshold,
                minLineLength,
                maxLineGap);

        std::vector<Point2> pts;

        for (auto l : lines) {
            cv::line(cdst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, CV_AA);

            Point2 pt1(l[0],l[1]);
            Point2 pt2(l[2],l[3]);
            Segment2 s(pt1,pt2);
            float b = l[1] - s.slope()*l[0];

            for (int i = l[0]; i < l[2];i++) {
                float y = s.slope()*i + b;
                Point2 pt(i,y);
                pts.push_back(pt);

            }

        }


        ROS_INFO_STREAM("Line n: " + std::to_string(lines.size()));

        // Project fieldlines from 2d to 3d
        ros::Subscriber amcl_pose = SoccerFieldlineDetector::nh.subscribe("/amcl_pose",100,&SoccerFieldlineDetector::pose_callback,this);

        Camera cam (pose,240,360);
        std::vector<Point3> rPts;
        for (auto p : pts) {
            Point3 p2 = cam.FindFloorCoordinate(p.x, p.y);
            rPts.push_back(p2);
        }

        // Publish fieldlines in a LaserScan data format
        sensor_msgs::PointCloud2 msg;

        msg.header.stamp = ros::Time::now();
        msg.height = 1;
        msg.width = rPts.size();
        msg.is_bigendian = false;
        msg.is_dense = false;
        msg.point_step = 12;
        msg.row_step = 12*rPts.size();

        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2Fields(1,"xyz");
        modifier.resize(rPts.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");

        for (auto p : rPts){
            *iter_x = p.x;
            *iter_y = p.y;
            *iter_z = p.z;

            ++iter_x;
            ++iter_y;
            ++iter_z;

        }


        ros::Publisher cloudIn = SoccerFieldlineDetector::nh.advertise<sensor_msgs::PointCloud2> ("/cloud_in",1);

        cloudIn.publish(msg);

        ros::Subscriber laserScan = SoccerFieldlineDetector::nh.subscribe("/scan",1, &SoccerFieldlineDetector::callback,this);



        cv::imshow("original view", image);
        cv::imshow("edge view", cdst);
        cv::waitKey(1);

    } catch (const cv_bridge::Exception &e) {
        ROS_ERROR_STREAM("CV Exception" << e.what());
    }
}






int main(int argc, char **argv) {
    ros::init(argc, argv, "soccer_fieldline_detector");

    SoccerFieldlineDetector detector;

    detector.initPose(0,0,0);

    ros::spin();

    return 0;
}