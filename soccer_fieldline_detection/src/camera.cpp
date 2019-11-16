#include <soccer_fieldline_detection/camera.hpp>
#include <geometry/point3.hpp>
#include <geometry/pose3.hpp>
#include <soccer_fieldline_detection/transform.hpp>
#include <tf2_ros/buffer.h>

Camera::Camera(const Pose3 &pose) {
    this->pose = pose;

    cameraInfoSubscriber = nh.subscribe<sensor_msgs::CameraInfo>("camera/camera_info", 1, &Camera::cameraInfoCallback, this);
}


void Camera::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &camera_info) {
    resolution_x = camera_info->width;
    resolution_y = camera_info->height;
}

void Camera::setPose(){
    tf2_ros::Buffer tfBuffer;

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

    Pose3 camera_position;
    camera_position.position.x = camera_pose.transform.translation.x;
    camera_position.position.y = camera_pose.transform.translation.y;
    camera_position.position.z = camera_pose.transform.translation.z;
    camera_position.orientation.w = camera_pose.transform.rotation.w;
    camera_position.orientation.x = camera_pose.transform.rotation.x;
    camera_position.orientation.y = camera_pose.transform.rotation.y;
    camera_position.orientation.z = camera_pose.transform.rotation.z;
    this->pose = camera_position;
}

void Camera::DrawPixelRayTrace(int pixel_y, int pixel_x) {

}


float Camera::VerticalFOV() {
    float f = std::sqrt(std::pow(resolution_x,2.f) + std::pow(resolution_y,2.f)) / 2.f * (1 / std::tan(diagonal_fov/2));
    return 2 * std::atan2(resolution_y / 2, f);
}

float Camera::HorizontalFOV() {
    float f = std::sqrt(std::pow(resolution_x,2.f) + std::pow(resolution_y,2.f)) / 2.f * (1 / std::tan(diagonal_fov/2));
    return 2 * std::atan2(resolution_x / 2, f);}

float Camera::ImageSensorHeight() {
    return std::tan(Camera::VerticalFOV() / 2.f) * 2.f * Camera::focal_length;
}

float Camera::ImageSensorWidth() {
    return tan(Camera::HorizontalFOV() / 2.f) * 2.f * Camera::focal_length;
}

float Camera::PixelHeight() {
    return Camera::ImageSensorHeight() / Camera::resolution_y;

}

float Camera::PixelWidth() {
    return Camera::ImageSensorWidth() / Camera::resolution_x;

}

float Camera::ImageSensorLocation_X(int pos_x, int pos_y) {
    return (pos_x - Camera::resolution_x / 2.f) * PixelWidth();

}

float Camera::ImageSensorLocation_Y(int pos_x, int pos_y) {
    return (pos_y - Camera::resolution_y / 2.f) * PixelHeight();

}

Point3 Camera::FindFloorCoordinate(int pos_x, int pos_y) {

    float tx = ImageSensorLocation_X(pos_x, pos_y);
    float ty = ImageSensorLocation_Y(pos_x, pos_y);


    float tmp1[3] = {focal_length / 1000, tx / 1000, ty / 1000};
    float tmp2[3] = {static_cast<float>(pose.position.x), static_cast<float>(pose.position.y),
                     static_cast<float>(pose.position.z)};

    float tmp3[4] = {static_cast<float>(pose.orientation.w), static_cast<float>(pose.orientation.x),
                     static_cast<float>(pose.orientation.y), static_cast<float>(pose.orientation.z)};

    auto pixelrelLocation3d = transform(tmp1);
    transform t2 = transform(tmp2, tmp3);

    transform pixelLocation3d = pixelrelLocation3d.ApplyTransformation(pixelrelLocation3d, t2);

    //Raytrace that pixel onto the floor
    float ratio = (pixelLocation3d.position[2] - pose.position.z) / pose.position.z;
    float xdelta = (pixelLocation3d.position[0] - pose.position.x) / ratio;
    float ydelta = (pixelLocation3d.position[1] - pose.position.y) / ratio;

    Point3 point(pose.position.x - xdelta, pose.position.y - ydelta, 0);

    return point;
}


