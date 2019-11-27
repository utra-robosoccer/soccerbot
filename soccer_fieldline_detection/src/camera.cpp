#include <soccer_fieldline_detection/camera.hpp>


Camera::Camera(const Pose3 &pose) {
    this->pose = pose;

    cameraInfoSubscriber = nh.subscribe<sensor_msgs::CameraInfo>("camera/camera_info", 1, &Camera::cameraInfoCallback, this);
}
Camera::Camera(const Pose3 &pose,int resx, int resy){
    this->pose = pose;
    resolution_x = resx;
    resolution_y = resy;

}

void Camera::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &camera_info) {
    resolution_x = camera_info->width;
    resolution_y = camera_info->height;
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
    return (Camera::resolution_x / 2.f - pos_x) * PixelWidth();

}

float Camera::ImageSensorLocation_Y(int pos_x, int pos_y) {
    return (Camera::resolution_y / 2.f - pos_y) * PixelHeight();

}

Point3 Camera::FindFloorCoordinate(int pos_x, int pos_y) {

    float tx = ImageSensorLocation_X(pos_x, pos_y);
    float ty = ImageSensorLocation_Y(pos_x, pos_y);

    //New version of code
    tf2::Transform pixel_pose(tf2::Quaternion  (0, 0, 0, 1), tf2::Vector3 (focal_length, tx, ty));
    tf2::Transform camera_pose (tf2::Quaternion (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w), tf2::Vector3 (pose.position.x, pose.position.y, pose.position.z));
    pixel_pose.mult(camera_pose, pixel_pose);

    //Raytrace that pixel onto the floor
    float ratio = (pose.position.z - pixel_pose.getOrigin().getZ()) / pose.position.z;
    float xdelta = (pixel_pose.getOrigin().getX() - pose.position.x) / ratio;
    float ydelta = (pixel_pose.getOrigin().getY() - pose.position.y) / ratio;

    Point3 point(xdelta, ydelta, 0);

    return point;
}

void Camera::setPose(const Pose3 &pose) {
    Camera::pose = pose;
}

Pose3 Camera::getPose() {
    return Camera::pose;
}

