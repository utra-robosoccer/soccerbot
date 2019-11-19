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
    return (pos_x - Camera::resolution_x / 2.f) * PixelWidth();

}

float Camera::ImageSensorLocation_Y(int pos_x, int pos_y) {
    return (pos_y - Camera::resolution_y / 2.f) * PixelHeight();

}

Point3 Camera::FindFloorCoordinate(int pos_x, int pos_y) {

    float tx = ImageSensorLocation_X(pos_x, pos_y);
    float ty = ImageSensorLocation_Y(pos_x, pos_y);

    //old Version of code
    /*float tmp1[3] = {focal_length / 1000, tx / 1000, ty / 1000};
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

    Point3 point(pose.position.x - xdelta, -1*(pose.position.y - ydelta), 0);
    */

    //New version of code
    tf2::Transform pixelLocation3dd(tf2::Quaternion  (0,0,0,1),tf2::Vector3 (focal_length / 1000, tx / 1000, ty / 1000));
    tf2::Transform t2 (tf2::Quaternion (pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w),tf2::Vector3 (pose.position.x,pose.position.y,pose.position.z));
    pixelLocation3dd.mult(t2,pixelLocation3dd);



    //Raytrace that pixel onto the floor
    float ratio = (pixelLocation3dd.getOrigin().getZ() - pose.position.z) / pose.position.z;
    float xdelta = (pixelLocation3dd.getOrigin().getX()- pose.position.x) / ratio;
    float ydelta = (pixelLocation3dd.getOrigin().getY() - pose.position.y) / ratio;

    Point3 point(pose.position.x - xdelta, -1*(pose.position.y - ydelta), 0);


    return point;
}

void Camera::setPose(const Pose3 &pose) {
    Camera::pose = pose;
}


