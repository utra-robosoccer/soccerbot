#include <soccer_fieldline_detection/camera.hpp>
#include <geometry/point3.hpp>
#include <geometry/pose3.hpp>
#include <soccer_fieldline_detection/transform.hpp>

Camera::Camera(const Pose3 &pose, int resolution_y, int resolution_x) {
    Camera::resolution_x = resolution_x;
    Camera::resolution_y = resolution_y;
    Camera::pose = pose;


}

void Camera::DrawPixelRayTrace(int pixel_y, int pixel_x) {

}

float Camera::VerticalFOV() {
    return Camera::resolution_y / sqrt(std::pow(Camera::resolution_x, 2.0) + std::pow(Camera::resolution_y, 2.0));
}

float Camera::HorizontalFOV() {
    return Camera::resolution_x / sqrt(pow(Camera::resolution_x, 2.0) + pow(Camera::resolution_y, 2.0));
}

float Camera::ImageSensorHeight() {
    return tan(Camera::VerticalFOV() / 2.0) * 2.0 * Camera::focal_length;

}

float Camera::ImageSensorWidth() {
    return tan(Camera::HorizontalFOV() / 2.0) * 2.0 * Camera::focal_length;
}

float Camera::PixelHeight() {
    return Camera::ImageSensorHeight() / Camera::resolution_y;

}

float Camera::PixelWidth() {
    return Camera::ImageSensorWidth() / Camera::resolution_x;

}

float Camera::ImageSensorLocation_X(int pos_x, int pos_y) {

    return (pos_x - Camera::resolution_x / 2.0) * PixelWidth();

}

float Camera::ImageSensorLocation_Y(int pos_x, int pos_y) {

    return (pos_y - Camera::resolution_y / 2.0) * PixelHeight();

}

Point3 Camera::FindFloorCoordinate(int pos_x, int pos_y) {


    float tx = Camera::ImageSensorLocation_X(pos_x, pos_y);
    float ty = Camera::ImageSensorLocation_Y(pos_x, pos_y);


    float tmp1[3] = {Camera::focal_length / 1000, tx / 1000, ty / 1000};
    float tmp2[3] = {static_cast<float>(Camera::pose.position.x), static_cast<float>(Camera::pose.position.y),
                     static_cast<float>(Camera::pose.position.z)};
    float tmp3[4] = {static_cast<float>(Camera::pose.orientation.w), static_cast<float>(Camera::pose.orientation.x),
                     static_cast<float>(Camera::pose.orientation.y), static_cast<float>(Camera::pose.orientation.z)};
    transform pixelrelLocation3d = transform(tmp1);
    transform t2 = transform(tmp2, tmp3);

    transform pixelLocation3d = pixelrelLocation3d.ApplyTransformation(pixelrelLocation3d, t2);

    //Raytrace that pixel onto the floor
    float ratio = (pixelLocation3d.position[2] - Camera::pose.position.z) / Camera::pose.position.z;
    float xdelta = (pixelLocation3d.position[0] - Camera::pose.position.x) / ratio;
    float ydelta = (pixelLocation3d.position[1] - Camera::pose.position.y) / ratio;

    Point3 point(Camera::pose.position.x - xdelta, Camera::pose.position.y - ydelta, 0);

    return point;
}