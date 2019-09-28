#include <soccer_fieldline_detection/camera.hpp>
#include <geometry/point3.hpp>

Camera::Camera(const Pose3 &pose, int resolution_y, int resolution_x) {
    Camera::resolution_x = resolution_x;
    Camera::resolution_y = resolution_y;
    Camera::pose = pose;

}

void Camera::DrawPixelRayTrace(int pixel_y, int pixel_x) {

}



float Camera::VerticalFOV() {
    return Camera::resolution_y / sqrt(Camera::resolution_x^2 + Camera::resolution_y^2);
}

float Camera::HorizontalFOV() {
    return Camera::resolution_x / sqrt(Camera::resolution_x^2 + Camera::resolution_y^2);
}

float Camera::ImageSensorHeight() {
    return tan(Camera::VerticalFOV() / 2) * 2 * Camera::focal_length;

}

float Camera::ImageSensorWidth() {
    return tan(Camera::HorizontalFOV() / 2) * 2 * Camera::focal_length;


}

int Camera::PixelHeight(){
    return Camera::ImageSensorHeight() / Camera::resolution_y;

}

int Camera::PixelWidth(){
    return Camera::ImageSensorWidth() / Camera::resolution_x;

}

int Camera::ImageSensorLocation_X (int pos_x, int pos_y) {

    return (pos_x - Camera::resolution_x / 2) * PixelWidth();

}

int Camera::ImageSensorLocation_Y (int pos_x, int pos_y) {

    return (pos_y - Camera::resolution_y / 2) * PixelHeight();

}

Point3 Camera::FindFloorCoordinate(int pos_x, int pos_y) {
    int tx = Camera::ImageSensorLocation_X(pos_x,pos_y);
    int ty = Camera::ImageSensorLocation_Y(pos_x,pos_y);

   // int  pixelrelLocation3d = Geometry.Transform(Camera::focal_length / 1000, tx / 1000, ty / 1000]);
    Camera::pose.position.x = focal_length / 1000;
    Camera::pose.position.y = tx / 1000;
    Camera::pose.position.z = ty / 1000;

    float H [4] [4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            H[i][j] = 0;
        }
    }
    H[1][4] = Camera::pose.position.x ;
    H[2][4] = Camera::pose.position.y ;
    H[3][4] = Camera::pose.position.z ;

}
/*



 function point3f = FindFloorCoordinate(obj, pixelx, pixely)
            [tx, ty] = obj.ImageSensorLocation(pixelx, pixely);

            % x is direction forward, backward
            % y is direction left right
            % z is direction up down
            pixelrelLocation3d = Geometry.Transform([obj.focal_length / 1000, tx / 1000, ty / 1000]);

            % Coordinate of pixel in real space
            pixelLocation3d = pixelrelLocation3d.ApplyTransformation(obj.pose);

            % Raytrace that pixel onto the floor
            ratio = (pixelLocation3d.Z - obj.pose.Z) / obj.pose.Z;
            xdelta = (pixelLocation3d.X - obj.pose.X) / ratio;
            ydelta = (pixelLocation3d.Y - obj.pose.Y) / ratio;

            point3f = Geometry.Point3f(obj.pose.X - xdelta, obj.pose.Y - ydelta, 0);
        end
 */