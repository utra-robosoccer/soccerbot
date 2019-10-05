#include <soccer_fieldline_detection/camera.hpp>
#include <geometry/point3.hpp>
#include <geometry/transform.hpp>
Camera::Camera(const Pose3 &pose, int resolution_y, int resolution_x) {
    Camera::resolution_x = resolution_x;
    Camera::resolution_y = resolution_y;
    Camera::pose = pose;


}

void Camera::DrawPixelRayTrace(int pixel_y, int pixel_x) {

}

float Camera::VerticalFOV() {
    return Camera::resolution_y / sqrt(Camera::resolution_x ^ 2 + Camera::resolution_y ^ 2);
}

float Camera::HorizontalFOV() {
    return Camera::resolution_x / sqrt(Camera::resolution_x ^ 2 + Camera::resolution_y ^ 2);
}

float Camera::ImageSensorHeight() {
    return tan(Camera::VerticalFOV() / 2) * 2 * Camera::focal_length;

}

float Camera::ImageSensorWidth() {
    return tan(Camera::HorizontalFOV() / 2) * 2 * Camera::focal_length;


}

int Camera::PixelHeight() {
    return Camera::ImageSensorHeight() / Camera::resolution_y;

}

int Camera::PixelWidth() {
    return Camera::ImageSensorWidth() / Camera::resolution_x;

}

int Camera::ImageSensorLocation_X(int pos_x, int pos_y) {

    return (pos_x - Camera::resolution_x / 2) * PixelWidth();

}

int Camera::ImageSensorLocation_Y(int pos_x, int pos_y) {

    return (pos_y - Camera::resolution_y / 2) * PixelHeight();

}

Point3 Camera::FindFloorCoordinate(int pos_x, int pos_y) {
    float tx = Camera::ImageSensorLocation_X(pos_x, pos_y);
    float ty = Camera::ImageSensorLocation_Y(pos_x, pos_y);

    float tmp1 [3]= {focal_length / 1000, tx / 1000, ty / 1000};
    transform pixelrelLocation3d = transform(tmp1);

    float tmp2 [3]= {static_cast<float>(Camera::pose.position.x),static_cast<float>(Camera::pose.position.y),static_cast<float>(Camera::pose.position.z)};
    transform t2 = transform(tmp2);

    transform pixelLocation3d = pixelrelLocation3d.ApplyTransformation(pixelrelLocation3d,t2);


    //Raytrace that pixel onto the floor
    float ratio = (pixelLocation3d.position[2]- Camera::pose.position.z) / Camera::pose.position.z;
    float xdelta = (pixelLocation3d.position[0] - Camera::pose.position.x) / ratio;
    float ydelta = (pixelLocation3d.position[1]- Camera::pose.position.y) / ratio;

    Point3  point (Camera::pose.position.x - xdelta,Camera::pose.position.y - ydelta,0);

    return point;







    /*//Initializing the H matrix
    float H1[4][4];
    float H2[4][4];
    float Hnew[4][4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            H1[i][j] = 0;

            H2[i][j] = 0;
            Hnew[i][j] = 0;
        }
    }
    H1[0][3] = focal_length / 1000;
    H1[1][3] = tx / 1000;
    H1[2][3] = ty / 1000;

    H2[0][3] = Camera::pose.position.x;
    H2[1][3] = Camera::pose.position.y;
    H2[2][3] = Camera::pose.position.z;

    //Matrix multiplication
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
           for (int k = 0; k< 4; k++){
               Hnew[i][j] +=  H1[i][k] * H2[k][j];
           }

        }
    }

    //Coordinate of pixel in real space
    float pixelLocation3d [3] = {Hnew[0][3],Hnew[1][3],Hnew[2][3]};

    //Raytrace that pixel onto the floor
    float ratio = (pixelLocation3d[2] - Camera::pose.position.z) / Camera::pose.position.z;
    float xdelta = (pixelLocation3d[0] - Camera::pose.position.x) / ratio;
    float ydelta = (pixelLocation3d[1] - Camera::pose.position.y) / ratio;

    Point3  point (Camera::pose.position.x - xdelta,Camera::pose.position.y - ydelta,0);

    return point;*/



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