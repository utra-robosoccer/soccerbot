#include <gtest/gtest.h>
#include <ros/ros.h>

class SoccerFieldlineDetectorFixture : public ::testing::Test {
protected:
};

TEST_F(SoccerFieldlineDetectorFixture, TestAdder) {

}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_fieldline_detector");
    return RUN_ALL_TESTS();
}