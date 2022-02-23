#include <google/protobuf/timestamp.pb.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include "robocup_extension.pb.h"
#include "utils.hpp"

int main(void) {
    // Create a new message which contains extensions
    robocup::humanoid::Message msg;

    // Set the transmission timestamp
    auto d       = std::chrono::system_clock::now().time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(d);
    auto nanos   = std::chrono::duration_cast<std::chrono::nanoseconds>(d - seconds);
    msg.mutable_timestamp()->set_seconds(seconds.count());
    msg.mutable_timestamp()->set_nanos(nanos.count());

    // Set player details
    msg.mutable_current_pose()->set_player_id(3);
    msg.mutable_current_pose()->set_team(robocup::humanoid::Team::BLUE);
    msg.mutable_current_pose()->mutable_position()->set_x(1.0f);
    msg.mutable_current_pose()->mutable_position()->set_y(3.0f);
    msg.mutable_current_pose()->mutable_position()->set_z(M_PI / 3);
    msg.mutable_current_pose()->mutable_covariance()->mutable_x()->set_x(1.0f);
    msg.mutable_current_pose()->mutable_covariance()->mutable_x()->set_y(0.0f);
    msg.mutable_current_pose()->mutable_covariance()->mutable_x()->set_z(0.0f);
    msg.mutable_current_pose()->mutable_covariance()->mutable_y()->set_x(0.0f);
    msg.mutable_current_pose()->mutable_covariance()->mutable_y()->set_y(1.0f);
    msg.mutable_current_pose()->mutable_covariance()->mutable_y()->set_z(0.0f);
    msg.mutable_current_pose()->mutable_covariance()->mutable_z()->set_x(0.0f);
    msg.mutable_current_pose()->mutable_covariance()->mutable_z()->set_y(0.0f);
    msg.mutable_current_pose()->mutable_covariance()->mutable_z()->set_z(1.0f);

    // Set ball details
    msg.mutable_ball()->mutable_position()->set_x(2.0f);
    msg.mutable_ball()->mutable_position()->set_y(3.0f);
    msg.mutable_ball()->mutable_position()->set_z(1.0f);
    msg.mutable_ball()->mutable_velocity()->set_x(0.5f);
    msg.mutable_ball()->mutable_velocity()->set_y(0.5f);
    msg.mutable_ball()->mutable_velocity()->set_z(-0.5f);
    msg.mutable_ball()->mutable_covariance()->mutable_x()->set_x(1.0f);
    msg.mutable_ball()->mutable_covariance()->mutable_x()->set_y(0.0f);
    msg.mutable_ball()->mutable_covariance()->mutable_x()->set_z(0.0f);
    msg.mutable_ball()->mutable_covariance()->mutable_y()->set_x(0.0f);
    msg.mutable_ball()->mutable_covariance()->mutable_y()->set_y(1.0f);
    msg.mutable_ball()->mutable_covariance()->mutable_y()->set_z(0.0f);
    msg.mutable_ball()->mutable_covariance()->mutable_z()->set_x(0.0f);
    msg.mutable_ball()->mutable_covariance()->mutable_z()->set_y(0.0f);
    msg.mutable_ball()->mutable_covariance()->mutable_z()->set_z(1.0f);

    // Set walk command
    msg.mutable_walk_command()->set_x(0.15f);
    msg.mutable_walk_command()->set_y(0.15f);
    msg.mutable_walk_command()->set_z(-0.05f);

    // Set target pose
    msg.mutable_target_pose()->set_player_id(3);
    msg.mutable_target_pose()->set_team(robocup::humanoid::Team::BLUE);
    msg.mutable_target_pose()->mutable_position()->set_x(2.0f);
    msg.mutable_target_pose()->mutable_position()->set_y(3.0f);
    msg.mutable_target_pose()->mutable_position()->set_z(-M_PI / 3);

    // Set kick target
    msg.mutable_kick_target()->set_x(4.5f);
    msg.mutable_kick_target()->set_y(0.0f);

    // Set state
    msg.set_state(robocup::humanoid::State::UNPENALISED);

    // Set some others details
    robocup::humanoid::Robot* other = msg.add_others();
    other->set_player_id(2);
    other->set_team(robocup::humanoid::Team::BLUE);
    other->mutable_position()->set_x(1.0f);
    other->mutable_position()->set_y(-3.0f);
    other->mutable_position()->set_z(M_PI / 6);
    other->mutable_covariance()->mutable_x()->set_x(1.0f);
    other->mutable_covariance()->mutable_x()->set_y(0.0f);
    other->mutable_covariance()->mutable_x()->set_z(0.0f);
    other->mutable_covariance()->mutable_y()->set_x(0.0f);
    other->mutable_covariance()->mutable_y()->set_y(1.0f);
    other->mutable_covariance()->mutable_y()->set_z(0.0f);
    other->mutable_covariance()->mutable_z()->set_x(0.0f);
    other->mutable_covariance()->mutable_z()->set_y(0.0f);
    other->mutable_covariance()->mutable_z()->set_z(1.0f);

    other = msg.add_others();
    other->set_player_id(1);
    other->set_team(robocup::humanoid::Team::RED);
    other->mutable_position()->set_x(4.5f);
    other->mutable_position()->set_y(0.0f);
    other->mutable_position()->set_z(M_PI);
    other->mutable_covariance()->mutable_x()->set_x(1.0f);
    other->mutable_covariance()->mutable_x()->set_y(0.0f);
    other->mutable_covariance()->mutable_x()->set_z(0.0f);
    other->mutable_covariance()->mutable_y()->set_x(0.0f);
    other->mutable_covariance()->mutable_y()->set_y(1.0f);
    other->mutable_covariance()->mutable_y()->set_z(0.0f);
    other->mutable_covariance()->mutable_z()->set_x(0.0f);
    other->mutable_covariance()->mutable_z()->set_y(0.0f);
    other->mutable_covariance()->mutable_z()->set_z(1.0f);

    other = msg.add_others();
    other->set_player_id(0);
    other->set_team(robocup::humanoid::Team::UNKNOWN_TEAM);
    other->mutable_position()->set_x(4.0f);
    other->mutable_position()->set_y(0.0f);
    other->mutable_position()->set_z(M_PI);
    other->mutable_covariance()->mutable_x()->set_x(1.0f);
    other->mutable_covariance()->mutable_x()->set_y(0.0f);
    other->mutable_covariance()->mutable_x()->set_z(0.0f);
    other->mutable_covariance()->mutable_y()->set_x(0.0f);
    other->mutable_covariance()->mutable_y()->set_y(1.0f);
    other->mutable_covariance()->mutable_y()->set_z(0.0f);
    other->mutable_covariance()->mutable_z()->set_x(0.0f);
    other->mutable_covariance()->mutable_z()->set_y(0.0f);
    other->mutable_covariance()->mutable_z()->set_z(1.0f);

    // ******************************
    // * Official message ends here *
    // ******************************

    // Set max walking speed (an extension value)
    msg.set_max_walking_speed(0.25f);

    // Dump serialised message to file
    std::ofstream ofs("extended_message.pb", std::ofstream::binary);
    std::string string_msg;
    msg.SerializeToString(&string_msg);
    ofs.write(string_msg.data(), string_msg.size());
    ofs.close();

    return 0;
}
