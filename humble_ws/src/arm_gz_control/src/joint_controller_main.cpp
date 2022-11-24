/*******************************************************************************
 *  Copyright (c) Gezp (https://github.com/gezp), All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify it 
 *  under the terms of the MIT License, See the MIT License for more details.
 *
 *  You should have received a copy of the MIT License along with this program.
 *  If not, see <https://opensource.org/licenses/MIT/>.
 *
 ******************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include <arm_gz_control/joint_position_controller.hpp>
#include <arm_gz_control/joint_trajectory_controller.hpp>

int main(int argc, char* argv[])
{
    //creat ros2 node
    rclcpp::init(argc, argv);
    auto ros_node = std::make_shared<rclcpp::Node>("joint_controller");
    // variable
    std::vector<std::string> joint_names;
    std::vector<std::string> gz_joint_topics;
    int update_rate;
    // parameters
    ros_node->declare_parameter<std::vector<std::string>>("joint_names");
    ros_node->declare_parameter<std::vector<std::string>>("gz_joint_topics");
    ros_node->declare_parameter<int64_t>("rate", 200);
    joint_names = ros_node->get_parameter("joint_names").as_string_array();
    gz_joint_topics = ros_node->get_parameter("gz_joint_topics").as_string_array();
    update_rate = ros_node->get_parameter("rate").as_int();
    // create controller
    auto joint_trajectory_controller = std::make_shared<arm_gz_control::jointTrajectoryController>(ros_node,
        joint_names, "set_joint_trajectory", gz_joint_topics ,update_rate);
    // create controller 
    auto joint_position_controller = std::make_shared<arm_gz_control::JointPositionController>(ros_node,
        joint_names, "set_joint_state", gz_joint_topics);
    // run node until it's exited
    rclcpp::spin(ros_node);
    //clean up
    rclcpp::shutdown();
    return 0;
}
