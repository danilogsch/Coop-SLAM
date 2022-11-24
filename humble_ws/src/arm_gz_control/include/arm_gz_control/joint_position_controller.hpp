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
#ifndef ARM_GZ_CONTROL_JOINT_POSITION_CONTROLLER_H
#define ARM_GZ_CONTROL_JOINT_POSITION_CONTROLLER_H

#include <gz/transport/Node.hh>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unordered_map>
#include <ignition/msgs.hh>
#include <gz/transport.hh>

namespace arm_gz_control {

class JointPositionController {
public:
    JointPositionController(const rclcpp::Node::SharedPtr& nh,
        const std::vector<std::string>& joint_names,
        const std::string& ros_cmd_topic,
        const std::vector<std::string>& gz_cmd_topics);
    ~JointPositionController() {};

private:
    void setJointPositionCb(const sensor_msgs::msg::JointState::SharedPtr msg);

private:
    rclcpp::Node::SharedPtr nh_;
    std::shared_ptr<gz::transport::Node> gz_node_;
    // ros pub and sub
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ros_cmd_joint_state_sub_;
    //GZ pub
    std::vector<std::shared_ptr<gz::transport::Node::Publisher>> gz_cmd_joint_pubs_;
    // joint names and map
    std::vector<std::string> joint_names_;
    std::unordered_map<std::string, int> joint_names_map_;
};
}
#endif //ARM_GZ_CONTROL_JOINT_POSITION_CONTROLLER_H
