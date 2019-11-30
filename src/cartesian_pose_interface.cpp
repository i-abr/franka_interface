// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <cmath>
#include <array>
#include <memory>
#include <stdexcept>
#include <string>

#include <controller_interface/controller_base.h>
#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_cartesian_command_interface.h>

#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/robot_hw.h>

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <geometry_msgs/Pose.h>

namespace franka_interface {


    class CartesianPoseInterface : public controller_interface::MultiInterfaceController<
                                            franka_hw::FrankaPoseCartesianInterface,
                                            franka_hw::FrankaStateInterface>
    {
    private:

        franka_hw::FrankaPoseCartesianInterface*                cartesian_pose_interface_;
        std::unique_ptr<franka_hw::FrankaCartesianPoseHandle>   cartesian_pose_handle_;
        ros::Duration           elapsed_time_;
        std::array<double, 16>  initial_pose_{};

        ros::Subscriber     pose_cmd_sub;
        bool                read_message = false;
        float               decay_rate   = 0.8;

        std::array<double, 6> pose_cmd        = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        std::array<double, 6> filtered_cmd    = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        float alpha = 0.9999;

    public:
        bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle)
        {

            pose_cmd_sub = node_handle.subscribe("/pose_cmd", 1, &CartesianPoseInterface::cmd_callback, this);

            cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();
            if (cartesian_pose_interface_ == nullptr) {
                ROS_ERROR(
                  "CartesianPoseExampleController: Could not get Cartesian Pose "
                  "interface from hardware");
                return false;
            }

            std::string arm_id;
            if (!node_handle.getParam("arm_id", arm_id)) {
                ROS_ERROR("CartesianPoseExampleController: Could not get parameter arm_id");
                return false;
            }

            try {
                cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
                                        cartesian_pose_interface_->getHandle(arm_id + "_robot")
                                    );
            } catch (const hardware_interface::HardwareInterfaceException& e) {
                ROS_ERROR_STREAM(
                    "CartesianPoseExampleController: Exception getting Cartesian handle: " << e.what());
                return false;
            }

            auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
            try {
                auto state_handle = state_interface->getHandle(arm_id + "_robot");

                std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
                for (size_t i = 0; i < q_start.size(); i++) {
                    if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
                        ROS_ERROR_STREAM(
                          "CartesianPoseExampleController: Robot is not in the expected starting position for "
                          "running this example. Move robot to init pose first.");
                        return false;
                    }
                }
            } catch (const hardware_interface::HardwareInterfaceException& e) {
                ROS_ERROR_STREAM(
                    "CartesianPoseExampleController: Exception getting state handle: " << e.what());
                return false;
            }

            return true;
        }

        void starting(const ros::Time&) {
            initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
            elapsed_time_ = ros::Duration(0.0);
        }

        void update(const ros::Time&, const ros::Duration& period) {

            // if (read_message == true) {
            //     // resetting the duration if there was a message
            //     elapsed_time_ = ros::Duration(0.);
            read_message  = false;
            // }
            // else {
            //     // updating if there wasn't, there could be something wrong
            //     elapsed_time_ += period;
            // }
            elapsed_time_ += period;

            // std::array<double, 16> new_pose = initial_pose_;

            // if (elapsed_time_.toSec() > 0.2) { // not sure if I want to look at 0.1 s instead....
            //     filtered_cmd[0] = decay_rate * filtered_cmd[0]; // slow the robot
            //     filtered_cmd[2] = decay_rate * filtered_cmd[2];
            // }
            // else {
            //     filtered_cmd[0] = alpha * filtered_cmd[0] + (1.0-alpha) * pose_cmd[0];
            //     filtered_cmd[2] = alpha * filtered_cmd[2] + (1.0-alpha) * pose_cmd[2];
            // }
            filtered_cmd[0] = alpha * filtered_cmd[0] + (1.0-alpha) * pose_cmd[0];
            filtered_cmd[2] = alpha * filtered_cmd[2] + (1.0-alpha) * pose_cmd[2];
            // double radius = 0.3;
            // double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
            // double delta_x = radius * std::sin(angle);
            // double delta_z = radius * (std::cos(angle) - 1);
            std::array<double, 16> new_pose = initial_pose_;
            // std::cout << filtered_cmd[0] << " " << filtered_cmd[2] << std::endl;
            //
            new_pose[12] -= filtered_cmd[0];
            new_pose[14] -= filtered_cmd[2];

            // std::cout << new_pose[12] << " " << new_pose[14] << std::endl;
            cartesian_pose_handle_->setCommand(new_pose);

            // double radius = 0.3;
            // double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
            // double delta_x = radius * std::sin(angle);
            // double delta_z = radius * (std::cos(angle) - 1);
            // std::array<double, 16> new_pose = initial_pose_;
            // std::cout << delta_x << " " << delta_z << std::endl;
            // new_pose[12] -= delta_x;
            // new_pose[14] -= delta_z;
            // cartesian_pose_handle_->setCommand(new_pose);
        }

        void cmd_callback(const geometry_msgs::Pose::ConstPtr& msgs) {
            pose_cmd[0] = msgs->position.x;
            pose_cmd[2] = msgs->position.z;
            read_message = true;
        }

    };
}  // namespace franka_interface

PLUGINLIB_EXPORT_CLASS(franka_interface::CartesianPoseInterface,
                       controller_interface::ControllerBase)
