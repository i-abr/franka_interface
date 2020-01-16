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
#include <std_msgs/Float32MultiArray.h>

namespace franka_interface {


    class CartesianPoseInterface : public controller_interface::MultiInterfaceController<
                                            franka_hw::FrankaPoseCartesianInterface,
                                            franka_hw::FrankaStateInterface>
    {
    private:

        franka_hw::FrankaPoseCartesianInterface*                cartesian_pose_interface_;
        std::unique_ptr<franka_hw::FrankaCartesianPoseHandle>   cartesian_pose_handle_;
        ros::Duration           elapsed_time_;
        std::array<double, 16>  current_pose_{};

        ros::Subscriber     pose_cmd_sub;
        bool                read_message = false;
        float               decay_rate   = 0.99;

        std::array<double, 16> raw_pose_cmd {};
        std::array<double, 16> filtered_raw_pose_cmd {};
        std::array<double, 16> filtered_target_cmd {};

        float alpha1   = 0.999;
        float alpha2   = 0.99;


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

                // std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
                // for (size_t i = 0; i < q_start.size(); i++) {
                //     if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
                //         ROS_ERROR_STREAM(
                //           "CartesianPoseExampleController: Robot is not in the expected starting position for "
                //           "running this example. Move robot to init pose first.");
                //         return false;
                //     }
                // }
            } catch (const hardware_interface::HardwareInterfaceException& e) {
                ROS_ERROR_STREAM(
                    "CartesianPoseExampleController: Exception getting state handle: " << e.what());
                return false;
            }

            return true;
        }

        void starting(const ros::Time&) {
            current_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
            filtered_raw_pose_cmd = current_pose_;
            raw_pose_cmd = current_pose_;

            elapsed_time_ = ros::Duration(0.0);
        }

        void update(const ros::Time&, const ros::Duration& period) {

            // update the current pose
            std::array<double, 16>  current_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;

            if (read_message == true) {
                // resetting the duration if there was a message
                elapsed_time_ = ros::Duration(0.);

                // if (elapsed_time_.toSec() < 0.2) { // not sure if I want to look at 0.1 s instead....
                    // update the target filter if the elapsed time is ok, otherwise
                    // just stay where it was commanded before

                read_message = false; // reset the message
            }
            else {
                // updating if there wasn't, there could be something wrong
                elapsed_time_ += period;
            }

            for (int i=0; i < 16; i++) {
                filtered_raw_pose_cmd[i] = alpha1 * filtered_raw_pose_cmd[i] + (1.0-alpha1) * raw_pose_cmd[i];
                current_pose_[i] = alpha2 * current_pose_[i] + (1.0-alpha2) * filtered_raw_pose_cmd[i];
            }

            cartesian_pose_handle_->setCommand(current_pose_);

        }

        void cmd_callback(const std_msgs::Float32MultiArray::ConstPtr& msgs) {

            for (int i = 0; i<16 ; i++) {
                raw_pose_cmd[i] = msgs->data[i];
            }
            read_message = true;
        }

    };
}  // namespace franka_interface

PLUGINLIB_EXPORT_CLASS(franka_interface::CartesianPoseInterface,
                       controller_interface::ControllerBase)
