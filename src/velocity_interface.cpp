// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <cmath>

#include <controller_interface/controller_base.h>
#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>

#include <pluginlib/class_list_macros.h>

namespace franka_interface {


    class VelocityInterface : public controller_interface::MultiInterfaceController<
                                hardware_interface::VelocityJointInterface, franka_hw::FrankaStateInterface>
    {
    private:

        hardware_interface::VelocityJointInterface* velocity_joint_interface_;
        std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
        ros::Subscriber     vel_cmd_sub;
        ros::Duration       elapsed_time_;
        bool                read_message = false;
        float               decay_rate   = 0.8;
        float               max_vel      = 0.2;

        // initialize to 0 so the robot doesn't do crazy things when starting

        std::array<float, 7> vel_cmd        = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        std::array<float, 7> filtered_cmd   = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        float alpha = 0.99;

    public:
        bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) {



            vel_cmd_sub = node_handle.subscribe("/jnt_cmd", 1, &VelocityInterface::cmd_callback, this);
            // Get the velocity HW interface
            velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
            if (velocity_joint_interface_ == nullptr) {
                ROS_ERROR(
                    "JointVelocityExampleController: Error getting velocity joint interface from hardware!");
                return false;
            }

            // Not sure if I will ever get this error but w/e
            std::vector<std::string> joint_names;
            if (!node_handle.getParam("joint_names", joint_names)) {
                ROS_ERROR("JointVelocityExampleController: Could not parse joint names");
            }
            if (joint_names.size() != 7) {
                ROS_ERROR_STREAM("JointVelocityExampleController: Wrong number of joint names, got "
                                 << joint_names.size() << " instead of 7 names!");
                return false;
            }

            velocity_joint_handles_.resize(7);
            for (size_t i = 0; i < 7; ++i) {
                try {
                    velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
                } catch (const hardware_interface::HardwareInterfaceException& ex) {
                  ROS_ERROR_STREAM(
                      "JointVelocityExampleController: Exception getting joint handles: " << ex.what());
                  return false;
                }
            }

            auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();

            if (state_interface == nullptr) {
                ROS_ERROR("JointVelocityExampleController: Could not get state interface from hardware");
                return false;
            }

            // I am removing this because starting the robot every time at a new place is ridiculous
            // try {
            //     auto state_handle = state_interface->getHandle("panda_robot");
            //
            //     std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
            //     for (size_t i = 0; i < q_start.size(); i++) {
            //       if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
            //         ROS_ERROR_STREAM(
            //             "JointVelocityExampleController: Robot is not in the expected starting position for "
            //             "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            //             "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
            //         return false;
            //       }
            //     }
            // } catch (const hardware_interface::HardwareInterfaceException& e) {
            //     ROS_ERROR_STREAM(
            //         "JointVelocityExampleController: Exception getting state handle: " << e.what());
            //         return false;
            // }

            return true;

        }

        void update(const ros::Time&, const ros::Duration& period) {

            // check to see if there were any messages recieved
            if (read_message == true) {
                // resetting the duration if there was a message
                elapsed_time_ = ros::Duration(0.);
                read_message = false;
            }
            else {
                // updating if there wasn't, there could be something wrong
                elapsed_time_ += period;
            }

            if (elapsed_time_.toSec() > 0.2) { // not sure if I want to look at 0.1 s instead....
                for (size_t i = 0; i < 7; i++) {
                    filtered_cmd[i] = decay_rate * filtered_cmd[i]; // slow the robot
                    velocity_joint_handles_[i].setCommand(filtered_cmd[i]);
                }
            }
            else {
                for ( size_t i = 0 ; i < 7; i ++) {
                    filtered_cmd[i] = alpha * filtered_cmd[i] + (1.0-alpha) * vel_cmd[i];
                    velocity_joint_handles_[i].setCommand(filtered_cmd[i]);
                }
            }

        }

        void starting(const ros::Time&) {
            elapsed_time_ = ros::Duration(0.0);
        }

        void stopping(const ros::Time&) {
            // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
            // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
            // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
        }

        void cmd_callback(const sensor_msgs::JointState::ConstPtr& msgs) {
            // pull in the current command
            // TODO: update this once debugging is complete
            float vel;
            for (size_t i=0; i<7; i++) {
                vel = msgs->velocity[i];
                if (msgs->velocity[i] < -max_vel) {
                    vel = -max_vel;
                }
                if (msgs->velocity[i] > max_vel) {
                    vel = max_vel;
                }
                vel_cmd[i] = vel;
            }
            read_message = true;
        }

    };
} // namespace franka_interface

PLUGINLIB_EXPORT_CLASS(franka_interface::VelocityInterface,
                       controller_interface::ControllerBase)
