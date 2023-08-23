#pragma once

#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>

// Define a convenient type for joint vectors
namespace Eigen
{
    typedef Matrix<double, 7, 1> Vector7d;
} // namespace Eigen

namespace fr_joint_velocity_controller
{

    class JointVelocityController : public controller_interface::MultiInterfaceController<
                                        hardware_interface::VelocityJointInterface,
                                        franka_hw::FrankaStateInterface>
    {

    public:
        bool init(hardware_interface::RobotHW *robot_hardware, ros::NodeHandle &node_handle) override;
        void update(const ros::Time &, const ros::Duration &period) override;
        void stopping(const ros::Time &) override;

    private:
        // Hardware interface things
        hardware_interface::VelocityJointInterface *velocity_joint_interface_;
        std::vector<hardware_interface::JointHandle> velocity_joint_handles_;

        // Target joint velocity subscriber
        ros::Subscriber sub_target_joint_velocity_;
        ros::Time last_target_timestamp_;
        void targetJointVelocityCallback(const sensor_msgs::JointStateConstPtr &msg);

        // Implement a moving average filter to update commanded velocities smoothly
        double velocity_smoothing_{0.002};
        double max_time_between_targets_{0.1}; // seconds
        Eigen::Vector7d velocity_command_;
        Eigen::Vector7d velocity_command_target_;
    };

} // namespace fr_joint_velocity_controller
