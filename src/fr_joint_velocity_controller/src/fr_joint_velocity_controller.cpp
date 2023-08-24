#include <fr_joint_velocity_controller/fr_joint_velocity_controller.h>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace fr_joint_velocity_controller
{

    bool JointVelocityController::init(hardware_interface::RobotHW *robot_hardware,
                                       ros::NodeHandle &node_handle)
    {
        // Initialize target pose subscriber
        sub_target_joint_velocity_ = node_handle.subscribe(
            "target_joint_velocity", 20, &JointVelocityController::targetJointVelocityCallback, this,
            ros::TransportHints().reliable().tcpNoDelay());

        // Initialize HW interface
        velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
        if (velocity_joint_interface_ == nullptr)
        {
            ROS_ERROR(
                "JointVelocityController: Error getting velocity joint interface from hardware!");
            return false;
        }

        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id))
        {
            ROS_ERROR("JointVelocityController: Could not get parameter arm_id");
            return false;
        }

        std::vector<std::string> joint_names;
        if (!node_handle.getParam("joint_names", joint_names))
        {
            ROS_ERROR("JointVelocityController: Could not parse joint names");
        }
        if (joint_names.size() != 7)
        {
            ROS_ERROR_STREAM("JointVelocityController: Wrong number of joint names, got "
                             << joint_names.size() << " instead of 7 names!");
            return false;
        }
        velocity_joint_handles_.resize(7);
        for (size_t i = 0; i < 7; ++i)
        {
            try
            {
                velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
            }
            catch (const hardware_interface::HardwareInterfaceException &ex)
            {
                ROS_ERROR_STREAM(
                    "JointVelocityController: Exception getting joint handles: " << ex.what());
                return false;
            }
        }

        auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr)
        {
            ROS_ERROR("JointVelocityController: Could not get state interface from hardware");
            return false;
        }

        try
        {
            auto state_handle = state_interface->getHandle(arm_id + "_robot");
        }
        catch (const hardware_interface::HardwareInterfaceException &e)
        {
            ROS_ERROR_STREAM(
                "JointVelocityController: Exception getting state handle: " << e.what());
            return false;
        }

        // Set initial velocity commands to zero
        last_target_timestamp_ = ros::Time::now();
        velocity_command_.setZero();
        velocity_command_target_.setZero();

        return true;
    } // init

    void JointVelocityController::update(const ros::Time &time,
                                         const ros::Duration & /*duration*/)
    {
        ROS_DEBUG("JointVelocityController: update");

        // If it's been too long since we received the last target, set the target
        // to zero. THIS IS A SAFETY FEATURE to bring the robot to a stop if the target
        // publisher stops for whatever reason.
        ros::Duration time_since_last_target = time - last_target_timestamp_;
        if (time_since_last_target.toSec() > max_time_between_targets_)
        {
            ROS_WARN_THROTTLE(1.0, "JointVelocityController: No target received for %f seconds, setting target to zero",
                              time_since_last_target.toSec());
            velocity_command_target_.setZero();
        }

        // Smoothly update the commanded velocity to match the target
        velocity_command_ = (1 - velocity_smoothing_) * velocity_command_ + velocity_smoothing_ * velocity_command_target_;

        // Send command to arm
        for (int i = 0; i < velocity_command_.size(); i++) {
            velocity_joint_handles_[i].setCommand(velocity_command_[i]);
        }

        ROS_DEBUG("JointVelocityController update complete");
    } // update

    void JointVelocityController::stopping(const ros::Time & /*time*/)
    {
        // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
        // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
        // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
    }

    void JointVelocityController::targetJointVelocityCallback(
        const sensor_msgs::JointStateConstPtr &msg)
    {
        // Make sure there are the right number of joint velocities
        if (msg->velocity.size() != 7)
        {
            ROS_ERROR("JointVelocityController: targetJointVelocityCallback expected 7 velocities, got %d",
                      int(msg->velocity.size()));
            return;
        }

        // Update the target joint velocity
        velocity_command_target_ << msg->velocity[0], msg->velocity[1], msg->velocity[2],
            msg->velocity[3], msg->velocity[4], msg->velocity[5], msg->velocity[6];

        // Reset the elapsed time since the last target
        last_target_timestamp_ = ros::Time::now();
    }

} // namespace fr_joint_velocity_controller

PLUGINLIB_EXPORT_CLASS(fr_joint_velocity_controller::JointVelocityController,
                       controller_interface::ControllerBase)