#pragma once

#include <memory>
#include <mutex>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/robot_hw.h>
#include <controller_interface/controller.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>

#include <franka_hw/franka_cartesian_command_interface.h>

#include <force_sensitive_cbf/ForceSensitiveCBFConfig.h>

// Define a convenient type for 6Dof vectors
namespace Eigen
{
    typedef Matrix<double, 6, 1> Vector6d;
} // namespace Eigen

namespace force_sensitive_cbf
{

    class ForceSensitiveCBFController : public controller_interface::Controller<franka_hw::FrankaVelocityCartesianInterface>
    {

    public:
        bool init(franka_hw::FrankaVelocityCartesianInterface *hw_interface, ros::NodeHandle &node_handle) override;
        void starting(const ros::Time &) override;
        void update(const ros::Time &, const ros::Duration &period) override;

    private:
        // Target pose subscriber
        ros::Subscriber sub_target_pose_;
        void targetPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);

        // Dynamic reconfigure
        std::unique_ptr<dynamic_reconfigure::Server<ForceSensitiveCBFConfig>>
            dynamic_param_server_;
        ros::NodeHandle dynamic_reconfigure_node_;
        void dynamicParamCallback(ForceSensitiveCBFConfig &config,
                                  uint32_t level);

        // ROS control interfaces
        std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> velocity_handle_;

        // TF interface
        tf::TransformListener tf_listener_;

        // Controller parameters. Implement a moving average filter to update these
        // smoothly
        double filter_params_{0.005};
        Eigen::Vector6d cartesian_force_limit_;
        Eigen::Vector6d cartesian_force_limit_target_;
        double linear_speed_limit_{0.5};
        double linear_speed_limit_target_{0.5};
        double angular_speed_limit_{2.0};
        double angular_speed_limit_target_{2.0};
        double cbf_rate_{1.0};
        double cbf_rate_target_{1.0};
        double clf_rate_{1.0};
        double clf_rate_target_{1.0};
        double clf_relaxation_penalty_{1.0};
        double clf_relaxation_penalty_target_{1.0};

        // Track the desired position and orientation, with targets for smoothing (same
        // as used for parameters above)
        Eigen::Vector3d position_d_;
        Eigen::Quaterniond orientation_d_;
        std::mutex position_and_orientation_d_target_mutex_;
        Eigen::Vector3d position_d_target_;
        Eigen::Quaterniond orientation_d_target_;
    };

} // namespace force_sensitive_cbf
