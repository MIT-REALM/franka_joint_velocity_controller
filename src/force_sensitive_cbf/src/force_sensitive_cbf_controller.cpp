#include <force_sensitive_cbf/force_sensitive_cbf_controller.h>

#include <cmath>
#include <memory>
#include <vector>
#include <limits>
#include <array>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include "OsqpEigen/OsqpEigen.h"
#include <pluginlib/class_list_macros.h>

namespace force_sensitive_cbf
{

    bool ForceSensitiveCBFController::init(franka_hw::FrankaVelocityCartesianInterface *hw_interface, ros::NodeHandle &node_handle)
    {
        // Initialize target pose subscriber
        sub_target_pose_ = node_handle.subscribe(
            "target_pose", 20, &ForceSensitiveCBFController::targetPoseCallback, this,
            ros::TransportHints().reliable().tcpNoDelay());

        // Get the arm ID from the ROS parameter server
        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id))
        {
            ROS_ERROR_STREAM("ForceSensitiveCBFController: Could not read parameter arm_id");
            return false;
        }

        // Initialize the velocity interface with the robot hardware
        try
        {
            velocity_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
                hw_interface->getHandle(arm_id + "_robot"));
        }
        catch (hardware_interface::HardwareInterfaceException &ex)
        {
            ROS_ERROR_STREAM(
                "ForceSensitiveCBFController: Exception getting velocity handle from interface: "
                << ex.what());
            return false;
        }

        // Set up the dynamic reconfiguration server
        dynamic_reconfigure_node_ =
            ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_node");
        dynamic_param_server_ = std::make_unique<dynamic_reconfigure::Server<ForceSensitiveCBFConfig>>(
            dynamic_reconfigure_node_);
        dynamic_param_server_->setCallback(
            boost::bind(&ForceSensitiveCBFController::dynamicParamCallback, this, _1, _2));

        // Set default values for controller parameters
        cartesian_force_limit_.setZero();
        cartesian_force_limit_target_.setZero();

        // Initialize positions and orientations to origin
        position_d_.setZero();
        orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
        position_d_target_.setZero();
        orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

        return true;
    } // init

    void ForceSensitiveCBFController::starting(const ros::Time & /*time*/)
    {
        // Get initial state and set target position and orientation to current state
        franka::RobotState initial_state = velocity_handle_->getRobotState();
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        position_d_ = initial_transform.translation();
        orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
        position_d_target_ = initial_transform.translation();
        orientation_d_target_ = Eigen::Quaterniond(initial_transform.rotation());
    } // starting

    void ForceSensitiveCBFController::update(const ros::Time & /*time*/,
                                             const ros::Duration & /*duration*/)
    {
        ROS_DEBUG("ForceSensitiveCBFController: update");
        // Get the current position and orientation of the end effector
        franka::RobotState robot_state = velocity_handle_->getRobotState();
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaterniond orientation(transform.rotation());
        // TODO is this going to bite me later? What's the difference between EE frame
        // and the TCP frame and the stiffness (K) frame?

        // Update the parameters and targets with a moving average filter
        position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
        orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
        cartesian_force_limit_ = filter_params_ * cartesian_force_limit_target_ +
                                 (1.0 - filter_params_) * cartesian_force_limit_;
        linear_speed_limit_ = filter_params_ * linear_speed_limit_target_ +
                              (1.0 - filter_params_) * linear_speed_limit_;
        angular_speed_limit_ = filter_params_ * angular_speed_limit_target_ +
                               (1.0 - filter_params_) * angular_speed_limit_;
        cbf_rate_ = filter_params_ * cbf_rate_target_ + (1.0 - filter_params_) * cbf_rate_;
        clf_rate_ = filter_params_ * clf_rate_target_ + (1.0 - filter_params_) * clf_rate_;
        clf_relaxation_penalty_ = filter_params_ * clf_relaxation_penalty_target_ +
                                  (1.0 - filter_params_) * clf_relaxation_penalty_;

        // Get the current external force. Franka claims they estimate this with
        // a moving-average filter and subtract gravity for us; we'll see how well that
        // works....
        Eigen::Vector6d external_force = Eigen::Vector6d::Map(
            robot_state.O_F_ext_hat_K.data());
        ROS_DEBUG("External force: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", external_force(0),
                  external_force(1), external_force(2), external_force(3),
                  external_force(4), external_force(5));

        // We need to define the CLF and CBF used for the controller.
        //
        // The CLF is defined as squared error in pose V = 1/2 ||pose - pose_d||^2
        //
        // The CBF is defined in terms of the forces on the end effector, and we
        // actually have 6 CBFs (one for each linear and angular DOF)
        //      h_i = 1/2 (f_i^2 - f_i_max^2)  for i = fx,fy,fz,taux,tauy,tauz

        // We solve for the control input (velocity) using a QP. The variables are the
        // 6DOF velocity of the end effector plus a scalar relaxation of the CLF
        // constraint.

        int n_vars = 7;        // 6DOF velocity + 1 relaxation
        int n_constraints = 8; // 6 CBFs + 1 CLF + non-negative relaxation

        int vx_idx = 0; // indices for decision variables
        int vy_idx = 1;
        int vz_idx = 2;
        int wx_idx = 3;
        int wy_idx = 4;
        int wz_idx = 5;
        int relaxation_idx = 6;

        // The objective of the CBF/CLF QP is to minimize the squared magnitude of
        // the velocity (our control input) plus penalized violation of the CLF
        // condition.

        Eigen::SparseMatrix<double> objective_matrix(n_vars, n_vars);
        const Eigen::Triplet<double> objective_triplets[] = {
            // Define entries in the objective matrix (i, j, value)
            // This includes a quadratic cost on the velocity
            {vx_idx, vx_idx, 1.0},
            {vy_idx, vy_idx, 1.0},
            {vz_idx, vz_idx, 1.0},
            {wx_idx, wx_idx, 1.0},
            {wy_idx, wy_idx, 1.0},
            {wz_idx, wz_idx, 1.0},
        };
        objective_matrix.setFromTriplets(std::begin(objective_triplets),
                                         std::end(objective_triplets));
        // We also have a linear cost on the relaxation variable
        Eigen::VectorXd objective_vector(n_vars);
        objective_vector.setZero();
        objective_vector(relaxation_idx) = clf_relaxation_penalty_;

        // We have a couple of constraints to add to the constraint matrix
        Eigen::SparseMatrix<double> constraint_matrix(n_constraints, n_vars);
        std::vector<Eigen::Triplet<double>> constraint_triplets;
        constraint_triplets.reserve(6 + 7 + 1); // 6 for CBFs + 7 for CLF + 1 for relaxation
        Eigen::VectorXd constraint_ub(n_constraints);
        constraint_ub.setZero();
        Eigen::VectorXd constraint_lb(n_constraints);
        constraint_lb.setZero();
        int constraint_idx = 0; // Keep track of which row we're on

        // The CLF relaxation must be non-negative
        constraint_triplets.push_back({constraint_idx, relaxation_idx, 1.0});
        constraint_ub(constraint_idx) = std::numeric_limits<double>::infinity(); // lower bound already 0
        constraint_idx++;

        // There is a constraint for the CLF condition, which we relax when needed
        // to prioritize safety. The constraint is:
        //     sum_i (pose_i - pose_desired_i) * velocity_i - relaxation <= - clf_rate * V

        // Get the pose error
        // position error
        Eigen::Matrix<double, 6, 1> error;
        error.head(3) << position - position_d_;

        // orientation error (from the Franka impedance controller implementation)
        if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0)
        {
            orientation.coeffs() << -orientation.coeffs();
        }
        // "difference" quaternion
        Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        // Transform to base frame
        error.tail(3) << -transform.rotation() * error.tail(3);

        // Get the CLF value
        double clf_value = error.squaredNorm();
        ROS_DEBUG("CLF value: || [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] ||^2 = %.2f",
                  error(0), error(1), error(2), error(3), error(4), error(5),
                  clf_value);

        // Add the CLF constraint
        constraint_triplets.push_back({constraint_idx, vx_idx, error(0)});
        constraint_triplets.push_back({constraint_idx, vy_idx, error(1)});
        constraint_triplets.push_back({constraint_idx, vz_idx, error(2)});
        constraint_triplets.push_back({constraint_idx, wx_idx, error(3)});
        constraint_triplets.push_back({constraint_idx, wy_idx, error(4)});
        constraint_triplets.push_back({constraint_idx, wz_idx, error(5)});
        constraint_triplets.push_back({constraint_idx, relaxation_idx, -1.0});
        constraint_ub(constraint_idx) = -clf_rate_ * clf_relaxation_penalty_;
        constraint_lb(constraint_idx) = -std::numeric_limits<double>::infinity();
        constraint_idx++;

        // There is a constraint for the CBF condition for each linear and angular DOF.
        // The constraint for each axis is:
        //      - force * velocity_i <= - cbf_rate * h_i

        for (int idx = 0; idx < 6; idx++)
        {
            // Get the CBF value
            double force_i = external_force(idx);
            double max_force_i = cartesian_force_limit_(idx);
            double cbf_value_i = force_i * force_i - max_force_i * max_force_i;
            ROS_DEBUG("CBF value: (%.2f)^2 - (%.2f)^2 = %.2f", force_i, max_force_i, cbf_value_i);

            // Add the constraint
            constraint_triplets.push_back({constraint_idx, idx, -force_i});
            constraint_ub(constraint_idx) = -cbf_rate_ * cbf_value_i;
            constraint_lb(constraint_idx) = -std::numeric_limits<double>::infinity();
            constraint_idx++;
        }

        // Assemble all constraint triples into a matrix
        constraint_matrix.setFromTriplets(constraint_triplets.begin(),
                                          constraint_triplets.end());

        // Now we can solve the QP and get the optimal solution
        OsqpEigen::Solver solver;
        solver.data()->setNumberOfVariables(n_vars);
        solver.data()->setNumberOfConstraints(n_constraints);
        solver.data()->setHessianMatrix(objective_matrix);
        solver.data()->setGradient(objective_vector);
        solver.data()->setLinearConstraintsMatrix(constraint_matrix);
        solver.data()->setLowerBound(constraint_lb);
        solver.data()->setUpperBound(constraint_ub);

        solver.initSolver();
        auto exit_code = solver.solveProblem();
        auto solver_status = solver.getStatus();
        if (exit_code != OsqpEigen::ErrorExitFlag::NoError && solver_status != OsqpEigen::Status::Solved)
        {
            // The solver should never fail (the QP should always be feasible)
            // If this somehow happens, stop the robot and set the desired position
            // and orientation to the current position and orientation
            position_d_ = position;
            orientation_d_ = orientation;
            std::array<double, 6> zero_array{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            velocity_handle_->setCommand(zero_array);

            ROS_ERROR("CBF/CLF QP solver failed! exit code %d, solver status %d",
                      int(exit_code), int(solver_status));
            return;
        }

        // Get the optimal solution
        Eigen::VectorXd optimal_solution = solver.getSolution();
        Eigen::Vector6d velocity_command;
        velocity_command << optimal_solution.head(6);
        ROS_DEBUG("Optimal solution: v = [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f], r = %.2f",
                  velocity_command(0), velocity_command(1), velocity_command(2),
                  velocity_command(3), velocity_command(4), velocity_command(5),
                  optimal_solution(6));

        // Normalize the given linear and angular velocities to the maximum allowed
        // linear and angular speeds
        double linear_speed = velocity_command.head(3).norm();
        if (linear_speed > linear_speed_limit_)
        {
            velocity_command.head(3) *= linear_speed_limit_ / linear_speed;
        }
        double angular_speed = velocity_command.tail(3).norm();
        if (angular_speed > angular_speed_limit_)
        {
            velocity_command.tail(3) *= angular_speed_limit_ / angular_speed;
        }

        ROS_DEBUG("Normalized solution: v = [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                  velocity_command(0), velocity_command(1), velocity_command(2),
                  velocity_command(3), velocity_command(4), velocity_command(5));

        // Send the velocity command to the robot
        // We need to convert from an Eigen::Vector6d to an array
        std::array<double, 6> velocity_command_array;
        Eigen::VectorXd::Map(&velocity_command_array[0], velocity_command.size()) = velocity_command;
        velocity_handle_->setCommand(velocity_command_array);

        ROS_DEBUG("ForceSensitiveCBFController update complete");

        return;
    } // update

    void ForceSensitiveCBFController::dynamicParamCallback(
        ForceSensitiveCBFConfig &config,
        uint32_t /*level*/)
    {
        // Set force limits from configured parameters
        // Update the targets (we'll smoothly transition to these from the current
        // parameters in the update step).
        cartesian_force_limit_target_ << config.force_limit_x,
            config.force_limit_y,
            config.force_limit_z,
            config.torque_limit_x,
            config.torque_limit_y,
            config.torque_limit_z;

        linear_speed_limit_target_ = config.linear_speed_limit;
        angular_speed_limit_target_ = config.angular_speed_limit;
        cbf_rate_target_ = config.cbf_rate;
        clf_rate_target_ = config.clf_rate;
        clf_relaxation_penalty_target_ = config.clf_relaxation_penalty;

        ROS_DEBUG_STREAM("Updated dynamic parameters"
                         << "\n\tforce_limit_x: " << cartesian_force_limit_target_(0)
                         << "\n\tforce_limit_y: " << cartesian_force_limit_target_(1)
                         << "\n\tforce_limit_z: " << cartesian_force_limit_target_(2)
                         << "\n\ttorque_limit_x: " << cartesian_force_limit_target_(3)
                         << "\n\ttorque_limit_y: " << cartesian_force_limit_target_(4)
                         << "\n\ttorque_limit_z: " << cartesian_force_limit_target_(5)
                         << "\n\tlinear_speed_limit: " << linear_speed_limit_
                         << "\n\tangular_speed_limit: " << angular_speed_limit_
                         << "\n\tcbf_rate: " << cbf_rate_target_
                         << "\n\tclf_rate: " << clf_rate_target_
                         << "\n\tclf_relaxation_penalty: " << clf_relaxation_penalty_target_);
    } // dynamicParamCallback

    void ForceSensitiveCBFController::targetPoseCallback(
        const geometry_msgs::PoseStampedConstPtr &msg)
    {
        // Convert the pose into the robot's base frame
        geometry_msgs::PoseStamped pose_in_robot_frame;
        try
        {
            tf_listener_.transformPose("fr3_link0", *msg, pose_in_robot_frame);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN_STREAM(
                "ForceSensitiveCBFController: Could not transform pose into robot frame: "
                << ex.what());
            return;
        }

        // Acquire lock on target position and orientation
        std::lock_guard<std::mutex> position_d_target_mutex_lock(
            position_and_orientation_d_target_mutex_);

        // Update position
        position_d_target_ << pose_in_robot_frame.pose.position.x,
            pose_in_robot_frame.pose.position.y,
            pose_in_robot_frame.pose.position.z;

        // Update orientation, flipping the sign if necessary to ensure the shortest path
        Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
        orientation_d_target_.coeffs() << pose_in_robot_frame.pose.orientation.x,
            pose_in_robot_frame.pose.orientation.y,
            pose_in_robot_frame.pose.orientation.z,
            pose_in_robot_frame.pose.orientation.w;
        if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0)
        {
            orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
        }

        ROS_DEBUG("Updated target pose to [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                  position_d_target_(0), position_d_target_(1), position_d_target_(2),
                  orientation_d_target_.x(), orientation_d_target_.y(), orientation_d_target_.z(),
                  orientation_d_target_.w());
    }

} // namespace force_sensitive_cbf

PLUGINLIB_EXPORT_CLASS(force_sensitive_cbf::ForceSensitiveCBFController,
                       controller_interface::ControllerBase)