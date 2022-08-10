#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <dynamic_reconfigure/server.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <realtime_tools/realtime_publisher.h>

#include <franka_example_controllers/JointTorqueComparison.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>
#include <franka_example_controllers/compliance_paramConfig.h>
#include <Eigen/Dense>
#include "geometry_msgs/PoseStamped.h"

namespace franka_feedback_controller {
class FeedbackController : public controller_interface::MultiInterfaceController<
                                        franka_hw::FrankaModelInterface,
                                        hardware_interface::EffortJointInterface,
                                        franka_hw::FrankaStateInterface> {
public:
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
    void starting(const ros::Time&) override;
    void update(const ros::Time&, const ros::Duration& period) override;

private: 
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
        const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;


    double coriolis_factor_{1.0};
    std::array<double, 7> dq_filtered_;
    ros::Duration elapsed_time_;
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    franka::RobotState cur_state;
    std::array<double, 16> robot_pose_;
    const double tol{5e-3};
    ros::Publisher pospub;
    ros::Publisher torquepub;
    ros::Publisher goalpub;
    Eigen::Matrix<double, 7, 1> q_d_nullspace_;
    Eigen::Vector3d position_d_;
    Eigen::Quaterniond orientation_d_;
    std::mutex position_and_orientation_d_target_mutex_;
    Eigen::Vector3d position_d_target_;
    Eigen::Quaterniond orientation_d_target_;
    const double delta_tau_max_{1.0};
    double filter_params_{0.005};
    double nullspace_stiffness_{20.0};
    double nullspace_stiffness_target_{20.0};
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_target_;

    std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>
        dynamic_server_compliance_param_;
    ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
    void complianceParamCallback(franka_example_controllers::compliance_paramConfig& config,
                                uint32_t level);
    ros::Subscriber sub_equilibrium_pose_;
    void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    

};

}
