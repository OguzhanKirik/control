// ur5_moveit_planner_node.cpp

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>

class UR5MoveItPlanner : public rclcpp::Node {
public:
  UR5MoveItPlanner() : Node("ur5_moveit_planner"),
    move_group_(std::shared_ptr<rclcpp::Node>(
                  std::static_pointer_cast<rclcpp::Node>(shared_from_this())),
                "manipulator")
  {
    // ---- Parameters ----
    declare_parameter<bool>("execute_with_moveit", true);
    declare_parameter<double>("stream_rate_hz", 200.0);
    declare_parameter<std::vector<double>>("target_joint_positions", {});
    declare_parameter<std::vector<double>>("target_pose_xyzrpy", {});
    declare_parameter<std::string>("target_pose_frame", "base_link"); // <- explicit root frame
    declare_parameter<std::string>("planner_id", "RRTConnectkConfigDefault");

    get_parameter("execute_with_moveit", execute_with_moveit_);
    get_parameter("stream_rate_hz", stream_rate_hz_);
    get_parameter("target_pose_frame", target_pose_frame_);
    get_parameter("planner_id", planner_id);

    // ---- Publisher (if streaming to custom controller) ----
    ref_pub_ = create_publisher<sensor_msgs::msg::JointState>("~/cmd_joint_state", 10);

    // ---- MoveIt basic setup ----
    move_group_.setPlanningTime(5.0);
    move_group_.setMaxVelocityScalingFactor(0.5);
    move_group_.setMaxAccelerationScalingFactor(0.5);
    move_group_.setStartStateToCurrentState();
    move_group_.setPlannerId(planner_id);

    // ---- Decide target ----
    std::vector<double> q_target, xyzrpy;
    get_parameter("target_joint_positions", q_target);
    get_parameter("target_pose_xyzrpy", xyzrpy);

    bool ok = false;
    if (!q_target.empty()) {
      ok = planToJointTarget(q_target);
    } else if (xyzrpy.size() == 6) {
      ok = planToPoseTarget(xyzrpy, target_pose_frame_);
    } else {
      RCLCPP_WARN(get_logger(), "No target provided. Using demo joint move (+0.2 rad on joint 0).");
      auto current = move_group_.getCurrentJointValues();
      if (current.size() >= 1) current[0] += 0.2;
      ok = planToJointTarget(current);
    }

    if (!ok) {
      RCLCPP_ERROR(get_logger(), "Planning failed.");
      return; // constructor returns; node will still spin, but nothing more happens
    }

    // ---- Execute or stream ----
    if (execute_with_moveit_) {
      auto success = (move_group_.execute(plan_) == moveit::core::MoveItErrorCode::SUCCESS);
      RCLCPP_INFO(get_logger(), success ? "Executed plan via MoveIt." : "Execution failed.");
    } else {
      startStreamingToController();
    }
  }

private:
  bool planToJointTarget(const std::vector<double>& joints) {
    // Validate joint size
    const auto joint_names = move_group_.getJointNames();
    if (!joint_names.empty() && joints.size() != joint_names.size()) {
      RCLCPP_ERROR(get_logger(),
                   "Joint target size (%zu) != group joint count (%zu).",
                   joints.size(), joint_names.size());
      return false;
    }
    move_group_.setStartStateToCurrentState();
    move_group_.setJointValueTarget(joints);
    return plan();
  }

  bool planToPoseTarget(const std::vector<double>& xyzrpy, const std::string& frame_id) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = frame_id; // explicit root/world/base frame
    pose.pose.position.x = xyzrpy[0];
    pose.pose.position.y = xyzrpy[1];
    pose.pose.position.z = xyzrpy[2];

    tf2::Quaternion q;
    q.setRPY(xyzrpy[3], xyzrpy[4], xyzrpy[5]);
    q.normalize();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    move_group_.setStartStateToCurrentState();
    move_group_.setPoseTarget(pose);
    return plan();
  }

  bool plan() {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto ok = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    // Clear pose targets to avoid residual state
    move_group_.clearPoseTargets();

    if (ok) {
      plan_ = plan;
      const auto& jt = plan_.trajectory_.joint_trajectory;
      RCLCPP_INFO(get_logger(), "Planning OK. Points: %zu (dt up to %.3f s)",
                  jt.points.size(),
                  jt.points.empty() ? 0.0 :
                  jt.points.back().time_from_start.sec +
                  jt.points.back().time_from_start.nanosec * 1e-9);
    }
    return ok;
  }

  void startStreamingToController() {
    const auto& jt = plan_.trajectory_.joint_trajectory;
    if (jt.points.empty()) {
      RCLCPP_ERROR(get_logger(), "Empty trajectory; nothing to stream.");
      return;
    }

    auto period = std::chrono::duration<double>(1.0 / stream_rate_hz_);
    current_idx_ = 0;
    base_time_ = now();
    current_trajectory_ = jt;

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&UR5MoveItPlanner::timerCallback, this)
    );

    RCLCPP_INFO(get_logger(), "Streaming planned trajectory to ~/cmd_joint_state at %.1f Hz",
                stream_rate_hz_);
  }

  void timerCallback() {
    rclcpp::Duration elapsed = now() - base_time_;

    while (current_idx_ + 1 < current_trajectory_.points.size() &&
           rclcpp::Duration(current_trajectory_.points[current_idx_ + 1].time_from_start) <= elapsed)
    {
      current_idx_++;
    }

    const auto& pt = current_trajectory_.points[current_idx_];
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = current_trajectory_.joint_names;
    msg.position = pt.positions;

    if (!pt.velocities.empty())    msg.velocity = pt.velocities;
    if (!pt.accelerations.empty()) msg.effort   = pt.accelerations; // if your controller expects qdd in effort

    ref_pub_->publish(msg);

    if (current_idx_ + 1 >= current_trajectory_.points.size()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Streaming complete.");
      timer_->cancel();
    }
  }

  // --- Params/state ---
  bool   execute_with_moveit_{true};
  double stream_rate_hz_{200.0};
  std::string planner_id;
  std::string target_pose_frame_{"base_link"};

  rclcpp::Time  base_time_;
  size_t        current_idx_{0};
  trajectory_msgs::msg::JointTrajectory current_trajectory_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr ref_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  moveit::planning_interface::MoveGroupInterface move_group_;
  moveit::planning_interface::MoveGroupInterface::Plan plan_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UR5MoveItPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
