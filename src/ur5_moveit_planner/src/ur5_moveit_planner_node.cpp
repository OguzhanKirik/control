//ur5_moveit_planner_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class UR5MoveItPlanner : public rclcpp::Node{

    public:
        UR5MoveItPlanner::UR5MoveItPlanner() : Node("ur5_moveit_planner"),
            move_group_(std::shared_ptr<rclcpp::Node>(std::static_pointer_cast<rclcpp::Node>(shared_from_this())),
            "manipulator"),
        {
            //Params
            declare_parameter<bool>("execute_with_moveit",true);
            declare_parameter<double>("stream_rate_hz",200.0);
            declare_parameter<std::vector<double>>("target_joint_positions", {});
            declare_parameter<std::vector<double>>("target_pose_xyzrpy",{});


            get_parameter("execute_with_moveit", execute_with_moveit_);
            get_parameter("stream_rate_hz", stream_rate_hz_);

            //Publisher to your torque/MPC controller(if streaming)
            ref_pub_ = creata_publisher<sensor_msgs::msg::JointState>("~/cmd_joint_state",10);

            //Basic Moveit setup
            move_group_.setPlanningTime(5.0);
            move_group_.setMaxVelocityScalingFactor(0.5);
            move_group_.setMaxAccelerationScalingFactor(0.5);

            //Pick target: joint state or Cartesian pose
            std::vector<double> q_target;
            std::vector<double> xyzrpy;
            get_parameter("target_joint_positions", q_target);
            get_parameter("target_pose_xyzrpy", xyzrpy);

            bool ok = false;
            if(!q_target.empty()){
                ok = planToJoinTarget(q_target);
            }else if(xyzrpy.size() == 6){
                ok = planToPoseTarget(xyzrpy);
            }else{
                RCLCPP_WARN(get_logger(), "No target provided. Using a small demo joint move.");
                //Demo: move+0.2 rad on shoulder_pan
                auto currect = move_group_.getCurrentJointValues();
                if (current.size() >= 6) current[0] += 0.2;
                ok = planToJointTarget(current);
            }
            if(!ok){
                RCLCPP_ERROR(get_logger(), "Planning failed.");
                return; 
            }
            
            if(execute_with_moveit_){
                auto success = (move_group_.execute(plan_) == moveit::core::MoveItErrorCode::SUCCESS);
                RCLCPP_INFO(get_logger(), success ? "Executed plan via MoveIt." : "Execution failed.");
            }else {
                // Stream to your controller as desired JointState at a fixed rate
                startStreamingToController();
            }
        
        } 
    private:
        bool planToJointTarget(const std::vector<double>& joints){
            move_group_.setJointValueTarget(joints);
            return plan();
        }

        bool planToPoseTarget(const std::vector<double>& xyzrpy){
            geotmetry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = move_group_.getPlanningFrame();
            pose.pose.position.x = xyzrpy[0];
            pose.pose.position.x = xyzrpy[1];
            pose.pose.position.x = xyzrpy[2];
            tf2::Quartenion q; 
            q.setRPY(xyzrpy[3],xyzrpy[4],xyzrpy[5]);
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();

            move_group_.setPoseTarget(pose);
            return plan();
        }

        bool plan() {
            // Plan with MoveIt
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto ok = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (ok) {
            plan_ = plan;
            // The plan contains time-parameterized joint trajectory:
            // plan_.trajectory_.joint_trajectory (points with positions, velocities, accelerations, time_from_start)
            RCLCPP_INFO(get_logger(), "Planning OK. Points: %zu",
                        plan_.trajectory_.joint_trajectory.points.size());
            }
            // Clear any pose targets to avoid residuals next calls
            move_group_.clearPoseTargets();
            return ok;
        }

        void startStreamingToController(){
            //Stream the planned joint_trajcetory as JointState references
            const auto& jt = plan_.trajectory_.joint_trajectoy;
            if(jt.points.empty()){
                RCLCPP_ERROR(get_logger(), "Empty trajectory; nothing to stream.");
                return;
            }
            // Build a timer to publish at stream_rate_hz_
            auto period = std::chrono::duration<double>(1.0 / stream_rate_hz_);
            size_t idx = 0;
            base_time_ = now();
            timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                [this, &jt, idx]() mutable {
                    // Advance according to time_from_start (interpolating by nearest index for simplicity)
                    rclcpp::Duration elapsed = now() - base_time_;
                    // Find the last point whose time_from_start <= elapsed
                    while (idx + 1 < jt.points.size() &&
                        rclcpp::Duration(jt.points[idx + 1].time_from_start) <= elapsed) {
                    idx++;
                    }

                    const auto& pt = jt.points[idx];
                    sensor_msgs::msg::JointState msg;
                    msg.name = jt.joint_names;
                    msg.position = pt.positions;

                    if (!pt.velocities.empty())
                    msg.velocity = pt.velocities;
                    if (!pt.accelerations.empty())
                    msg.effort = pt.accelerations;  // <-- your controller uses effort field as qdd_ref

                    ref_pub_->publish(msg);

                    if (idx + 1 >= jt.points.size()) {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Streaming complete.");
                    timer_->cancel(); // stop
                    }
                });

                RCLCPP_INFO(get_logger(), "Streaming planned trajectory to ~/cmd_joint_state at %.1f Hz",
                            stream_rate_hz_);
            }

            bool execute_with_moveit_{true};
            double stream_rate_hz_{200.0};
            rclcpp::Time base_time_;

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
        

