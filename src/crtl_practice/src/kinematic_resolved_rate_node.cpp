// kinematic_resolved_rate_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include "crtl_practice/ur5_kin_helpers.hpp" // fk_and_jacobian(...), rotvec_error(...), DH_UR5
using Eigen::VectorXd; using Eigen::MatrixXd; using Eigen::Vector3d; using Eigen::Matrix3d;


class ResolvedRate : public rclcpp::Node{
    public:
        ResolvedRate() : Node("resolved_rate"){
            rate_hz_ = declare_parameter("rate_hz", 250.0);
            kp_pos_  = declare_parameter("kp_pos", 2.0);
            kp_ori_  = declare_parameter("kp_ori", 2.0);
            lambda_  = declare_parameter("lambda", 1e-3);

            // Initialize matrices
            q_.setZero(6);
            qd_.setZero(6);
            T_.setIdentity();
            J_.setZero(6,6);
            p_d_.setZero();
            R_d_.setIdentity();

            js_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_state",10,
                std::bind(&ResolvedRate::onJointState,this, std::placeholders::_1));

            pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("~/cmd_pose",10,
                std::bind(&ResolvedRate::onPoseStamped,this, std::placeholders::_1));

            qdot_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/joint_velocity_cmd",10);
            timer_ = create_wall_timer(std::chrono::duration<double>(1.0/rate_hz_), 
                std::bind(&ResolvedRate::loop,this));
        }

    private:

        void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg){
            if(msg->position.size() < 6) return;
            for(int i=0;i<6;++i){
                q_[i]=msg->position[i];
                if(msg->velocity.size()>i) qd_[i]=msg->velocity[i];
            }
            ctrl::fk_and_jacobian(ctrl::DH_UR5, q_, T_, J_);
            have_state_=true; 
        }

        void onPoseStamped(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
            p_d_ = {msg->pose.position.x,msg->pose.position.y,msg->pose.position.z};
            Eigen::Quaterniond Q(msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z);
            R_d_ = Q.normalized().toRotationMatrix(); 
            have_ref_=true;
        }

        void loop(){
            if(!have_state_||!have_ref_) return;
            Vector3d p = T_.block<3,1>(0,3);
            Matrix3d R = T_.block<3,3>(0,0);
            Vector3d ep = p_d_ - p;
            Vector3d eo = ctrl::rotvec_error(R, R_d_);
            VectorXd xdot_des(6); xdot_des << kp_pos_*ep, kp_ori_*eo;      // PD in task
            
            // Damped Least Squares pseudo-inverse
            MatrixXd JJT = J_ * J_.transpose();
            MatrixXd lambda_I = lambda_ * MatrixXd::Identity(JJT.rows(), JJT.cols());
            VectorXd qdot = J_.transpose() * (JJT + lambda_I).inverse() * xdot_des;
            
            std_msgs::msg::Float64MultiArray out;
            out.data.assign(qdot.data(), qdot.data()+6);
            qdot_pub_->publish(out);
        }

        double rate_hz_, kp_pos_, kp_ori_, lambda_;
        bool have_state_{false}, have_ref_{false};
        VectorXd q_, qd_; 
        Eigen::Matrix4d T_; 
        MatrixXd J_;
        Vector3d p_d_; 
        Matrix3d R_d_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr qdot_pub_;
        rclcpp::TimerBase::SharedPtr timer_;
                
};

int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ResolvedRate>());
    rclcpp::shutdown();
}
