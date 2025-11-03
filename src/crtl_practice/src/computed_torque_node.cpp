// computed_torque_node.cpp // inverse dynammic
// given the desired position, velocity, accel, 
//we commuput the tau for achiving it/ inverse dynamic




#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include "crtl_practice/ur5_dynamics.hpp" // M_of(q), C_of(q,qd), G_of(q)
using Eigen::VectorXd; using Eigen::MatrixXd;


class ComputedTorque : public rclcpp::Node{
    public:
        ComputedTorque(): Node("computed_torque"){
            rate_hz_ = declare_parameter("rate_hz", 250.0);
            kp_ = declare_parameter("kp", 80.0); kd_ = declare_parameter("kd", 16.0);
            tau_max_ = declare_parameter("tau_max", 120.0);

            js_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,
            [&](const sensor_msgs::msg::JointState::SharedPtr m){
            if(m->position.size()<6) return; 
            for(int i=0;i<6;++i){ 
                q_[i]=m->position[i]; 
                if(m->velocity.size()>i) qd_[i]=m->velocity[i];
            }
            have_state_=true;});
            
            ref_sub_ = create_subscription<sensor_msgs::msg::JointState>("~/cmd_joint_state",10,
            [&](const sensor_msgs::msg::JointState::SharedPtr m){
            if(m->position.size()>=6) for(int i=0;i<6;++i) qref_[i]=m->position[i];
            if(m->velocity.size()>=6) for(int i=0;i<6;++i) qdref_[i]=m->velocity[i];
            if(m->effort.size()  >=6) for(int i=0;i<6;++i) qddref_[i]=m->effort[i];
            have_ref_=true;});
            tau_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/joint_torque_cmd",10);
            timer_   = create_wall_timer(std::chrono::duration<double>(1.0/rate_hz_), [&]{loop();});
            q_.setZero(6); qd_.setZero(6); qref_.setZero(6); qdref_.setZero(6); qddref_.setZero(6);
    }
    private:
        void loop(){
            //τ=M(q)(q¨​ref​+Kp​(qref​−q)+Kd​(q˙​ref​−q˙​))+C(q,q˙​)+G(q)
            if(!have_state_||!have_ref_) return;
            VectorXd e = qref_ - q_, ed = qdref_ - qd_;
            VectorXd qdd_cmd = qddref_ + kp_*e + kd_*ed;
            
            MatrixXd M = ctrl::M_of(q_);
            VectorXd C = ctrl::C_of(q_, qd_);
            VectorXd G = ctrl::G_of(q_);

            VectorXd tau = M*qdd_cmd + C + G;
            for(int i=0;i<6;++i) tau[i] = std::clamp(tau[i], -tau_max_, tau_max_);
            std_msgs::msg::Float64MultiArray out;
            out.data.assign(tau.data(), tau.data()+6);
            tau_pub_->publish(out);
        }
            double rate_hz_, kp_, kd_, tau_max_;
            bool have_state_{false}, have_ref_{false};
            VectorXd q_{6}, qd_{6}, qref_{6}, qdref_{6}, qddref_{6};
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_, ref_sub_;
            rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tau_pub_;
            rclcpp::TimerBase::SharedPtr timer_;
  };

  int main(int argc,char** argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ComputedTorque>());
    rclcpp::shutdown();
}
