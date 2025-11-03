// pid_joint_node.cpp
/*

τ=Kp​e+Kd​e˙+Ki​∫edt

| Term   | Responds to             | Effect              | Physical Analogy          |
| ------ | ----------------------- | ------------------- | ------------------------- |
| **Kp** | Present error           | Pulls toward target | Spring stiffness          |
| **Kd** | Rate of change of error | Damps motion        | Shock absorber            |
| **Ki** | Accumulated past error  | Eliminates offset   | Long-term bias correction |
*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
using Eigen::VectorXd;



class PIDJoint : public rclcpp::Node {
    public:
        PIDJoint() : Node("pid_joint"){
            rate_hz_ = declare_parameter("rate_hz", 250.0);
            kp_ = declare_parameter("kp", 50.0);
            kd_ = declare_parameter("kd", 10.0);
            ki_ = declare_parameter("ki", 0.0);
            tau_max_ = declare_parameter("tau_max", 120.0);

            js_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_state", 10,
            [&](const sensor_msgs::msg::JointState::SharedPtr m) {if(m->position.size()<6) return;
            for(int i=0;i<6;++i){ q_[i]=m->position[i];if(m->velocity.size()>i) qd_[i]=m->velocity[i];}
            have_state_=true;});

            ref_sub_ = create_subscription<sensor_msgs::msg::JointState>("~/cmd_joint_state", 10,
            [&](const sensor_msgs::msg::JointState::SharedPtr m) {if(m->position.size() <6) return;
            for(int i=0;i<6;++i){qref_[i]= m->position[i]; if(m->velocity.size()>i) qdref_[i]=m->velocity[i];}
            have_ref_= true;});

            tau_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/joint_torque_cmd", 10);

            timer_ = create_wall_timer(std::chrono::duration<double>(1.0/rate_hz_), [&]{loop();});

            q_.setZero(6); qd_.setZero(6); qref_.setZero(6); qdref_.setZero(6); eint_.setZero(6);

            last_ = now();
            }

        private:
            void loop(){
                if(!have_state_ || !have_ref_) return;
                double dt = (now()-last_).seconds(); if(dt<=0) dt=1.0/rate_hz_; last_=now();
                VectorXd e =qref_ - q_, ed = qdref_ - qd_;
                eint_ += e * dt; //integral
                //anti-windup clamp
                for(int i=0;i<6;++i) eint_[i] = std::clamp(eint_[i], -5.0, 5.0);
                VectorXd tau = kp_*e + kd_*ed + ki_*eint_;
                for(int i=0;i<6;++i) tau[i] = std::clamp(tau[i], -tau_max_, tau_max_);

                std_msgs::msg::Float64MultiArray out; 
                out.data.assign(tau.data(), tau.data()+6);
                tau_pub_->publish(out);
            }

            double rate_hz_, kp_, kd_, ki_, tau_max_;
            bool have_state_{false}, have_ref_{false};
            VectorXd q_{6}, qd_{6}, qref_{6}, qdref_{6}, eint_{6};
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_, ref_sub_;
            rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tau_pub_;
            rclcpp::TimerBase::SharedPtr timer_; rclcpp::Time last_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDJoint>());
    rclcpp::shutdown();
    return 0;
}