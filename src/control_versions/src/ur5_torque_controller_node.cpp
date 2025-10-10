//ur5_torque_controller_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include "control_versions/ur5_dynamics.hpp"

using Eigen::VectorXd; using Eigen::MatrixXd;

class UR5TorqueController : public rclcpp::Node{
    public:
        UR5TorqueController() : Node("ur5_torque_controller"){

            declare_parameter<double>("rate_hz", 250.0);
            declare_parameter<double>("kp", 80.0);
            declare_parameter<double>("kd", 16.0);

            get_parameter("rate_hz", rate_hz_);
            get_parameter("kp", kp_);
            get_parameter("kd", kd_);

            js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
                "/joint_state", 10, std::bind(&UR5TorqueController::onJs, this, std::placeholders::_1));
            
            //Desired joint state:positions, velocities, use efforts for desired acceleration
            des_sub_ = create_subscription<sensor_msgs::msg::JointState>(
                "~/cmd_joint_state",10, std::bind(&UR5TorqueController::onRef, this, std::placeholders::_1));

            tau_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/joint_torque_cmd",10);

            auto period = std::chrono::duration<double>(1.0 / rate_hz_);
            timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                std::bind(&UR5TorqueController::loop, this));

                        
            q_.setZero(6); qd_.setZero(6);
            qd_ref_.setZero(6); q_ref_.setZero(6); qdd_ref_.setZero(6);
            RCLCPP_INFO(get_logger(), "UR5 torque controller started.");
    
        }
    private:
        double rate_hz_{250.0};
        double kp_{80.0}, kd_{16.0};
        bool have_state_{false}, have_ref_{false};
        VectorXd q_{6}, qd_{6}, q_ref_{6}, qd_ref_{6}, qdd_ref_{6};
        
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_, des_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tau_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        void onJs(sensor_msgs::msg::JointState::SharedPtr msg ){
            if(msg->position.size()<6) return;
            for(int i=0;i<6;++i) q_(i) = msg->position[i];
            if (msg->velocity.size()>=6) for(int i=0;i<6;++i) qd_(i)=msg->velocity[i];
            have_state_=true;       
        }

        void onRef(const sensor_msgs::msg::JointState::SharedPtr msg){
            if (msg->position.size()>=6) for(int i=0;i<6;++i) q_ref_(i)=msg->position[i];
            if (msg->velocity.size()>=6) for(int i=0;i<6;++i) qd_ref_(i)=msg->velocity[i];
            if (msg->effort.size()  >=6) for(int i=0;i<6;++i) qdd_ref_(i)=msg->effort[i];
            have_ref_=true;
        }

        void loop(){
            if(!have_state_ || !have_ref_) return;

            // 1️. Compute tracking errors
            //τ=M(q)q¨​cmd​+h(q,q˙​)+g(q).
            VectorXd e  = q_ref_ - q_;
            VectorXd ed = qd_ref_ - qd_;

            // 2️, Desired joint acceleration (PD + feedforward)
            //     qdd_cmd = qdd_ref + Kp*(q_ref - q) + Kd*(qd_ref - qd)
            VectorXd qdd_cmd = qdd_ref_ + kp_ * e + kd_ * ed;

            // 3️. Compute dynamic model terms
            //     M(q) : joint-space inertia matrix
            //     h(q, qd) : Coriolis/centrifugal terms
            //     g(q) : gravity torque
            //             | Symbol        | Meaning                                           |
            // | ------------- | ------------------------------------------------- |
            // | (q)           | joint positions (radians)                         |
            // | (\dot q)      | joint velocities (rad/s)                          |
            // | (\ddot q)     | joint accelerations (rad/s²)                      |
            // | (M(q))        | inertia matrix (depends on robot geometry + mass) |
            // | (h(q,\dot q)) | Coriolis & centrifugal terms                      |
            // | (g(q))        | gravity torques                                   |
            // | (\tau)        | actuator torques (your control output)            |

            MatrixXd M = mass_matrix(q_);
            VectorXd h = nonlinear_term(q_, qd_);
            VectorXd g = inverse_dynamics(q_, VectorXd::Zero(6), VectorXd::Zero(6), /*include_gravity=*/true);

            // 4️⃣ Computed Torque Control Law
            //     τ = M(q) * qdd_cmd + h(q, qd) + g(q)
            // If the model is perfect, this cancels robot dynamics
            // and the closed-loop error behaves like a simple PD system.
            VectorXd tau = M * qdd_cmd + h + g;

            // ------------------------------------------------------------------------
            // 5️⃣ Publish torque command to the effort controller
            // ------------------------------------------------------------------------
            std_msgs::msg::Float64MultiArray out;
            out.data.assign(tau.data(), tau.data() + 6);
            tau_pub_->publish(out);
}

};

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UR5TorqueController>());
    rclcpp::shutdown();
    return 0;
}