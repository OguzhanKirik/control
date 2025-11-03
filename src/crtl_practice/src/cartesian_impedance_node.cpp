// cartesian_impedance_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include "crtl_practice/ur5_kin_helpers.hpp"
#include "crtl_practice/ur5_dynamics.hpp"

/*1.Forward Kinematics: x = FK(q)
2. Position error: e_p = p_d − p
3. Orientation error: e_o = log(Rᵀ R_d)
4. Combine both: e = [e_p; e_o]
5. Diffrential Kinematics: J = Jacobian(q)
6. Cartesian Velocity: xdot = J q̇
7. Veloctiy error: ė = ẋ_d − ẋ 
8. Cartesian Spring-damper: F = K ⊙ e + D ⊙ ė
9. Robot dynamics terms: C,G = Dynamics(q, q̇)
10. Map wrench to joint torques: τ = Jᵀ F + C + G
Conceptually: τ = Jᵀ (K ⊙ e + D ⊙ ė) + C + G
*/
using Eigen::VectorXd; using Eigen::MatrixXd; using Eigen::Vector3d; using Eigen::Matrix3d;

class CartesianImpedance : public rclcpp::Node {
public:
  CartesianImpedance(): Node("cartesian_impedance"){
    rate_hz_ = declare_parameter("rate_hz", 250.0);
    K_lin_ = declare_parameter("K_lin", 800.0);
    D_lin_ = declare_parameter("D_lin", 40.0);
    K_ang_ = declare_parameter("K_ang", 50.0);
    D_ang_ = declare_parameter("D_ang", 4.0);
    tau_max_ = declare_parameter("tau_max", 120.0);

    js_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,[&](auto m){
      if(m->position.size()<6) return; for(int i=0;i<6;++i){ q_[i]=m->position[i]; if(m->velocity.size()>i) qd_[i]=m->velocity[i];}
      fk_and_jacobian(DH_UR5, q_, T_, J_); have_state_=true; });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("~/cmd_pose",10,[&](auto m){
      p_d_ = {m->pose.position.x,m->pose.position.y,m->pose.position.z};
      Eigen::Quaterniond Q(m->pose.orientation.w,m->pose.orientation.x,m->pose.orientation.y,m->pose.orientation.z);
      R_d_ = Q.normalized().toRotationMatrix(); have_ref_=true; });

    tau_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/joint_torque_cmd",10);
    timer_   = create_wall_timer(std::chrono::duration<double>(1.0/rate_hz_), [&]{loop();});
  }
private:
  void loop(){
    if(!have_state_||!have_ref_) return;
    Vector3d p = T_.block<3,1>(0,3); Matrix3d R = T_.block<3,3>(0,0);
    Vector3d ep = p_d_ - p; Vector3d eo = rotvec_error(R, R_d_);

    Eigen::Matrix<double,6,1> e; e << ep, eo;
    Eigen::Matrix<double,6,1> xdot = J_*qd_;
    Eigen::Matrix<double,6,1> edot = -xdot; // if no desired xdot

    Eigen::Matrix<double,6,1> K, D;
    K << K_lin_,K_lin_,K_lin_, K_ang_,K_ang_,K_ang_;
    D << D_lin_,D_lin_,D_lin_, D_ang_,D_ang_,D_ang_;
    //Jacobain transforms cartesian wrench to tau
    Eigen::Matrix<double,6,1> F = K.cwiseProduct(e) + D.cwiseProduct(edot);
    VectorXd C = C_of(q_, qd_), G = G_of(q_);
    VectorXd tau = J_.transpose()*F + C + G; //desired Cartesian force F=Ke+De˙
    for(int i=0;i<6;++i) tau[i] = std::clamp(tau[i], -tau_max_, tau_max_);
    std_msgs::msg::Float64MultiArray out; out.data.assign(tau.data(), tau.data()+6); tau_pub_->publish(out);
  }
  double rate_hz_, K_lin_,D_lin_,K_ang_,D_ang_, tau_max_;
  bool have_state_{false}, have_ref_{false};
  VectorXd q_{6}, qd_{6}; Eigen::Matrix4d T_{Eigen::Matrix4d::Identity()}; MatrixXd J_{MatrixXd::Zero(6,6)};
  Vector3d p_d_{0,0,0}; Matrix3d R_d_{Matrix3d::Identity()};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tau_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int a,char**b){rclcpp::init(a,b);rclcpp::spin(std::make_shared<CartesianImpedance>());rclcpp::shutdown();}
