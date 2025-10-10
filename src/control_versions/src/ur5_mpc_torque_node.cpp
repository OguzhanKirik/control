// ur5_mpc_torque_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <Eigen/Dense>
#include "control_versions/ur5_dynamics.hpp"

using Eigen::MatrixXd; using Eigen::VectorXd;

class UR5MPC : public rclcpp::Node {
public:
  UR5MPC() : Node("ur5_mpc_torque")
  {
    declare_parameter<double>("rate_hz", 200.0);
    declare_parameter<int>("N", 10);
    declare_parameter<double>("dt", 0.005);
    declare_parameter<double>("q_weight", 50.0);
    declare_parameter<double>("qd_weight", 5.0);
    declare_parameter<double>("tau_weight", 1e-3);

    get_parameter("rate_hz", rate_hz_);
    get_parameter("N", N_);
    get_parameter("dt", dt_);
    get_parameter("q_weight", wq_);
    get_parameter("qd_weight", wqd_);
    get_parameter("tau_weight", wr_);

    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&UR5MPC::onJS, this, std::placeholders::_1));

    ref_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "~/cmd_joint_state", 10, std::bind(&UR5MPC::onRef, this, std::placeholders::_1));

    tau_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/joint_torque_cmd", 10);

    auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&UR5MPC::loop, this));

    x_ref_.setZero(12); q_.setZero(6); qd_.setZero(6);
    RCLCPP_INFO(get_logger(), "UR5 MPC (torque) started. N=%d, dt=%.3g", N_, dt_);
  }

private:
  double rate_hz_{200.0}, dt_{0.005};
  int N_{10};
  double wq_{50.0}, wqd_{5.0}, wr_{1e-3};
  bool have_state_{false}, have_ref_{false};
  VectorXd q_{6}, qd_{6}, x_ref_{12};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_, ref_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tau_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void onJS(const sensor_msgs::msg::JointState::SharedPtr msg){
    if (msg->position.size()<6) return;
    for(int i=0;i<6;++i) q_(i)=msg->position[i];
    if (msg->velocity.size()>=6) for(int i=0;i<6;++i) qd_(i)=msg->velocity[i];
    have_state_ = true;
  }
  void onRef(const sensor_msgs::msg::JointState::SharedPtr msg){
    // Desired joint positions/velocities; efforts ignored here
    x_ref_.setZero(12);
    for(int i=0;i<6 && i<(int)msg->position.size(); ++i) x_ref_(i)=msg->position[i];
    for(int i=0;i<6 && i<(int)msg->velocity.size(); ++i) x_ref_(6+i)=msg->velocity[i];
    have_ref_ = true;
  }

  // Build stacked least-squares: solve for Tau = [tau_0; ...; tau_{N-1}] (6N x 1)
  void loop(){
    if(!have_state_ || !have_ref_) return;

    // Linearized (time-invariant over the horizon) model at current state
    MatrixXd M = mass_matrix(q_);
    MatrixXd Minv = M.inverse(); // 6x6
    VectorXd h = nonlinear_term(q_, qd_);
    VectorXd g = inverse_dynamics(q_, VectorXd::Zero(6), VectorXd::Zero(6), /*gravity=*/true);
    VectorXd b = -Minv*(h+g); // bias accel from nonlinear terms

    // Discrete-time matrices
    // x = [q; qd], u = tau
    MatrixXd A = MatrixXd::Zero(12,12);
    A.block<6,6>(0,0) = MatrixXd::Identity(6,6);
    A.block<6,6>(0,6) = dt_ * MatrixXd::Identity(6,6);
    A.block<6,6>(6,6) = MatrixXd::Identity(6,6);

    MatrixXd B = MatrixXd::Zero(12,6);
    B.block<6,6>(6,0) = dt_ * Minv; // qd_{k+1} += dt * Minv * tau_k

    VectorXd c = VectorXd::Zero(12);
    c.tail(6) = dt_ * b; // bias on qd

    // Build block prediction matrices (Toeplitz) for N steps
    const int nx=12, nu=6;
    MatrixXd Sx = MatrixXd::Zero(nx*N_, nx);
    MatrixXd Su = MatrixXd::Zero(nx*N_, nu*N_);
    VectorXd sc = VectorXd::Zero(nx*N_);

    MatrixXd A_pow = MatrixXd::Identity(nx, nx);
    for(int k=0;k<N_;++k){
      A_pow = (k==0) ? A : (A_pow * A);
      Sx.block(nx*k, 0, nx, nx) = A_pow;
      // input influence
      MatrixXd Akm = MatrixXd::Identity(nx,nx);
      for(int j=0;j<=k;++j){
        if(j>0) Akm = Akm * A;
        Su.block(nx*k, nu*j, nx, nu) += Akm * B;
        sc.segment(nx*k, nx) += Akm * c;
      }
    }

    // Cost weights
    MatrixXd Q = MatrixXd::Zero(nx, nx);
    Q.block<6,6>(0,0) = wq_*MatrixXd::Identity(6,6);
    Q.block<6,6>(6,6) = wqd_*MatrixXd::Identity(6,6);
    MatrixXd R = wr_ * MatrixXd::Identity(nu*N_, nu*N_);

    // Stack cost: 0.5 || (Sx x0 + Su Tau + sc - Xref_stack) ||_Qbar^2 + 0.5 ||Tau||_R^2
    VectorXd x0(nx); x0 << q_, qd_;
    VectorXd Xref(nx*N_); for(int k=0;k<N_;++k) Xref.segment(nx*k, nx) = x_ref_;

    // Block-diagonal Qbar
    MatrixXd Qbar = MatrixXd::Zero(nx*N_, nx*N_);
    for(int k=0;k<N_;++k) Qbar.block(nx*k, nx*k, nx, nx) = Q;

    // Normal equations: (Su^T Qbar Su + R) Tau = Su^T Qbar (Xref - Sx x0 - sc)
    MatrixXd H = Su.transpose() * Qbar * Su + R;
    VectorXd rhs = Su.transpose() * Qbar * (Xref - Sx * x0 - sc);

    // Solve and take first control
    VectorXd Tau = H.ldlt().solve(rhs);
    VectorXd tau0 = Tau.head(6);

    // Publish first torque
    std_msgs::msg::Float64MultiArray out;
    out.data.assign(tau0.data(), tau0.data()+6);
    tau_pub_->publish(out);
  }
};

int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<UR5MPC>());
  rclcpp::shutdown();
  return 0;
}
