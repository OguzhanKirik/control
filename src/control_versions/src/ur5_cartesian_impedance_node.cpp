/*
Problem definion
Given: Robot joint angles and velocities
Goal: Move to desired pose, and possibly track a desired velocity
Controller's Job: Compute joint torques that make the robot behave like a 
6D virtual spring-damper

1.Forward Kinematics: x = FK(q)
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



#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <vector>
#include <string>

using Eigen::VectorXd;
using Eigen::MatrixXd;

class CartesianImpedanceUR5KDL : public rclcpp::Node {
public:
  CartesianImpedanceUR5KDL() : Node("ur5_cartesian_impedance_node")
  {
    // ---- Parameters ----
    base_link_ = declare_parameter<std::string>("base_link", "base_link");
    ee_link_   = declare_parameter<std::string>("ee_link",   "tool0");
    rate_hz_   = declare_parameter<double>("rate_hz", 250.0);

    auto K_lin = declare_parameter<std::vector<double>>("K_lin", {800,800,800});
    auto K_ang = declare_parameter<std::vector<double>>("K_ang", {50,50,50});
    auto D_lin = declare_parameter<std::vector<double>>("D_lin", {40,40,40});
    auto D_ang = declare_parameter<std::vector<double>>("D_ang", {4,4,4});

    tau_max_      = declare_parameter<double>("tau_max",      120.0);
    tau_rate_max_ = declare_parameter<double>("tau_rate_max", 2000.0);

    K_.setZero(); D_.setZero();
    for (int i=0;i<3;++i){ K_(i)=K_lin.at(i); K_(i+3)=K_ang.at(i); }
    for (int i=0;i<3;++i){ D_(i)=D_lin.at(i); D_(i+3)=D_ang.at(i); }

    // ---- Build KDL chain from URDF on parameter server ----
    std::string urdf_xml;
    if (!get_parameter("robot_description", urdf_xml)) {
      RCLCPP_FATAL(get_logger(), "robot_description param not set.");
      throw std::runtime_error("No robot_description");
    }
    
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(urdf_xml, tree)) {
      RCLCPP_FATAL(get_logger(), "Failed to parse URDF into KDL tree.");
      throw std::runtime_error("URDF parse failed");
    }
    if (!tree.getChain(base_link_, ee_link_, chain_)) {
      RCLCPP_FATAL(get_logger(), "Failed to get KDL chain %s -> %s", base_link_.c_str(), ee_link_.c_str());
      throw std::runtime_error("Chain build failed");
    }

    nj_ = chain_.getNrOfJoints();
    q_.resize(nj_); qdot_.resize(nj_);
    q_.data.setZero(); qdot_.data.setZero();
    tau_prev_.setZero(nj_);

    fk_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
    jac_.reset(new KDL::ChainJntToJacSolver(chain_));
    gravity_ = KDL::Vector(0,0,-9.81);
    dyn_.reset(new KDL::ChainDynParam(chain_, gravity_));

    // ---- IO ----
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&CartesianImpedanceUR5KDL::onJointState, this, std::placeholders::_1));

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/target_pose", 10,
      std::bind(&CartesianImpedanceUR5KDL::onTargetPose, this, std::placeholders::_1));

    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "~/target_twist", 10,
      std::bind(&CartesianImpedanceUR5KDL::onTargetTwist, this, std::placeholders::_1));

    tau_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("~/joint_torque_cmd", 10);

    // Loop timer
    auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                               std::bind(&CartesianImpedanceUR5KDL::loop, this));

    dx_d_.setZero();
    have_state_ = false; have_pose_ = false;

    RCLCPP_INFO(get_logger(), "KDL Cartesian impedance running (chain %s->%s, joints=%zu).",
                base_link_.c_str(), ee_link_.c_str(), nj_);
  }

private:
  // Params
  std::string base_link_, ee_link_;
  double rate_hz_{250.0}, tau_max_{120.0}, tau_rate_max_{2000.0};

  // KDL
  KDL::Chain chain_;
  size_t nj_{0};
  KDL::JntArray q_, qdot_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_;
  std::unique_ptr<KDL::ChainDynParam> dyn_;
  KDL::Vector gravity_;

  // State/targets
  bool have_state_{false}, have_pose_{false};
  KDL::Frame x_d_;                       // desired pose (base frame)
  Eigen::Matrix<double,6,1> dx_d_;       // desired spatial vel (base frame)
  Eigen::VectorXd tau_prev_;

  // Gains
  Eigen::Matrix<double,6,1> K_, D_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tau_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- helpers ---
  static Eigen::Vector3d rotLog(const KDL::Rotation& R_err){
    // convert to axis-angle (small-angle safe)
    double x,y,z,w; R_err.GetQuaternion(x,y,z,w);
    w = std::clamp(w, -1.0, 1.0);
    double angle = 2.0 * std::acos(w);
    double s = std::sqrt(std::max(1.0 - w*w, 0.0));
    Eigen::Vector3d axis(0,0,0);
    if (s > 1e-6) axis = Eigen::Vector3d(x/s, y/s, z/s);
    return angle * axis;
  }

  void saturateAndRateLimit(Eigen::VectorXd& tau, double dt){
    const double max_step = tau_rate_max_ * dt;
    for (int i=0;i<tau.size(); ++i){
      tau[i] = std::clamp(tau[i], -tau_max_, tau_max_);
      tau[i] = std::clamp(tau[i], tau_prev_[i]-max_step, tau_prev_[i]+max_step);
    }
    tau_prev_ = tau;
  }

  // --- callbacks ---
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg){
    if (msg->position.size() < nj_) return; // simple assumption: order matches chain
    for (size_t i=0;i<nj_; ++i) q_(i) = msg->position[i];
    if (msg->velocity.size() >= nj_) for (size_t i=0;i<nj_; ++i) qdot_(i) = msg->velocity[i];
    have_state_ = true;
  }

  void onTargetPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    const auto& p = msg->pose.position;
    const auto& o = msg->pose.orientation;
    x_d_.p = KDL::Vector(p.x, p.y, p.z);
    x_d_.M = KDL::Rotation::Quaternion(o.x, o.y, o.z, o.w);  // (x,y,z,w)
    have_pose_ = true;
  }

  void onTargetTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg){
    dx_d_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
             msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;
  }

  // --- main loop ---
  void loop(){
    if (!have_state_ || !have_pose_) return;
    const double dt = 1.0 / rate_hz_;

    // FK pose (base frame)
    KDL::Frame x;
    fk_->JntToCart(q_, x);

    // Pose error
    KDL::Vector ep_kdl = x_d_.p - x.p;                   // base frame pos error
    KDL::Rotation R_err = x.M.Inverse() * x_d_.M;        // R^T * R_d
    Eigen::Vector3d ep(ep_kdl.x(), ep_kdl.y(), ep_kdl.z());
    Eigen::Vector3d eo = rotLog(R_err);

    Eigen::Matrix<double,6,1> e; e << ep, eo;

    // Jacobian (base frame) & spatial velocity
    KDL::Jacobian J_kdl(nj_);
    jac_->JntToJac(q_, J_kdl);
    MatrixXd J(6, nj_);
    for (size_t r=0;r<6;++r) for (size_t c=0;c<nj_; ++c) J(r,c) = J_kdl(r,c);

    Eigen::Matrix<double,6,1> dx = J * qdot_.data;   // base-frame twist
    Eigen::Matrix<double,6,1> edot = dx_d_ - dx;

    // Impedance wrench
    Eigen::Matrix<double,6,1> Fcmd = K_.cwiseProduct(e) + D_.cwiseProduct(edot);

    // Dynamics (Coriolis + Gravity)
    KDL::JntSpaceInertiaMatrix M_kdl(nj_);
    KDL::JntArray C_kdl(nj_), G_kdl(nj_);
    dyn_->JntToMass(q_, M_kdl);
    dyn_->JntToCoriolis(q_, qdot_, C_kdl);
    dyn_->JntToGravity(q_, G_kdl);

    VectorXd C(nj_), G(nj_);
    for (size_t i=0;i<nj_; ++i){ C(i)=C_kdl(i); G(i)=G_kdl(i); }

    // Map to joint torques
    VectorXd tau = J.transpose() * Fcmd + C + G;

    // Safety + publish
    saturateAndRateLimit(tau, dt);
    std_msgs::msg::Float64MultiArray out;
    out.data.assign(tau.data(), tau.data()+tau.size());
    tau_pub_->publish(out);
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianImpedanceUR5KDL>());
  rclcpp::shutdown();
  return 0;
}
