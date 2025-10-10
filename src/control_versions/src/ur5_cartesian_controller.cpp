#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <Eigen/Dense>
#include <array> //Vector??
#include <deque>

//#include "control_version/ur5_dynamics.hpp"

using Eigen::Matrix4d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

struct DHRow {double a, alpha, d, theta0;};

static const std::array<DHRow,6> DH_UR5 = {{
  {  0.0,            M_PI/2, 0.089159, 0.0},
  { -0.425,          0.0,    0.0,      0.0},
  { -0.39225,        0.0,    0.0,      0.0},
  {  0.0,            M_PI/2, 0.10915,  0.0},
  {  0.0,           -M_PI/2, 0.09465,  0.0},
  {  0.0,            0.0,    0.0823,   0.0}
}};

static inline Matrix4d dhT(double a, double alpha, double d, double theta){
    const double ca = std::cos(alpha); 
    const double sa = std::sin(alpha);
    const double ct = std::cos(theta);
    const double st = std::sin(theta);

    Matrix4d T;

    T <<    ct, -st*ca,  st*sa, a*ct,
            st,  ct*ca, -ct*sa, a*st,
            0,     sa,     ca,    d,
            0,      0,      0,    1;
    return T;
}




static inline void rpy_from_R_zyx(const Eigen::Matrix3d& R,
                                  double& phi, double& theta, double& psi)
{
  psi   = std::atan2(R(1,0), R(0,0));
  theta = std::asin(-R(2,0));
  phi   = std::atan2(R(2,1), R(2,2));
}

static inline Eigen::Matrix3d W_rpy_zyx(double phi, double theta)
{
  const double sφ = std::sin(phi),  cφ = std::cos(phi);
  const double tθ = std::tan(theta), cθ = std::cos(theta);
  Eigen::Matrix3d W;
  W << 1,   sφ*tθ,    cφ*tθ,
       0,   cφ,      -sφ,
       0,   sφ/cθ,    cφ/cθ;
  return W;
}


// FK that also collects all frames
static inline void fk_all(const std::array<DHRow,6>& DH,
                          const Eigen::Matrix<double,6,1>& q,
                          std::array<Eigen::Matrix4d,7>& T,
                          std::array<Eigen::Vector3d,7>& o,
                          std::array<Eigen::Vector3d,7>& z)
{
  T[0] = Eigen::Matrix4d::Identity();
  o[0] = Eigen::Vector3d::Zero();
  z[0] = Eigen::Vector3d(0,0,1);
  for (int i = 0; i < 6; ++i) {
    const auto& r = DH[i];
    const double theta = r.theta0 + q(i);
    Eigen::Matrix4d A = dhT(r.a, r.alpha, r.d, theta);
    T[i+1] = T[i] * A;
    o[i+1] = T[i+1].block<3,1>(0,3);
    z[i+1] = T[i+1].block<3,1>(0,2);
  }
}

// orientation error as rotation vector (R_des * R_cur^T log map)
static Vector3d rotvec_error(const Matrix3d& R_cur, const Matrix3d& R_des){
    Matrix3d Rerr = R_des * R_cur.transpose();
    double tr = Rerr.trace();
    double cos_th = std::max (-1.0, std::min(1.0, (tr - 1.0)/2.0));
    double th = std::acos(cos_th);
    if (th < 1e-9) return Vector3d::Zero();
    Matrix3d K = (Rerr - Rerr.transpose())/(2.0*std::sin(th));
    Vector3d axis(K(2,1), K(0,2), K(1,0));
    return th * axis;
  }

static void fk_and_jacobian(const std::array<DHRow,6>& DH,
                            const VectorXd& q,
                            Matrix4d& T06, 
                            MatrixXd& Jg)
{
  // std::array<Vector3d,7> o; // origins of frames
  // std::array<Vector3d,7> z; // z axes of frames

  // o[0] = Vector3d::Zero(); // base origin
  // z[0] = Vector3d(0,0,1); // base z-axis
  // 1) FK for all frames
  std::array<Eigen::Matrix4d,7> T;
  std::array<Eigen::Vector3d,7> o;
  std::array<Eigen::Vector3d,7> z;
 // 1. Forward kinematics
  fk_all(DH,q, T, o ,z);
  T06 = T[6];

  //2. Geometric Jacobian [v;ω]=Jg∗q˙​ 
  //   J(q) maps joint velocities to Cartesian velocities,
  Jg.setZero();
  const Vector3d pe = o[6];
  for(int i = 0; i < 6; ++i){
    Vector3d zi_1 = z[i];
    Vector3d oi_1 = o[i];
    Vector3d Jv = zi_1.cross(pe - oi_1);
    Jg.block<3,1>(0,i) = Jv; // linear part (3x1) Take a 3×1 block; Starting at row 0, column i
    Jg.block<3,1>(3,i) = zi_1; // angular part (3x1)  Take a 3×1 block; Starting at row 3, column i
  }



  //3.  Analytical Jacobian (position + ZYX RPY rate) ---
  //Extract roll (φ), pitch (θ), yaw (ψ) from T06 rotation
  const Matrix3d R06 = T06.block<3,3>(0,0);
  double phi, theta, psi;
  rpy_from_R_zyx(R06, phi, theta, psi);

  // Build W(φ, θ) and combine: Ja = [ Jv ; W * Jω ]
  const Matrix3d W = W_rpy_zyx(phi, theta);

  MatrixXd Ja = MatrixXd::Zero(6,6); //v;φ˙​;θ˙;ψ˙​]=Ja∗q˙​
  //Ja.setZero(6,6);
  Ja.block<3,6>(0,0) = Jg.block<3,6>(0,0);                 // position part unchanged
  Ja.block<3,6>(3,0) = W * Jg.block<3,6>(3,0);             // orientation rate part
  // Optional: warn on gimbal lock when |cosθ| is small

  if (std::abs(std::cos(theta)) < 1e-6) {
  //   // You can handle this as needed in your app (throw, clamp, or log)
    std::cerr << "Warning: ZYX analytical Jacobian near gimbal lock (|cos(theta)| ~ 0)\n";
  }


}

  // static Matrix3d skew(const Vector3d& v){
  //   Matrix3d S;
  //   S << 0, -v.z(), v.y(),
  //       v.z(), 0, -v.x(),
  //       -v.y(), v.x(), 0;
  //   return S;
  // }



// Solve q_des for a desired Cartesian pose using iterative IK (DLS on pose error)
static bool solve_ik_pose(const std::array<DHRow,6>& DH,
                          const Eigen::Vector3d& p_des,
                          const Eigen::Matrix3d& R_des,
                          Eigen::Matrix<double,6,1>& q_io, // in: seed, out: q_des
                          int max_iters = 60,
                          double eps_pos = 1e-4,            // ~0.1 mm
                          double eps_ori = 1e-3,            // ~0.057 rad
                          double lambda = 1e-3,
                          double alpha = 1.0,               // step scaling 0..1
                          const Eigen::Matrix<double,6,6>& W = (Eigen::Matrix<double,6,6>() <<
                              1,0,0, 0,0,0,
                              0,1,0, 0,0,0,
                              0,0,1, 0,0,0,
                              0,0,0, 1,0,0,
                              0,0,0, 0,1,0,
                              0,0,0, 0,0,1).finished())
{
  for (int k=0; k<max_iters; ++k) {
    Eigen::Matrix4d T; Eigen::MatrixXd J(6,6);
    fk_and_jacobian(DH, q_io, T, J);

    const Eigen::Vector3d p_cur = T.block<3,1>(0,3);
    const Eigen::Matrix3d R_cur = T.block<3,3>(0,0);

    const Eigen::Vector3d e_pos = p_des - p_cur;
    const Eigen::Vector3d e_ori = rotvec_error(R_cur, R_des); // axis-angle error

    if (e_pos.norm() < eps_pos && e_ori.norm() < eps_ori) return true;

    Eigen::Matrix<double,6,1> e; e << e_pos, e_ori;

    // Weighted DLS step: dq = (J^T W J + λ² I)^(-1) J^T W e
    const Eigen::Matrix<double,6,6> JT_W = J.transpose() * W;
    const Eigen::Matrix<double,6,6> H    = JT_W * J + (lambda*lambda) * Eigen::Matrix<double,6,6>::Identity();
    const Eigen::Matrix<double,6,1> g    = JT_W * e;

    Eigen::Matrix<double,6,1> dq = alpha * H.ldlt().solve(g);

    // Optional: clamp step to stay smooth
    const double max_step = 0.15; // rad
    double n = dq.norm();
    if (n > max_step) dq *= (max_step / n);

    q_io += dq;
  }
  return false; // did not converge
}


 // 
  // damped pseudoinverse via normal eqss, another method would be "gradientenabstieg", you can implement it
  // damped least square is effective against singularity

  ///J may not be square (redundant or underactuated robots),
  //or it may not be invertible (singular configurations).
  //So we use the Moore–Penrose pseudoinverse, denoted  J+
  // Psedo inverse -> q˙​=(J-1)+x˙, jacobian if not square or not invertable
  // thats why  we use q˙​=(J+)x˙d​
  // J+=J−1
  // J+=J⊤(JJ⊤)−1
  //After taking the pseudoinverse of the Jacobian,
// you’re effectively solving a least-squares problem to 
// find the joint velocities that best reproduce the desired 
// end-effector velocity.
  static VectorXd dls(const MatrixXd& J, const VectorXd& e, double lambda){
    MatrixXd JT = J.transpose();
   //J+=(JTJ)−1 JT
    MatrixXd H  = JT * J + (lambda*lambda)*MatrixXd::Identity(J.cols(), J.cols()); // q˙​cmd​= (JTJ+λ2I)−1JT x_des​  
    VectorXd g  = JT * e;
    return H.ldlt().solve(g); 
  }

class UR5CartesianController : public rclcpp::Node {
  public:
    UR5CartesianController() : Node("ur5_cartesian_controller"){
    // Params
    declare_parameter<std::string>("joint_trajectory_topic", "~/joint_trajectory_cmd");
    declare_parameter<double>("rate_hz", 250.0);          // control loop
    declare_parameter<double>("lambda", 1e-3);            // DLS damping
    declare_parameter<double>("kp_pos", 2.0);             // position gain (m/s per m error)
    declare_parameter<double>("kp_ori", 2.0);             // orientation gain (rad/s per rad error)
    declare_parameter<std::string>("mode", "position");   // "position" | "velocity" | "acceleration"

    get_parameter("joint_trajectory_topic", traj_topic_);
    get_parameter("rate_hz", rate_hz_);
    get_parameter("lambda", lambda_);
    get_parameter("kp_pos", kp_pos_);
    get_parameter("kp_ori", kp_ori_);
    get_parameter("mode", mode_);

    //publishers/subscribers
    traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(traj_topic_, 10);
    
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",10,
    std::bind(&UR5CartesianController::onJointState, this, std::placeholders::_1));
    
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/cmd_pose", 10,
      std::bind(&UR5CartesianController::onPoseCmd, this, std::placeholders::_1)
    );

    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "~/cmd_twist", 10,
      std::bind(&UR5CartesianController::onTwistCmd, this, std::placeholders::_1)
    );

    accel_sub_ = create_subscription<geometry_msgs::msg::AccelStamped>(
      "~/cmd_accel",10,
      std::bind(&UR5CartesianController::onAccelCmd, this, std::placeholders::_1)
    );

    // Timer loop
    auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                               std::bind(&UR5CartesianController::loop, this));

    //Default joint order for UR
    joint_names_ = {"shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
                    "wrist_1_joint","wrist_2_joint","wrist_3_joint"};

    last_time_ = now();
    RCLCPP_INFO(get_logger(), "UR5 Cartesian controller started. Mode=%s", mode_.c_str());
    }
    private: 
    //state
    bool have_state_ = false;
    VectorXd q_ = VectorXd::Zero(6);
    VectorXd qd_ = VectorXd::Zero(6);
    VectorXd qdd_ = VectorXd::Zero(6);
    Matrix4d T_curr_ = Matrix4d::Identity();
    MatrixXd  J_cur_ = MatrixXd::Zero(6,6);

    //Commands
    bool have_pose_cmd_ = false;
    bool have_twist_cmd_ = false;
    bool have_accel_cmd_ = false;

    Vector3d p_des_ = Vector3d::Zero();
    Matrix3d R_des_ = Matrix3d::Identity();
    VectorXd xd_des_ = VectorXd::Zero(6); // twist [vx vy vz wx wy wz]
    VectorXd xdd_des_ = VectorXd::Zero(6); // spatial acce
    
    //Parameters
    std::string traj_topic_;
    double rate_hz_;
    double lambda_;
    double kp_pos_;
    double kp_ori_;
    std::string mode_;
    
    // ROS
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr accel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;
    std::deque<MatrixXd> J_hist_;
    std::deque<VectorXd> qd_hist_;

    void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg){
      if(msg->position.size() < 6) return;
      for(int i = 0; i<6; ++i) q_[i] = msg->position[i];
      if(msg->velocity.size() >= 6){
        for(int i=0; i<6; ++i) qd_[i] = msg->velocity[i];
      }
      //FK & J
      fk_and_jacobian(DH_UR5, q_, T_curr_, J_cur_);
      have_state_ = true;
    }

    void onPoseCmd(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      p_des_ = Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
      //Build R from quaternion
      const auto &q = msg->pose.orientation;
      Eigen::Quaterniond Q(q.w,q.x,q.y,q.z);
      R_des_ = Q.normalized().toRotationMatrix();
      have_pose_cmd_ = true;
    }

  void onTwistCmd(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    xd_des_ << msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
               msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z;
    have_twist_cmd_ = true;
  }

  void onAccelCmd(const geometry_msgs::msg::AccelStamped::SharedPtr msg) {
    xdd_des_ << msg->accel.linear.x, msg->accel.linear.y, msg->accel.linear.z,
                msg->accel.angular.x, msg->accel.angular.y, msg->accel.angular.z;
    have_accel_cmd_ = true;
  }

  void loop(){
    if(!have_state_) return;
    auto t = now();
    double dt = (t - last_time_).seconds();
    if (dt <=0) dt = 1.0 / rate_hz_;
    last_time_  = t;
    
    // Keep short histories for Jdot*qdot estimate
    if(J_hist_.size()>5) J_hist_.pop_front();
    if(qd_hist_.size()>5) qd_hist_.pop_front();
    J_hist_.push_back(J_cur_);
    qd_hist_.push_back(qd_);

    VectorXd qd_cmd = VectorXd::Zero(6);
    VectorXd qdd_cmd = VectorXd::Zero(6);

    if( mode_ == "position" && have_pose_cmd_){ // Differential IK
      //resolved rated: cartesian error -> twist_d
      Vector3d p_cur = T_curr_.block<3,1>(0,3);
      Matrix3d R_cur = T_curr_.block<3,3>(0,0);
      Vector3d e_pos = p_des_ - p_cur;
      Vector3d e_ori = rotvec_error(R_cur, R_des_); // can be disabled for position-only

      VectorXd xdot_des(6); //e_pos, e_ori) is position error,the output (xdot_des) is a velocity command, by definition of the control law.
      xdot_des.head<3>() = kp_pos_ * e_pos; //Position gain:  how fast you want to correct the error
      xdot_des.tail<3>() = kp_ori_ * e_ori; // set kp_ori_=0 for pure position // orienation gain

      // Robot kinematic:  x_dot˙=J(q)q_dot  
      // ˙​q_ cmd​=J+(q)x_des​ // J+: pseudoinverse
      qd_cmd = dls(J_cur_, xdot_des, lambda_);

    }else if (mode_ == "velocity" && have_twist_cmd_){
      // position and rotataion error is already provided. it assummes, it is rather diffuclt
      qd_cmd = dls(J_cur_, xd_des_, lambda_);

    }else if (mode_ == "acceleration" && have_accel_cmd_){
      //x˙=J(q)q˙​,x¨=J(q)q¨​+J˙(q,q˙​)q˙​
      // q¨​=J+(x¨des​−J˙q˙​)
      VectorXd Jdot_qdot = VectorXd::Zero(6);

      if (J_hist_.size() >= 2 && qd_hist_.size() >= 2) {
        const MatrixXd& Jk = J_hist_.back();
        const MatrixXd& Jk1= J_hist_[J_hist_.size()-2];
        // crude finite-diff for Jdot: (Jk - Jk1)/dt * qdot (use latest qdot)
        MatrixXd Jdot = (Jk - Jk1) / std::max(1e-4, dt);
        Jdot_qdot = Jdot * qd_;
      }
      qdd_cmd = dls(J_cur_, xdd_des_ - Jdot_qdot, lambda_);
      // integrate
      qd_cmd = qd_ + qdd_cmd * dt;
  }else if(mode_ == "position_direct" && have_pose_cmd_){ // IK solver
    Eigen::Matrix<double,6,1> q_goal = q_;
    // (Optional) tune weights: prioritize position over orientation, for example
    MatrixXd W = MatrixXd::Identity(6,6);
    // W(3,3)=W(4,4)=W(5,5)=0.5; // uncomment to de-emphasize orientation
    bool ok = solve_ik_pose(DH_UR5, p_des_, R_des_, q_goal,
                          /*max_iters*/80,
                          /*eps_pos*/1e-4,
                          /*eps_ori*/1e-3,
                          /*lambda*/lambda_,
                          /*alpha*/1.0,
                          W);
    if(!ok) {
      RCLCPP_WARN(get_logger(), "IK (position_direct) did not converge; publishing best found q.");
    }
    // Publish a single-point position commad
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.resize(6);
    pt.velocities.resize(6); std::fill(pt.velocities.begin(), pt.velocities.end(), 0.0);
    pt.accelerations.resize(6); std::fill(pt.accelerations.begin(), pt.accelerations.end(), 0.0);

    for(int i=0; i<6;++i) pt.positions[i] = q_goal[i];
      //give some move time(tune it)
      pt.time_from_start = rclcpp::Duration::from_seconds(1.0 / std::max(1.0, rate_hz_));
      traj.points.push_back(pt);
      traj_pub_->publish(traj); 
      return;

  }else{
    return;
  }
      // Integrate to position for the trajectory point (open-loop)
    VectorXd q_cmd = q_ + qd_cmd * dt;


    // Publish a single-point JointTrajectory (time_from_start = dt)
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.resize(6);
    pt.velocities.resize(6);
    pt.accelerations.resize(6);

    for (int i=0;i<6;++i) {
      pt.positions[i]     = q_cmd[i];
      pt.velocities[i]    = qd_cmd[i];
      pt.accelerations[i] = qdd_cmd[i];
    }
    pt.time_from_start = rclcpp::Duration::from_seconds(dt);
    traj.points.push_back(pt);
    traj_pub_->publish(traj);
  }
    std::vector<std::string> joint_names_;

};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UR5CartesianController>());
  rclcpp::shutdown();
  return 0;
}

