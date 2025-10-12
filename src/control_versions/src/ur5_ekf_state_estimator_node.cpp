// ur5_state_estimator.cpp
// Minimal ROS2 state estimator for UR5.
// - Linear KF per joint for [q, qdot] from /joint_states
// - Optional EKF-style fusion with PoseStamped (camera/tool) via FK & Jacobian
// - Publishes filtered JointState, EE PoseStamped, EE TwistStamped

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <Eigen/Dense>
#include <array>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>


using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;

struct DHRow { double a, alpha, d, theta0; };

// ---------- UR5 Standard DH (NOT MDH) ----------
static const std::array<DHRow,6> DH_UR5 = {{
  {  0.0,            M_PI/2, 0.089159, 0.0},
  { -0.425,          0.0,    0.0,      0.0},
  { -0.39225,        0.0,    0.0,      0.0},
  {  0.0,            M_PI/2, 0.10915,  0.0},
  {  0.0,           -M_PI/2, 0.09465,  0.0},
  {  0.0,            0.0,    0.0823,   0.0}
}};

// ---------- Kinematics ----------
static inline Matrix4d dhT(double a,double alpha,double d,double theta){
  const double ca=std::cos(alpha), sa=std::sin(alpha);
  const double ct=std::cos(theta),  st=std::sin(theta);
  Matrix4d T;
  T <<  ct, -st*ca,  st*sa, a*ct,
        st,  ct*ca, -ct*sa, a*st,
         0,     sa,     ca,    d,
         0,      0,      0,    1;
  return T;
}

static void fk_and_jacobian(const std::array<DHRow,6>& DH, const Eigen::Ref<const VectorXd>& q,
                            Matrix4d& T06, MatrixXd& J){


    std::array<Vector3d, 7> o; o[0] = Vector3d::Zero();
    std::array<Vector3d, 7> z; z[0] = Vector3d(0,0,1);

    Matrix4d T = Matrix4d::Identity();
    for(int i = 0; i<6; ++i){
        const auto& r = DH[i];
        Matrix4d A = dhT(r.a,r.alpha,r.d,r.theta0 + q[i]);
        T = T * A;
        o[i+1] = T.block<3,1>(0,3);  // position vector (3x1)
        z[i+1] = T.block<3,1>(0,2);  // z-axis vector (3rd column of rotation)
    }

    // Store final transformation
    T06 = T;

    //geometric Jacibain (revolute)
    J = MatrixXd::Zero(6,6);
    Vector3d pe = o[6];
    for(int i=0;i<6; ++i){
        Vector3d zi = z[i];
        Vector3d oi = o[i];
        J.block<3,1>(0,i) = zi.cross(pe-oi);
        J.block<3,1>(3,i) = zi;
    }
}

static Matrix3d quatToR(double w,double x,double y,double z){
  Eigen::Quaterniond Q(w,x,y,z); Q.normalize();
  return Q.toRotationMatrix();
}

static Vector3d rotvec_error(const Matrix3d& R_cur, const Matrix3d& R_des){
  Matrix3d Rerr = R_des * R_cur.transpose();
  double tr = Rerr.trace();
  double c = std::max(-1.0, std::min(1.0, (tr - 1.0)/2.0));
  double th = std::acos(c);
  if (th < 1e-12) return Vector3d::Zero();
  Matrix3d K = (Rerr - Rerr.transpose())/(2.0*std::sin(th));
  Vector3d axis(K(2,1), K(0,2), K(1,0));
  return th * axis;
}

// EKF node
class UR5EKFStateEstimator : public rclcpp::Node{
    public: 
        UR5EKFStateEstimator() : Node("ur5_ekf_state_estimator"){
            //Params
        declare_parameter<std::vector<std::string>>("joint_names", {
        "shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
        "wrist_1_joint","wrist_2_joint","wrist_3_joint"});     
            declare_parameter<double>("rate_hz", 250.0);
        // Process noise (diag): q_pos (rad^2/s^3), q_vel (rad^2/s)
        declare_parameter<double>("q_pos_rw", 1e-7);   // affects position via integration
        declare_parameter<double>("q_vel_rw", 1e-3);   // random-walk on qdot
        // Encoder measurement noise (rad^2)
        declare_parameter<double>("r_enc_pos", 1e-6);
        // Camera measurement noise
        declare_parameter<bool>("use_camera_pose", false);
        declare_parameter<double>("r_cam_pos", 1e-4);  // m^2
        declare_parameter<double>("r_cam_ori", 1e-3);  // rad^2
        
        
        get_parameter("joint_names", joint_names_);
        get_parameter("rate_hz", rate_hz_);
        get_parameter("q_pos_rw", q_pos_rw_);
        get_parameter("q_vel_rw", q_vel_rw_);
        get_parameter("r_enc_pos", r_enc_pos_);
        get_parameter("use_camera_pose", use_cam_pose_);
        get_parameter("r_cam_pos", r_cam_pos_);
        get_parameter("r_cam_ori", r_cam_ori_);
    
        x_.setZero(12);
        P_.setZero(12,12);
        P_ *= 1e-3; //small inital uncertainty

        sub_js_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",10, std::bind(&UR5EKFStateEstimator::onJointState,this, std::placeholders::_1));
    
        if(use_cam_pose_){
            sub_cam_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "~/camera_pose", 10, std::bind(&UR5EKFStateEstimator::onCamPose, this, std::placeholders::_1));
        }

        pub_js_filtered_ = create_publisher<sensor_msgs::msg::JointState>(
            "~/joint_states",10);
        pub_ee_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "~/ee_pose",10);
        pub_ee_twist_= create_publisher<geometry_msgs::msg::TwistStamped>(
            "~/ee_twist",10);

        //timer
        auto period = std::chrono::milliseconds((int)std::round(1000/rate_hz_));
        timer_ = create_wall_timer(period, std::bind(
            &UR5EKFStateEstimator::tick,this));
        last_time_ = now();

        RCLCPP_INFO(get_logger(), "UR5 EKF state estimator started. use_camera_pose=%s",
                use_cam_pose_ ? "true" : "false");
    }
private:
    // Parameters
    std::vector<std::string> joint_names_;
    double rate_hz_{250.0};
    double q_pos_rw_{1e-7}, q_vel_rw_{1e-3};
    double r_enc_pos_{1e-6};
    bool use_cam_pose_{false};
    double r_cam_pos_{1e-4}, r_cam_ori_{1e-3};

    // EKF state
    VectorXd x_{12};     // [q(6); qdot(6)]
    MatrixXd P_{12,12};

    // Inputs
    bool have_enc_{false};
    std::array<double,6> enc_q_{};
    bool have_cam_{false};
    Vector3d cam_p_{Vector3d::Zero()};
    Matrix3d cam_R_{Matrix3d::Identity()};

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_cam_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_js_filtered_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_ee_pose_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_ee_twist_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;

    //ROS Callbacks
    void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg){
        for (int i=0;i<6;++i){
            auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
            if (it == msg->name.end()) continue;
            size_t idx = it - msg->name.begin();
            enc_q_[i] = (idx < msg->position.size()) ? msg->position[idx] : 0.0;
        }
        have_enc_ = true;
        }


    void onCamPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        cam_p_ = Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        cam_R_ = quatToR(msg->pose.orientation.w, msg->pose.orientation.x,
                            msg->pose.orientation.y, msg->pose.orientation.z);
        have_cam_ = true;
    }

    // EKF predict
    void predict(double dt){
        // x = [q;qdot]
        // f(x) = q += qdot*dt; qdot +=0
        VectorXd x_new(12); x_new = x_;
        x_new.segment<6>(0) += x_.segment<6>(6) * dt;
        x_ = x_new;

        //Jacbian F
        MatrixXd F = MatrixXd::Identity(12,12);
        F.block<6,6>(0,6) = MatrixXd::Identity(6,6) * dt;

        //Process noise Q
        MatrixXd Q = MatrixXd::Zero(12,12);

        //Position RW from integrity velicty nouse
        for (int i=0;i<6;++i){
            Q(i,i)         = q_pos_rw_; // tuned in rad^2
            Q(6+i,6+i)     = q_vel_rw_; // tuned in rad^2
        }

        P_ = F * P_ * F.transpose() + Q;        
    }

    // ---------- EKF update: encoders (linear) ----------
    void update_encoders(){
      // z = q (6x1), H = [I6  0]
      Eigen::Matrix<double,6,1> z;
      for (int i=0;i<6;++i) z(i) = enc_q_[i];

      MatrixXd H = MatrixXd::Zero(6,12);
      H.block<6,6>(0,0) = MatrixXd::Identity(6,6);

      MatrixXd R = MatrixXd::Identity(6,6) * r_enc_pos_; // diag

      VectorXd y = z - H * x_;
      MatrixXd S = H * P_ * H.transpose() + R;
      MatrixXd K = P_ * H.transpose() * S.inverse();

      x_ += K * y;
      MatrixXd I = MatrixXd::Identity(12,12);
      P_ = (I - K*H) * P_;
    }

    // ---------- EKF update: camera pose (nonlinear) ----------
    void update_camera(){
        // h(x): FK(q) -> position p and orientation residual as rotvec
        VectorXd q= x_.segment<6>(0);
        Matrix4d T; MatrixXd J;
        fk_and_jacobian(DH_UR5,q,T,J);
        Vector3d p_curr = T.block<3,1>(0,3);
        Matrix3d R_curr = T.block<3,3>(0,0);        
        
        Vector3d e_pos = cam_p_  - p_curr;
        Vector3d e_ori = rotvec_error(R_curr, cam_R_);
        VectorXd r(6); r << e_pos, e_ori; // residual z- h(x)

        // H = [J,_v; J_w] wrt, and zeros wrt qdot
        MatrixXd H = MatrixXd::Zero(6,12);
        H.block<6,6>(0,0) = J; // depends on q only

        MatrixXd Rm = MatrixXd::Zero(6,6);
        Rm.block<3,3>(0,0) = r_cam_pos_ * Matrix3d::Identity();
        Rm.block<3,3>(3,3) = r_cam_ori_ * Matrix3d::Identity();

        MatrixXd S = H * P_ * H.transpose() + Rm;
        MatrixXd K = P_ * H.transpose() * S.inverse();
        x_ += K * r;
        MatrixXd I = MatrixXd::Identity(12,12);
        P_ = (I - K*H) * P_;
    }

    // TImer Loop
    void tick(){
        double dt = (now()-last_time_).seconds();
        if(dt <= 0.0 || dt > 1.0) dt = 1.0/ rate_hz_;
        last_time_ = now();
        if(!have_enc_) return;

        //Predict
        predict(dt);
        //Update with encoders(always)
        update_encoders();
        //Optional camera fusion
        if(use_cam_pose_ && have_cam_) update_camera();

        // Publish filtered joint state
        auto js = sensor_msgs::msg::JointState();
        js.header.stamp = now();
        js.name = joint_names_;
        js.position.resize(6);
        js.velocity.resize(6);
        for(int i=0;i<6;++i){
            js.position[i] = x_(i);
            js.velocity[i] = x_(6+i);
        }
        pub_js_filtered_->publish(js);

        //Publish EE pose& twish from EKF State
        Matrix4d T; MatrixXd J;
        fk_and_jacobian(DH_UR5,x_.segment<6>(0), T, J);

        geometry_msgs::msg::PoseStamped pmsg;
        pmsg.header = js.header;
        pmsg.header.frame_id = "base_link";
        pmsg.pose.position.x = T(0,3);
        pmsg.pose.position.y = T(1,3);
        pmsg.pose.position.z = T(2,3);
        Eigen::Quaterniond Q(T.block<3,3>(0,0)); Q.normalize();
        pmsg.pose.orientation.w = Q.w();
        pmsg.pose.orientation.x = Q.x();
        pmsg.pose.orientation.y = Q.y();
        pmsg.pose.orientation.z = Q.z();
        pub_ee_pose_->publish(pmsg);


        geometry_msgs::msg::TwistStamped tmsg;
        tmsg.header = pmsg.header;
        VectorXd xdot = J * x_.segment<6>(6); // [v; w]
        tmsg.twist.linear.x  = xdot[0];
        tmsg.twist.linear.y  = xdot[1];
        tmsg.twist.linear.z  = xdot[2];
        tmsg.twist.angular.x = xdot[3];
        tmsg.twist.angular.y = xdot[4];
        tmsg.twist.angular.z = xdot[5];
        pub_ee_twist_->publish(tmsg);
    }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UR5EKFStateEstimator>());
  rclcpp::shutdown();
  return 0;
}