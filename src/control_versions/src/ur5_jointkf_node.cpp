// ur5_kf_node_.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <algorithm>

using Eigen::Matrix2d; using Eigen::Vector2d;
using Eigen::MatrixXd; using Eigen::VectorXd;


struct JointKF{
    //State and Covariance
    Vector2d x = Vector2d::Zero(); // [q,qdot]
    Matrix2d P = Matrix2d::Identity();

    //Noise(tune?)
    double q_pos = 1e-6; // process noise on position random walk
    double q_vel = 1e-3; // process noise on velocity
    double r_pos = 1e-5; // measurement noise (encoder)
    double r_vel = 1e-2; // measurement nouse (raw vel);

    //whetherto use velocity measurement
    bool  use_vel_measurement = false;

    void predict(const double& dt){
        Matrix2d F; F << 1, dt, 0, 1;
        Matrix2d Q; Q << q_pos, 0, 0, q_vel;
        x = F * x; 
        P = F * P * F.transpose() + Q;
    }

    void update_pos(double zq){
        Eigen::RowVector2d H; H << 1, 0;
        double y = zq - H * x;
        double S = (H * P * H.transpose())(0,0) + r_pos;
        Eigen::Vector2d K = ( P * H.transpose()) / S;
        x += K * y;
        P = (Matrix2d::Identity() - K * H) * P;
    }

    void update_pose_vel(double zq, double zqd){
        Matrix2d H, R;
        H << 1, 0,
             0, 1;
        R << r_pos, 0,
             0, r_vel;
        Vector2d z;  z<< zq, zqd;
        Vector2d y = z - H * x;
        Matrix2d S = H * P * H.transpose() + R;
        Matrix2d K = P * H.transpose() * S.inverse();
        x += K *y;
        P = (Matrix2d::Identity() - K * H) * P;
    }
};

class UR5_JointKF : public rclcpp::Node{
    public:
        UR5_JointKF() : Node("ur5_jointkf_node"){
            declare_parameter<std::vector<std::string>>("joint_names",{
            "shoulder_pan_joint","shoulder_lift_joint","elbow_joint",
            "wrist_1_joint","wrist_2_joint","wrist_3_joint"});
            declare_parameter<double>("rate_hz",250);
            declare_parameter<bool>("use_velocity_measurement",false);
            declare_parameter<double>("q_pos",1e-6);
            declare_parameter<double>("q_vel",1e-3);
            declare_parameter<double>("r_pos",1e-5);
            declare_parameter<double>("r_vel",1e-2);

            
            get_parameter("joint_name",joint_names_);
            get_parameter("rate_hz",rate_hz_);
            bool use_vel; get_parameter("use_velocity_measurement", use_vel);
            get_parameter("q_pos", q_pos_);
            get_parameter("q_vel", q_vel_);
            get_parameter("r_pos", r_pos_);
            get_parameter("r_vel", r_vel_);

            kfs_.resize(joint_names_.size());
            for(auto &kf : kfs_){
                kf.use_vel_measurement = use_vel;
                kf.q_pos = q_pos_;
                kf.q_vel = q_vel_;
                kf.r_pos = r_pos_;
                kf.r_vel = r_vel_;
            }

            sub_ = create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states",50,
                std::bind(&UR5_JointKF::on_joint_state, this, std::placeholders::_1));
            
            pub_ = create_publisher<sensor_msgs::msg::JointState>(
                "/joint_state_filtered", 10);
            
            last_time_ = now();

            timer_ = create_wall_timer(
                std::chrono::milliseconds((int)std::round(1000.0/rate_hz_)),
                std::bind(&UR5_JointKF::tick, this));
            


        }
    private: 
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Time last_time_;
        std::vector<JointKF> kfs_;
        std::vector<std::string> joint_names_;
        double q_vel_, q_pos_, r_pos_, r_vel_;
        double rate_hz_;

        //latest raw measurement
        bool have_measurement_ = false;
        std::vector<double> z_pos_, z_vel_;


        void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg){
            //map by name for safety
            z_pos_.assign(joint_names_.size(),0.0);
            z_vel_.assign(joint_names_.size(),0.0);
            for( size_t i=0; i<joint_names_.size(); ++i){
                auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
                if(it == msg->name.end()) continue;
                size_t idx = it - msg->name.begin();
                if(idx < msg->position.size()) z_pos_[i] = msg->position[idx];
                if(idx < msg->velocity.size()) z_pos_[i] = msg->velocity[idx];
            }
            have_measurement_ = true;
        }

        void tick(){
            if(!have_measurement_) return;
            double dt = (now()- last_time_).seconds();
            if(dt <= 0.0 || dt>0.1) dt = 1.0/rate_hz_;
            last_time_ = now();

            //predict + update per jpint
            for(size_t i=0; i<kfs_.size();++i){
                auto &kf = kfs_[i];
                kf.predict(dt);
                if(kf.use_vel_measurement)kf.update_pose_vel(z_pos_[i],z_vel_[i]);
                else    kf.update_pos(z_pos_[i]);
            }

            //publish filtered joint state
            auto out =sensor_msgs::msg::JointState();
            out.header.stamp =now();
            out.name = joint_names_;
            out.position.resize(kfs_.size());
            out.velocity.resize(kfs_.size());
            out.effort.resize(0);
            for(size_t i=0; i<kfs_.size(); ++i){
                out.position[i] = kfs_[i].x[0];
                out.velocity[i] = kfs_[i].x[1];
            }

            pub_->publish(out);
        }

};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UR5_JointKF>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}