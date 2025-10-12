#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/eigen.hpp>
#include "camera/calibration_core.hpp"

class HandEyeNode : public rclcpp::Node{
    public:
        HandEyeNode() : Node("cam_handeye_node"), 
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_){
            
            image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_color");
            camera_frame_= declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
            base_frame_  = declare_parameter<std::string>("base_frame", "base_link");
            ee_frame_    = declare_parameter<std::string>("ee_frame", "tool0");
            dict_name_   = declare_parameter<std::string>("aruco_dict", "DICT_4X4_50");
            squares_x_   = declare_parameter<int>("squares_x", 5);
            squares_y_   = declare_parameter<int>("squares_y", 7);
            square_len_  = declare_parameter<double>("square_length", 0.030);
            marker_len_  = declare_parameter<double>("marker_length", 0.020);
            min_samples_ = declare_parameter<int>("min_samples", 20);
            method_      = declare_parameter<std::string>("method", "Tsai");
            
            //Load intrinsics (assume your camera driver publish camerainfo or load zaml externally ann pass K,D here)
            //for simplciity , we read form pareters if provided
            std::vector<double> Kp = declare_parameter<std::vector<double>>("K", std::vector<double>());
            std::vector<double> Dp = declare_parameter<std::vector<double>>("D", std::vector<double>());
            if(Kp.size()==9){
                K_ = (cv::Mat_<double>(3,3) << Kp[0],Kp[1],Kp[2], Kp[3],Kp[4],Kp[5], Kp[6],Kp[7],Kp[8]);
            }
            if(!Dp.empty()){
                D_ = cv::Mat(1, (int)Dp.size(), CV_64F);
                for(int i=0;i<(int)Dp.size(); ++i) D_.at<double>(i)=Dp[i];
            }
            dict_ = calib::arucoDict(dict_name_);
            board_ = calib::makeCharucoBoard(squares_x_, squares_y_, float(square_len_), float(marker_len_), dict_);

            img_sub_ = create_subscription<sensor_msgs::msg::Image>(
                image_topic_, rclcpp::SensorDataQoS(),
                std::bind(&HandEyeNode::imageCb, this, std::placeholders::_1));
            
            srv_capture_ = create_service<std_srvs::srv::Trigger>(
            "handeye/capture", std::bind(&HandEyeNode::srvCapture, this, std::placeholders::_1, std::placeholders::_2));
            srv_solve_ = create_service<std_srvs::srv::Trigger>(
            "handeye/solve", std::bind(&HandEyeNode::srvSolve, this, std::placeholders::_1, std::placeholders::_2));
            srv_reset_ = create_service<std_srvs::srv::Trigger>(
            "handeye/reset", std::bind(&HandEyeNode::srvReset, this, std::placeholders::_1, std::placeholders::_2));

            RCLCPP_INFO(get_logger(), "HandEyeNode ready. Services: /handeye/capture, /handeye/solve, /handeye/reset");
        }

    private: 
        void imageCb(const sensor_msgs::msg::Image::SharedPtr msg){
            last_img_ = msg;
        }

        void srvCapture(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res){
            
            if (!last_img_) { res->success=false; res->message="No image yet."; return; }
            if (K_.empty() || D_.empty()) { res->success=false; res->message="Intrinsics (K,D) not set."; return; }

            // TF base->ee
            geometry_msgs::msg::TransformStamped T_be;
            try{
                T_be = tf_buffer_.lookupTransform(base_frame_, ee_frame_, last_img_->header.stamp,
                                                rclcpp::Duration::from_seconds(0.05));
            }
            catch(const std::exception& e){
                res->success=false; res->message=std::string("TF Base->ee failed: ")+e.what(); return;
            }

            // Image and board pose (cam->board)
            cv::Mat img;
            try{img = cv_bridge::toCvCopy(last_img_, "bgr8")->image;}
            catch(const std::exception& e){res->success=false;res->message= std::string("cv_bridge:")+e.what(); return;}

            cv::Mat R_cb, t_cb;
            if(!calib::estimateBoardPoseCamToBoard(img, K_, D_, dict_, board_, R_cb, t_cb)){
                res->success=false; res->message="Board pose estimation failed."; return;
            }
            //Convert to TF to cv::Mat r_be, t_bw
                // Convert TF to cv::Mat R_be, t_be
            Eigen::Quaterniond q(T_be.transform.rotation.w, T_be.transform.rotation.x,
                                T_be.transform.rotation.y, T_be.transform.rotation.z);
            q.normalize();
            Eigen::Matrix3d Re = q.toRotationMatrix();
            cv::Mat R_be(3,3,CV_64F); cv::eigen2cv(Re, R_be);
            cv::Mat t_be = (cv::Mat_<double>(3,1) << T_be.transform.translation.x,
                                                    T_be.transform.translation.y,
                                             T_be.transform.translation.z);

            ee_R_.push_back(R_be.clone()); ee_t_.push_back(t_be.clone());
            tgt_R_.push_back(R_cb.clone()); tgt_t_.push_back(t_cb.clone());
            res->success=true; res->message="Captured sample "+std::to_string(ee_R_.size())+
                                    " / target >= "+std::to_string(min_samples_);
        }

        void srvSolve(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res){
            
            if ((int)ee_R_.size() < std::max(6, min_samples_)) { res->success=false; res->message="Not enough samples."; return; }
            std::vector<cv::Mat> Rg,tg,Rt,tt;
            calib::buildRelativeMotions(ee_R_, ee_t_, tgt_R_, tgt_t_, Rg, tg, Rt, tt);

            cv::Mat R_c2g, t_c2g;
            if(!calib::handEyeSolve(Rg,tg,Rt,tt,R_c2g,t_c2g, method_)){
                res->success=false; res->message="Hand–eye failed."; return; 
            }
            // Print both directions
            cv::Mat R_g2c, t_g2c; calib::invertRt(R_c2g, t_c2g, R_g2c, t_g2c);
            auto matToQuat = [](const cv::Mat& R){
            Eigen::Matrix3d Re; cv::cv2eigen(R, Re);
            Eigen::Quaterniond q(Re); q.normalize();
            return std::array<double,4>{q.x(),q.y(),q.z(),q.w()}; // x,y,z,w
            };
            
            auto q_c2g = matToQuat(R_c2g);
            auto q_g2c = matToQuat(R_g2c);

            RCLCPP_INFO(get_logger(), "=== Hand–Eye Result ===");
            RCLCPP_INFO(get_logger(), "camera -> ee: t=[%.6f %.6f %.6f], q=[%.6f %.6f %.6f %.6f]",
            t_c2g.at<double>(0), t_c2g.at<double>(1), t_c2g.at<double>(2),
            q_c2g[0], q_c2g[1], q_c2g[2], q_c2g[3]);
            RCLCPP_INFO(get_logger(), "ee -> camera: t=[%.6f %.6f %.6f], q=[%.6f %.6f %.6f %.6f]",
            t_g2c.at<double>(0), t_g2c.at<double>(1), t_g2c.at<double>(2),
            q_g2c[0], q_g2c[1], q_g2c[2], q_g2c[3]);

            res->success=true; res->message="Solved. See logs.";
        }

        void srvReset(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
            ee_R_.clear(); ee_t_.clear(); tgt_R_.clear(); tgt_t_.clear();
            res->success=true; res->message="Reset.";
        }
        // params/state
        std::string image_topic_, camera_frame_, base_frame_, ee_frame_, dict_name_, method_;
        int squares_x_, squares_y_, min_samples_; double square_len_, marker_len_;
        cv::Mat K_, D_;

        // ros
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_capture_, srv_solve_, srv_reset_;
        tf2_ros::Buffer tf_buffer_; tf2_ros::TransformListener tf_listener_;

        // detection / samples
        cv::Ptr<cv::aruco::Dictionary> dict_;
        cv::Ptr<cv::aruco::GridBoard> board_;
        std::vector<cv::Mat> ee_R_, ee_t_, tgt_R_, tgt_t_;

        // cache
        sensor_msgs::msg::Image::SharedPtr last_img_;
}; 

int main(int argc,char** argv){
    rclcpp::init(argc,argv); 
    rclcpp::spin(std::make_shared<HandEyeNode>()); 
    rclcpp::shutdown(); 
    return 0;
}
