#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "camera/calibration_core.hpp"

class IntrinsicsNode : public rclcpp::Node{
    public:
        IntrinsicsNode() : Node("intrinsic_node"){
            image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_color");
            camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
            output_yaml_  = declare_parameter<std::string>("output_yaml", "camera_intrinsics.yaml");
            publish_camera_info_ = declare_parameter<bool>("publish_camera_info", true);

            dict_name_ = declare_parameter<std::string>("aruco_dict", "DICT_4X4_50");
            squares_x_ = declare_parameter<int>("squares_x", 5);
            squares_y_ = declare_parameter<int>("squares_y", 7);
            square_len_= declare_parameter<double>("square_length", 0.030);
            marker_len_= declare_parameter<double>("marker_length", 0.020);
            min_samples_= declare_parameter<int>("min_samples", 30);
            min_corner_frac_ = declare_parameter<double>("min_corner_frac", 0.6);
                
            dict_ = calib::arucoDict(dict_name_);
            board_ = calib::makeCharucoBoard(squares_x_, squares_y_, float(square_len_), float(marker_len_), dict_);
            det_params_ = cv::aruco::DetectorParameters::create();

            img_sub_ = create_subscription<sensor_msgs::msg::Image>(
                image_topic_, rclcpp::SensorDataQoS(), 
                std::bind(&IntrinsicsNode::imageCb, this, std::placeholders::_1));
            
            caminfo_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
                    "camera_info",10);
            
            srv_capture_ = create_service<std_srvs::srv::Trigger>(
                "intrinsics/capture", std::bind(&IntrinsicsNode::srvCapture, this,
                    std::placeholders::_1, std::placeholders::_2));
            srv_solve_ = create_service<std_srvs::srv::Trigger>(
                "intrinsics/solve", std::bind(&IntrinsicsNode::srvSolve, this,
                std::placeholders::_1, std::placeholders::_2));
            
            srv_reset_ = create_service<std_srvs::srv::Trigger>(
                "intrinsics/reset", std::bind(&IntrinsicsNode::srvReset, this,
            std::placeholders::_1,std::placeholders::_2));

            RCLCPP_INFO(get_logger(), "IntrinsicNode ready. /intrinsics/capture, /intrinsics/solve, /intrinsics/reset");
    
    }
    private:
        void imageCb(const sensor_msgs::msg::Image::SharedPtr msg){
            last_img_ = msg;
            if(calibrated_ && publish_camera_info_){
                sensor_msgs::msg::CameraInfo ci;
                ci.header = msg->header;
                ci.header.frame_id = camera_frame_;
                ci.width = img_size_.width; ci.height = img_size_.height;
                ci.distortion_model = "plumb_bob";

                ci.k = { K_.at<double>(0,0), K_.at<double>(0,1), K_.at<double>(0,2),
                        K_.at<double>(1,0), K_.at<double>(1,1), K_.at<double>(1,2),
                        K_.at<double>(2,0), K_.at<double>(2,1), K_.at<double>(2,2) };            
                    
                
                ci.d.resize(size_t(D_.total()));
                for(size_t i=0;i<ci.d.size();++i) ci.d[i] = D_.at<double>(int(i));
                ci.r = {1,0,0, 0,1,0, 0,0,1};
                std::array<double,12> P{}; //K[0]
                P[0]=K_.at<double>(0,0); P[1]=K_.at<double>(0,1); P[2]=K_.at<double>(0,2);
                P[4]=K_.at<double>(1,0); P[5]=K_.at<double>(1,1); P[6]=K_.at<double>(1,2);
                P[8]=K_.at<double>(2,0); P[9]=K_.at<double>(2,1); P[10]=K_.at<double>(2,2);    
                ci.p = P;
                caminfo_pub_->publish(ci);
            }
        }

        void srvCapture(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
            std::shared_ptr<std_srvs::srv::Trigger::Response> res){
            
            if(!last_img_){res->success=false; res->message="No image yet."; return;}
            cv::Mat img;
            try{img = cv_bridge::toCvCopy(last_img_, "bgr8")->image;}
            catch(const std::exception& e) {res->success=false;res->message=std::string("cv_bridge: ")+e.what(); return; }
                
            std::vector<std::vector<cv::Point2f>> corners;
            std::vector<int> ids;
            cv::aruco::detectMarkers(img, dict_, corners, ids, det_params_);
            if(ids.size() < 4){
                res->success=false; res->message="No/low ArUco markers detected."; return;
            }
            img_size_ = {img.cols, img.rows};

            if(ids.size() < size_t(min_corner_frac_ * squares_x_ * squares_y_)){
                res->success=false; res->message="not enough markers visible"; return;
            }
        
            // Store detected corners and IDs for later calibration
            // For simplicity, we'll convert the vector<vector<Point2f>> to a Mat
            std::vector<cv::Point2f> all_corners_flat;
            for(const auto& marker_corners : corners) {
                all_corners_flat.insert(all_corners_flat.end(), marker_corners.begin(), marker_corners.end());
            }
            
            all_corners_.push_back(cv::Mat(all_corners_flat).reshape(2, -1));
            all_ids_.push_back(cv::Mat(ids));
            res->success=true;
            res->message = "Captured sample " + std::to_string(all_ids_.size()) +
            " / target >= " + std::to_string(min_samples_);           
        }

        void srvSolve(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> res){

            if((int)all_ids_.size()< std::max(8, min_samples_)){
                res->success=false; res->message="Not enough sammples.";return;
            }
            double rms=0.0;
            if (!calib::calibrateIntrinsicsCharuco(all_corners_, all_ids_, board_, img_size_, K_, D_, rms)) {
            res->success=false; res->message="Calibration failed."; return;
            }
            calibrated_= true;
            saveYaml();
            res->success=true; res->message="Intrinsics RMS="+std::to_string(rms) + ", saved to" + output_yaml_;
            RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
        }

        void srvReset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> res){

            all_corners_.clear(); all_ids_.clear(); K_.release(); D_.release(); calibrated_=false;
            res->success=true; res->message="Reset.";
        }

        void saveYaml(){
            cv::FileStorage fs(output_yaml_, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
            fs << "image_width"  << img_size_.width;
            fs << "image_height" << img_size_.height;
            fs << "camera_name"  << camera_frame_;
            fs << "camera_matrix" << K_;
            fs << "distortion_model" << "plumb_bob";
            fs << "distortion_coefficients" << D_;
            fs << "rectification_matrix" << cv::Mat::eye(3,3,CV_64F);    
            cv::Mat P = cv::Mat::zeros(3,4,CV_64F); K_.copyTo(P(cv::Rect(0,0,3,3)));
            fs << "projection_matrix" << P ; 
            fs.release();
        }

        //params
        std::string image_topic_,camera_frame_, output_yaml_, dict_name_;
        bool publish_camera_info_{true};
        int squares_x_, squares_y_, min_samples_;
        double square_len_, marker_len_, min_corner_frac_;

        //state
        sensor_msgs::msg::Image::SharedPtr last_img_;
        cv::Size img_size_;
        bool calibrated_{false};
        std::vector<cv::Mat> all_corners_, all_ids_;
        cv::Mat K_, D_;

        //ROS
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_pub_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_capture_, srv_solve_,srv_reset_;

        //aruco
        cv::Ptr<cv::aruco::Dictionary> dict_;
        cv::Ptr<cv::aruco::GridBoard> board_;
        cv::Ptr<cv::aruco::DetectorParameters> det_params_;

    };

    int main(int argc, char** argv){
        rclcpp::init(argc,argv);
        rclcpp::spin(std::make_shared<IntrinsicsNode>());
        rclcpp::shutdown();
        return 0;
    }