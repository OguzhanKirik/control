#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <string>

namespace calib{
    inline cv::Ptr<cv::aruco::Dictionary> arucoDict(const std::string& name){
        int id  = cv::aruco::DICT_4X4_50;
        if(name.rfind("DICT_",0) == 0){
        #if OPENCV_VERSION_MAJOR >= 4
            id = cv::aruco::getPredefinedDictionaryName(name);
        #endif
        }
        return cv::aruco::getPredefinedDictionary(id);
    }
    
    
    inline cv::Ptr<cv::aruco::GridBoard> makeCharucoBoard(
        int squares_x, int squares_y, float square_len_m, float marker_len_m,
        const cv::Ptr<cv::aruco::Dictionary>& dict) {
        return cv::aruco::GridBoard::create(squares_x, squares_y, square_len_m, marker_len_m, dict);
    } 
    
    inline bool calibrateIntrinsicsCharuco(
        const std::vector<cv::Mat>& all_charuco_corners,
        const std::vector<cv::Mat>& all_charuco_ids,
        const cv::Ptr<cv::aruco::GridBoard>& board,
        const cv::Size& image_size,
        cv::Mat& K, cv::Mat& D, double& rms){
            if(all_charuco_corners.size() < 8) return false;
            std::vector<cv::Mat> rvecs, tvecs;
            rms = cv::aruco::calibrateCameraAruco(
                all_charuco_corners, all_charuco_ids, all_charuco_corners.size(), board, image_size, K, D, rvecs, tvecs);
            return true;
        }

    inline bool estimateBoardPoseCamToBoard(
        const cv::Mat& bgr_or_gray, const cv::Mat& K, const cv::Mat& D,
        const cv::Ptr<cv::aruco::Dictionary>& dict, 
        const cv::Ptr<cv::aruco::GridBoard>& board,
        cv::Mat& R_cb, cv::Mat& t_cb){
            cv::Mat img = bgr_or_gray.clone();
            std::vector<std::vector<cv::Point2f>> corners; std::vector<int> ids;
            cv::aruco::detectMarkers(img,dict,corners,ids);
            if(ids.size() < 4 ) return false;
            cv::Vec3d rvec, tvec;
            int ok = cv::aruco::estimatePoseBoard(corners,ids, board,K,D,rvec,tvec);
            if(ok <= 4) return false;
            cv::Rodrigues(rvec,R_cb);
            t_cb = (cv::Mat_<double>(3,1)) << tvec[0], tvec[1], tvec[2];
            return true;
        }
    
        // ----- Small SE3 helpers -----
    inline void invertRt(const cv::Mat& R, const cv::Mat& t, cv::Mat& Ri, cv::Mat& ti) {
        Ri = R.t();
        ti = -Ri * t;
    }

    inline void buildRelativeMotions(
            const std::vector<cv::Mat>& ee_abs_R, const std::vector<cv::Mat>& ee_abs_t,
            const std::vector<cv::Mat>& tgt_abs_R, const std::vector<cv::Mat>& tgt_abs_t,
            std::vector<cv::Mat>& Rg, std::vector<cv::Mat>& tg,
            std::vector<cv::Mat>& Rt, std::vector<cv::Mat>& tt){
        
        Rg.clear(); tg.clear(); Rt.clear(); tt.clear();
        for (size_t i=0; i+1<ee_abs_R.size(); ++i) {
            const cv::Mat &R1=ee_abs_R[i], &t1=ee_abs_t[i];
            const cv::Mat &R2=ee_abs_R[i+1], &t2=ee_abs_t[i+1];

            cv::Mat R1i, t1i; invertRt(R1, t1, R1i, t1i);
            cv::Mat RA = R1i * R2;
            cv::Mat tA = R1i * (t2 - t1);

            const cv::Mat &Rb1=tgt_abs_R[i], &tb1=tgt_abs_t[i];
            const cv::Mat &Rb2=tgt_abs_R[i+1], &tb2=tgt_abs_t[i+1];
            cv::Mat Rb2i, tb2i; invertRt(Rb2, tb2, Rb2i, tb2i);
            cv::Mat RB = Rb1 * Rb2i;
            cv::Mat tB = Rb1 * tb2i + tb1;

            Rg.push_back(RA); tg.push_back(tA);
            Rt.push_back(RB); tt.push_back(tB);
        }        
    }

    inline bool handEyeSolve(
        const std::vector<cv::Mat>& Rg, const std::vector<cv::Mat>& tg,
        const std::vector<cv::Mat>& Rt, const std::vector<cv::Mat>& tt,
        cv::Mat& R_cam2gripper, cv::Mat& t_cam2gripper,
        const std::string& method = "Tsai"){
        if(Rg.size() < 6)  return false;
        cv::HandEyeCalibrationMethod algo = (method.rfind("park",0) == 0 || method=="park") ? cv::CALIB_HAND_EYE_PARK
        : cv::CALIB_HAND_EYE_TSAI;
        
        cv::calibrateHandEye(Rg, tg, Rt, tt, R_cam2gripper, t_cam2gripper, algo);
        return true;
    } 

} // namespace calib