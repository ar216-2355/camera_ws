#include <opencv2/opencv.hpp>

int main(){
    cv::FileStorage fs("camera_calib.yml", cv::FileStorage::READ);
    cv::Mat cameraMatrix, distCoeffs;
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    cv::VideoCapture cap(0);
    cv::Mat frame, und, map1, map2;

    cv::Size imageSize(640, 480); // 実際のキャプチャサイズに合わせる
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                                cameraMatrix, imageSize, CV_16SC2, map1, map2);

    while(true){
        cap >> frame;
        if(frame.empty()) break;

        cv::remap(frame, und, map1, map2, cv::INTER_LINEAR);
        cv::imshow("undistorted", und);

        if(cv::waitKey(1) == 'q') break;
    }
}


