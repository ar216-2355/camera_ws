#include <opencv2/opencv.hpp>
#include <vector>

int main(){
    cv::FileStorage fs("camera_calib.yml", cv::FileStorage::READ);
    cv::Mat cameraMatrix, distCoeffs;
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    cv::VideoCapture cap(0);

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);
    cv::Mat frame, und, map1, map2;

    cv::Size imageSize(640, 480); // 実際のキャプチャサイズに合わせる
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                                cameraMatrix, imageSize, CV_16SC2, map1, map2);

    while(true){
        cap >> frame;
        if(frame.empty()) break;

        cv::remap(frame, und, map1, map2, cv::INTER_LINEAR);

        auto hsv = cv::Mat();
        cv::cvtColor(und, hsv, cv::COLOR_BGR2HSV);

        //AKA色の範囲
        cv::Scalar lower_red1 = cv::Scalar(0, 100, 10);
        cv::Scalar upper_red1 = cv::Scalar(10, 255, 255);
        cv::Scalar lower_red2 = cv::Scalar(160, 100, 10);
        cv::Scalar upper_red2 = cv::Scalar(179, 255, 255);

        cv::Mat mask1, mask2, mask;
        cv::inRange(hsv, lower_red1, upper_red1, mask1);
        cv::inRange(hsv, lower_red2, upper_red2, mask2);
        cv::bitwise_or(mask1, mask2, mask);

        //ノイズ除去
        // auto kernel = 
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1,-1), 2);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1,-1), 2);

        //輪郭検出
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        constexpr uint32_t area_min = 500;
        constexpr uint32_t area_max = 5000;

        //円形度閾値
        constexpr double circularity_threshold = 0.7;

        for(const auto& contour : contours){
            double area = cv::contourArea(contour);
            if(area < area_min || area > area_max) continue;

            double perimeter = cv::arcLength(contour, true);
            double circularity = 4 * CV_PI * area / (perimeter * perimeter);
            if(circularity < circularity_threshold) continue;

            cv::Moments mu = cv::moments(contour);
            cv::Point2f center(mu.m10/mu.m00, mu.m01/mu.m00);
            float radius;
            cv::minEnclosingCircle(contour, center, radius);

            cv::circle(und, center, static_cast<int>(radius), cv::Scalar(0, 255, 0), 2);
            cv::circle(und, center, 2, cv::Scalar(0, 0, 255), -1);
        }

        cv::imshow("undistorted", und);

        if(cv::waitKey(1) == 'q') break;
    }
}


