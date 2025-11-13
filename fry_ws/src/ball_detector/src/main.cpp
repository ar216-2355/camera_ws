#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
// #include <ament_index_cpp/get_package_share_directory.hpp>
// #include <filesystem>

using namespace std;

struct HomographyMapper {
    cv::Mat H, Hinv;
    bool initialized = false;

    void setPoints(const std::vector<cv::Point2f>& imgPts,
                   const std::vector<cv::Point2f>& worldPts) {
        H = cv::findHomography(imgPts, worldPts);
        Hinv = H.inv();
        initialized = true;
    }

    cv::Point2f imageToWorld(const cv::Point2f& p) const {
        if (!initialized) return {0, 0};
        cv::Mat pt = (cv::Mat_<double>(3, 1) << p.x, p.y, 1);
        cv::Mat w = H * pt;
        return cv::Point2f(w.at<double>(0) / w.at<double>(2),
                           w.at<double>(1) / w.at<double>(2));
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::Logger logger_ = rclcpp::get_logger("ball_detector");
    RCLCPP_INFO(logger_, "Ball detector node started.");
    auto node = rclcpp::Node::make_shared("ball_publisher");
    auto pub = node->create_publisher<geometry_msgs::msg::PointStamped>("/ball_position", 10);
    RCLCPP_INFO(logger_, "Publisher created.");

    // === カメラパラメータ読み込み ===

    cv::FileStorage fs("/home/crs3/camera_ws/camera_ws/fry_ws/src/ball_detector/camera_calib.yml", cv::FileStorage::READ);//仕事PC

    // cv::FileStorage fs("/home/mihiro/camera_ws/collab_ws/camera_ws/fry_ws/src/ball_detector/camera_calib.yml", cv::FileStorage::READ);//自宅PC

    // std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("ball_detector");
    // std::string calib_path = pkg_share_dir + "/camera_calib.yml";

    // cv::FileStorage fs(calib_path, cv::FileStorage::READ);
    RCLCPP_INFO(logger_, "yml file loaded.");

    cv::Mat cameraMatrix, distCoeffs;
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();
    RCLCPP_INFO(logger_, "Camera parameters loaded.");

    cameraMatrix.convertTo(cameraMatrix, CV_64F);
    distCoeffs.convertTo(distCoeffs, CV_64F);

    RCLCPP_INFO(logger_, "cameraMatrix size: %dx%d", cameraMatrix.rows, cameraMatrix.cols);

    double fx = cameraMatrix.at<double>(0, 0);
    double fy = cameraMatrix.at<double>(1, 1);
    double cx = cameraMatrix.at<double>(0, 2);
    double cy = cameraMatrix.at<double>(1, 2);
    RCLCPP_INFO(logger_, "Camera fx=%.2f, fy=%.2f", fx, fy);

    // === カメラ設定 ===

    cv::VideoCapture cap("/dev/video0", cv::CAP_V4L2);//仕事PC

    // cv::VideoCapture cap("http://192.168.1.9:8080/video");//自宅PC

    if (!cap.isOpened()) {
        RCLCPP_ERROR(logger_, "Failed to open video stream!");
        return 0;
    }   
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);

    // === 歪み補正 ===
    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                                cameraMatrix, cv::Size(640, 480), CV_16SC2, map1, map2);

    RCLCPP_INFO(logger_, "Camera initialized.");

    // === ホモグラフィ設定 ===
    HomographyMapper mapper;
    std::vector<cv::Point2f> imgPts = {
        {0, 0}, {640, 0}, {640, 480}, {0, 480}};
    // 対応するロボット座標[m]（任意に変更可）
    std::vector<cv::Point2f> worldPts = {
        {-0.3, 0.3}, {0.3, 0.3}, {0.3, -0.3}, {-0.3, -0.3}};
    mapper.setPoints(imgPts, worldPts);

    RCLCPP_INFO(logger_, "Homography mapper initialized.");

    // === パラメータ ===
    const double ball_radius_m = 0.15;  // 15 cm
    double camera_height_m = 0.40;      // 40 cm（変更可能）

    cv::Mat frame, und, hsv, gray, mask;

    RCLCPP_INFO(logger_, "Starting main loop.");

    while (rclcpp::ok()) {
        cap >> frame;
        if (frame.empty()) break;

        cv::remap(frame, und, map1, map2, cv::INTER_LINEAR);
        // === HSV変換 ===
        cv::cvtColor(und, hsv, cv::COLOR_BGR2HSV);
        // cv::cvtColor(und, gray, cv::COLOR_BGR2GRAY);
        // cv::threshold(gray, mask, 60, 255, cv::THRESH_BINARY_INV);
        // // cv::imshow("gray", gray);
        // cv::imshow("mask", mask);
        

// --- GUI初期化（1回だけ） ---
static bool trackbar_initialized = false;
static int h_low_r = 0, h_high_r = 17, s_low_r = 80, s_high_r = 255, v_low_r = 80, v_high_r = 255;
static int h_low_r2 = 160, h_high_r2 = 179; // 赤の2つ目領域
static int h_low_b = 85, h_high_b = 145, s_low_b = 80, s_high_b = 255, v_low_b = 80, v_high_b = 255;
static int h_low_y = 17, h_high_y = 35, s_low_y = 80, s_high_y = 255, v_low_y = 80, v_high_y = 255;

if (!trackbar_initialized) {
    cv::namedWindow("mask", cv::WINDOW_NORMAL);
    cv::resizeWindow("mask", 480, 360);

    // 赤1
    cv::createTrackbar("R1_H_low", "mask", &h_low_r, 179);
    cv::createTrackbar("R1_H_high", "mask", &h_high_r, 179);
    cv::createTrackbar("R_S_low", "mask", &s_low_r, 255);
    cv::createTrackbar("R_S_high", "mask", &s_high_r, 255);
    cv::createTrackbar("R_V_low", "mask", &v_low_r, 255);
    cv::createTrackbar("R_V_high", "mask", &v_high_r, 255);

    // 赤2
    cv::createTrackbar("R2_H_low", "mask", &h_low_r2, 179);
    cv::createTrackbar("R2_H_high", "mask", &h_high_r2, 179);

    // 青
    cv::createTrackbar("B_H_low", "mask", &h_low_b, 179);
    cv::createTrackbar("B_H_high", "mask", &h_high_b, 179);
    cv::createTrackbar("B_S_low", "mask", &s_low_b, 255);
    cv::createTrackbar("B_S_high", "mask", &s_high_b, 255);
    cv::createTrackbar("B_V_low", "mask", &v_low_b, 255);
    cv::createTrackbar("B_V_high", "mask", &v_high_b, 255);

    // 黄
    cv::createTrackbar("Y_H_low", "mask", &h_low_y, 179);
    cv::createTrackbar("Y_H_high", "mask", &h_high_y, 179);
    cv::createTrackbar("Y_S_low", "mask", &s_low_y, 255);
    cv::createTrackbar("Y_S_high", "mask", &s_high_y, 255);
    cv::createTrackbar("Y_V_low", "mask", &v_low_y, 255);
    cv::createTrackbar("Y_V_high", "mask", &v_high_y, 255);

    trackbar_initialized = true;
}

// --- HSV範囲に基づくマスク作成 ---
cv::Mat mask_r1, mask_r2, mask_r, mask_b, mask_y, mask;

cv::inRange(hsv, cv::Scalar(h_low_r, s_low_r, v_low_r), cv::Scalar(h_high_r, s_high_r, v_high_r), mask_r1);
cv::inRange(hsv, cv::Scalar(h_low_r2, s_low_r, v_low_r), cv::Scalar(h_high_r2, s_high_r, v_high_r), mask_r2);
cv::bitwise_or(mask_r1, mask_r2, mask_r);

cv::inRange(hsv, cv::Scalar(h_low_b, s_low_b, v_low_b), cv::Scalar(h_high_b, s_high_b, v_high_b), mask_b);
cv::inRange(hsv, cv::Scalar(h_low_y, s_low_y, v_low_y), cv::Scalar(h_high_y, s_high_y, v_high_y), mask_y);

// --- 色ごとのマスク統合 ---
cv::bitwise_or(mask_r, mask_b, mask);
cv::bitwise_or(mask, mask_y, mask);

// --- ノイズ除去 ---
cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1,-1), 2);
cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1,-1), 2);

// --- 確認用ウィンドウ ---
cv::imshow("mask", mask);


        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < 300 || 640 * 480 / 2 < area) continue;
            double perimeter = cv::arcLength(contour, true);
            double circularity = 4 * CV_PI * area / (perimeter * perimeter);
            if (circularity < 0.7) continue;

            cv::Moments mu = cv::moments(contour);
            cv::Point2f center(mu.m10 / mu.m00, mu.m01 / mu.m00);
            float radius_px;
            cv::minEnclosingCircle(contour, center, radius_px);

            // === 平均色で色判定 ===
            cv::Mat mask_contour = cv::Mat::zeros(mask.size(), CV_8UC1);
            cv::drawContours(mask_contour, std::vector<std::vector<cv::Point>>{contour}, -1, 255, -1);
            cv::Scalar mean_color = cv::mean(hsv, mask_contour);
            double hue = mean_color[0];

            std::string color;
            if ((hue < 17) || (hue > 160)) color = "Red";
            else if (hue < 35)  color = "Yellow";
            else if (hue < 145 && hue > 85)
             color = "Blue";
            else continue;  // 他の色は無視

            // === ホモグラフィによる座標変換 ===
            cv::Point2f xy = mapper.imageToWorld(center);

            // === 高さZの推定 ===
            double Z = (fx * ball_radius_m) / radius_px; // 単純なスケール推定
            Z = std::max(0.0, Z - camera_height_m);      // カメラ高さとの差分として補正

            // === 結果表示 ===
            cv::circle(und, center, (int)radius_px, cv::Scalar(0, 255, 0), 2);
            cv::putText(und, color, center + cv::Point2f(5, -5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);

            std::cout << color << " ball: (x=" << xy.x << ", y=" << xy.y << ", z=" << Z << ")\n";

            // === ROS2送信 ===
            geometry_msgs::msg::PointStamped msg;
            msg.header.stamp = node->now();
            msg.header.frame_id = "robot_base";
            msg.point.x = xy.x;
            msg.point.y = xy.y;
            msg.point.z = Z;
            pub->publish(msg);
        }

        cv::imshow("undistorted", und);
        if (cv::waitKey(1) == 'q') break;
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
