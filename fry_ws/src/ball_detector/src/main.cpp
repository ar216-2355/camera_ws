#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

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
    cv::FileStorage fs("camera_calib.yml", cv::FileStorage::READ);
    cv::Mat cameraMatrix, distCoeffs;
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();
    RCLCPP_INFO(logger_, "Camera parameters loaded.");

    double fx = cameraMatrix.at<double>(0, 0);
    double fy = cameraMatrix.at<double>(1, 1);
    double cx = cameraMatrix.at<double>(0, 2);
    double cy = cameraMatrix.at<double>(1, 2);
    std::cout << "Camera fx=" << fx << ", fy=" << fy << std::endl;

    // === カメラ設定 ===
    cv::VideoCapture cap(0);
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
        cv::cvtColor(und, hsv, cv::COLOR_BGR2HSV);
        cv::cvtColor(und, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, mask, 60, 255, cv::THRESH_BINARY_INV);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area < 300) continue;
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
            if ((hue < 10) || (hue > 160)) color = "Red";
            else if (hue < 35) color = "Yellow";
            else if (hue < 130) color = "Blue";
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
