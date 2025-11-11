#include <opencv2/opencv.hpp>

int main(){
    cv::Mat img;

    cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);
    while(true){
        cap.read(img);

        cv::imshow("VideoCapture",img);

        unsigned char key = cv::waitKey(2);
        if(key == '\x1b')break;

    }

}