#ifndef GETP_TO_HAND_H
#define GETP_TO_HAND_H

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include <memory>

//定义一个目标检测及中心获取的类
class detect_center
{
public:
    std::vector<cv::Point2f> corners2D;
    cv::Point2f center2D;

    detect_center()
    {
        corners2D.clear();
    }

    bool Detector(cv::Mat &image);

};
typedef std::shared_ptr<detect_center> detect_center_ptr;


#endif // GETP_TO_HAND_H
