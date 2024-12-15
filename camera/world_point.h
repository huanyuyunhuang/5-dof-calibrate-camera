#ifndef WORLD_POINT_H
#define WORLD_POINT_H

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

#define ku 1
#define kv 1

#define offsetx 400
#define offsety 0
#define worldz  5


cv::Point3f w_point(cv::Point2f p);

#endif // WORLD_POINT_H
