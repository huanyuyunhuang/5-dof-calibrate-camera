#include "getp_to_hand.h"
#include "QTime"
#include "QCoreApplication"
using namespace std;
using namespace cv;

#define max_num 255
/*
#define threshold_low 60000
#define threshold_high 350000
*/
//方块
#define threshold_low 5000
#define threshold_high 100000
#define lenth_width 3
/*
 //方格
#define threshold_low 1000
#define threshold_high 50000
*/
bool detect_center:: Detector(cv::Mat &image)
{
    Mat src, dst1, dst2, dst;
    bool flag = false;
    //RGB图像转HSV图像
    cvtColor(image, src, COLOR_BGR2HSV);
    //图片二值化
    inRange(src, Scalar(0, 0, 0), Scalar(150, 150, 150), dst); //black
    //inRange(src, Scalar(0, 0, 0), Scalar(200, 200, 200), dst); //black

    //中值滤波
    medianBlur(dst, dst, 3);
    //边沿检测
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(dst, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<RotatedRect> minEllipse;
    vector<Point2f> center;
    vector<cv::Point2f> pts;
    cv::Point2f centerPoint;
    minEllipse.clear();
    pts.clear();


    for (int i = 0; i < contours.size(); i++)
    {
        RotatedRect minRect = minAreaRect(contours[i]);
        //对所检测出来的边沿的面积进行限制，其面积大小应在（hreshold_low，threshold_high）
        if (minRect.size.area() > threshold_low && minRect.size.area() < threshold_high)
        {
            Point2f ttt[4];
            minRect.points(ttt);
            for (int a = 0; a < 4; a++)
            {
                //https://blog.csdn.net/weixin_28949185/article/details/84839167
                line(image, ttt[a % 4], ttt[(a + 1) % 4], Scalar(255, 0, 0), 1, 8);
            }


            cv::Point2f left_up, left_down, right_up, right_down;

 //方格
            Point2f t[4];
            minRect.points(t);

            left_up=t[0];
            left_down=t[1];

           right_down=t[2];

            right_up=t[3];



/*
 //方块
            int left_up_min = 10000, right_up_max = -10000, left_down_min = 10000, right_down_max = -10000;
            for (auto pt : contours[i])
            {
                if (left_up_min > pt.x + pt.y)
                {
                    left_up_min = pt.x + pt.y;
                    left_up = pt;
                }
                if (right_up_max < pt.y - pt.x)
                {
                    right_up_max = pt.y - pt.x;
                    right_up = pt;
                }
                if (left_down_min > pt.y - pt.x)
                {
                    left_down_min = pt.y - pt.x;
                    left_down = pt;
                }
                if (right_down_max < pt.x + pt.y)
                {
                    right_down_max = pt.x + pt.y;
                    right_down = pt;
                }
            }
            */

            //矩形长、宽的平均长度
            float L1 = (sqrt((left_up.x - right_up.x) * (left_up.x - right_up.x) + (left_up.y - right_up.y) * (left_up.y - right_up.y)) +
                        sqrt((left_down.x - right_down.x) * (left_down.x - right_down.x) + (left_down.y - right_down.y) * (left_down.y - right_down.y))) /
                       2;
            float L2 = (sqrt((left_down.x - left_up.x) * (left_down.x - left_up.x) + (left_down.y - left_up.y) * (left_down.y - left_up.y)) +
                        sqrt((right_down.x - right_up.x) * (right_down.x - right_up.x) + (right_down.y - right_up.y) * (right_down.y - right_up.y))) /
                       2;
            //对所检测出来的边沿（长方形）的长、宽比进行限制，其长宽比应在（1/2，2）
            if (max(L1 / L2, L2 / L1) < lenth_width)
            {
                centerPoint = (left_up + left_down + right_up + right_down) / 4;
                if (center.size() != 0)
                {
                    bool judge_continue = false;
                    for (Point2f c : center)
                    {
                        float distance = (c.x - centerPoint.x) * (c.x - centerPoint.x) +
                                         (c.y - centerPoint.y) * (c.y - centerPoint.y);
                        if (distance < 300)
                        {
                            judge_continue = true;
                            break;
                        }
                    }
                }

                pts.push_back(left_up);
                pts.push_back(left_down);
                pts.push_back(right_down);
                pts.push_back(right_up);
                cout<<"find 4 points"<<endl;

                center.push_back(centerPoint);

                 line(image, left_up, left_down, Scalar(0, 0, 255), 1, 8); //画矩形四条边
                 line(image, left_down, right_down, Scalar(0, 0, 255), 1, 8);
                 line(image, right_down, right_up, Scalar(0, 0, 255), 1, 8);
                 line(image, right_up, left_up, Scalar(0, 0, 255), 1, 8);
            }
        }
    }
     cout <<"center.size="<< center.size() << endl;
     cout <<"pts.size="<< pts.size() << endl;


    if (pts.size() == 4)
    {
        vector<Point2f> point;

        //左上 左下 右下 右上（检测物体的4个顶点）
        corners2D.push_back(pts[0]);
        corners2D.push_back(pts[1]);
        corners2D.push_back(pts[2]);
        corners2D.push_back(pts[3]);



        //检测物体的中心点
        center2D = centerPoint;
        cout<<center2D<<endl;
        flag = true;
        cout<<"flag="<<1<<endl;
    }
//https://blog.csdn.net/jgj123321/article/details/95057025
    //https://www.cnblogs.com/mangoroom/p/10999125.html
    imshow("hsv", dst);

    imshow("dealing", image);
    return flag;

}
