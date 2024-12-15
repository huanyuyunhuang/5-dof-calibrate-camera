#include"world_point.h"

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

#include<iostream>
using namespace std;
using namespace cv;


cv::Mat inner_transform_matrix = (Mat_<double>(4, 4) <<979.9386,0,654.4046,0,
                            0, 974.9962, 292.7627,0,
                            0, 0, 1,0,
                            0,0,0,1 );

cv::Mat external_transform_matrix = (Mat_<double>(4, 4) <<0,-1,0,0,
                            -1,0,0,300,
                            0, 0, -1,380,
                             0,0,0,1);

cv::Point3f w_point(cv::Point2f p2)
{
    /*
    cv::Point3f p3;
   //Vec<float,3> p21,p31;
    cv::Mat p21= (Mat_<double>(4, 1) <<p2.x,p2.y,1,1);
    cv::Mat p31 = (Mat_<double>(4, 1) <<0,0,0,0);

    p31=external_transform_matrix.inv()*inner_transform_matrix.inv()*p21;
    cout<<"p31: "<<p31<<endl;

    p3.x=p31.at<double>(0,0);
    p3.y=p31.at<double>(1,0);
    p3.z=worldz;
      */
     cv::Point3f p3;
    double u0,v0;
    u0=320;
    v0=240;
    double ax=970.9,ay=974.9;
    double zc=380;

    double xc,yc;
    double u,v;

    u=p2.x;
    v=p2.y;

    xc=(u-u0)*zc/ax;
    yc=(v-v0)*zc/ay;

    p3.x=(300-yc)/10.0;
    p3.y=-xc/10.0;
    p3.z=worldz;
    if(u>380)p3.y=p3.y-3;
    if(u<260)p3.y=p3.y+5;
    //if(u<180)p3.y=p3.y+7;
    if(v>280)p3.x=p3.x-3;
    if(v<200)p3.x=p3.x;
    cout<<p3<<endl;
    return p3;
}
