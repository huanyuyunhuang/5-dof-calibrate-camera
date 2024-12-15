#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QCamera>
#include <QList>
#include <QCameraViewfinder>
#include <QCameraImageCapture>
#include <QFileDialog>
#include <QCameraInfo>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QString>
#include <QTextEdit>
#include <QtMath>
#include <QCheckBox>
#include <QPushButton>
#include <QSpinBox>
#include <QElapsedTimer>
#include <QIODevice>
#include <QObject>
#include <QEventLoop>
#include <QTime>
#include <opencv2/opencv.hpp>
using namespace cv;
namespace Ui {
class MainWindow;
}
class QCamera;
class QCameraViewfinder;
class QCameraImageCapture;
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    QSerialPort serial;
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
private slots:
    //void captureImage();
    //void displayImage(int , QImage image);
    void on_Start_Button_clicked();//打开摄像头功能函数
    void on_End_button_clicked();//图像处理关闭，摄像头关闭
    void on_openPort_Button_clicked();//打开串口处理函数
    void on_pointNUM_Button_clicked();//确认规划点个数处理函数
    void on_point_check_Button_clicked();//确认此点处理函数
    void on_point_delete_Button_clicked();//取消此点处理函数
    void on_build_Button_clicked();//确认规划处理函数
    void on_unbuild_Button_clicked();//取消规划处理函数
    void on_checkBox_clicked();//滑条控制标志位处理
    void on_checkBox_2_clicked();//笛卡尔规划标志位处理
    void on_checkBox_3_clicked();//关节规划标志位处理
    void on_checkBox_4_clicked();
    void on_checkBox_5_clicked();
    void delay_3(int msec);
    int max(int point);
    int LIMIT(int mun,int max,int min);
    void Cv_to_Qimage(cv::Mat image);
    //解算相关函数：
    void reset();
    void send_data();
    double ANG2RAD(double N);
    bool InverseKinematics(double x, double y, double z);
    bool limit_1(double theta1,double theta2);
    bool limit_2(double theta1,double theta2,double theta3);
    bool limit_3(double theta1,double theta2,double theta3);
    //void inverseKinematics(double x, double y, double z);//笛卡尔空间规划
    void Joint_space_inverse(double x, double y, double z, double theta4, double theta5);//关节空间规划
    void Three_interpolation(double theta_last,double theta_now, int channel);//三项插值函数
    void track_view();
    void track_data();
    void numeric_data();
    void numericUpDown_view();
    void copy(double a[],double b[], int size);
    QString data_to_Fstring(int m);
    cv::Mat Forward_kinematics(double theta[],double alpha[],double a[],double b[],int size);
    void set_PWM(double theta0,double theta1,double theta2,double theta3,double theta4,double theta5);
    void set_TIME(double time0,double time1,double time2,double time3,double time4,double time5);

    void on_horizontalSlider_sliderMoved(int position);
    void on_horizontalSlider_2_sliderMoved(int position);
    void on_horizontalSlider_4_sliderMoved(int position);
    void on_horizontalSlider_5_sliderMoved(int position);
    void on_horizontalSlider_3_sliderMoved(int position);
    void on_horizontalSlider_6_sliderMoved(int position);

    void on_spinBox_valueChanged(int arg1);
    void on_spinBox_2_valueChanged(int arg1);
    void on_spinBox_3_valueChanged(int arg1);
    void on_spinBox_4_valueChanged(int arg1);
    void on_spinBox_5_valueChanged(int arg1);
    void on_spinBox_6_valueChanged(int arg1);

    //TIME
    void on_spinBox_7_valueChanged(int arg1);
    void on_spinBox_8_valueChanged(int arg1);
    void on_spinBox_9_valueChanged(int arg1);
    void on_spinBox_10_valueChanged(int arg1);
    void on_spinBox_11_valueChanged(int arg1);
    void on_spinBox_12_valueChanged(int arg1);

    void on_exit_Button_clicked();
    void on_Help_Button_clicked();
    void on_pushButton_clicked();

    void on_spinBox_14_valueChanged(int arg1);
    void on_Capture_button_clicked();


private:
    Ui::MainWindow *ui;

    double L1 = 10;//10
    double L2 = 9.6;//9.6
    double L3 = 16;//11
    double h = 8;
    //1410\1440\1400\1450\1430\(1150-1740)
    double init_theta[6]={1410,1440,1430,1440,1430,1445};
    double theta0 = 0;//笛卡尔逆运动求解得解
    double theta1 = 0;
    double theta2 = 0;
    double theta3 = 0;

    bool send_flag_L = false;
    bool send_flag_R = false;
    bool send_flag_M = false;

    bool calibration_flag=false;
    bool tracking_flag=false;
    bool go_flag=true;

    QString send_DATA_P[6];//PWM数据
    QString send_DATA_T[6]={"T1000!","T1000!","T1000!","T1000!","T1000!","T1000!"};//TIME数据

    double X;//逆运动学变量
    double Y;
    double Z;

    double X_CV;
    double Y_CV;
    double Z_CV;

    double X_1;//规划时解算出的各点坐标
    double Y_1;
    double Z_1;

    double X_last;
    double Y_last;
    double Z_last;

    int precision=50;
    bool build_flag=true;
    double Pi=3.1415926535898;
    double RAD2ANG = 3.1415926535898 / 180.0;
    int button7_count = 0;
    int point_num;//规划中间点的个数
    int P_count = 0;//规划相关计数位
    double input_X[10];//记录输入各点的xyz坐标
    double input_Y[10];
    double input_Z[10];
    double build_X[10];//插值节点
    double build_Y[10];
    double build_Z[10];
    double PI = 3.141592;
    double J_last[6] = {0,0,0,0,0,0};
    double theta[5]={0,0,0,0,0};
    double alpha[5]={0,-Pi/2.0,0,0,0};
    double a[5]={0,0,L1,L2,L3};
    double d[5]={h,0,0,0,0};

    double data0[100];
    double data1[100];
    double data2[100];
    double data3[100];
    double data4[100];
    double data5[100];
    double input_theta4[20];
    double input_theta5[20];
    //1410,1440,1430,1440,1430,1445
    QString action[9]=
    {
        "{#000P1410T1000!#001P1500T1000!#002P1500T1000!#003P1500T1000!#004P1430T1000!#005P1700T1000!}",//抬升状态
        "{#000P1410T1000!#001P1600T1000!#002P1750T1000!#003P1750T1000!#004P1430T1000!#005P1445T1000!}",
        "{#000P1410T1600!#001P1600T1000!#002P1850T1000!#003P1850T1000!#004P1430T1000!#005P1100T1000!}",

        "{#000P1410T0500!#001P1440T0500!#002P1800T0500!#003P1700T0500!#004P1430T0500!#005P1445T0500!}",//0°,49.95,35.1
        "{#000P1743T0500!#001P1440T0500!#002P1800T0500!#003P1700T0500!#004P1430T0500!#005P1445T0500!}",//45°
        "{#000P2076T0500!#001P1000T0500!#002P1800T0500!#003P1700T0500!#004P1430T0500!#005P1445T0500!}",//90°

        "{#000P1076T0500!#001P1440T0500!#002P1800T0500!#003P1700T0500!#004P1430T0500!#005P1445T0500!}",//-45°
        "{#000P0743T0500!#001P1440T0500!#002P1800T0500!#003P1700T0500!#004P1430T0500!#005P1445T0500!}",//-90°
    };

    double A[5][6]={{0,0,50,85,0,-30},  //eye-in-hand搜索位置数组
                    {-45,0,50,85,0,-30},
                    {-90,0,50,85,0,-30},
                    {45,0,50,85,0,-30},
                    {90,0,50,85,0,-30}};
    double offset[5][2]={{-18,-2},
                         {-14,12},
                         {0,20},
                         {-12,-14},
                         {2,-19}
                        };
    cv::Mat T_X=(Mat_<double>(4, 4) <<0,0,1,-4.5,
                                      0,1,0,3,
                                      -1,0,0,0,
                                      0,0,0,1);
    cv::Mat T0=(Mat_<double>(4, 4) <<-1,0,0,20,
                                     0,-1,0,0,
                                     0,0,1,0,
                                     0,0,0,1);
    cv::Mat T_in=(Mat_<double>(4, 4) <<97.99386,0,65.64046,0,
                                     0,97.49962,29.27627,0,
                                     0,0,1,0,
                                     0,0,0,1);
    QCamera *camera;
    QCameraViewfinder *viewfinder;
    QCameraImageCapture *imageCapture;
};

#endif // MAINWINDOW_H

