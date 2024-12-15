#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "iostream"
#include <QApplication>
#include <opencv2/opencv.hpp>
#include "get_pose.h"
#include "QPainter"
#include "getp_to_hand.h"
#include"world_point.h"
using namespace std;
using namespace cv;
cv::VideoCapture capture;
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //QObject::connect(&serial, &QSerialPort::readyRead, this, &MainWindow::serialPort_readyRead);
    ui->comboBox->clear();
    foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
    {
        ui->comboBox->addItem(info.portName());
    }
    capture=opencv_init();
}

MainWindow::~MainWindow()
{
    delete ui;
}

//控件函数
void MainWindow::on_Start_Button_clicked()
{
    if(calibration_flag==true)//eye-in-hand标志位为真时（对应checkBox_6勾选）
    {
        bool flag=false;
        detect_center_ptr obj0{new detect_center};
        QByteArray data;//存储传输的控制数组
        double H;
        double theta_a;//弧度
        double theta_b;//弧度
        double L;
        double Zc;
        double Xc;
        double Yc;
        int i=ui->spinBox_15->value();
        set_PWM(A[i][0],A[i][1],A[i][2],A[i][3],A[i][4],A[i][5]);//设置指定位置的pwm
        set_TIME(500,500,500,500,500,500);
        send_data();
        delay_3(2000);//到达初始位置
        theta[0]=A[i][0];theta[1]=A[i][1];theta[2]=A[i][2];theta[3]=A[i][3];theta[4]=0;
        cv::Mat image=capture_img(capture);
        do{
            image=capture_img(capture);
            delay_3(500);//等待摄像头稳定
            image=capture_img(capture);
            flag = obj0->Detector(image);
            if(flag==true)
            {
                cv::Point2f point_0=obj0->center2D;//看到的二维点
                cv::Mat T2=Forward_kinematics(theta,alpha,a,d,5);//正运动学计算当前末端姿态
                cout<<"T2"<<T2<<endl;
                cv::Mat T_temp0=T2*T_X;//得到摄像头当前姿态
                cout<<"TTT:"<<T_temp0<<endl;
                H=h+L1+L2*qCos(theta[2]*RAD2ANG)+(L3-4)*qCos((theta[2]+theta[3])*RAD2ANG)
                        +4*qCos((theta[2]+theta[3]+90)*RAD2ANG);
                theta_a=qAtan((point_0.y-240)/974.9962);//弧度
                theta_b=(180-theta[2]-theta[3])*RAD2ANG+theta_a;//弧度
                L=H/qCos(theta_b);
                Zc=qAbs(L*qCos(theta_a));//计算出Zc
                Xc=Zc*(point_0.x-320)/979.9386;
                Yc=Zc*(point_0.y-240)/974.9962;
                ui->plainTextEdit->appendPlainText("Zc:"+QString::number(Zc));
                cout<<"Xc:"<<Xc<<"Yc:"<<Yc<<"Zc:"<<Zc<<endl;
                cv::Mat T_temp=T_temp0;
                cout<<"T_temp"<<T_temp<<endl;
                double x=T_temp.at<double>(0,0)*Xc+T_temp.at<double>(0,1)*Yc+T_temp.at<double>(0,2)*Zc+T_temp.at<double>(0,3)+offset[i][0];//-18;
                double y=T_temp.at<double>(1,0)*Xc+T_temp.at<double>(1,1)*Yc+T_temp.at<double>(1,2)*Zc+T_temp.at<double>(1,3)+offset[i][1];
                ui->plainTextEdit->appendPlainText("X:"+QString::number(x)+"  Y:"+QString::number(y));
                if(ui->checkBox_6->isChecked()==true&&go_flag==true)//跟踪模式
                {

                    InverseKinematics(x,y,7);//解算期望位置的角度，只用到theta0
                    theta[0]=theta0;theta[1]=A[i][1];theta[2]=A[i][2];theta[3]=A[i][3];theta[4]=A[i][4];//重写DH参数
                    ui->plainTextEdit->appendPlainText("theta0:"+QString::number(theta[0]));
                    set_PWM(theta[0],theta[1],theta[2],theta[3],theta[4],-30);//移动到目标点
                    set_TIME(200,200,200,200,200,200);
                    send_data();
                    delay_3(800);
                }
                if(ui->checkBox_6->isChecked()==false)//抓取模式
                {
                    InverseKinematics(x,y,7);//解算期望位置的角度
                    set_PWM(theta0,theta1,theta2,theta3,0,-30);//移动到目标点
                    set_TIME(500,500,500,500,500,500);
                    send_data();
                    delay_3(1000);
                    set_PWM(theta0,theta1,theta2,theta3,0,15);//抓取
                    send_data();
                    delay_3(1000);
                    data=action[0].toUtf8();//抬升
                    serial.write(data);
                    delay_3(1000);
                }
            }
        }
        while(ui->checkBox_6->isChecked()==true&&go_flag==true);//跟踪模式则进入循环，否则退出
    }
    if(tracking_flag==true)
    {
        reset();//复位
        delay_3(500);
        bool flag=false;
        QByteArray data;//存储传输指令的变量
        detect_center_ptr obj1{new detect_center};
        data = action[1].toUtf8();//位置1控制字符串，用于防止机械臂从复位状态移动后撞击摄像头
        serial.write(data);
        delay_3(1000);
        data = action[2].toUtf8();//位置2控制字符串，用于防止机械臂从复位状态移动后撞击摄像头
        serial.write(data);
        delay_3(1000);//进入初始位置
        cv::Mat image=capture_img(capture);
        do{
            image=capture_img(capture);
            delay_3(500);
            image=capture_img(capture);//获取图像
            flag = obj1->Detector(image);//识别
            if(flag==true)
            {
                cv::Point3f point_w=w_point(obj1->center2D);//返回识别到的坐标
                if(ui->checkBox_6->isChecked()==true&&go_flag==true)//跟踪模式
                {
                    InverseKinematics(point_w.x,point_w.y,point_w.z);//解算theta0
                    set_PWM(theta0,0,50,85,0,-20);//设置跟踪的PWM
                    set_TIME(200,200,200,200,200,200);
                    send_data();
                    delay_3(300);
                    ui->plainTextEdit->appendPlainText("theta0:"+QString::number(theta0));

                }
                if(ui->checkBox_6->isChecked()==false)//抓取模式
                {
                    InverseKinematics(point_w.x,point_w.y,point_w.z);//解算关节角
                    set_PWM(theta0,theta1,theta2,theta3,0,-20);//设定指定位置的PWM
                    ui->plainTextEdit->appendPlainText("j0:"+QString::number(theta0)+"j1:"+QString::number(theta1)+
                                            "j2:"+QString::number(theta2)+"j3:"+QString::number(theta3));
                    send_data();
                    delay_3(1000);
                    set_PWM(theta0,theta1,theta2,theta3,0,20);//抓取物体
                    send_data();
                    delay_3(1000);
                    data = action[0].toUtf8();//抬升机械臂
                    serial.write(data);
                }
            }
        }while(ui->checkBox_6->isChecked()==true&&go_flag==true);
    }
}
void MainWindow::Cv_to_Qimage(cv::Mat image)
{
    cv::Mat rgb;
    QImage img;
    if(image.channels() == 3)
    {
        cvtColor(image,rgb,COLOR_RGB2BGR);
        img = QImage((const unsigned char*)(rgb.data),
        rgb.cols,rgb.rows,rgb.cols*rgb.channels(),//rgb.cols*rgb.channels()可以替换为image.step
            QImage::Format_RGB888);
    }
    else
    {
        img = QImage((const unsigned char*)(image.data),
        image.cols,image.rows,rgb.cols*image.channels(),
        QImage::Format_RGB888);
    }
    ui->IMAGE->setPixmap(QPixmap::fromImage(img).scaled(ui->IMAGE->size()));
    ui->IMAGE->resize(ui->IMAGE->pixmap()->size());

}
void MainWindow::on_Capture_button_clicked()
{
    go_flag=true;
//    if(calibration_flag==true)
//    {
//        //先低头姿态，保证看到地面视野
//        cout<<"start work!!!"<<endl;
//        //reset();
//        set_PWM(0,0,50,80,0,0);
//        set_TIME(500,500,500,500,500,500);
//        send_data();
//        delay_3(2000);
//        theta[0]=0;theta[1]=0;theta[2]=50;theta[3]=80;theta[4]=0;
//        cv::Mat T1=Forward_kinematics(theta,alpha,a,d,5);
//        cout<<"T1:"<<T1<<endl;
//        cv::Mat image=capture_img(capture);
//        image=capture_img(capture);
//        cv::Mat T_world = opencv_work(image)/10.0;
//        cout<<"T_world:"<<T_world<<endl;
//        T_X=T1.inv()*T0*T_world;
//        cout<<"T_X:"<<T_X<<endl;
//        ui->textEdit_8->setPlainText("Okay!!!");
//    }
}
void MainWindow::on_End_button_clicked()
{
    go_flag=false;
}
void MainWindow::on_openPort_Button_clicked()//打开串口处理函数
{
    if(ui->openPort_Button->text()==QString("打开串口"))
     {
        //设置串口名
        serial.setPortName(ui->comboBox->currentText());
        serial.setBaudRate(ui->comboBox_2->currentText().toInt());
        serial.setFlowControl(QSerialPort::NoFlowControl);
        if(serial.open(QIODevice::ReadWrite))
        {
            ui->comboBox->setEnabled(false);
            ui->comboBox_2->setEnabled(false);
            ui->openPort_Button->setText(QString("关闭串口"));
            ui->textEdit_8->setPlainText("串口已打开！！");
        }
        else
        {
            ui->textEdit_8->setPlainText("串口打开失败！！");
        }
    }
    else
    {
        serial.close();
        ui->comboBox->setEnabled(true);
        ui->comboBox_2->setEnabled(true);
        ui->openPort_Button->setText(QString("打开串口"));
    }
}
void MainWindow::on_pointNUM_Button_clicked()//确认规划点个数处理函数
{
    if (((send_flag_R == true  && send_flag_M == false)|| (send_flag_M == true && send_flag_R == false)) && send_flag_L == false)
    {
        button7_count++;
        if (button7_count % 2 != 0)
        { ui->pointNUM_Button->setText("取消") ; ui->textEdit_2->setPlainText("1"); point_num = ui->textEdit->toPlainText().toInt();}
        else { ui->pointNUM_Button->setText("确定"); ui->textEdit_2->setPlainText(""); ui->textEdit->setPlainText(""); P_count = 0;point_num = 0; }
    }
}
void MainWindow::on_point_check_Button_clicked()//确认此点处理函数
{
    if (ui->textEdit_3->toPlainText() == NULL || ui->textEdit_4->toPlainText() == NULL
            || ui->textEdit_5->toPlainText() == NULL || ui->textEdit_6->toPlainText() == NULL || ui->textEdit_7->toPlainText() == NULL)
    {
        ui->textEdit_8->setPlainText("当前输入参数的数量不足，请补全！！");
    }
    else
    {
        if(InverseKinematics(ui->textEdit_3->toPlainText().toDouble(),ui->textEdit_4->toPlainText().toDouble(),ui->textEdit_5->toPlainText().toDouble()))
        {
            if (((send_flag_R == false && send_flag_M == true)||(send_flag_R == true && send_flag_M == false)) && send_flag_L == false)
            {
                input_X[P_count] = ui->textEdit_3->toPlainText().toDouble();
                input_Y[P_count] = ui->textEdit_4->toPlainText().toDouble();
                input_Z[P_count] = ui->textEdit_5->toPlainText().toDouble();
                input_theta4[P_count] = ui->textEdit_6->toPlainText().toDouble();
                input_theta5[P_count] = ui->textEdit_7->toPlainText().toDouble();
                P_count++;
                ui->textEdit_3->setPlainText("");
                ui->textEdit_4->setPlainText("");
                ui->textEdit_5->setPlainText("");
                ui->textEdit_6->setPlainText("");
                ui->textEdit_7->setPlainText("");
                int count_temp = P_count + 1;
                if (P_count != point_num) ui->textEdit_2->setPlainText( QString::number(count_temp));
            }
        }
        else
        {
            ui->textEdit_8->setPlainText("此点机械臂无法到达，原因是不在工作空间中，请重新输入！");
        }

    }
}
void MainWindow::on_point_delete_Button_clicked()//取消此点处理函数
{
    ui->textEdit_3->setPlainText("");
    ui->textEdit_4->setPlainText("");
    ui->textEdit_5->setPlainText("");
    ui->textEdit_6->setPlainText("");
    ui->textEdit_7->setPlainText("");
    input_X[P_count]=0;
    input_Y[P_count]=0;
    input_Z[P_count]=0;
    input_theta4[P_count]=0;
    input_theta5[P_count]=0;
    ui->textEdit_2->setPlainText( QString::number(P_count));
    if(P_count>0)
        P_count--;
}
void MainWindow::on_build_Button_clicked()//确认规划处理函数
{
    build_flag=true;
    if (send_flag_R == true && send_flag_L == false && send_flag_M == false)
    {
        for(int i=0;i<6;i++)
            send_DATA_T[i]="T"+data_to_Fstring(ui->spinBox_13->value())+"!";
       if (P_count != point_num)
       {
           ui->textEdit_8->setPlainText("当前已经输入的点的个数少于预定的数量！请输入剩下的"+QString::number(point_num - P_count)+"个节点！");
            return;
        }
        send_DATA_P[5] = "#005P"+QString::number(1200);
        X_last=(L1*qSin(J_last[1]*RAD2ANG)+L2*qSin((J_last[1]+J_last[2])*RAD2ANG)
                +L3*qSin((J_last[1]+J_last[2]+J_last[3])*RAD2ANG))*qCos(J_last[0]*RAD2ANG);
        Y_last=(L1*qSin(J_last[1]*RAD2ANG)+L2*qSin((J_last[1]+J_last[2])*RAD2ANG)
                +L3*qSin((J_last[1]+J_last[2]+J_last[3])*RAD2ANG))*qSin(J_last[0]*RAD2ANG);
        Z_last=h+L1*qCos(J_last[1]*RAD2ANG)+L2*qCos((J_last[1]+J_last[2])*RAD2ANG)
                +L3*qCos((J_last[1]+J_last[2]+J_last[3])*RAD2ANG);
        ui->plainTextEdit->appendPlainText("X:"+QString::number(X_last)+"Y:"+QString::number(Y_last)+
                                           "Z:"+QString::number(Z_last));
        int i = 0;
        double stepx; double stepy; double stepz;
        for (; i < P_count; i++)
        {
            if(i==0)
            {
                stepx = (input_X[i] - X_last) / precision;
                stepy = (input_Y[i] - Y_last) / precision;
                stepz = (input_Z[i] - Z_last) / precision;
                for (int d = 1; d <= precision; d++)
                {
                    if(build_flag==true)
                    {
                        delay_3(ui->spinBox_13->value());
                        InverseKinematics(X_last + stepx * d, Y_last + stepy * d, Z_last + stepz * d);
                        set_PWM(theta0,theta1,theta2,theta3,input_theta4[i],-20);
                        if(d==precision)
                        {
                            set_PWM(theta0,theta1,theta2,theta3,input_theta4[i],input_theta5[i]);
                            J_last[5]=input_theta5[i];
                        }
                        ui->plainTextEdit->appendPlainText("j0:"+QString::number(theta0)+"j1:"+QString::number(theta1)+
                                                        "j2:"+QString::number(theta2)+"j3:"+QString::number(theta3)+
                                                           "j4:"+QString::number(J_last[4])+"j5:"+QString::number(J_last[5]));
                        send_data();
                        J_last[0] = theta0; J_last[1] = theta1; J_last[2] = theta2; J_last[3] = theta3;J_last[4]=input_theta4[i];

                    }
                }
            }
            else
            {
                stepx = (input_X[i] - input_X[i - 1]) / precision;
                stepy = (input_Y[i] - input_Y[i - 1]) / precision;
                stepz = (input_Z[i] - input_Z[i - 1]) / precision;

                for (int d = 1; d <= precision; d++)
                {
                    if(build_flag==true)
                    {
                        delay_3(ui->spinBox_13->value());
                        InverseKinematics(input_X[i - 1] + stepx * d, input_Y[i - 1] + stepy * d, input_Z[i - 1] + stepz * d);
                        set_PWM(theta0,theta1,theta2,theta3,input_theta4[i],15);
                        if(d==precision)
                        {
                            set_PWM(theta0,theta1,theta2,theta3,input_theta4[i],input_theta5[i]);
                            J_last[5]=input_theta5[i];
                        }
                        ui->plainTextEdit->appendPlainText("j0:"+QString::number(theta0)+"j1:"+QString::number(theta1)+
                                                        "j2:"+QString::number(theta2)+"j3:"+QString::number(theta3)+
                                                        "j4:"+QString::number(J_last[4])+"j5:"+QString::number(J_last[5]));
                        send_data();
                        J_last[0] = theta0; J_last[1] = theta1; J_last[2] = theta2; J_last[3] = theta3;J_last[4]=input_theta4[i];

                    }
                }
            }
        }
        P_count = 0;

    }
    if (send_flag_R == false && send_flag_L == false && send_flag_M == true)
    {
        if (P_count != point_num)
        {
            ui->textEdit_8->setPlainText("当前已经输入的点的个数少于预定的数量！请输入剩下的"+ QString::number(point_num - P_count)+"个节点！");
            return;
        }
        send_DATA_P[5] = "#005P"+data_to_Fstring(qCeil(1200));
        for (int i = 0; i < point_num; i++)
        {
            Joint_space_inverse(input_X[i], input_Y[i], input_Z[i], input_theta4[i], input_theta5[i]);
            for (int j = 0; j < 50; j++)
            {
                if(build_flag==true)
                {
                    if(i!=0)
                        set_PWM(data0[j],data1[j],data2[j],data3[j],data4[j],input_theta5[i-1]);//非第一点规划时末端保持过去点抓取值
                    else
                        set_PWM(data0[j],data1[j],data2[j],data3[j],data4[j],-25);//第一个点规划时保持给定的角度
                    set_TIME((50/(qAbs(data0[j+50])+1)),(50/(qAbs(data1[j+50])+1)),//利用速度量求响应时间，比例系数50，“+1”防止计算所得速度为0
                            (50/(qAbs(data2[j+50])+1)),(50/(qAbs(data3[j+50])+1)),(50/(qAbs(data4[j+50])+1)),500);
                    if(j>=48)//设置末端为目标点输入值
                    {
                        set_PWM(data0[j],data1[j],data2[j],data3[j],data4[j],input_theta5[i]);
                        set_TIME((50/(qAbs(data0[j+50])+1)),(50/(qAbs(data1[j+50])+1)),
                                (50/(qAbs(data2[j+50])+1)),(50/(qAbs(data3[j+50])+1)),(50/(qAbs(data4[j+50])+1)),500);
                    }

                    ui->plainTextEdit->appendPlainText("j0:"+QString::number(data0[j])+"j1:"+QString::number(data1[j])+
                                                    "j2:"+QString::number(data2[j])+"j3:"+QString::number(data3[j])+
                                                     "j4:"+QString::number(data4[j])+"j5:"+QString::number(data5[j]));
                    send_data();
                    delay_3(max(j));//延时
                }
            }
        }
    }
}
void MainWindow::on_unbuild_Button_clicked()//取消规划处理函数
{
    if (send_flag_L == false && (send_flag_R == true || send_flag_M == true))
    {
        ui->horizontalSlider->setValue(init_theta[0]);
        ui->horizontalSlider->setValue(init_theta[1]);
        ui->horizontalSlider->setValue(init_theta[2]);
        ui->horizontalSlider->setValue(init_theta[3]);
        ui->horizontalSlider->setValue(init_theta[4]);
        ui->horizontalSlider->setValue(init_theta[5]);
        track_view();
        ui->spinBox_7->setValue(1000); ui->spinBox_8->setValue(1000);
        ui->spinBox_9->setValue(1000); ui->spinBox_10->setValue(1000);
        ui->spinBox_11->setValue(1000); ui->spinBox_12->setValue(1000);
        numeric_data();
        send_data();
        build_flag=false;
    }
}
void MainWindow::on_checkBox_clicked()//滑条控制标志位处理
{
    if(ui->checkBox->isChecked()==true&&send_flag_M==false&&send_flag_R==false)
    {
        send_flag_L=true;
        reset();
    }
    else
    {
        send_flag_L=false;
        ui->checkBox->setChecked(false);
    }
}
void MainWindow::on_checkBox_2_clicked()//笛卡尔规划标志位处理
{
    if(ui->checkBox_2->isChecked()==true&&send_flag_M==false&&send_flag_L==false)
    {
        send_flag_R=true;
        reset();
    }
    else
    {
        send_flag_R=false;
        ui->checkBox_2->setChecked(false);
    }
}
void MainWindow::on_checkBox_3_clicked()//关节规划标志位处理
{
    if(ui->checkBox_3->isChecked()==true&&send_flag_R==false&&send_flag_L==false)
    {
        send_flag_M=true;
        reset();
    }
    else
    {
        send_flag_M=false;
        ui->checkBox_3->setChecked(false);
    }
}
void MainWindow::on_checkBox_4_clicked()
{
    if(ui->checkBox_4->isChecked()==true&&send_flag_L==false&&send_flag_M==false&&send_flag_R==false)
    {
        tracking_flag=true;
    }
    else
    {
        tracking_flag=false;
        ui->checkBox_4->setChecked(false);
    }
}
void MainWindow::on_checkBox_5_clicked()
{
    if(ui->checkBox_5->isChecked()==true&&send_flag_L==false&&send_flag_M==false&&send_flag_R==false)
    {
        calibration_flag=true;
    }
    else
    {
        calibration_flag=false;
        ui->checkBox_5->setChecked(false);
    }
}

//PWM
void MainWindow::on_horizontalSlider_sliderMoved(int position)
{
    if (send_flag_L == true && send_flag_R == false && send_flag_M == false)
    {
        track_view();
        send_data();
    }
    else {  ui->textEdit_8->setPlainText( "当前不是滑条控制模式，请勾选滑条控制模式后再进行操作！"); }
}
void MainWindow::on_horizontalSlider_2_sliderMoved(int position)
{
    if (send_flag_L == true && send_flag_R == false && send_flag_M == false)
    {
        track_view();
        send_data();
    }
    else {  ui->textEdit_8->setPlainText( "当前不是滑条控制模式，请勾选滑条控制模式后再进行操作！"); }
}
void MainWindow::on_horizontalSlider_4_sliderMoved(int position)
{
    if (send_flag_L == true && send_flag_R == false && send_flag_M == false)
    {
        track_view();
        send_data();
    }
    else {  ui->textEdit_8->setPlainText( "当前不是滑条控制模式，请勾选滑条控制模式后再进行操作！"); }
}
void MainWindow::on_horizontalSlider_5_sliderMoved(int position)
{
    if (send_flag_L == true && send_flag_R == false && send_flag_M == false)
    {
        track_view();
        send_data();
    }
    else {  ui->textEdit_8->setPlainText( "当前不是滑条控制模式，请勾选滑条控制模式后再进行操作！"); }
}
void MainWindow::on_horizontalSlider_3_sliderMoved(int position)
{
    if (send_flag_L == true && send_flag_R == false && send_flag_M == false)
    {
        track_view();
        send_data();
    }
    else {  ui->textEdit_8->setPlainText( "当前不是滑条控制模式，请勾选滑条控制模式后再进行操作！"); }
}
void MainWindow::on_horizontalSlider_6_sliderMoved(int position)
{
    if (send_flag_L == true && send_flag_R == false && send_flag_M == false)
    {
        track_view();
        send_data();
    }
    else {  ui->textEdit_8->setPlainText( "当前不是滑条控制模式，请勾选滑条控制模式后再进行操作！"); }
}
void MainWindow::on_spinBox_valueChanged(int arg1){numericUpDown_view(); send_data();}
void MainWindow::on_spinBox_2_valueChanged(int arg1){numericUpDown_view(); send_data();}
void MainWindow::on_spinBox_3_valueChanged(int arg1){numericUpDown_view(); send_data();}
void MainWindow::on_spinBox_4_valueChanged(int arg1){numericUpDown_view(); send_data();}
void MainWindow::on_spinBox_5_valueChanged(int arg1){numericUpDown_view(); send_data();}
void MainWindow::on_spinBox_6_valueChanged(int arg1){numericUpDown_view(); send_data();}
//TIME
void MainWindow::on_spinBox_7_valueChanged(int arg1){numeric_data(); send_data();}
void MainWindow::on_spinBox_8_valueChanged(int arg1){numeric_data(); send_data();}
void MainWindow::on_spinBox_9_valueChanged(int arg1){numeric_data(); send_data();}
void MainWindow::on_spinBox_10_valueChanged(int arg1){numeric_data(); send_data();}
void MainWindow::on_spinBox_11_valueChanged(int arg1){numeric_data(); send_data();}
void MainWindow::on_spinBox_12_valueChanged(int arg1){numeric_data(); send_data();}
//退出程序
void MainWindow::on_exit_Button_clicked()
{
    QApplication::exit();
}
void MainWindow::on_Help_Button_clicked()
{
    ui->plainTextEdit->clear();
    ui->plainTextEdit->appendPlainText("关于本上位机的使用:\r\n");
    ui->plainTextEdit->appendPlainText("1、串口模块用于选择要连接的串口并设置其波特率，按下‘打开串口’即可使用；");
    ui->plainTextEdit->appendPlainText("‘打开串口’按下后会变成‘关闭串口’，按下实现串口关闭功能。\r\n");
    ui->plainTextEdit->appendPlainText("2、基础功能有三种，勾选对应的功能后才能进行实现。\r\n");
    ui->plainTextEdit->appendPlainText("3、滑条独立模式:此模式下可以实时控制机械臂，通过移动各关节滑块或直接更改PWM部分的QspinBox中的值改变PWM；");
    ui->plainTextEdit->appendPlainText("更改TIME栏的QspinBox值可以改变响应时间。\r\n");
    ui->plainTextEdit->appendPlainText("4、笛卡尔空间规划模式:输入待规划点的数目点击‘确定’，依次输入各点后点击‘确认此点’将该点保存，或‘取消该点’则不对输入数据保存；");
    ui->plainTextEdit->appendPlainText("完成输入后点击‘确认规划’即可。\r\n");
    ui->plainTextEdit->appendPlainText("5、关节空间规划模式:类似笛卡尔空间规划模式的过程。\r\n");
    ui->plainTextEdit->appendPlainText("6、传输提示框:对传输的指令做显示。\r\n");
    ui->plainTextEdit->appendPlainText("7、输出框:对中间结果及最终结果做输出显示，比如规划时的各关节角度等。\r\n");
}
void MainWindow::on_pushButton_clicked()
{
    reset();
}
void MainWindow::on_spinBox_14_valueChanged(int arg1)
{
    precision=ui->spinBox_14->value();
}

//功能函数
void MainWindow::reset()
{
    ui->horizontalSlider->setValue(init_theta[0]);   ui->horizontalSlider_2->setValue(init_theta[1]);
    ui->horizontalSlider_3->setValue(init_theta[2]);   ui->horizontalSlider_4->setValue(init_theta[3]);
    ui->horizontalSlider_5->setValue(init_theta[4]);   ui->horizontalSlider_6->setValue(init_theta[5]);

    ui->spinBox->setValue(init_theta[0]);    ui->spinBox_2->setValue(init_theta[1]);
    ui->spinBox_3->setValue(init_theta[2]);    ui->spinBox_4->setValue(init_theta[3]);
    ui->spinBox_5->setValue(init_theta[4]);    ui->spinBox_6->setValue(init_theta[5]);

    ui->spinBox_7->setValue(1000);  ui->spinBox_8->setValue(1000);
    ui->spinBox_9->setValue(1000);  ui->spinBox_10->setValue(1000);
    ui->spinBox_11->setValue(1000);  ui->spinBox_12->setValue(1000);

    ui->spinBox_13->setValue(100);
    track_data();
    numeric_data();
    send_data();
    //J_last[0]=0;J_last[1]=0;J_last[2]=0;
    //J_last[3]=0;J_last[4]=0;J_last[5]=0;
}
void MainWindow::delay_3(int msec)
{
    QTime dieTime = QTime::currentTime().addMSecs(msec);
        while( QTime::currentTime() < dieTime )
            QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}
void MainWindow::send_data()
{
    QString sendbuff = "{";
    for (int i = 0; i < 6; i++)
    {
        sendbuff = sendbuff+send_DATA_P[i]+send_DATA_T[i];
    }
    sendbuff = sendbuff+"}";
    QByteArray data = sendbuff.toUtf8();
    ui->textEdit_8->setPlainText(data);
    serial.write(data);
}
double MainWindow::ANG2RAD(double N)
{
    return (N) * (180.0 / 3.1415926535898);
}
bool MainWindow::InverseKinematics(double x, double y, double z)//笛卡尔空间规划
{
    double a,b;
    double m,n,p,q;
    double j0,j1,j2,j3;//解算出来的角度，最后存放在theta变量
    double x1, y1, z1;//逆解后正解算出来的值，看是否与逆解值相等
    double A,B;
    char i='0';
    double max = 1;
    double change = 500;
    if((qAbs(y)<=0.05&&x==0)||(qAbs(x)<=0.05&&y==0))j0=0;//在输入坐标的xy有一方为0另一方很小时插值点可能出现很小的点但会带来大的角度变化
                                                            //这里直接给定j0=0；
    else j0 = qAtan2(y, x);
    a = qSqrt(qPow(x, 2) + qPow(y, 2));
    b = z-h;
    for(j1=-90;j1<=90;j1++)
    {

        j1 *= RAD2ANG;
        m=L1*qCos(j1);
        n=a-L1*qSin(j1);
        p=b-m;
        q=qSqrt(qPow(n,2)+qPow(p,2));
        if(qAbs(y)<=0.1&&qAbs(x)<=0.1&&z<=30)//特殊位置求解
        {
            double D2=qPow(L1,2)+qPow((z-h),2)-2*L1*(z-h)*qCos(j1);
            j3=qAcos((D2-qPow(L2,2)-qPow(L3,2))/(2.0*L2*L3));
            A=L2*qCos(j1)+L3*qCos(j1+j3);
            B=L2*qSin(j1)+L3*qSin(j1+j3);
            j2=qAcos(((z-h-L1*qCos(j1))*A-L1*qSin(j1)*B)/(qPow(A,2)+qPow(B,2)));
        }
        else
        {
            j3=qAcos(-(qPow(L2,2)+qPow(L3,2)-qPow(q,2))/(2.0*L2*L3));
            A=L2+L3*qCos(j3);
            B=L3*qSin(j3);
            j2=qAcos((B*n+A*p)/(qPow(A,2)+qPow(B,2)))-j1;
        }
        x1 = (L1 * qSin(j1) + L2 * qSin(j1 + j2) + L3 * qSin(j1 + j2 + j3)) * qCos(j0);
        y1 = (L1 * qSin(j1) + L2 * qSin(j1 + j2) + L3 * qSin(j1 + j2 + j3)) * qSin(j0);
        z1 = L1 * qCos(j1) + L2 * qCos(j1 + j2) + L3 * qCos(j1 + j2 + j3)+h;
        j1 = ANG2RAD(j1);
        j2 = ANG2RAD(j2);
        j3 = ANG2RAD(j3);
        if (j3 > 141.9) { j3 = 141.9; }
        if (j3 < -128.4) { j3 = -128.4; }
        if (j2 > 148.6) { j2 = 148.6; }
        if (j2 < -121.6) { j2 = -121.6; }
        j0 = ANG2RAD(j0);
        if (x1 < (x + max) && x1 > (x - max) && y1 < (y + max) && y1 > (y - max) && z1 < (z + max) && z1 > (z - max)&&limit_1(j1,j2)&&limit_3(j1,j2,j3))
        {
            if (0.3*qAbs(j0 - J_last[0]) + 0.5*qAbs(j1 - J_last[1]) + qAbs(j2 - J_last[2]) + qAbs(j3 - J_last[3]) < change)//取变化小的解
            {
                change = 0.3*qAbs(j0 - J_last[0]) + 0.5*qAbs(j1 - J_last[1]) +qAbs(j2 - J_last[2]) + qAbs(j3 - J_last[3]);//更新变化量
                X_1=x1;Y_1=y1;Z_1=z1;
                //ui->textEdit_8->setPlainText("j0:"+QString::number(j0)+ "  j1:"+QString::number(j1)+ "  j2:"+QString::number(j2)+"  j3:"+QString::number(j3)+ "  x:"+QString::number(x1)+"  y:"+QString::number(y1)+"  z:"+QString::number(z1));
                i = '1';
                theta0 = j0; theta1 = j1; theta2 = j2; theta3 = j3;
            }
        }
        else  j0=j0*RAD2ANG;
    }
    for(j1=-90;j1<=90;j1++)//找j3的另一种解，j3正负
    {
        j1 *= RAD2ANG;
        m=L1*qCos(j1);
        n=a-L1*qSin(j1);
        p=b-m;
        q=qSqrt(qPow(n,2)+qPow(p,2));
        if(qAbs(y)<=0.1&&qAbs(x)<=0.1&&z<=30)
        {
            double D2=qPow(L1,2)+qPow((z-h),2)-2*L1*(z-h)*qCos(j1);
            j3=-qAcos((D2-qPow(L2,2)-qPow(L3,2))/(2.0*L2*L3));
            A=L2*qCos(j1)+L3*qCos(j1+j3);
            B=L2*qSin(j1)+L3*qSin(j1+j3);
            j2=qAcos(((z-h-L1*qCos(j1))*A-L1*qSin(j1)*B)/(qPow(A,2)+qPow(B,2)));
        }
        else
        {
            j3=-qAcos(-(qPow(L2,2)+qPow(L3,2)-qPow(q,2))/(2.0*L2*L3));//j3取负
            A=L2+L3*qCos(j3);
            B=L3*qSin(j3);
            j2=qAcos((B*n+A*p)/(qPow(A,2)+qPow(B,2)))-j1;
        }
        x1 = (L1 * qSin(j1) + L2 * qSin(j1 + j2) + L3 * qSin(j1 + j2 + j3)) * qCos(j0);
        y1 = (L1 * qSin(j1) + L2 * qSin(j1 + j2) + L3 * qSin(j1 + j2 + j3)) * qSin(j0);
        z1 = L1 * qCos(j1) + L2 * qCos(j1 + j2) + L3 * qCos(j1 + j2 + j3);
        j1 = ANG2RAD(j1);
        j2 = ANG2RAD(j2);
        j3 = ANG2RAD(j3);
        if (j3 > 141.9) { j3 = 141.9; }
        if (j3 < -128.4) { j3 = -128.4; }
        if (j2 > 148.6) { j2 = 148.6; }
        if (j2 < -121.6) { j2 = -121.6; }
        j0 = ANG2RAD(j0);
        if (x1 < (x + max) && x1 > (x - max) && y1 < (y + max) && y1 > (y - max) && z1 < (z + max) && z1 > (z - max)&&limit_1(j1,j2)&&limit_3(j1,j2,j3))
        {
            if (0.3*qAbs(j0 - J_last[0]) + 0.5*qAbs(j1 - J_last[1]) + qAbs(j2 - J_last[2]) + qAbs(j3 - J_last[3]) < change)
            {
                change = 0.3*qAbs(j0 - J_last[0]) + 0.5*qAbs(j1 - J_last[1]) +qAbs(j2 - J_last[2]) + qAbs(j3 - J_last[3]);
                X_1=x1;Y_1=y1;Z_1=z1;
                //ui->textEdit_8->setPlainText("j0:"+QString::number(j0)+ "  j1:"+QString::number(j1)+ "  j2:"+QString::number(j2)+"  j3:"+QString::number(j3)+ "  x:"+QString::number(x1)+"  y:"+QString::number(y1)+"  z:"+QString::number(z1));
                i = '1';
                theta0 = j0; theta1 = j1; theta2 = j2; theta3 = j3;
            }
        }
        else  j0=j0*RAD2ANG;
    }
    if (i == '0') {ui->textEdit_8->setPlainText("无解");return false;}
    else
    {
        ui->plainTextEdit->appendPlainText("  x:"+QString::number(X_1)+"  y:"+QString::number(Y_1)+"  z:"+QString::number(Z_1));
        //J0_last = theta0; J1_last = theta1; J2_last = theta2; J3_last = theta3;
        return true;
    }
}
void MainWindow::Joint_space_inverse(double x, double y, double z, double theta4, double theta5)
{
    InverseKinematics(x, y, z);
    Three_interpolation(J_last[0], theta0, 0);
    Three_interpolation(J_last[1], theta1, 1);
    Three_interpolation(J_last[2], theta2, 2);
    Three_interpolation(J_last[3], theta3, 3);
    Three_interpolation(J_last[4], theta4, 4);
    J_last[0] = theta0; J_last[1] = theta1; J_last[2] = theta2;
    J_last[3] = theta3; J_last[4] = theta4; J_last[5] = theta5;
}
void MainWindow::Three_interpolation(double theta_last,double theta_now,int channel)//三项插值函数
{
    double V_start = 0;
    double V_end = 0;
    int T_start = 0;
    double T_end = 50.0;
    double a0, a1, a2, a3;//三次插值系数
    double theta[100];
    a0 = theta_last;
    a1 = V_start;
    a2 = (3 / qPow(T_end, 2)) * (theta_now - theta_last) - (1 / T_end) * (2 * V_start + V_end);
    a3 = (2 / qPow(T_end, 3)) * (theta_last - theta_now) + (1 / qPow(T_end,2)) * (V_start + V_end);//计算三次多项式系数
    for (int i = T_start; i < T_end; i++)
    {
        theta[i] = a0 + a1 * i + a2 * qPow(i, 2) + a3 * qPow(i, 3);//位置，前50位
        theta[i+50] = a1 + 2*a2 * i + 3*a3 * qPow(i, 2);//速度，后50位
    }
    switch(channel)
    {
        case 0:copy(data0,theta,100);break;
        case 1:copy(data1,theta,100);break;
        case 2:copy(data2,theta,100);break;
        case 3:copy(data3,theta,100);break;
        case 4:copy(data4,theta,100);break;
    }
}
void MainWindow::copy(double a[],double b[], int size)
{
    for(int i=0;i<size;i++)
        a[i]=b[i];
}
void MainWindow::track_view()
{
   ui->textEdit_8->setPlainText("当前为关节滑条独立控制模式");
   ui->spinBox->setValue(ui->horizontalSlider->value());
   ui->spinBox_2->setValue(ui->horizontalSlider_2->value());
   ui->spinBox_3->setValue(ui->horizontalSlider_3->value());
   ui->spinBox_4->setValue(ui->horizontalSlider_4->value());
   ui->spinBox_5->setValue(ui->horizontalSlider_5->value());
   ui->spinBox_6->setValue(ui->horizontalSlider_6->value());
   track_data();

}
void MainWindow::track_data()
{
    send_DATA_P[0] = "#000P" + data_to_Fstring(ui->horizontalSlider->value());
    send_DATA_P[1] = "#001P" + data_to_Fstring(ui->horizontalSlider_2->value());
    send_DATA_P[2] = "#002P" + data_to_Fstring(ui->horizontalSlider_3->value());
    send_DATA_P[3] = "#003P" + data_to_Fstring(ui->horizontalSlider_4->value());
    send_DATA_P[4] = "#004P" + data_to_Fstring(ui->horizontalSlider_5->value());
    send_DATA_P[5] = "#005P" + data_to_Fstring(ui->horizontalSlider_6->value());
    J_last[0] = (ui->horizontalSlider->value()-init_theta[0])* 13.5 /100.0;
    J_last[1] = (ui->horizontalSlider_2->value() - init_theta[1]) * 13.5 / 100.0;
    J_last[2] = (ui->horizontalSlider_3->value() - init_theta[2]) * 13.5 / 100.0;
    J_last[3] = (ui->horizontalSlider_4->value() - init_theta[3]) * 13.5 / 100.0;
    J_last[4] = (ui->horizontalSlider_5->value() - init_theta[4]) * 13.5 / 100.0;
    J_last[5] = (ui->horizontalSlider_6->value() - init_theta[5]) * 13.5 / 100.0;
    for(int i=0;i<4;i++)theta[i]=J_last[i];
    theta[4]=0;
    cv::Mat C=Forward_kinematics(theta,alpha,a,d,5);
    ui->plainTextEdit->appendPlainText("X:"+QString::number(C.at<double>(0,3))+"Y:"+QString::number(C.at<double>(1,3))+"Z:"+QString::number(C.at<double>(2,3)));
}
void MainWindow::numeric_data()
{
    send_DATA_T[0] = "T" + data_to_Fstring(ui->spinBox_7->value()) + "!";
    send_DATA_T[1] = "T" + data_to_Fstring(ui->spinBox_8->value()) + "!";
    send_DATA_T[2] = "T" + data_to_Fstring(ui->spinBox_9->value()) + "!";
    send_DATA_T[3] = "T" + data_to_Fstring(ui->spinBox_10->value()) + "!";
    send_DATA_T[4] = "T" + data_to_Fstring(ui->spinBox_11->value()) + "!";
    send_DATA_T[5] = "T" + data_to_Fstring(ui->spinBox_12->value()) + "!";
}
QString MainWindow::data_to_Fstring(int m)
{
    if(m>=1000)return QString::number(m);
    else return "0"+QString::number(m);
}
void MainWindow::numericUpDown_view()
{
    ui->textEdit_8->setPlainText("当前为关节滑条独立控制模式");
    ui->horizontalSlider->setValue(ui->spinBox->value());
    ui->horizontalSlider_2->setValue(ui->spinBox_2->value());
    ui->horizontalSlider_3->setValue(ui->spinBox_3->value());
    ui->horizontalSlider_4->setValue(ui->spinBox_4->value());
    ui->horizontalSlider_5->setValue(ui->spinBox_5->value());
    ui->horizontalSlider_6->setValue(ui->spinBox_6->value());
    track_data();
}
cv::Mat MainWindow::Forward_kinematics(double theta[],double alpha[],double a[],double d[],int size)
{
    for(int i=0;i<5;i++)
    {
        theta[i]=theta[i]*RAD2ANG;
        if(i==1)
            theta[i]=(theta[i]-90)*RAD2ANG;
    }
    Mat Forward_matrix=(Mat_<double>(4, 4) <<1.0,0,0,0,
                                         0,1.0,0,0,
                                         0,0,1.0,0,
                                         0,0,0,1.0);//单位阵
    for(int i=0;i<size;i++)
    {
        Forward_matrix*=(Mat_<double>(4, 4) <<
                              qCos(theta[i]),               -qSin(theta[i]),             0,              a[i],
                         qSin(theta[i])*qCos(alpha[i]),qCos(theta[i])*qCos(alpha[i]),-qSin(alpha[i]),-qSin(alpha[i])*d[i],
                         qSin(theta[i])*qSin(alpha[i]),qCos(theta[i])*qSin(alpha[i]),qCos(alpha[i]), qCos(alpha[i])*d[i],
                                       0,                          0,                    0,                 1.0);
    }
    cout<<Forward_matrix<<endl;
    return Forward_matrix;
}
bool MainWindow::limit_1(double theta1,double theta2)
{
    theta1=theta1*RAD2ANG;
    theta2=theta2*RAD2ANG;
    double H=L1*qCos(theta1)-L2*qCos(theta1+theta2);
    if(H+2<=h)return true;
    else return false;
}
bool MainWindow::limit_2(double theta1,double theta2,double theta3)
{
    theta1=theta1*RAD2ANG;
    theta2=theta2*RAD2ANG;
    theta3=theta3*RAD2ANG;
    double XB=L1*qSin(theta1)+L2*qSin(theta1+theta2)+(L3+4)*qSin(theta1+theta2+theta3);
    double YB=L1*qCos(theta1)+L2*qCos(theta1+theta2)+(L3+4)*qCos(theta1+theta2+theta3);
    if(XB<=10&&YB<=h+1)return false;
    else return true;
}
bool MainWindow::limit_3(double theta1,double theta2,double theta3)
{
    theta1=theta1*RAD2ANG;
    theta2=theta2*RAD2ANG;
    theta3=theta3*RAD2ANG;
    double XB=L1*qSin(theta1)+L2*qSin(theta1+theta2)+(L3+4)*qSin(theta1+theta2+theta3);
    double YB=L1*qCos(theta1)+L2*qCos(theta1+theta2)+(L3+4)*qCos(theta1+theta2+theta3);
    if(theta1!=0)
    {
        double k=qTan((Pi-theta1));
        if(theta1>0&&theta2>0)
        {
            if(k*XB-YB>3)return true;
            else return false;
        }
        if(theta1<0&&theta2>0)
        {
            if(YB-k*XB>3)return true;
            else return false;
        }
        if(theta1>0&&theta2<0)
        {
            if(YB-k*XB>3)return true;
            else return false;
        }
        if(theta1<0&&theta2<0)
        {
            if(k*XB-YB>3)return true;
            else return false;
        }
    }
    else
    {
        if(qAbs(XB)<3&&YB<L1)return false;
        else return true;
    }
}
int MainWindow::max(int point)
{
    double min=20000;
    if(data0[50+point]<min)min=data0[50+point];
    if(data1[50+point]<min)min=data1[50+point];
    if(data2[50+point]<min)min=data2[50+point];
    if(data3[50+point]<min)min=data3[50+point];
    if(data4[50+point]<min)min=data4[50+point];
    if(data5[50+point]<min)min=data5[50+point];
    return LIMIT(qCeil(100 / qAbs(min+1)),500,20);
}
int MainWindow::LIMIT(int mun,int max,int min)
{
    if(mun>max)mun=max;
    if(mun<min)mun=min;
    return mun;
}
void MainWindow::set_PWM(double theta0,double theta1,double theta2,double theta3,double theta4,double theta5)
{
    send_DATA_P[0] = "#000P"+data_to_Fstring((qCeil((init_theta[0] + theta0 * 100 / 13.5))));
    send_DATA_P[1] = "#001P"+data_to_Fstring((qCeil((init_theta[1] - theta1 * 100 / 13.5))));
    send_DATA_P[2] = "#002P"+data_to_Fstring((qCeil((init_theta[2] + theta2 * 100 / 13.5))));
    send_DATA_P[3] = "#003P"+data_to_Fstring((qCeil((init_theta[3] + theta3 * 100 / 13.5))));
    send_DATA_P[4] = "#004P"+data_to_Fstring((qCeil((init_theta[4] + theta4 * 100 / 13.5))));
    send_DATA_P[5] = "#005P"+data_to_Fstring((qCeil((init_theta[5] + theta5 * 100 / 13.5))));

}
void MainWindow::set_TIME(double time0,double time1,double time2,double time3,double time4,double time5)
{
    send_DATA_T[0] = "T"+data_to_Fstring(LIMIT((qCeil(time0)), 500, 20))+ "!";
    send_DATA_T[1] = "T"+data_to_Fstring(LIMIT((qCeil(time1)), 500, 20))+ "!";
    send_DATA_T[2] = "T"+data_to_Fstring(LIMIT((qCeil(time2)), 500, 20))+ "!";
    send_DATA_T[3] = "T"+data_to_Fstring(LIMIT((qCeil(time3)), 500, 20))+ "!";
    send_DATA_T[4] = "T"+data_to_Fstring(LIMIT((qCeil(time4)), 500, 20))+ "!";
    send_DATA_T[5] = "T"+data_to_Fstring(LIMIT((qCeil(time5)), 500, 20))+ "!";
}


