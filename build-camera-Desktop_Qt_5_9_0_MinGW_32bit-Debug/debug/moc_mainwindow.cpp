/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../camera/mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[94];
    char stringdata0[1409];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 23), // "on_Start_Button_clicked"
QT_MOC_LITERAL(2, 35, 0), // ""
QT_MOC_LITERAL(3, 36, 21), // "on_End_button_clicked"
QT_MOC_LITERAL(4, 58, 26), // "on_openPort_Button_clicked"
QT_MOC_LITERAL(5, 85, 26), // "on_pointNUM_Button_clicked"
QT_MOC_LITERAL(6, 112, 29), // "on_point_check_Button_clicked"
QT_MOC_LITERAL(7, 142, 30), // "on_point_delete_Button_clicked"
QT_MOC_LITERAL(8, 173, 23), // "on_build_Button_clicked"
QT_MOC_LITERAL(9, 197, 25), // "on_unbuild_Button_clicked"
QT_MOC_LITERAL(10, 223, 19), // "on_checkBox_clicked"
QT_MOC_LITERAL(11, 243, 21), // "on_checkBox_2_clicked"
QT_MOC_LITERAL(12, 265, 21), // "on_checkBox_3_clicked"
QT_MOC_LITERAL(13, 287, 21), // "on_checkBox_4_clicked"
QT_MOC_LITERAL(14, 309, 21), // "on_checkBox_5_clicked"
QT_MOC_LITERAL(15, 331, 7), // "delay_3"
QT_MOC_LITERAL(16, 339, 4), // "msec"
QT_MOC_LITERAL(17, 344, 3), // "max"
QT_MOC_LITERAL(18, 348, 5), // "point"
QT_MOC_LITERAL(19, 354, 5), // "LIMIT"
QT_MOC_LITERAL(20, 360, 3), // "mun"
QT_MOC_LITERAL(21, 364, 3), // "min"
QT_MOC_LITERAL(22, 368, 12), // "Cv_to_Qimage"
QT_MOC_LITERAL(23, 381, 7), // "cv::Mat"
QT_MOC_LITERAL(24, 389, 5), // "image"
QT_MOC_LITERAL(25, 395, 5), // "reset"
QT_MOC_LITERAL(26, 401, 9), // "send_data"
QT_MOC_LITERAL(27, 411, 7), // "ANG2RAD"
QT_MOC_LITERAL(28, 419, 1), // "N"
QT_MOC_LITERAL(29, 421, 17), // "InverseKinematics"
QT_MOC_LITERAL(30, 439, 1), // "x"
QT_MOC_LITERAL(31, 441, 1), // "y"
QT_MOC_LITERAL(32, 443, 1), // "z"
QT_MOC_LITERAL(33, 445, 7), // "limit_1"
QT_MOC_LITERAL(34, 453, 6), // "theta1"
QT_MOC_LITERAL(35, 460, 6), // "theta2"
QT_MOC_LITERAL(36, 467, 7), // "limit_2"
QT_MOC_LITERAL(37, 475, 6), // "theta3"
QT_MOC_LITERAL(38, 482, 7), // "limit_3"
QT_MOC_LITERAL(39, 490, 19), // "Joint_space_inverse"
QT_MOC_LITERAL(40, 510, 6), // "theta4"
QT_MOC_LITERAL(41, 517, 6), // "theta5"
QT_MOC_LITERAL(42, 524, 19), // "Three_interpolation"
QT_MOC_LITERAL(43, 544, 10), // "theta_last"
QT_MOC_LITERAL(44, 555, 9), // "theta_now"
QT_MOC_LITERAL(45, 565, 7), // "channel"
QT_MOC_LITERAL(46, 573, 10), // "track_view"
QT_MOC_LITERAL(47, 584, 10), // "track_data"
QT_MOC_LITERAL(48, 595, 12), // "numeric_data"
QT_MOC_LITERAL(49, 608, 18), // "numericUpDown_view"
QT_MOC_LITERAL(50, 627, 4), // "copy"
QT_MOC_LITERAL(51, 632, 8), // "double[]"
QT_MOC_LITERAL(52, 641, 1), // "a"
QT_MOC_LITERAL(53, 643, 1), // "b"
QT_MOC_LITERAL(54, 645, 4), // "size"
QT_MOC_LITERAL(55, 650, 15), // "data_to_Fstring"
QT_MOC_LITERAL(56, 666, 1), // "m"
QT_MOC_LITERAL(57, 668, 18), // "Forward_kinematics"
QT_MOC_LITERAL(58, 687, 5), // "theta"
QT_MOC_LITERAL(59, 693, 5), // "alpha"
QT_MOC_LITERAL(60, 699, 7), // "set_PWM"
QT_MOC_LITERAL(61, 707, 6), // "theta0"
QT_MOC_LITERAL(62, 714, 8), // "set_TIME"
QT_MOC_LITERAL(63, 723, 5), // "time0"
QT_MOC_LITERAL(64, 729, 5), // "time1"
QT_MOC_LITERAL(65, 735, 5), // "time2"
QT_MOC_LITERAL(66, 741, 5), // "time3"
QT_MOC_LITERAL(67, 747, 5), // "time4"
QT_MOC_LITERAL(68, 753, 5), // "time5"
QT_MOC_LITERAL(69, 759, 31), // "on_horizontalSlider_sliderMoved"
QT_MOC_LITERAL(70, 791, 8), // "position"
QT_MOC_LITERAL(71, 800, 33), // "on_horizontalSlider_2_sliderM..."
QT_MOC_LITERAL(72, 834, 33), // "on_horizontalSlider_4_sliderM..."
QT_MOC_LITERAL(73, 868, 33), // "on_horizontalSlider_5_sliderM..."
QT_MOC_LITERAL(74, 902, 33), // "on_horizontalSlider_3_sliderM..."
QT_MOC_LITERAL(75, 936, 33), // "on_horizontalSlider_6_sliderM..."
QT_MOC_LITERAL(76, 970, 23), // "on_spinBox_valueChanged"
QT_MOC_LITERAL(77, 994, 4), // "arg1"
QT_MOC_LITERAL(78, 999, 25), // "on_spinBox_2_valueChanged"
QT_MOC_LITERAL(79, 1025, 25), // "on_spinBox_3_valueChanged"
QT_MOC_LITERAL(80, 1051, 25), // "on_spinBox_4_valueChanged"
QT_MOC_LITERAL(81, 1077, 25), // "on_spinBox_5_valueChanged"
QT_MOC_LITERAL(82, 1103, 25), // "on_spinBox_6_valueChanged"
QT_MOC_LITERAL(83, 1129, 25), // "on_spinBox_7_valueChanged"
QT_MOC_LITERAL(84, 1155, 25), // "on_spinBox_8_valueChanged"
QT_MOC_LITERAL(85, 1181, 25), // "on_spinBox_9_valueChanged"
QT_MOC_LITERAL(86, 1207, 26), // "on_spinBox_10_valueChanged"
QT_MOC_LITERAL(87, 1234, 26), // "on_spinBox_11_valueChanged"
QT_MOC_LITERAL(88, 1261, 26), // "on_spinBox_12_valueChanged"
QT_MOC_LITERAL(89, 1288, 22), // "on_exit_Button_clicked"
QT_MOC_LITERAL(90, 1311, 22), // "on_Help_Button_clicked"
QT_MOC_LITERAL(91, 1334, 21), // "on_pushButton_clicked"
QT_MOC_LITERAL(92, 1356, 26), // "on_spinBox_14_valueChanged"
QT_MOC_LITERAL(93, 1383, 25) // "on_Capture_button_clicked"

    },
    "MainWindow\0on_Start_Button_clicked\0\0"
    "on_End_button_clicked\0on_openPort_Button_clicked\0"
    "on_pointNUM_Button_clicked\0"
    "on_point_check_Button_clicked\0"
    "on_point_delete_Button_clicked\0"
    "on_build_Button_clicked\0"
    "on_unbuild_Button_clicked\0on_checkBox_clicked\0"
    "on_checkBox_2_clicked\0on_checkBox_3_clicked\0"
    "on_checkBox_4_clicked\0on_checkBox_5_clicked\0"
    "delay_3\0msec\0max\0point\0LIMIT\0mun\0min\0"
    "Cv_to_Qimage\0cv::Mat\0image\0reset\0"
    "send_data\0ANG2RAD\0N\0InverseKinematics\0"
    "x\0y\0z\0limit_1\0theta1\0theta2\0limit_2\0"
    "theta3\0limit_3\0Joint_space_inverse\0"
    "theta4\0theta5\0Three_interpolation\0"
    "theta_last\0theta_now\0channel\0track_view\0"
    "track_data\0numeric_data\0numericUpDown_view\0"
    "copy\0double[]\0a\0b\0size\0data_to_Fstring\0"
    "m\0Forward_kinematics\0theta\0alpha\0"
    "set_PWM\0theta0\0set_TIME\0time0\0time1\0"
    "time2\0time3\0time4\0time5\0"
    "on_horizontalSlider_sliderMoved\0"
    "position\0on_horizontalSlider_2_sliderMoved\0"
    "on_horizontalSlider_4_sliderMoved\0"
    "on_horizontalSlider_5_sliderMoved\0"
    "on_horizontalSlider_3_sliderMoved\0"
    "on_horizontalSlider_6_sliderMoved\0"
    "on_spinBox_valueChanged\0arg1\0"
    "on_spinBox_2_valueChanged\0"
    "on_spinBox_3_valueChanged\0"
    "on_spinBox_4_valueChanged\0"
    "on_spinBox_5_valueChanged\0"
    "on_spinBox_6_valueChanged\0"
    "on_spinBox_7_valueChanged\0"
    "on_spinBox_8_valueChanged\0"
    "on_spinBox_9_valueChanged\0"
    "on_spinBox_10_valueChanged\0"
    "on_spinBox_11_valueChanged\0"
    "on_spinBox_12_valueChanged\0"
    "on_exit_Button_clicked\0on_Help_Button_clicked\0"
    "on_pushButton_clicked\0on_spinBox_14_valueChanged\0"
    "on_Capture_button_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      58,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,  304,    2, 0x08 /* Private */,
       3,    0,  305,    2, 0x08 /* Private */,
       4,    0,  306,    2, 0x08 /* Private */,
       5,    0,  307,    2, 0x08 /* Private */,
       6,    0,  308,    2, 0x08 /* Private */,
       7,    0,  309,    2, 0x08 /* Private */,
       8,    0,  310,    2, 0x08 /* Private */,
       9,    0,  311,    2, 0x08 /* Private */,
      10,    0,  312,    2, 0x08 /* Private */,
      11,    0,  313,    2, 0x08 /* Private */,
      12,    0,  314,    2, 0x08 /* Private */,
      13,    0,  315,    2, 0x08 /* Private */,
      14,    0,  316,    2, 0x08 /* Private */,
      15,    1,  317,    2, 0x08 /* Private */,
      17,    1,  320,    2, 0x08 /* Private */,
      19,    3,  323,    2, 0x08 /* Private */,
      22,    1,  330,    2, 0x08 /* Private */,
      25,    0,  333,    2, 0x08 /* Private */,
      26,    0,  334,    2, 0x08 /* Private */,
      27,    1,  335,    2, 0x08 /* Private */,
      29,    3,  338,    2, 0x08 /* Private */,
      33,    2,  345,    2, 0x08 /* Private */,
      36,    3,  350,    2, 0x08 /* Private */,
      38,    3,  357,    2, 0x08 /* Private */,
      39,    5,  364,    2, 0x08 /* Private */,
      42,    3,  375,    2, 0x08 /* Private */,
      46,    0,  382,    2, 0x08 /* Private */,
      47,    0,  383,    2, 0x08 /* Private */,
      48,    0,  384,    2, 0x08 /* Private */,
      49,    0,  385,    2, 0x08 /* Private */,
      50,    3,  386,    2, 0x08 /* Private */,
      55,    1,  393,    2, 0x08 /* Private */,
      57,    5,  396,    2, 0x08 /* Private */,
      60,    6,  407,    2, 0x08 /* Private */,
      62,    6,  420,    2, 0x08 /* Private */,
      69,    1,  433,    2, 0x08 /* Private */,
      71,    1,  436,    2, 0x08 /* Private */,
      72,    1,  439,    2, 0x08 /* Private */,
      73,    1,  442,    2, 0x08 /* Private */,
      74,    1,  445,    2, 0x08 /* Private */,
      75,    1,  448,    2, 0x08 /* Private */,
      76,    1,  451,    2, 0x08 /* Private */,
      78,    1,  454,    2, 0x08 /* Private */,
      79,    1,  457,    2, 0x08 /* Private */,
      80,    1,  460,    2, 0x08 /* Private */,
      81,    1,  463,    2, 0x08 /* Private */,
      82,    1,  466,    2, 0x08 /* Private */,
      83,    1,  469,    2, 0x08 /* Private */,
      84,    1,  472,    2, 0x08 /* Private */,
      85,    1,  475,    2, 0x08 /* Private */,
      86,    1,  478,    2, 0x08 /* Private */,
      87,    1,  481,    2, 0x08 /* Private */,
      88,    1,  484,    2, 0x08 /* Private */,
      89,    0,  487,    2, 0x08 /* Private */,
      90,    0,  488,    2, 0x08 /* Private */,
      91,    0,  489,    2, 0x08 /* Private */,
      92,    1,  490,    2, 0x08 /* Private */,
      93,    0,  493,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   16,
    QMetaType::Int, QMetaType::Int,   18,
    QMetaType::Int, QMetaType::Int, QMetaType::Int, QMetaType::Int,   20,   17,   21,
    QMetaType::Void, 0x80000000 | 23,   24,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Double, QMetaType::Double,   28,
    QMetaType::Bool, QMetaType::Double, QMetaType::Double, QMetaType::Double,   30,   31,   32,
    QMetaType::Bool, QMetaType::Double, QMetaType::Double,   34,   35,
    QMetaType::Bool, QMetaType::Double, QMetaType::Double, QMetaType::Double,   34,   35,   37,
    QMetaType::Bool, QMetaType::Double, QMetaType::Double, QMetaType::Double,   34,   35,   37,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,   30,   31,   32,   40,   41,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Int,   43,   44,   45,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 51, 0x80000000 | 51, QMetaType::Int,   52,   53,   54,
    QMetaType::QString, QMetaType::Int,   56,
    0x80000000 | 23, 0x80000000 | 51, 0x80000000 | 51, 0x80000000 | 51, 0x80000000 | 51, QMetaType::Int,   58,   59,   52,   53,   54,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,   61,   34,   35,   37,   40,   41,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,   63,   64,   65,   66,   67,   68,
    QMetaType::Void, QMetaType::Int,   70,
    QMetaType::Void, QMetaType::Int,   70,
    QMetaType::Void, QMetaType::Int,   70,
    QMetaType::Void, QMetaType::Int,   70,
    QMetaType::Void, QMetaType::Int,   70,
    QMetaType::Void, QMetaType::Int,   70,
    QMetaType::Void, QMetaType::Int,   77,
    QMetaType::Void, QMetaType::Int,   77,
    QMetaType::Void, QMetaType::Int,   77,
    QMetaType::Void, QMetaType::Int,   77,
    QMetaType::Void, QMetaType::Int,   77,
    QMetaType::Void, QMetaType::Int,   77,
    QMetaType::Void, QMetaType::Int,   77,
    QMetaType::Void, QMetaType::Int,   77,
    QMetaType::Void, QMetaType::Int,   77,
    QMetaType::Void, QMetaType::Int,   77,
    QMetaType::Void, QMetaType::Int,   77,
    QMetaType::Void, QMetaType::Int,   77,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   77,
    QMetaType::Void,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->on_Start_Button_clicked(); break;
        case 1: _t->on_End_button_clicked(); break;
        case 2: _t->on_openPort_Button_clicked(); break;
        case 3: _t->on_pointNUM_Button_clicked(); break;
        case 4: _t->on_point_check_Button_clicked(); break;
        case 5: _t->on_point_delete_Button_clicked(); break;
        case 6: _t->on_build_Button_clicked(); break;
        case 7: _t->on_unbuild_Button_clicked(); break;
        case 8: _t->on_checkBox_clicked(); break;
        case 9: _t->on_checkBox_2_clicked(); break;
        case 10: _t->on_checkBox_3_clicked(); break;
        case 11: _t->on_checkBox_4_clicked(); break;
        case 12: _t->on_checkBox_5_clicked(); break;
        case 13: _t->delay_3((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 14: { int _r = _t->max((*reinterpret_cast< int(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 15: { int _r = _t->LIMIT((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = std::move(_r); }  break;
        case 16: _t->Cv_to_Qimage((*reinterpret_cast< cv::Mat(*)>(_a[1]))); break;
        case 17: _t->reset(); break;
        case 18: _t->send_data(); break;
        case 19: { double _r = _t->ANG2RAD((*reinterpret_cast< double(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< double*>(_a[0]) = std::move(_r); }  break;
        case 20: { bool _r = _t->InverseKinematics((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 21: { bool _r = _t->limit_1((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 22: { bool _r = _t->limit_2((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 23: { bool _r = _t->limit_3((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 24: _t->Joint_space_inverse((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5]))); break;
        case 25: _t->Three_interpolation((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3]))); break;
        case 26: _t->track_view(); break;
        case 27: _t->track_data(); break;
        case 28: _t->numeric_data(); break;
        case 29: _t->numericUpDown_view(); break;
        case 30: _t->copy((*reinterpret_cast< double(*)[]>(_a[1])),(*reinterpret_cast< double(*)[]>(_a[2])),(*reinterpret_cast< int(*)>(_a[3]))); break;
        case 31: { QString _r = _t->data_to_Fstring((*reinterpret_cast< int(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< QString*>(_a[0]) = std::move(_r); }  break;
        case 32: { cv::Mat _r = _t->Forward_kinematics((*reinterpret_cast< double(*)[]>(_a[1])),(*reinterpret_cast< double(*)[]>(_a[2])),(*reinterpret_cast< double(*)[]>(_a[3])),(*reinterpret_cast< double(*)[]>(_a[4])),(*reinterpret_cast< int(*)>(_a[5])));
            if (_a[0]) *reinterpret_cast< cv::Mat*>(_a[0]) = std::move(_r); }  break;
        case 33: _t->set_PWM((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5])),(*reinterpret_cast< double(*)>(_a[6]))); break;
        case 34: _t->set_TIME((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5])),(*reinterpret_cast< double(*)>(_a[6]))); break;
        case 35: _t->on_horizontalSlider_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 36: _t->on_horizontalSlider_2_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 37: _t->on_horizontalSlider_4_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 38: _t->on_horizontalSlider_5_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 39: _t->on_horizontalSlider_3_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 40: _t->on_horizontalSlider_6_sliderMoved((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 41: _t->on_spinBox_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 42: _t->on_spinBox_2_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 43: _t->on_spinBox_3_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 44: _t->on_spinBox_4_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 45: _t->on_spinBox_5_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 46: _t->on_spinBox_6_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 47: _t->on_spinBox_7_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 48: _t->on_spinBox_8_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 49: _t->on_spinBox_9_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 50: _t->on_spinBox_10_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 51: _t->on_spinBox_11_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 52: _t->on_spinBox_12_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 53: _t->on_exit_Button_clicked(); break;
        case 54: _t->on_Help_Button_clicked(); break;
        case 55: _t->on_pushButton_clicked(); break;
        case 56: _t->on_spinBox_14_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 57: _t->on_Capture_button_clicked(); break;
        default: ;
        }
    }
}

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow.data,
      qt_meta_data_MainWindow,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 58)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 58;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 58)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 58;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
