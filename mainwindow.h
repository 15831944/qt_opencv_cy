/**
* @file mainwindow.h
* 界面流程程序
* 该文件源码描述了界面涉及的触发函数，各类指针等
* @date 2018-02-02
*/

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QCloseEvent>
#include <QLabel>
#include <QImage>
#include <opencv2/opencv.hpp>
#include <QTimer>
#include "cy_preproc.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    inline cv::Mat* getImg(){return m_frame;}
    void setTable(int id, double x, double y);

public slots:
    void slot_Exit();
    void slot_Proc();
    void slot_setThreshValue(int value);
    void slot_setHoleRValue(int value);
    void slot_Invert(bool checked);
    void slot_setS2HValue(int value);
    void slot_setH2HValue(int value);
    void slot_setXValue(int value);
    void slot_setYValue(int value);
    void slot_setLValue(int value);
    void slot_setHValue(int value);
    void slot_openCam();
    void slot_closeCam();
    void slot_updateImg();
    void slot_mainProc();
    void slot_setROI(int value);
    void slot_setFrameRate(int value);
    void slot_setHoughValue(int value);
    void updateAlgoTime(int value);
    void slot_setAlgoType(int value);

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::MainWindow *ui;
    QLabel *m_imgLabel;	//<原始图像窗口
    QLabel *m_imgbwLabel;	//<二值化图像窗口
    QLabel *m_imgResultLabel;//<结果图像窗口
    cv::VideoCapture *m_vid;	//<视频指针
    cv::Mat *m_frame;	//<原始图像指针(过渡态)
    cv::Mat *m_rawframe;	//<原始图像指针
    cv::Mat *m_bwframe;	//<二值化图像指针(过渡态)
    cv::Mat *m_roiframe;	//<ROI区域图像指针
    QTimer *m_timer;	//<定时器指针
    cv::Rect *m_ROI;	//<ROI矩形指针
    cy_preproc *m_cy;	//<冲压预处理类指针，保存冲压变量和预处理

    int AlgorithmType;	//<算法类型
    int TrackbarVorP_value;	//<状态标签，空闲|处理
    int TrackbarRorE_value;	//<状态标签，运行|退出
};

#endif // MAINWINDOW_H
