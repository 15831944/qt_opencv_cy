#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QCloseEvent>
#include <QLabel>
#include <QImage>
#include "opencv2/opencv.hpp"
#include <QTimer>
#include <chongya.h>



namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void setImgLabel(const QString &fileName);
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

protected:
    void closeEvent(QCloseEvent *event);

private:
    Ui::MainWindow *ui;
    QLabel *m_imgLabel;
    QLabel *m_imgbwLabel;
    QLabel *m_imgResultLabel;
    cv::VideoCapture *m_vid;
    cv::Mat *m_frame;
    cv::Mat *m_rawframe;
    cv::Mat *m_bwframe;
    cv::Mat *m_roiframe;
    QTimer *m_timer;
    cv::Rect *m_ROI;
    cy *m_cy;

    int TrackbarVorP_value;
    int TrackbarRorE_value;
};

#endif // MAINWINDOW_H
