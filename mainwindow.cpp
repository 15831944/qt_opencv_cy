#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "globalParam.h"
#include <QDebug>
#include <qimagereader.h>
#include <QMessageBox>
#include <opencv2/opencv.hpp>
#include "chongya.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //Var init
    TrackbarVorP_value = 0;
    TrackbarRorE_value = 0;

    //Alocate memery
    m_cy = new cy;
    m_vid = new cv::VideoCapture;
    m_frame = new cv::Mat;
    m_rawframe = new cv::Mat;
    m_bwframe = new cv::Mat;
    m_roiframe = new cv::Mat;
    m_timer = new QTimer;
    m_imgLabel = new QLabel(ui->scrollArea);
    m_imgbwLabel = new QLabel(ui->scrollAreaBw);
    m_imgResultLabel = new QLabel(ui->scrollAreaResult);
    m_ROI = new cv::Rect();

    //ROI init
    connect(ui->spinBoxTop, SIGNAL(valueChanged(int)), this, SLOT(slot_setROI(int)));
    connect(ui->spinBoxButton, SIGNAL(valueChanged(int)), this, SLOT(slot_setROI(int)));
    connect(ui->spinBoxLeft, SIGNAL(valueChanged(int)), this, SLOT(slot_setROI(int)));
    connect(ui->spinBoxRight, SIGNAL(valueChanged(int)), this, SLOT(slot_setROI(int)));
    ui->spinBoxTop->setValue(0);
    ui->spinBoxLeft->setValue(0);
    ui->spinBoxButton->setValue(300);
    ui->spinBoxRight->setValue(400);

    //Timer init
    m_timer->setInterval(30);
    connect(m_timer, SIGNAL(timeout()), this, SLOT(slot_updateImg()));
    connect(m_timer, SIGNAL(timeout()), this, SLOT(slot_mainProc()));

    //Set slider max value
    ui->sliderThresh->setMaximum(CY_bw_thresh_Max);
    ui->sliderHoleR->setMaximum(CY_r_Max);
    ui->sliderS2H->setMaximum(CY_dist_Max);
    ui->sliderH2H->setMaximum(CY_delta_Max);
    ui->spinBoxX->setMaximum(CY_ROI_x_Max);
    ui->spinBoxY->setMaximum(CY_ROI_y_Max);
    ui->spinBoxL->setMaximum(CY_ROI_L_Max);
    ui->spinBoxH->setMaximum(CY_ROI_H_Max);

    //Table
    ui->tableResult->setRowCount(400);
    ui->tableResult->setColumnCount(2);
    ui->tableResult->setColumnWidth(0,60);
    ui->tableResult->setColumnWidth(1,60);

    //Signal connect to slot
    connect(ui->pushButtonExit, SIGNAL(clicked(bool)), this, SLOT(slot_Exit()));
    connect(ui->pushButtonProc, SIGNAL(clicked(bool)), this, SLOT(slot_Proc()));
    connect(ui->sliderThresh, SIGNAL(valueChanged(int)), this, SLOT(slot_setThreshValue(int)));
    connect(ui->sliderHoleR, SIGNAL(valueChanged(int)), this, SLOT(slot_setHoleRValue(int)));
    connect(ui->checkBoxInvert, SIGNAL(toggled(bool)), this, SLOT(slot_Invert(bool)));
    connect(ui->sliderS2H, SIGNAL(valueChanged(int)), this, SLOT(slot_setS2HValue(int)));
    connect(ui->sliderH2H, SIGNAL(valueChanged(int)), this, SLOT(slot_setH2HValue(int)));
    connect(ui->spinBoxX, SIGNAL(valueChanged(int)), this, SLOT(slot_setXValue(int)));
    connect(ui->spinBoxY, SIGNAL(valueChanged(int)), this, SLOT(slot_setYValue(int)));
    connect(ui->spinBoxL, SIGNAL(valueChanged(int)), this, SLOT(slot_setLValue(int)));
    connect(ui->spinBoxH, SIGNAL(valueChanged(int)), this, SLOT(slot_setHValue(int)));

    //Set initial value
    ui->sliderThresh->setValue(127);
    ui->sliderHoleR->setValue(20);
    ui->sliderS2H->setValue(0);
    ui->sliderH2H->setValue(0);
    ui->spinBoxX->setValue(0);
    ui->spinBoxY->setValue(0);
    ui->spinBoxL->setValue(320);
    ui->spinBoxH->setValue(240);

    slot_openCam();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    qDebug()<<"It will close!";
    slot_closeCam();
    slot_Exit();
    event->accept();
}

void MainWindow::slot_Exit()
{
    TrackbarRorE_value = 1;
//    qDebug()<<"TrackbarRorE_value: "<<TrackbarRorE_value;
}

void MainWindow::slot_Proc()
{
    TrackbarVorP_value = 1;
//    qDebug()<<"TrackbarVorP_value: "<<TrackbarVorP_value;
}

void MainWindow::slot_setThreshValue(int value)
{
    QString str;
//    m_cy->CY_bw_thresh = value;
    m_cy->set(_CY_bw_thresh, value);
    str = str.number(value);
    ui->labelThreshValue->setText(str);
}

void MainWindow::slot_setHoleRValue(int value)
{
    QString str;
//    m_cy->CY_r = value;
    m_cy->set(_CY_r, value);
    str = str.number(value);
    ui->labelHoleRValue->setText(str);
}

void MainWindow::slot_Invert(bool checked)
{
    if(checked)
//        m_cy->CY_img_invert = 1;
        m_cy->set(_CY_img_invert, 1);
    else
//        m_cy->CY_img_invert = 0;
        m_cy->set(_CY_img_invert, 0);
}

void MainWindow::slot_setS2HValue(int value)
{
    QString str;
//    m_cy->CY_dist = value;
    m_cy->set(_CY_dist, value);

    str = str.number(value);
    ui->labelS2HValue->setText(str);
}

void MainWindow::slot_setH2HValue(int value)
{
    QString str;
//    m_cy->CY_delta = value;
    m_cy->set(_CY_delta, value);

    str = str.number(value);
    ui->labelH2HValue->setText(str);
}

void MainWindow::slot_setXValue(int value)
{
//    qDebug()<<"X: "<<value;
//    m_cy->CY_ROI_x = value;
    m_cy->set(_CY_ROI_x, value);
}

void MainWindow::slot_setYValue(int value)
{
//    qDebug()<<"Y: "<<value;
//    m_cy->CY_ROI_y = value;
    m_cy->set(_CY_ROI_y, value);
}

void MainWindow::slot_setLValue(int value)
{
//    qDebug()<<"L: "<<value;
//    m_cy->CY_ROI_L = value;
    m_cy->set(_CY_ROI_L, value);
}

void MainWindow::slot_setHValue(int value)
{
//    qDebug()<<"H: "<<value;
//    m_cy->CY_ROI_H  = value;
    m_cy->set(_CY_ROI_H, value);
}

void MainWindow::slot_openCam()
{

    if(!m_vid->open(CAM_INDEX))
    {
         qDebug()<<"Can not open CAM!";
         return;
    }
    else
    {
         qDebug()<<"Success to open CAM!";
         m_timer->start();
    }
}

void MainWindow::slot_closeCam()
{
    m_timer->stop();
    m_vid->release();
    delete m_vid;
    m_vid = NULL;
    delete m_frame;
    m_frame = NULL;
    delete m_timer;
    m_timer = NULL;
}

void MainWindow::slot_updateImg()
{
    *m_vid >> *m_frame;
    m_frame->copyTo(*m_rawframe);

    //ROI frame
    /// ROI valid revise
    if(m_ROI->x>m_frame->cols)
        m_ROI->x = m_frame->cols;
    if(m_ROI->y>m_frame->rows)
        m_ROI->y = m_frame->rows;
    if(m_ROI->x + m_ROI->width > m_frame->cols)
    {
        m_ROI->width = m_frame->cols-m_ROI->x;
    }
    if(m_ROI->y + m_ROI->height > m_frame->rows)
    {
        m_ROI->height = m_frame->rows-m_ROI->y;
    }

    *m_bwframe = m_cy->resize2bw(*m_frame);
    *m_bwframe = (*m_bwframe)(*m_ROI);
    *m_roiframe = *m_bwframe;   //m_roiframe should be Binary img

    //ROI Rect
    cv::rectangle(*m_frame, *m_ROI, cv::Scalar(0, 0, 255), 1);
//    qDebug()<<m_ROI->x<<","<<m_ROI->y<<","<<m_ROI->width<<","<<m_ROI->height;

    cv::cvtColor(*m_frame, *m_frame, cv::COLOR_RGB2BGR);
    QImage img(m_frame->data, m_frame->cols, m_frame->rows, int(m_frame->step), QImage::Format_RGB888);

    cv::cvtColor(*m_bwframe, *m_bwframe, cv::COLOR_GRAY2BGR);
    QImage imgbw(m_bwframe->data, m_bwframe->cols, m_bwframe->rows, int(m_bwframe->step), QImage::Format_RGB888);

    if (img.isNull() || imgbw.isNull())
    {
        qDebug()<<"Image is NULL!";
    }

    m_imgLabel->setPixmap(QPixmap::fromImage(img));
    ui->scrollArea->setWidget(m_imgLabel);

    m_imgbwLabel->setPixmap(QPixmap::fromImage(imgbw));
    ui->scrollAreaBw->setWidget(m_imgbwLabel);
}

void MainWindow::slot_mainProc()
{
    cv::Mat frame;
    cv::Mat resultFrame;
    frame = *m_roiframe;

    if (TrackbarVorP_value)
    {
        resultFrame = m_cy->chongya(frame);
        TrackbarVorP_value = 0;

        ///Display result
        cv::cvtColor(resultFrame, resultFrame, cv::COLOR_GRAY2BGR);
        QImage imgResult(resultFrame.data, resultFrame.cols, resultFrame.rows, int(resultFrame.step), QImage::Format_RGB888);
        m_imgResultLabel->setPixmap(QPixmap::fromImage(imgResult));
        ui->scrollAreaResult->setWidget(m_imgResultLabel);

        /// Rad table
        ui->tableResult->clearContents();
        for(int i=0; i<m_cy->get_voutf().length(); i++)
        {
            setTable(i, m_cy->get_voutf()[i].x, m_cy->get_voutf()[i].y);
        }
    }
    if (TrackbarRorE_value)
    {
        this->close();
    }
}

void MainWindow::slot_setROI(int value)
{
    QString strName;
    strName = this->sender()->objectName();
    qDebug()<<strName;

    /// Value valid check step 1
    if(value<0)
        value = 0;

    if(strName == "spinBoxTop")
    {
        m_ROI->y = value;
    }
    else if(strName == "spinBoxLeft")
    {
        m_ROI->x = value;
    }
    else if(strName == "spinBoxRight")
    {
        m_ROI->width = value;
    }
    else if(strName == "spinBoxButton")
    {
        m_ROI->height = value;
    }
    else
    {
        qDebug()<<"set ROI error!";
    }
}

void MainWindow::setTable(int id, double x, double y)
{
    int row, col;
    row = id;
    QTableWidgetItem *itemx = new QTableWidgetItem(tr("%1").arg(x));
    QTableWidgetItem *itemy = new QTableWidgetItem(tr("%1").arg(y));

    col = 0;
    ui->tableResult->setItem(row, col, itemx);
    col = 1;
    ui->tableResult->setItem(row, col, itemy);
}
