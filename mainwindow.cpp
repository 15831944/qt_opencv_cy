#include <QDebug>
#include <qimagereader.h>
#include <QMessageBox>
#include <opencv2/opencv.hpp>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "globalParam.h"
#include "cy_preproc.h"
#include "cy_algorithm.h"
#include "cy_performance.h"

/*
* @def DEBUG_USE_VID
* @brief Use VIDEO or TEST IMAGES
* if DEBUG_USE_VID is defined, VIDEO is used as sources.
* else TEST IMAGES is used as sources.
*
* @note Used in function:slot_updateImg()
* @see slot_updateImg()
*/
//#define DEBUG_USE_VID	

unsigned int IMG_INDEX = 1; //<Test images index


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //Var init
    TrackbarVorP_value = 0;
    TrackbarRorE_value = 0;

    //Alocate memery
    m_cy = new cy_preproc;
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
    ui->spinBoxButton->setValue(800);
    ui->spinBoxRight->setValue(500);

    //Timer init
    m_timer->setInterval(30);	//<frame interval:30ms
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
    //Frame rate
    ui->sliderFrameRate->setMaximum(1000);
    ui->sliderFrameRate->setMinimum(30);
    //Hough value
    ui->sliderHoughValue->setMaximum(180);
    ui->sliderHoughValue->setMinimum(100);
    //Algorithm type
    ui->comboBoxAlgoChoose->addItem(tr("I-CY(Circle)-0"), 0);
    ui->comboBoxAlgoChoose->addItem(tr("H-CY(Circle)-1"), 1);
    ui->comboBoxAlgoChoose->addItem(tr("H-CY(Poly)-2"), 2);
    ui->comboBoxAlgoChoose->addItem(tr("H-CY(Rect)-3"), 3);
    ui->comboBoxAlgoChoose->addItem(tr("H-W-CY(Circle)-4"), 4);
    ui->comboBoxAlgoChoose->addItem(tr("I-W-CY(Abnormity)-5"), 5);
	ui->comboBoxAlgoChoose->addItem(tr("H-CY(A_Positive)-6"), 6);
	ui->comboBoxAlgoChoose->addItem(tr("H-CY(A_Negtive)-7"), 7);
	ui->comboBoxAlgoChoose->addItem(tr("--Reserved-8"), 8);
	ui->comboBoxAlgoChoose->addItem(tr("--Reserved-9"), 9);
    AlgorithmType = 1;	//<default algorithm type
    ui->comboBoxAlgoChoose->setCurrentIndex(AlgorithmType);

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
    //Frame rate
    connect(ui->sliderFrameRate, SIGNAL(valueChanged(int)), this, SLOT(slot_setFrameRate(int)));
    //Hough value
    connect(ui->sliderHoughValue, SIGNAL(valueChanged(int)), this, SLOT(slot_setHoughValue(int)));
    //Algorithm type
    connect(ui->comboBoxAlgoChoose, SIGNAL(currentIndexChanged(int)), this, SLOT(slot_setAlgoType(int)));

    //Set initial value
    ui->sliderThresh->setValue(127);
    ui->sliderHoleR->setValue(20);
    ui->sliderS2H->setValue(0);
    ui->sliderH2H->setValue(0);
    ui->spinBoxX->setValue(0);
    ui->spinBoxY->setValue(0);
    ui->spinBoxL->setValue(320);
    ui->spinBoxH->setValue(240);
    //Frame Rate
    ui->sliderFrameRate->setValue(200);
    //Hough value
    ui->sliderHoughValue->setValue(150);

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
	//qDebug()<<"TrackbarVorP_value: "<<TrackbarVorP_value;
    //IMG_INDEX++;
}

void MainWindow::slot_setThreshValue(int value)
{
    QString str;
    m_cy->set(_CY_bw_thresh, value);
    str = str.number(value);
    ui->labelThreshValue->setText(str);
}

void MainWindow::slot_setHoleRValue(int value)
{
    QString str;
    m_cy->set(_CY_r, value);
    str = str.number(value);
    ui->labelHoleRValue->setText(str);
}

void MainWindow::slot_setFrameRate(int value)
{
    QString str;
    m_timer->setInterval(value);
    str = str.number(value);
    ui->labelFrameRateValue->setText(str);
}

void MainWindow::slot_setHoughValue(int value)
{
    QString str;
    m_cy->CY_houghValue = value;
    str = str.number(value);
    ui->labelHoughValue->setText(str);
}

/** set Algorithm Type */
void MainWindow::slot_setAlgoType(int value)
{
    AlgorithmType = value;
    std::cout<<"Algo type :"<<value<<std::endl;
}

/** update Algorithm time */
void MainWindow::updateAlgoTime(int value)
{
    QString str;
    str = str.number(value);
    str += " ms";
    ui->labelAlgoTimeValue->setText(str);
}

/** Image color invert */
void MainWindow::slot_Invert(bool checked)
{
    if(checked)
        m_cy->set(_CY_img_invert, 1);
    else
        m_cy->set(_CY_img_invert, 0);
}

/** set dist(side to hole) */
void MainWindow::slot_setS2HValue(int value)
{
    QString str;
    m_cy->set(_CY_dist, value);

    str = str.number(value);
    ui->labelS2HValue->setText(str);
}

/** set space|delta(hole to hole) */
void MainWindow::slot_setH2HValue(int value)
{
    QString str;
    m_cy->set(_CY_delta, value);

    str = str.number(value);
    ui->labelH2HValue->setText(str);
}

void MainWindow::slot_setXValue(int value)
{
    m_cy->set(_CY_ROI_x, value);
}

void MainWindow::slot_setYValue(int value)
{
    m_cy->set(_CY_ROI_y, value);
}

void MainWindow::slot_setLValue(int value)
{
    m_cy->set(_CY_ROI_L, value);
}

void MainWindow::slot_setHValue(int value)
{
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
         // If 1024x768 is bigger than the Maxium size, it will be set to the Maxium size
         m_vid->set(CV_CAP_PROP_FRAME_HEIGHT,480);
         m_vid->set(CV_CAP_PROP_FRAME_WIDTH,640);

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

/** Choose frame sources and pre-process */
void MainWindow::slot_updateImg()
{
#ifdef DEBUG_USE_VID
	// Get a frame from video
    *m_vid >> *m_frame;
#else
    // Test Images Name Choose 
	cv::String fileNamePre;
	switch (AlgorithmType)
	{
	case 6: { //positive
		fileNamePre = "F:\\project vs\\qt_opencv_cy_20180127\\qt_opencv_cy\\images\\duokuai_";
		break;
	}
	case 7: { //negtive
		fileNamePre = "F:\\project vs\\qt_opencv_cy_20180127\\qt_opencv_cy\\images\\Pos_images\\Positive_test_";
		break;
	}
	default: //normal
		fileNamePre = "F:\\project vs\\qt_opencv_cy_20180127\\qt_opencv_cy\\images\\duokuai_";
		break;
	}
	//cv::String fileNamePre = "F:\\project vs\\qt_opencv_cy_20180127\\qt_opencv_cy\\images\\Pos_images\\Positive_test_";
	//cv::String fileNamePre = "F:\\project vs\\qt_opencv_cy_20180127\\qt_opencv_cy\\images\\duokuai_";
	//cv::String fileNamePre = "F:\\project vs\\qt_opencv_cy_20180127\\qt_opencv_cy\\images\\duokuai_";
	//cv::String fileNamePre = "F:\\project vs\\qt_opencv_cy_20180127\\qt_opencv_cy\\images\\test_img_";
	//cv::String fileNamePre = "F:\\project vs\\qt_opencv_cy_20180127\\qt_opencv_cy\\images\\problem_1\\test_";
	//cv::String fileNamePre = "F:\\project vs\\qt_opencv_cy_20180127\\qt_opencv_cy\\images\\problem_1\\narrow_";
	//cv::String fileNamePre = "F:\\PunchSystem\\CYAlgorithm\\qt_opencv_cy_20180127\\qt_opencv_cy\\images\\test_";
    if(IMG_INDEX>6)
        IMG_INDEX = 1;
    cv::String fileNameNum = cv::String(std::to_string(IMG_INDEX));
    cv::String fileNameEnd = ".jpg";
    cv::String fileName = fileNamePre+fileNameNum+fileNameEnd;
	// Get a frame from test imamge
    *m_frame = cv::imread(fileName);
#endif

    m_frame->copyTo(*m_rawframe);

    // ROI valid check
    if(m_ROI->x>m_frame->cols)
        m_ROI->x = m_frame->cols;
    if(m_ROI->y>m_frame->rows)
        m_ROI->y = m_frame->rows;
    if(m_ROI->x + m_ROI->width > m_frame->cols)
        m_ROI->width = m_frame->cols-m_ROI->x;
    if(m_ROI->y + m_ROI->height > m_frame->rows)
        m_ROI->height = m_frame->rows-m_ROI->y;

    *m_bwframe = m_cy->rgb2bw(*m_frame);	// Convert m_frame(rgb) to m_bwframe(binary)
    *m_bwframe = (*m_bwframe)(*m_ROI);	// Get ROI image 
    *m_roiframe = *m_bwframe;   // m_roiframe should be Binary image

    // Draw ROI Rectangle
    cv::rectangle(*m_frame, *m_ROI, cv::Scalar(0, 0, 255), 1);

	// OpenCV-Mat to Qt-QImage 
    cv::cvtColor(*m_frame, *m_frame, cv::COLOR_RGB2BGR); //Qt use BGR rather than RGB
    QImage img(m_frame->data, m_frame->cols, m_frame->rows, int(m_frame->step), QImage::Format_RGB888);
    cv::cvtColor(*m_bwframe, *m_bwframe, cv::COLOR_GRAY2BGR);	//Qt use BGR rather than RGB
    QImage imgbw(m_bwframe->data, m_bwframe->cols, m_bwframe->rows, int(m_bwframe->step), QImage::Format_RGB888);

	// Check img|imgbw is Null or not
    if (img.isNull() || imgbw.isNull())
    {
        qDebug()<<"Image is NULL!";
    }

	// Show images
    m_imgLabel->setPixmap(QPixmap::fromImage(img));
    ui->scrollArea->setWidget(m_imgLabel);
    m_imgbwLabel->setPixmap(QPixmap::fromImage(imgbw));
    ui->scrollAreaBw->setWidget(m_imgbwLabel);
}

/** Choose algorithm according to AlgorithmType */
void MainWindow::slot_mainProc()
{
    cv::Mat frame;
    cv::Mat resultFrame;
    frame = *m_roiframe;

    if (TrackbarVorP_value)
    {

        cy_algorithm algo;
        cy_performance perform;

        perform.time_start();	// Time counter start
		cv::String stereotype_normal_filename = "F:\\project vs\\qt_opencv_cy_20180127\\qt_opencv_cy\\images\\problem_2\\test_5.jpg";
		cv::String stereotype_pos_neg_filename = "F:\\project vs\\qt_opencv_cy_20180127\\qt_opencv_cy\\images\\problem_2\\test_8.jpg";
        switch (AlgorithmType) {
        case 0:
            algo.chongya(frame, m_cy->CY_r, m_cy->CY_dist, m_cy->CY_delta, m_cy->vout, 0);
            break;
        case 1:
            //algo.chongyaFowardCircle(frame, m_cy->CY_r, m_cy->CY_dist, m_cy->CY_delta, m_cy->vout, 0);
			algo.chongyaFowardCircleSmartHorizontal(frame, m_cy->CY_r, m_cy->CY_dist, m_cy->CY_delta, m_cy->vout, 0);
            break;
        case 2:
            algo.chongyaFowardPoly(frame, m_cy->CY_r, m_cy->CY_dist, m_cy->CY_delta, m_cy->vout, 0);
            break;
        case 3:
            algo.chongyaFowardRect(frame, m_cy->CY_r, 2*(m_cy->CY_r), m_cy->CY_dist, m_cy->CY_delta, m_cy->vout, 0);
            break;
        case 4:
            algo.chongyaFowardCircle_w(frame, m_cy->CY_r, m_cy->CY_dist, m_cy->CY_delta, m_cy->vout, 0);
            break;
		case 5: {
			cv::Mat img_with_stereotype = cv::imread(stereotype_normal_filename);
			//获得样板
			cv::Mat stereotype = algo.getStereotype(img_with_stereotype);
			//设置样板
			algo.setStereotype(stereotype);
			//异形冲压（只有设置过样板异形冲压才会进行，否则返回错误）
			algo.chongyaFowardAbnormitySmartHorizontal(frame, m_cy->CY_dist, m_cy->CY_delta, m_cy->vout, 0);
			break; }
		case 6: {
			cv::Mat img_with_stereotype = cv::imread(stereotype_pos_neg_filename);
			//获得样板
			cv::Mat stereotype = algo.getStereotype(img_with_stereotype);
			//设置样板
			algo.setStereotype(stereotype);
			//异形正面排孔冲压（只有设置过样板异形冲压才会进行，否则返回错误）
			algo.chongyaFowardAbnormityPositive(frame, m_cy->CY_dist, m_cy->CY_delta, m_cy->vout, 0);
			break; }
		case 7: {
			cv::Mat img_with_stereotype = cv::imread(stereotype_pos_neg_filename);
			//获得样板
			cv::Mat stereotype = algo.getStereotype(img_with_stereotype);
			//设置样板
			algo.setStereotype(stereotype);
			//异形反面排孔冲压（只有设置过样板异形冲压才会进行，否则返回错误）
			algo.chongyaFowardAbnormityNegtive(frame, m_cy->CY_dist, m_cy->CY_delta, m_cy->vout, 0);
			break; }
        default:
            std::cerr<<"Algorithm Type Error!";
            break;
        }
        int time_diff = perform.time_end();	// Time counter end

		// Algorithm time update
        this->updateAlgoTime(time_diff);

        int len = m_cy->vout.length();
        //std::cout<< len<<std::endl;

        m_cy->voutf.clear();
        for(int i=0; i<len; i++)
        {
          m_cy->voutf.append((cv::Point_<double>)m_cy->vout[i]);
        }
        resultFrame = frame;

        TrackbarVorP_value = 0;

        // Display result
        cv::cvtColor(resultFrame, resultFrame, cv::COLOR_GRAY2BGR);
        QImage imgResult(resultFrame.data, resultFrame.cols, resultFrame.rows, int(resultFrame.step), QImage::Format_RGB888);
        m_imgResultLabel->setPixmap(QPixmap::fromImage(imgResult));
        ui->scrollAreaResult->setWidget(m_imgResultLabel);

        // Rad table
        ui->tableResult->clearContents();
        for(int i=0; i<m_cy->get_voutf().length(); i++)
        {
            setTable(i, m_cy->get_voutf()[i].x, m_cy->get_voutf()[i].y);
        }
		// Image index increase
		IMG_INDEX++;
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

    // Value valid check step 1
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

