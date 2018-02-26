#include <iostream>
#include <float.h>
#include <QVector>
#include <QDebug>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "globalParam.h"
#include "cy_preproc.h"

using namespace std;
using namespace cv;

/**
 *  @brief : ���캯����ִ�г�ʼ������
*/
cy_preproc::cy_preproc()
{
    cyVarInit();
    std::cout<<"TEST : cy_preproc "<<std::endl;
}

/**
 * @brief: ������ʼ��
*/
void cy_preproc::cyVarInit()
{
    // ROI
    CY_ROI_x = 0;	//<���������������x
    CY_ROI_y = 0;	//<���������������y
    CY_ROI_L = 320;	//<������򳤶�L
    CY_ROI_H = 240;	//<�������߶�H

    // CY related
    CY_bw_thresh = 127;		//<�Ҷ���ֵ
    CY_r = 20;				//<��װ뾶
    CY_maxStep = 400;		//<�������������
    CY_delta = 0;			//<��׼��
    CY_dist = 0;			//<��߼��
    CY_img_invert = 0;		//<��ɫѡ��
}

/**
*	@brief : RGBͼ��ת��ֵ��ͼ��
*	@param imgin : ԭͼ������
*/
cv::Mat cy_preproc::rgb2bw(cv::Mat& imgin)
{
	Mat imgout(imgin);

	cvtColor(imgout, imgout, CV_RGB2GRAY);
	threshold(imgout, imgout, CY_bw_thresh, 255, THRESH_BINARY);
	if (CY_img_invert)
		imgout = ~imgout;
	return imgout;
}

