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
 *  @brief : 构造函数，执行初始化参数
*/
cy_preproc::cy_preproc()
{
    cyVarInit();
    std::cout<<"TEST : cy_preproc "<<std::endl;
}

/**
 * @brief: 变量初始化
*/
void cy_preproc::cyVarInit()
{
    // ROI
    CY_ROI_x = 0;	//<检测区域左上坐标x
    CY_ROI_y = 0;	//<检测区域左上坐标y
    CY_ROI_L = 320;	//<检测区域长度L
    CY_ROI_H = 240;	//<检测区域高度H

    // CY related
    CY_bw_thresh = 127;		//<灰度阈值
    CY_r = 20;				//<冲孔半径
    CY_maxStep = 400;		//<单次最大冲孔数量
    CY_delta = 0;			//<冲孔间距
    CY_dist = 0;			//<随边间距
    CY_img_invert = 0;		//<反色选择
}

/**
*	@brief : RGB图像转二值化图像
*	@param imgin : 原图像引用
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

