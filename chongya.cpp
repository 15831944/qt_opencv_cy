#include <iostream>
#include <opencv2/opencv.hpp>
#include "cyproc.h"
#include "globalParam.h"
#include "chongya.h"
#include <float.h>
#include <QVector>
#include <QDebug>

using namespace std;
using namespace cv;

/// Resize img's rows and cols


/*
 *  @brief : 构造函数，执行初始化参数
*/
cy::cy()
{
    cyVarInit();
    std::cout<<"This is a cy test!"<<std::endl;
}

/*
 * @brief: 变量初始化
*/
void cy::cyVarInit()
{
    /// ROI
    CY_ROI_x = 0;	//检测区域左上坐标x
    CY_ROI_y = 0;	//检测区域左上坐标y
    CY_ROI_L = 320;	//检测区域长度L
    CY_ROI_H = 240;	//检测区域高度H

    /// CY related
    CY_bw_thresh = 127;		//灰度阈值
    CY_r = 20;				// 冲孔半径
    CY_maxStep = 400;		// 单次最大冲孔数量
    CY_delta = 0;			// 冲孔间距
    CY_dist = 0;			// 随边间距
    CY_img_invert = 0;		// 反色选择
}

/*
 * @brief: 主处理
 * @param:
 *      img: opencv二值化图像
*/
cv::Mat cy::chongya(cv::Mat& img)
{
    Mat img_raw,imgEdge;
	unsigned int rows, cols;
    int numOfPoints;

    /// 图像有效性检测
	if (img.empty())
	{
		cerr << "Image read error!" << endl;
        return img;
	}
    img.copyTo(img_raw);

    /// 显示图像信息
	rows = img.rows;
	cols = img.cols;
    cout <<"ROI size(pix): "<< rows << "," << cols << endl;

    /// 得到图像边缘
	imgEdge = edgesbw(img);

    /// 加边框边界，若不需要，移除以下四条语句
    imgEdge.row(0) = IMGBW_WITHE;
    imgEdge.row(rows-1) = IMGBW_WITHE;
    imgEdge.col(0) = IMGBW_WITHE;
    imgEdge.col(cols - 1) = IMGBW_WITHE;
	
    /// 得到首个圆心区域
	img = img | circleSub(imgEdge, CY_r+ CY_dist);

    /// 主处理循环，获得像素坐标点集合
	Point rad;
    vout.clear();
    for (int i = 0; i < CY_maxStep; i++)
	{
        /// 检测是否存在有效圆心区域，
        /// 若存在，获取一像素坐标；若不存在，退出处理循环
        rad = findValueStore(img, IMGBW_BLACK);
        if (rad.x<0 || rad.y<0) break;

        /// 像素坐标数组
        vout.append(rad);

        /// 减圆操作 (半径为：2r+delta)
		plotc(img, rad, 2*CY_r+CY_delta);
		imgEdge = edgesbw(img);
	}
    ///得到点集有效个数
    numOfPoints = vout.length();

    ///冲压顺序排序（像素点）
    vout = pointPixSort(vout);

    ///结果显示
    voutf.clear();
    for(int j=0; j<numOfPoints; j++)
    {
        /// 坐标映射与输出
        voutf.append(mapPoint(vout[j], Point(img.cols, img.rows), CY_ROI_x, CY_ROI_y, CY_ROI_L, CY_ROI_H));
        cout << voutf[j] << endl;
        /// 冲压结果画圆
        cv::circle(img_raw, vout[j], CY_r, cv::Scalar(255, 255, 255), 1);
        /// 顺序显示
        cv::putText(img_raw, String(to_string(j+1)), vout[j]-cv::Point(CY_r/5.0, -CY_r/5.0), CV_FONT_HERSHEY_COMPLEX, CY_r*2.0/CY_r_Max, Scalar(255, 0, 0), 1, cv::LINE_AA);
    }

    /// 总个数显示
    cout << "Total : " << numOfPoints << endl;

    /// 显示处理后图像(For debug)
    // imshow("img_raw", img_raw);

    return img_raw;
}


/*
*	@brief :[缩放尺寸],得到二值化图像
*	@param :
*           imgin: 原图像引用
*/
cv::Mat cy::resize2bw(cv::Mat& imgin)
{
	Mat imgout(imgin);

#ifdef IMG_RESIZE
    int imgRows = 320;
    int imgCols = 240;
    double imgFactor = 1.3; // 图像比例
	if (imgin.cols > imgCols)
	{
		imgFactor = double(imgin.cols) / imgin.rows;
		imgCols = ceil(imgRows*imgFactor);
		resize(imgin, imgout, Size(imgCols, imgRows));
	}
#endif // IMG_RESIZE

	cvtColor(imgout, imgout, CV_RGB2GRAY);
	threshold(imgout, imgout, CY_bw_thresh, 255, THRESH_BINARY);
	if (CY_img_invert)
		imgout = ~imgout;
	return imgout;
}

/*
*	@brief :映射坐标点，从像素坐标映射到实际坐标
*	@param :
*           pin: 像素坐标
*           pinMax: 像素坐标最大值
*           x, y: ROI左上坐标
*           L, H: ROI实际尺寸,宽L, 高H
*   @return: 实际坐标
*/
cv::Point_<double> cy::mapPoint(cv::Point pin, cv::Point pinMax, double x, double y, double L, double H)
{
	cv::Point_<double> pout;
	pout = Point_<double>(x, y) + Point_<double>(L*double(pin.x)/pinMax.x, H*double(pin.y)/pinMax.y);
	return pout;
}


/*
*	@brief :映射尺寸，从实际尺寸映射到像素尺寸
*	@param :
*           realScale: 实际尺寸
*           pinMax: 像素坐标最大值
*           L, H: ROI实际尺寸,宽L, 高H
*   @return: 像素尺寸
*/
int cy::mapScale(double realScale, cv::Point pinMax, double L, double H)
{
    int pixScale;
    double scaleFactor;
    scaleFactor = (double(pinMax.x)/L+double(pinMax.y)/H)/2;
    pixScale = realScale*scaleFactor;
    return pixScale;
}
/*
 *  @brief : 像素点排序
 *  @param :
 *          vin : 像素点的引用
*/
QVector<Point> cy::pointPixSort(QVector<cv::Point> &vin)
{
    Point startPoint;
    QVector<cv::Point> vout(vin.length());
    startPoint = vin[0];
    vout.clear();
    vout.append(startPoint);
    vin.erase(&vin[0]);

    while(vin.length()>0)
    {
        double distMin = INT_MAX;
        int indexDistMin = 0;
        for(int i=0; i<vin.length(); i++)
        {
            double dist = pow(vin[i].x-startPoint.x, 2)+pow(vin[i].y-startPoint.y, 2);
            if(dist<distMin)
            {
                distMin = dist;
                indexDistMin = i;
            }
        }
        startPoint = vin[indexDistMin];  //由于迭代器的更新，保留startPoint的数值
        vout.append(startPoint);
        vin.erase(&vin[indexDistMin]);
    }
    vin = vout;
    return vin;
}


