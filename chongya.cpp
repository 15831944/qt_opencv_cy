#include <iostream>
#include <opencv2/opencv.hpp>
#include "cyproc.h"
#include "globalParam.h"
#include "delay.h"
#include "chongya.h"
//#include <set>
#include <float.h>
#include <QVector>
#include <QDebug>

using namespace std;
using namespace cv;

/// Resize img's rows and cols
int imgRows = 320;
int imgCols = 240;
double imgFactor = 1.3; // 图像比例

/// ROI
int CY_ROI_x = 0;	//检测区域左上坐标x
int CY_ROI_y = 0;	//检测区域左上坐标y
int CY_ROI_L = 320;	//检测区域长度L
int CY_ROI_H = 240;	//检测区域高度H

/// Global parameter definition
int CY_bw_thresh = 127;		//灰度阈值
int CY_r = 20;				// 冲孔半径
int CY_maxStep = 400;		// 单次最大冲孔数量
int CY_delta = 0;			// 冲孔间距
int CY_dist = 0;			// 随边间距
int CY_img_invert = 0;		// 反色选择

Point_<double> RadArray[400+1];
Point RadArrayInt[400+1];

/*
 * @brief: main process
 * @param:
 *      img: opencv binary img
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
    int i;
	Point rad;
    for (i = 0; i < CY_maxStep; i++)
	{
        /// 检测是否存在有效圆心区域，
        /// 若存在，获取一像素坐标；若不存在，退出处理循环
        rad = findValueStore(img, IMGBW_BLACK);
//        rad = findValueStoreS(img, IMGBW_BLACK);
        if (rad.x<0 || rad.y<0) break;

        /// 像素坐标数组
        RadArrayInt[i] = rad;

        /// 减圆操作 (半径为：2r+delta)
		plotc(img, rad, 2*CY_r+CY_delta);
		imgEdge = edgesbw(img);
	}
    ///添加结束点标志位,得到点集有效个数
    RadArray[i] = Point(-1, -1);
    numOfPoints = i;

    ///冲压顺序排序（像素点）
    pointPixSort(RadArrayInt, numOfPoints);


    ///结果显示
    voutf.clear();
    for(int j=0; j<numOfPoints; j++)
    {
        /// 坐标映射与输出
        RadArray[j] = mapPoint(RadArrayInt[j], Point(img.cols, img.rows), CY_ROI_x, CY_ROI_y, CY_ROI_L, CY_ROI_H);
        voutf.append(mapPoint(RadArrayInt[j], Point(img.cols, img.rows), CY_ROI_x, CY_ROI_y, CY_ROI_L, CY_ROI_H));
        cout << RadArray[j] << endl;
        /// 冲压结果画圆
        cv::circle(img_raw, RadArrayInt[j], CY_r, cv::Scalar(255, 255, 255), 1);
        /// 顺序显示
        cv::putText(img_raw, String(to_string(j+1)), RadArrayInt[j]-cv::Point(CY_r/5.0, -CY_r/5.0), CV_FONT_HERSHEY_COMPLEX, CY_r*2.0/CY_r_Max, Scalar(255, 0, 0), 1, cv::LINE_AA);
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
 * @brief: 冲孔圆心排序
 *         选择一个起点，在剩余点中选出几何距离最近的点，作为输出的下一个点和新的起点，重复此步骤至点集空
 * @param:
 *          rawPoint:输入点集
 *          n: 点集元素个数
 * @note: 此函数将改变输入点集
*/
template <typename T>
T* cy::pointSort(T *rawPoint, int n)
{
    /// Array to QVector
    QVector<T> vin(n);
    qCopy(rawPoint, rawPoint+n, vin.begin());
    QVector<T> vout;

    T startPoint;
    startPoint = vin[0];
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
    qCopy(vout.begin(), vout.begin()+vout.length(), rawPoint);
    return rawPoint;
}


QVector<Point> cy::pointPixSort(Point *rawPoint, int n)
{
    /// Array to QVector
    QVector<Point> vin(n);
    qCopy(rawPoint, rawPoint+n, vin.begin());

    Point startPoint;
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
    qCopy(vout.begin(), vout.begin()+vout.length(), rawPoint);

    return vout;
}

cy::cy()
{
    std::cout<<"This is a cy test!"<<std::endl;
}




/*
%% 算法原理
%{
#1 得到扫描二值图
#2 按照半径内缩腐蚀，得到首个圆心区域
#3 检测圆心区域是否存在，否则退出
#4 在圆形区域边缘左侧选取一点P，打印P点坐标
#5 减去P点圆心，2r半径的圆，得到新的圆心区域，重复#3, 4, 5步骤
%}
*/

/*
NOTE :
opencv 中的位操作都是用0和255来代表逻辑值不是0和1

---还有很多可以优化的地方---
1.现在的二值化采取的是简单的灰度化的值，特征值是亮度，太过单一，分化度不高
	如果将图像转化到HSV或者YCrCb色域上，通过三个通道的阈值进行二值化可以达到到比较好的分化，
	肤色提取也可用此方法，效果中等偏上

2.二值化后需要一系列的边缘缩进和寻找白点（或者黑点），现在采取简单的从头遍历，但是事实上应该
	可以保存上一次找到的点，作为此次的遍历开始点，可以降低时间复杂度

3.在进行二值图边缘提取的时候有一点特别需要注意：
	opencv中的提取边缘函数有个问题就是存在"与运算缩扩"
	即：提取的边缘与原图像（或原图像的取反）进行与运算，---（*）
	得到的图像再次进行（*）操作，图像的黑色（或者白色）会不断的缩进或者扩展
	导致得到错误低效的坐标点，这应该是因为opencv的边缘提取基本上都是以求导预算为基本运算得到
	边缘的，所以写了一个自己的‘四方向二值图边缘检测’【cv::Mat edgesbw(cv::Mat imgbw)】克服这种现象。


4.现在的程序都是以圆形为冲压图像的，但是转化到无旋转的正方形和正六边形原理是不变的，是需要改少量的代码，
	就是边缘缩进的代码需要修改（原理在纸质推导上）

5.关于上下左右边缘的处理还不太清楚具体要求，但是如果要把这四个边缘考虑到随边缩进来，只要在得到二值图后把
二值图的上下左右四个行/列的颜色置为白色值（不是材料的颜色值）就可以了，在opencv中简单几条语句就可以实现。

6.程序仓促有很多要优化的地方。。。

*/

/*
{
1.原始，从左到右
2.采用s形状路线
3.欧几里得距离 ->目前暂时采用这种
}
*/
