﻿/**
* @file cy_algorithm.h
* 算法文件
* 该文件源码描述了算法所涉及的函数，参数
* @date 2018-02-02
*/

#ifndef CY_ALGORITHM_H
#define CY_ALGORITHM_H

#include <QObject>
#include <QVector>
#include <vector>
#include <opencv2/opencv.hpp>

#define CY_MAXSTEP	(400)			// 单次最大冲孔数量
#define CY_R_MAX	(100)			// 冲孔半径最大值
#define IMGBW_BLACK (0)				//opencv黑色值
#define IMGBW_WITHE (255)			//opencv白色值
#define IMGBW_THRESH_B2W (200)		//四方向二值图边缘检测阈值

class cy_algorithm : public QObject
{
    Q_OBJECT
public:
    explicit cy_algorithm(QObject *parent = 0);
    ~cy_algorithm();

	// 异形孔预处理相关函数
	cv::Mat getStereotype(cv::Mat & img_with_stereotype);
	cv::Mat setStereotype(cv::Mat & stereotype);

	//智能横排
	int chongyaFowardCircleSmartHorizontal(cv::Mat & img, int radius, int dist, int space, QVector<cv::Point>& vec, int overLap, double new_tablet_scanRange_factor=0.3, double scanRange_factor=0.22);
	int chongyaFowardCircleSmartHorizontalMirror(cv::Mat & img, int radius, int dist, int space, QVector<cv::Point>& vec, int overLap, double new_tablet_scanRange_factor=0.3, double scanRange_factor=0.22);
	int chongyaFowardPolySmartHorizontal(cv::Mat & img, int radius, int dist, int space, QVector<cv::Point>& vec, int overLap, double new_tablet_scanRange_factor=0.3, double scanRange_factor=0.22);//六边形随边智能横排
	int chongyaFowardAbnormitySmartHorizontal(cv::Mat & img, int dist, int space, QVector<cv::Point>& vec, int overLap, double scanFactor=0.3, double x_extra_space_factor=1, double y_extra_space_factor=1, bool superNarrow=false);

	//智能排孔（圆形）
    int chongya(cv::Mat& img, int radius, int dist, int space, QVector<cv::Point> &vec, int overLap);//圆形智能排
    
	int chongyaFowardCircle(cv::Mat& img, int radius, int dist, int space, QVector<cv::Point> &vec, int overLap);//圆形普通三角形排法-修改顶部扫描方式
    int chongyaFowardPoly(cv::Mat& img, int radius, int dist, int space, QVector<cv::Point> &vec, int overLap);	//六边形普通三角形排法-强行后退0.5r方式（未修改为顶部扫描方式）
	int chongyaFowardRect(cv::Mat & img, int xradius, int yradius, int dist, int space, QVector<cv::Point>& vec, int overLap, double new_tablet_scanRange_factor = 0.3);//矩形排法
	int chongyaFowardCircle_w(cv::Mat& img, int radius, int dist, int space, QVector<cv::Point> &vec, int overLap, double new_tablet_scanRange_factor = 0.3, double scanRange_factor = 0.22);//智能竖排

	//正反排孔
	int chongyaFowardAbnormityPositive(cv::Mat & img, int dist, int space, QVector<cv::Point>& vec, int overLap, double scanFactor=0, double x_extra_space_factor=1, double y_extra_space_factor=1, double jumpFactor=1, bool superNarrow=false);
	int chongyaFowardAbnormityNegtive(cv::Mat & img, int dist, int space, QVector<cv::Point>& vec, int overLap, double scanFactor = 0.7, double x_extra_space_factor = 1, double y_extra_space_factor = 1, double jumpFactor = 0.7, bool superNarrow = false);
	
	//公共变量
	cv::Mat tmp_img;
private:
	//异形相关
	bool stereotypeInfoGet(const cv::Mat & shape, int & x_dist, int & y_dist);
	double getJumpfactor(cv::Mat & stereotype_sub, cv::Point & stereotype_sub_center, int x_length, int y_length);
	cv::Mat stereotypeSub(cv::Mat & imgbw, cv::Mat stereotype, cv::Point center);
	cv::Mat & plota(cv::Mat & img_raw, cv::Mat & stereotype, cv::Point rad, cv::Point center);

	//排序
	QVector<cv::Point> pointPixSort(QVector<cv::Point> &vin);
	QVector<cv::Point> pointPixForwardSort(QVector<cv::Point> &vin);
	QVector<cv::Point> pointPixSortHlimit(QVector<cv::Point> &vin, int imgHeight, int radius);

	//画实心孔
	cv::Mat plotc(cv::Mat& img, cv::Point rad, int r);
	cv::Mat plotr(cv::Mat& img, cv::Point rad, int xr, int yr);
	cv::Mat plotp(cv::Mat& img, cv::Point rad, int r);

	//画空心孔
	cv::Mat plotcEmpty(cv::Mat& img, cv::Point rad, int r);
	cv::Mat plotrEmpty(cv::Mat& img, cv::Point rad, int xr, int yr);
	cv::Mat plotpEmpty(cv::Mat& img, cv::Point rad, int r);

	//寻值函数
	cv::Point findValueStore(const cv::Mat &mat, uchar value);
	cv::Point findValueLine(const cv::Mat &mat, uchar value, int lines);

	//基本图像处理函数
	cv::Mat edgesbw(cv::Mat imgbw);
	cv::Mat circleSub(cv::Mat& imgbw, int r);
	cv::Mat polySub(cv::Mat& imgbw, int r);
	cv::Mat rectSub(cv::Mat& imgbw, int xr, int yr);

	//获得异形Y方向基本间距
	int getAbnormityPosYdist(cv::Mat & img, int space);
signals:

public slots:
};

#endif // CY_ALGORITHM_H
