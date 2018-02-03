/**
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

    cv::Mat edgesbw(cv::Mat imgbw);
    cv::Mat circleSub(cv::Mat& imgbw, int r);
    cv::Mat polySub(cv::Mat& imgbw, int r);
    cv::Mat rectSub(cv::Mat& imgbw, int xr, int yr);

    cv::Mat plotc(cv::Mat& img, cv::Point rad, int r);
    cv::Mat plotr(cv::Mat& img, cv::Point rad, int xr, int yr);
    cv::Mat plotp(cv::Mat& img, cv::Point rad, int r);

    cv::Mat plotcEmpty(cv::Mat& img, cv::Point rad, int r);
    cv::Mat plotrEmpty(cv::Mat& img, cv::Point rad, int xr, int yr);
    cv::Mat plotpEmpty(cv::Mat& img, cv::Point rad, int r);

    cv::Point findValueStore(const cv::Mat &mat, uchar value);
    cv::Point findValueLine(const cv::Mat &mat, uchar value, int lines);

    QVector<cv::Point> pointPixSort(QVector<cv::Point> &vin);
    QVector<cv::Point> pointPixForwardSort(QVector<cv::Point> &vin);
    QVector<cv::Point> pointPixSortHlimit(QVector<cv::Point> &vin, int imgHeight, int radius);

    int chongyaFowardCircleSmartHorizontal(cv::Mat& img, int radius, int dist, int space, QVector<cv::Point> &vec, int overLap);//圆形随边智能横排
    int chongyaFowardPolySmartHorizontal(cv::Mat& img, int radius, int dist, int space, QVector<cv::Point> &vec, int overLap);//六边形随边智能横排

    int chongya(cv::Mat& img, int radius, int dist, int space, QVector<cv::Point> &vec, int overLap);   //圆形智能排
    int chongyaFowardCircle(cv::Mat& img, int radius, int dist, int space, QVector<cv::Point> &vec, int overLap);//圆形普通三角形排法-修改顶部扫描方式
    int chongyaFowardPoly(cv::Mat& img, int radius, int dist, int space, QVector<cv::Point> &vec, int overLap);//六边形普通三角形排法-强行后退0.5r方式（未修改为顶部扫描方式）
    int chongyaFowardRect(cv::Mat& img, int xradius, int yradius, int dist, int space, QVector<cv::Point> &vec, int overLap);//矩形排法
    int chongyaFowardCircle_w(cv::Mat& img, int radius, int dist, int space, QVector<cv::Point> &vec, int overLap);//圆形竖型排法--未修改为随边

	cv::Mat tmp_img;
signals:

public slots:
};

#endif // CY_ALGORITHM_H
