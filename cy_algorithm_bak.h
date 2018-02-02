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
    cv::Mat stereotypeSub(cv::Mat& imgbw, cv::Mat stereotype, cv::Point center);


    cv::Mat plotc(cv::Mat& img, cv::Point rad, int r);
    cv::Mat plotr(cv::Mat& img, cv::Point rad, int xr, int yr);
    cv::Mat plotp(cv::Mat& img, cv::Point rad, int r);
    cv::Mat &plota(cv::Mat& img_raw, cv::Mat&sterotype, cv::Point rad, cv::Point center);

    cv::Mat plotcEmpty(cv::Mat& img, cv::Point rad, int r);
    cv::Mat plotrEmpty(cv::Mat& img, cv::Point rad, int xr, int yr);
    cv::Mat plotpEmpty(cv::Mat& img, cv::Point rad, int r);
    cv::Point findValueStore(const cv::Mat &mat, uchar value);
    cv::Point findValueLine(const cv::Mat &mat, uchar value, int lines);
    QVector<cv::Point> pointPixSort(QVector<cv::Point> &vin);

    QVector<cv::Point> pointPixSortHlimit(QVector<cv::Point> &vin, int imgHeight, int radius);


    QVector<cv::Point> pointPixForwardSort(QVector<cv::Point> &vin);
    int chongya(cv::Mat& img, int radius, int dist, int space, QVector<cv::Point> &vec, int overLap);
    int chongyaAllShape(cv::Mat& img, cv::Mat&shape, int dist, int space, QVector<cv::Point> &vec, int overLap);

	bool stereotypeInfoGet(const cv::Mat & shape, int & x_dist, int & y_dist);
	double getJumpfactor(cv::Mat& stereotype_sub, cv::Point& stereotype_sub_center, int x_length, int y_length);
	cv::Mat & getStereotype(cv::Mat & img_with_stereotype);

    int chongyaFowardCircle(cv::Mat& img, int radius, int dist, int space, QVector<cv::Point> &vec, int overLap);
    int chongyaFowardCircle_w(cv::Mat& img, int radius, int dist, int space, QVector<cv::Point> &vec, int overLap);

    int chongyaFowardPoly(cv::Mat& img, int radius, int dist, int space, QVector<cv::Point> &vec, int overLap);

    int chongyaFowardRect(cv::Mat& img, int xradius, int yradius, int dist, int space, QVector<cv::Point> &vec, int overLap);


    // Get factor
    double averageDif(std::vector<cv::Vec2f> &v);
    void removeOverlapLines(std::vector<cv::Vec2f> &v);
    cv::Vec2f getFactor(cv::Mat& img, int houghValue, cv::Mat &imgResult);

signals:

public slots:
};

#endif // CY_ALGORITHM_H
