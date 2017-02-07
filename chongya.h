#ifndef __CHONGYA_H
#define __CHONGYA_H

#include <opencv2/opencv.hpp>
#include <QVector>

class cy
{
public:
    cy();
    cv::Mat chongya(cv::Mat& img);  // Main process => generate vout&voutf and result image
    cv::Mat resize2bw(cv::Mat& img);

    QVector<cv::Point> vout;    //point vector of int -- Pix location
    QVector<cv::Point_<double>> voutf;  //point vector of float --  Real location
private:
    cv::Point_<double> mapPoint(cv::Point pin, cv::Point pinMax, double x, double y, double L, double H);
    int cy::mapScale(double realScale, cv::Point pinMax, double L, double H);
    template <typename T>
    T* pointSort(T *rawPoint, int n);
    QVector<cv::Point> cy::pointPixSort(cv::Point *rawPoint, int n);

};

#endif // !__CHONGYA_H
