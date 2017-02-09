#ifndef __CYPROC_H
#define __CYPROC_H
#include <opencv2/opencv.hpp>

cv::Mat edgesbw(cv::Mat imgbw);
cv::Mat circleSub(cv::Mat& img, int r);
cv::Point findValue(const cv::Mat &mat, uchar value);
cv::Point findValueStore(const cv::Mat &mat, uchar value);
cv::Point findValueStoreS(const cv::Mat &mat, uchar value);
cv::Mat plotc(cv::Mat& img, cv::Point rad, int r);

template<typename T>
const T* findDir(const T* _first, const T* _last, T val, bool back=false);

#endif // !__CYPROC_H
