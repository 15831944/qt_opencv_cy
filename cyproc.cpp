#include "cyproc.h"
#include "globalParam.h"
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;

/*
*	@brief : 边缘减圆
*	@note : 输入为边缘二值图，白色为边缘
*
*/
cv::Mat circleSub(cv::Mat& imgbw, int r)
{
    cv::Mat img;
    imgbw.copyTo(img);
    int rows, cols;
    rows = imgbw.rows;
    cols = imgbw.cols;
    int i, j;
    for (i = 0; i < rows; i++)
    {
        for (j = 0; j < cols; j++)
        {
            if (imgbw.at<uchar>(i, j) > IMGBW_THRESH_B2W) //withe area
            {
                cv::Point center = cv::Point(j, i);
                cv::circle(img, center, r, cv::Scalar(255, 255, 255), -1);
            }
        }
    }
    return img;
}

/*
*	@brief : 四方向二值图边缘检测
*	@note : 边缘线为白色，处于原图白色域
*			opencv 自带边缘检测存在与运算缩扩，
*			故针对二值图写此边缘检测。
*/
cv::Mat edgesbw(cv::Mat imgbw)
{
    cv::Mat img;
    imgbw.copyTo(img);
    img = cv::Scalar(IMGBW_BLACK);
    int rows, cols;

    rows = imgbw.rows;
    cols = imgbw.cols;

    int left_index;
    int right_index;
    int up_index;
    int down_index;
    int i, j;
    for (i = 0; i < rows; i++)
    {
        for (j = 0; j < cols; j++)
        {
            if (imgbw.at<uchar>(i, j) > IMGBW_THRESH_B2W)
            {
                //Index boundary check
                left_index = j > 0 ? j - 1 : j;
                right_index = j <cols-1 ? j + 1 : j;
                up_index = i > 0 ? i - 1 : i;
                down_index = i <rows - 1 ? i + 1 : i;
                //Get imgbw edge
                if (imgbw.at<uchar>(i, left_index) < IMGBW_THRESH_B2W \
                    || imgbw.at<uchar>(i, right_index) < IMGBW_THRESH_B2W\
                    || imgbw.at<uchar>(up_index, j) < IMGBW_THRESH_B2W\
                    || imgbw.at<uchar>(down_index, j) < IMGBW_THRESH_B2W)
                {
                    img.at<uchar>(i, j) = IMGBW_WITHE;
                }
            }
        }
    }
    return img;
}

/*
*	@brief : 在Mat变量中寻找第一个value值，返回其坐标
*	@note : 坐标为point类型
*
*/
cv::Point findValue(const cv::Mat &mat, uchar value)
{
    int i;
    cv::Point rad(-1,-1);
    const uchar *p;
    for (i = 0; i < mat.rows; i++)
    {
        const uchar* row = mat.ptr<uchar>(i);
        if ((p=std::find(row, row + mat.cols, value)) != row + mat.cols)
        {
            rad.y = i;
            rad.x = int(p - row);
            return rad;
        }
    }
    return rad;
}

/*
*	@brief : 在Mat变量中寻找第一个value值，返回其坐标,可以储蓄上一个点的raw值作为下一个的开始点，减少时间复杂度
*	@note : 坐标为point类型
*
*/
cv::Point findValueStore(const cv::Mat &mat, uchar value)
{
    static int lastLine = 0;
    cv::Point rad(-1, -1);
    int i;

    const uchar *p;
    for (i = lastLine; i < mat.rows; i++)
    {
        if (i == mat.rows - 1)//find in last line
            lastLine = 0;
        else
            lastLine = i;
        const uchar* row = mat.ptr<uchar>(i);

        if ((p = std::find(row, row + mat.cols, value)) != row + mat.cols)
        {
            rad.y = i;
            rad.x = int(p - row);
            return rad;
        }
    }
    return rad;
}

/*
*	@brief : S形状路线,在Mat变量中寻找第一个value值，返回其坐标,可以储蓄上一个点的raw值作为下一个的开始点，减少时间复杂度
*	@note : 坐标为point类型
*
*/
cv::Point findValueStoreS(const cv::Mat &mat, uchar value)
{
    static int lastLine = 0;
    static cv::Point lastPoint(0, 0);
    cv::Point rad(-1, -1);
    int i;
    static bool last_dir = false;
    bool now_dir = false;

    const uchar *p;
    for (i = lastLine; i < mat.rows; i++)
    {
        if (i == mat.rows - 1)  //find in last line
            lastLine = 0;
        else
            lastLine = i;

        /// Derection control
        if(i == lastPoint.y)
        {
            now_dir = last_dir;
        }
        else
        {
            if(lastPoint.x < mat.cols/2)
                now_dir = false;
            else
                now_dir = true;
        }
        last_dir = now_dir;    //update last_dir

        const uchar* row = mat.ptr<uchar>(i);
        if ((p = findDir(row, row + mat.cols, value, now_dir)) != row + mat.cols)
        {
            rad.y = i;
            rad.x = int(p - row);
            lastPoint = rad;
            return rad;
        }
    }
    return rad;
}

/*
 * @brief: 带方向参数find函数
 *         range: [first, last)
*/
template<typename T>
const T* findDir(const T* _first, const T* _last, T val, bool back)
{
    T *first, *last;
    first = (T *)_first;
    last = (T *)_last;

    while (first!=last)
    {
        if(back)
        {
            if (*(last-1)==val)
            return last-1;
            --last;
        }
        else
        {
            if (*first==val)
            return first;
            ++first;
        }
    }
    return _last;
}

/*
*	@brief :画一个实心圆
*	@note :
*
*/
cv::Mat plotc(cv::Mat& img, cv::Point rad, int r)
{
    cv::circle(img, rad, r, cv::Scalar(255, 255, 255), -1);
    return img;
}
