#include "cy_algorithm.h"
using namespace cv;


cy_algorithm::cy_algorithm(QObject *parent) : QObject(parent)
{

}

cy_algorithm::~cy_algorithm()
{

}

/**
* @brief 四方向二值图边缘检测
* @param imgbw 二值化图像
* @return 四方向边缘图像
* @note 边缘线为白色，处于原图白色域
*		opencv 自带边缘检测存在与运算缩扩，
*		故针对二值图写此边缘检测。
*/
Mat cy_algorithm::edgesbw(cv::Mat imgbw)
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
                // Index boundary check
                left_index = j > 0 ? j - 1 : j;
                right_index = j < cols - 1 ? j + 1 : j;
                up_index = i > 0 ? i - 1 : i;
                down_index = i < rows - 1 ? i + 1 : i;
                // Get imgbw edge
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

/**
* @brief 边缘减圆
* @param imgbw 边缘二值化图像
* @param r 半径 
* @return 边缘减圆结果二值图
* @note 输入为边缘二值图，白色为边缘
*/
Mat cy_algorithm::circleSub(cv::Mat& imgbw, int r)
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
* @brief 边缘减六边形
* @param imgbw 边缘二值化图像
* @param r 六边形内切圆半径
* @return 边缘减六边形结果二值图
* @note 输入为边缘二值图，白色为边缘
*/
Mat cy_algorithm::polySub(cv::Mat& imgbw, int r)
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
                plotp(img, center, r);
            }
        }
    }
    return img;
}

/*
*	@brief : 边缘减矩形
*	@note : 输入为边缘二值图，白色为边缘
*
*/
Mat cy_algorithm::rectSub(cv::Mat& imgbw, int xr, int yr)
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
                plotr(img, center, xr, yr);
            }
        }
    }
    return img;
}

/*
*	@brief : 在Mat变量中寻找第一个value值，返回其坐标,可以储蓄上一个点的raw值作为下一个的开始点，减少时间复杂度
*	@note : 坐标为point类型
*
*/
Point cy_algorithm::findValueStore(const cv::Mat &mat, uchar value)
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
*	@brief : 在Mat变量lines中寻找第一个value值，返回其坐标
*	@note : 坐标为point类型
*
*/
Point cy_algorithm::findValueLine(const cv::Mat &mat, uchar value, int lines)
{
    cv::Point rad(-1,-1);
    const uchar *p;

    const uchar* row = mat.ptr<uchar>(lines);
    if ((p=std::find(row, row + mat.cols, value)) != row + mat.cols)
    {
        rad.y = lines;
        rad.x = int(p - row);
        return rad;
    }

    return rad;
}

/*
*  @brief : 像素点排序
*  @param :
*          vin : 像素点的引用
*/
/*
    File : chongya.cpp
    Function : pointPixSort
    Changes Location:
    /// --Changes Begin
    ...
    /// --Changes End
    Note : parameters can be changed to fit expectation
    xLeftWeight=1.1, xRightWeight=1.1;
    yUpWeight=1.1, yDownWeight=2.3;
*/
QVector<Point> cy_algorithm::pointPixSort(QVector<cv::Point> &vin)
{
    if(vin.length()<=2)
        return vin;

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
            /// Parameter for tunning direction
            /// --Changes Begin
//            const double xLeftWeight=1.1, xRightWeight=1.1;
//            const double yUpWeight=1.1, yDownWeight=2.3;
            const double xLeftWeight=1.1, xRightWeight=1.1;
            const double yUpWeight=1.1, yDownWeight=2.3;

            double xDist = vin[i].x-startPoint.x;
            double yDist = vin[i].y-startPoint.y;
            xDist = xDist>0? xDist*xRightWeight : -xDist*xLeftWeight;
            yDist = yDist>0? yDist*yDownWeight : -yDist*yUpWeight;
            double dist = pow(xDist, 2)+pow(yDist, 2);
            /// --Changes End

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

/*
*  @brief : 像素点排序(用于不后退)
*  @param :
*          vin : 像素点的引用
*/
QVector<Point> cy_algorithm::pointPixForwardSort(QVector<cv::Point> &vin)
{
    if(vin.length()<=2)
        return vin;

    Point startPoint;
    QVector<cv::Point> vout(vin.length());
    QVector<cv::Point> vline(vin.length());
    startPoint = vin[0];
    static Point lastLinePoint = startPoint;
    vout.clear();
    vin.erase(&vin[0]);

    vline.clear();
    vline.append(startPoint);
    while(vin.length()>0)
    {
        if(vin[0].y == startPoint.y)    // The same line
        {
            vline.append(vin[0]);
        }
        else    // Different line
        {
            double distFisrt = pow(vline.first().x-lastLinePoint.x, 2)+pow(vline.first().y-lastLinePoint.y, 2);
            double distLast = pow(vline.last().x-lastLinePoint.x, 2)+pow(vline.last().y-lastLinePoint.y, 2);
            if(distFisrt<distLast)  //add vline
            {
                lastLinePoint = vline.last(); // startPoint update
                vout+=vline;
            }
            else // Reverse add vline
            {
                lastLinePoint = vline.first(); // startPoint update
                int vlineLen = vline.length();
                for(int i=0; i<vlineLen; i++)
                {
                    vout+=vline[vlineLen-1-i];
                }
            }
            startPoint = vin[0];    // startPoint update
            vline.clear();
            vline.append(startPoint);
        }
        vin.erase(&vin[0]);
    }

    double distFisrt = pow(vline.first().x-lastLinePoint.x, 2)+pow(vline.first().y-lastLinePoint.y, 2);
    double distLast = pow(vline.last().x-lastLinePoint.x, 2)+pow(vline.last().y-lastLinePoint.y, 2);
    if(distFisrt<distLast)  // Last line process(add)
    {
        lastLinePoint = vline.last(); // startPoint update
        vout+=vline;
    }
    else // Last line process(reverse add)
    {
        lastLinePoint = vline.first(); // startPoint update
        int vlineLen = vline.length();
        for(int i=0; i<vlineLen; i++)
        {
            vout+=vline[vlineLen-1-i];
        }
    }
    vin = vout;
    return vin;
}


/*
*  @brief : 像素点排序，按高度切片排序
*  @param :
*          vin : 像素点的引用
*          imgHeight : 图像像素高度
*          radius : 冲孔半径(或冲孔半径+间隙)
*  @return : 排序好的像素坐标
*/
QVector<cv::Point> cy_algorithm::pointPixSortHlimit(QVector<cv::Point> &vin, int imgHeight, int radius)
{
    if(vin.length()<=2)
        return vin;
////////////////////////////////////////////////////////////////
    int sliceHeight = (int)(radius*2);  ///单位切片高度
////////////////////////////////////////////////////////////////
    QVector<Point> vout;
    vout.clear();
    Point lastSlicePoint = vin[0];

    int vecLength = vin.length();
    for(int lines=0; lines<imgHeight; lines+=sliceHeight)
    {
        int begin = lines;
        int end = lines + sliceHeight;

        QVector<Point> vecSlice;
        vecSlice.clear();
        for(int i=0; i<vecLength; i++)
        {
            if((begin<=vin[i].y) && (vin[i].y<end))
            vecSlice += vin[i];
        }
        if(vecSlice.length()>0)
        {
            vecSlice.insert(0, lastSlicePoint); // make the lastSlicePoint to be the first point for sort
            vecSlice = pointPixSort(vecSlice);
            vecSlice.remove(0); // remove lastSlicePoint insert before
            lastSlicePoint = *vecSlice.end(); // Update lastSlicePoint
            vout += vecSlice;
        }
    }
    vin = vout;
    return vin;
}
/*
* @brief: 主处理
* @param:
*		img:	opencv二值化图像
*		radius: 冲孔半径
*		dist:	随边间距
*		space:	冲孔间距
*		vec:	点序列
*       overLap:重叠区域纵向像素值（必须大于等于零）
*@return:
*		<0: 函数运行错误
*		other:	冲孔个数
*/
int cy_algorithm::chongya(cv::Mat& img, int radius, int dist, int space, QVector<cv::Point> &vec, int overLap)
{
    Mat img_raw, imgEdge;
    unsigned int rows, cols;
    int numOfPoints;

    /// 图像有效性检测
    if (img.empty())
    {
        return -1;
    }
    img.copyTo(img_raw);

    /// Image concat
    static Mat img_remain, last_frameRaw, img_rawRaw;
    Mat img_cpartRem, img_cpartNow, img_cpartRemRaw,img_cpartNowRaw;


    int img_remain_h = (int)img_raw.rows/4.0;
    if(overLap>img_remain_h)
        img_remain_h = overLap;
    if(img_remain.empty())  //第一帧用空白填充连接部分
    {
        img_raw.copyTo(img_remain);
        img_raw.copyTo(last_frameRaw);
        img_remain = IMGBW_WITHE;
        last_frameRaw = IMGBW_WITHE;
        img_cpartRem = img_remain(Rect(0, img_remain.rows-img_remain_h, img_remain.cols, img_remain_h));
        img_cpartRemRaw = last_frameRaw(Rect(0, last_frameRaw.rows-img_remain_h, last_frameRaw.cols, img_remain_h));

        // 首帧后退处理
//        img_cpartRem.copyTo(cpartRemStore);
//        img_cpartRemRaw.copyTo(cpartRemStoreRaw);

        img_cpartNow = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));
        img_cpartNowRaw = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));

//        vconcat(img_cpartRem, img_cpartNow, img_raw);
//        vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
        //modified by czh
        if (img_cpartRem.cols == img_cpartNow.cols)
        {
            vconcat(img_cpartRem, img_cpartNow, img_raw);
            vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
        }
        else
        {
            img_cpartNow.copyTo(img_raw);
            img_cpartNowRaw.copyTo(img_rawRaw);
        }
    }
    else
    {
        img_cpartRem = img_remain(Rect(0, img_remain.rows-img_remain_h, img_remain.cols, img_remain_h));
        img_cpartRemRaw = last_frameRaw(Rect(0, last_frameRaw.rows-img_remain_h, last_frameRaw.cols, img_remain_h));
        // 首帧后退处理
//        img_cpartRem.copyTo(cpartRemStore);
//        img_cpartRemRaw.copyTo(cpartRemStoreRaw);

        img_cpartNow = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));
        img_cpartNowRaw = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));

//        vconcat(img_cpartRem, img_cpartNow, img_raw);
//        vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
        //modified by czh
        if (img_cpartRem.cols == img_cpartNow.cols)
        {
            vconcat(img_cpartRem, img_cpartNow, img_raw);
            vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
        }
        else
        {
            img_cpartNow.copyTo(img_raw);
            img_cpartNowRaw.copyTo(img_rawRaw);
        }
    }
    img_raw.copyTo(img_remain);
    img_raw.copyTo(last_frameRaw);

    // Debug
    Mat img_remain_Debug;
    img_remain.copyTo(img_remain_Debug);
//    imshow("img_remain", img_remain);
    // Debug End

//    // Debug
//    imshow("img_remain", img_remain);
//    // Debug End

    /// 显示图像信息
    rows = img_raw.rows;
    cols = img_raw.cols;

    /// 随边处理
    Mat img_rawRawEdge = edgesbw(img_rawRaw);
    img_rawRawEdge.col(0) = IMGBW_WITHE;
    img_rawRawEdge.col(cols - 1) = IMGBW_WITHE;
    img_rawRaw = img_rawRaw | circleSub(img_rawRawEdge, dist);

    /// 随边与冲孔图像整合
    img_raw = img_raw | img_rawRaw;

    /// 得到整合图像边缘
    imgEdge = edgesbw(img_raw);

    /// 加边框边界，若不需要，移除以下四条语句
    imgEdge.row(0) = IMGBW_WITHE;
    imgEdge.row(rows - 1) = IMGBW_WITHE;
    imgEdge.col(0) = IMGBW_WITHE;
    imgEdge.col(cols - 1) = IMGBW_WITHE;

    /// 得到首个圆心区域
    img_raw = img_raw | circleSub(imgEdge, radius+space);

    /// 主处理循环，获得像素坐标点集合
    Point rad;
    vec.clear();
    for (int i = 0; i < CY_MAXSTEP; ++i)
    {
        /// 检测是否存在有效圆心区域，
        /// 若存在，获取一像素坐标；若不存在，退出处理循环
        rad = findValueStore(img_raw, IMGBW_BLACK);
        if (rad.x < 0 || rad.y < 0) break;

        /// 像素坐标数组
        vec.push_back(rad);

        /// 减圆操作 (半径为：2r+delta)
        plotc(img_raw, rad, 2 * radius + space);
        imgEdge = edgesbw(img_raw);

        /// Image concat
        plotc(img_remain, rad, radius);
    }

    ///得到点集有效个数
    numOfPoints = vec.size();

    ///冲压顺序排序（像素点）
//    vec = pointPixSort(vec);
    vec = pointPixSortHlimit(vec, img_raw.rows, radius);

    /// Image concat ,Point modify
    for(int j=0; j<numOfPoints; j++)
    {
        vec[j].y -= img_remain_h;   //remain concat modify
        vec[j].y += overLap;    // overlap modify
    }

    // Debug
    ///结果显示 1
    //img = img_remain_Debug;
//    for(int j=0; j<numOfPoints; j++)
//    {
//        /// 坐标映射与输出
//        std::cout << vec[j] << std::endl;
//        /// 冲压结果画圆
//        Point tmp_Point(vec[j]);
//        tmp_Point.y -= overLap;
//        tmp_Point.y += img_remain_h;
//        cv::circle(img, tmp_Point, radius, cv::Scalar(255, 255, 255), 1);
//        /// 顺序显示
//        cv::putText(img, String(std::to_string(j+1)), tmp_Point-cv::Point(radius/5.0, -radius/5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(255, 0, 0), 1, cv::LINE_AA);
//    }
//	img.copyTo(tmp_img);
    // Debug end

//    // Debug 结果显示 2
//    ///结果显示
//    for(int j=0; j<numOfPoints; j++)
//    {
//        /// 坐标映射与输出
//        std::cout << vec[j] << std::endl;
//        /// 冲压结果画圆
//        cv::circle(img, vec[j], radius, cv::Scalar(255, 255, 255), 1);
//        /// 顺序显示
//        cv::putText(img, String(std::to_string(j+1)), vec[j]-cv::Point(radius/5.0, -radius/5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(255, 0, 0), 1, cv::LINE_AA);
//    }
//    // Debug end

    return numOfPoints;
}


typedef struct MRCYU_TAG
{
    QVector<cv::Point> vec;
    int maxNum;
    int lineInd;
} MRCYU;


/*
* @brief: 圆形主处理(不后退)--随边智能横排
* @param:
*		img:	opencv二值化图像
*		radius: 冲孔半径
*		dist:	随边间距
*		space:	冲孔间距
*		vec:	点序列
*       overLap:重叠区域纵向像素值（必须大于等于零）
*@return:
*		<0: 函数运行错误
*		other:	冲孔个数
*/
/*
* 修改记录
* build180126:修改圆形智能横排排孔BUG，防止非拼接区域一排孔只排几个
*/
int cy_algorithm::chongyaFowardCircleSmartHorizontal(cv::Mat& img, int radius, int dist, int space, QVector<cv::Point> &vec, int overLap)
{
    Mat img_raw, imgEdge;
    unsigned int rows, cols;
    int numOfPoints;
    Mat cpartRemStore, cpartRemStoreRaw;   //opencv 很多图像操作都是对同一块内存的，为了存储中间过程的图像，要用copyTo开一块新的内存保留

    /// 图像有效性检测
    if (img.empty())
    {
        return -1;
    }
    img.copyTo(img_raw);

    /// Image concat
    static Mat img_remain, last_frameRaw, img_rawRaw;
    Mat img_cpartRem, img_cpartNow, img_cpartRemRaw,img_cpartNowRaw;
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    int img_remain_h = (int)img_raw.rows/3.0;   ///拼接高度，如果要保证半径为r绝对不出错，h>2r+1,否则小概率出错,当h=图像高度，绝对不出错，但是计算很大
//modified by czh 20170615
    int img_remain_h = (int)(radius+space)*2.5;
    int img_remain_h_without_overlap = img_remain_h;
    int img_remain_h_limit = (int)(img_raw.rows);
    if(img_remain_h > img_remain_h_limit)
        return -1;
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(overLap>img_remain_h)
        img_remain_h = overLap;
    if(img_remain.empty())  //第一帧用空白填充连接部分
    {
        img_raw.copyTo(img_remain);
        img_raw.copyTo(last_frameRaw);
        img_remain = IMGBW_WITHE;
        last_frameRaw = IMGBW_WITHE;
        img_cpartRem = img_remain(Rect(0, img_remain.rows-img_remain_h, img_remain.cols, img_remain_h));
        img_cpartRemRaw = last_frameRaw(Rect(0, last_frameRaw.rows-img_remain_h, last_frameRaw.cols, img_remain_h));

        // 首帧后退处理pointPixForwardSort(x)
        img_cpartRem.copyTo(cpartRemStore);
        img_cpartRemRaw.copyTo(cpartRemStoreRaw);

        img_cpartNow = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));
        img_cpartNowRaw = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));

        if (img_cpartRem.cols == img_cpartNow.cols)
        {
            vconcat(img_cpartRem, img_cpartNow, img_raw);
            vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
        }
        else
        {
            if (img_cpartRem.cols == img_cpartNow.cols)
            {
                vconcat(img_cpartRem, img_cpartNow, img_raw);
                vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
            }
            else
            {
                img_cpartNow.copyTo(img_raw);
                img_cpartNowRaw.copyTo(img_rawRaw);
            }
        }
    }
    else
    {
        img_cpartRem = img_remain(Rect(0, img_remain.rows-img_remain_h, img_remain.cols, img_remain_h));
        img_cpartRemRaw = last_frameRaw(Rect(0, last_frameRaw.rows-img_remain_h, last_frameRaw.cols, img_remain_h));
        // 首帧后退处理(x)
        img_cpartRem.copyTo(cpartRemStore);
        img_cpartRemRaw.copyTo(cpartRemStoreRaw);

        img_cpartNow = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));
        img_cpartNowRaw = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));

        if (img_cpartRem.cols == img_cpartNow.cols)
        {
            vconcat(img_cpartRem, img_cpartNow, img_raw);
            vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
        }
        else
        {
            img_cpartNow.copyTo(img_raw);
            img_cpartNowRaw.copyTo(img_rawRaw);
        }
    }
    img_raw.copyTo(img_remain);
    img_raw.copyTo(last_frameRaw);

    // Debug
    ///结果显示 1
    ///delete by czh 20170711
    Mat img_remain_Debug;
    img_remain.copyTo(img_remain_Debug);
//    imshow("img_remain", img_remain);
    // Debug End

    /// 显示图像信息
    rows = img_raw.rows;
    cols = img_raw.cols;

    /// 随边处理
    Mat img_rawRawEdge = edgesbw(img_rawRaw);
    img_rawRawEdge.col(0) = IMGBW_WITHE;
    img_rawRawEdge.col(cols - 1) = IMGBW_WITHE;
    img_rawRaw = img_rawRaw | circleSub(img_rawRawEdge, dist);

    /// 随边与冲孔图像整合
    img_raw = img_raw | img_rawRaw;

    /// 得到整合图像边缘
    imgEdge = edgesbw(img_raw);

    /// 加边框边界，若不需要，移除以下四条语句
    imgEdge.row(0) = IMGBW_WITHE;
    imgEdge.row(rows - 1) = IMGBW_WITHE;
    imgEdge.col(0) = IMGBW_WITHE;
    imgEdge.col(cols - 1) = IMGBW_WITHE;

    /// 得到首个圆心区域
    img_raw = img_raw | circleSub(imgEdge, radius);

    // Debug
//    imshow("centerArea", img_raw);
    // Debug End

    /// 主处理循环，获得像素坐标点集合
    Point rad;
    vec.clear();
    int num = 0;
    int rowStart;

    ///减去上一帧最后两行得到顶部圆心区域
    static QVector<cv::Point> lastFramelastRVec;
    Point lastFramePoint = Point(cols, rows);
    if(lastFramelastRVec.length()>0)
    {
        lastFramePoint = lastFramelastRVec.last();
        for(int k=0; k<lastFramelastRVec.length(); k++)
        {
            lastFramelastRVec[k].y = img_remain_h-lastFramelastRVec[k].y;
            plotc(img_raw, lastFramelastRVec[k], 2*radius+space);
        }
    }
    // Debug
//    imshow("centerArea", img_raw);
    // Debug End

    /// 连接图像扫描首行值
    rowStart = img_remain_h-lastFramePoint.y+(radius+space*0.5f)*1.732f+1;

    //{
    /// 补足前一帧最后一行冲孔
    int lastFrameLastrow = img_remain_h-lastFramePoint.y+2;
    if(lastFrameLastrow > 0)
    {
        for(int j=0; j<(int)cols;) //Col scan
        {
            if(img_raw.at<uchar>(lastFrameLastrow, j)==IMGBW_BLACK)
            {
                rad.x = j;
                rad.y = lastFrameLastrow;
                ///////////////////////////////
//                rad.y = lastFrameLastrow+1;//
                //////////////////////////////
                vec.append(rad);
                //modified by czh 20170711
                /// 拼接图冲孔
                plotc(img_remain, rad, radius);
                /// 得到新的圆心区域
                plotc(img_raw, rad, 2*radius+space);

                j += 2*radius+space;
            }
            else
                j += 1;
        }
    }
    //} Modified by Duan20170710

    if(rowStart < 0)
    {
        rowStart = 0;
    }

    /// 冲压处理
    MRCYU mrpoints;
    mrpoints.vec.clear();

    //{
    QVector<cv::Point> pointsVecBuf;
    pointsVecBuf.clear();
    //} Modified by Duan20170710

    for (int i = rowStart; i < (int)rows;)   // Row scan
    {
        rad = findValueLine(img_raw, IMGBW_BLACK, i);

        if (rad.x<0 || rad.y<0)
        {
            i++;
        }
        else
        {
            bool existValidPoint = false;
            int linePoints = 0;

            //在ScanRange内扫描，得到最大行冲压单元
            ////////////////////////////////////////////////
            int ScanRange = (int)(0.3f*(radius+space*0.5f));   ///扫描范围, 0.268f=2-sqrt(3),系数大于这个值可能会在最多相切与行冲点最多之间产生抉择，但是大于0.268f小于1的效果时比较好的;
            ///////////////////////////////////////////////////

            //{
            pointsVecBuf = mrpoints.vec;
            //} Modified by Duan20170710
            mrpoints.vec.clear();
            mrpoints.maxNum = mrpoints.vec.length();
            mrpoints.lineInd = i;

            for(int r_index=i; (r_index<rows)&&(r_index<i+ScanRange); r_index++)// Scan Range
            {
                QVector<cv::Point> tmpvec;
                tmpvec.clear();

//                // Debug
//                Mat tmp_raw_img;
//                img_raw.copyTo(tmp_raw_img);
//                // Debug end
                for(int j=0; j<(int)cols;) //Col scan
                {
                    if(img_raw.at<uchar>(r_index, j)==IMGBW_BLACK)
                    {
                        existValidPoint = true;
                        /// 像素坐标数组
                        rad.x = j;
                        rad.y = r_index;
                        tmpvec.append(rad);
                        j += 2*radius+space;

//                        // Debug
//                        circle(tmp_raw_img, rad, radius, cv::Scalar(0, 255, 0), 1);
//                        // Debug end
                    }
                    else
                    {
                        j += 1;
                    }
                }
//                // Debug end
//                cv::putText(tmp_raw_img, String(std::to_string(r_index)), cv::Point(20, 20), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(0, 255, 0), 1, cv::LINE_AA);
//                imshow("debug_img_raw", tmp_raw_img);
//                waitKey(0);
//                // Debug end

                //更新最大行冲压单元
                if(tmpvec.length()>mrpoints.maxNum)
                {
                    mrpoints.maxNum = tmpvec.length();
                    mrpoints.lineInd = r_index;
                    mrpoints.vec = tmpvec;
                }
                /// If the points number is bigger than CY_maxStep then stop
                if(num+mrpoints.maxNum>=CY_MAXSTEP) break;
            }

            ///以行最后一个点为起点反冲一次，避免大间距
            if(mrpoints.maxNum>0)
            {
                Point lineLastPoint = mrpoints.vec.last();
                Point resort_rad;
                QVector<cv::Point> tmpvec;
                for(int j=lineLastPoint.x; j>0; j -= 2*radius+space) //Col scan
                {
                    if(img_raw.at<uchar>(lineLastPoint.y, j)==IMGBW_BLACK)
                    {
                        /// 像素坐标数组
                        resort_rad.x = j;
                        resort_rad.y = lineLastPoint.y;
                        tmpvec.append(resort_rad);
                    }
                }
                //如果反冲数量不减少，则最大行冲压单元更新为反冲结果，以此避免大间距
                if(tmpvec.length()>=mrpoints.maxNum)
                {
                    mrpoints.maxNum = tmpvec.length();
                    mrpoints.lineInd = lineLastPoint.y;
                    mrpoints.vec = tmpvec;
                }
            }
            ///更新特征量
            linePoints = mrpoints.maxNum;
            num += linePoints;
            i = mrpoints.lineInd;
            vec += mrpoints.vec;

            ///拼接图冲孔和当前行去圆形区域
            for(int j=0; j<mrpoints.maxNum; j++)
            {
                /// 拼接图冲孔
                plotc(img_remain, mrpoints.vec[j], radius);
                /// 得到新的圆心区域
                plotc(img_raw, mrpoints.vec[j], 2*radius+space);
            }

            /// 更新下一行
            if(existValidPoint)
            {
                i += (radius+space*0.5f)*1.732f;   //sqr(3)=1.732f
            }
            else
            {
                i += 1;
            }
            //Modified by Duan20171228
            if(rows-i+radius+(int)(space/2) < img_remain_h_without_overlap)
                break;
        }
    }

    ///得到点集有效个数
    numOfPoints = vec.size();

    ///更新一帧最后一行冲孔
    if(numOfPoints>0)
    {
        //{
        lastFramelastRVec = pointsVecBuf + mrpoints.vec;
        //} Modified by Duan20170710
        for(int k=0; k<lastFramelastRVec.length(); k++)
        {
            lastFramelastRVec[k].y = rows-lastFramelastRVec[k].y;
        }
    }
    else
        lastFramelastRVec.clear();

    ///冲压顺序排序（像素点）
    vec = pointPixForwardSort(vec);

    /// 图像拼接与最后一行冲点坐标修正
    for(int j=0; j<numOfPoints; j++)
    {
        vec[j].y -= img_remain_h;   //remain concat modify
        vec[j].y += overLap;    // overlap modify
    }

    // Debug
    ///结果显示 1
    img = img_remain_Debug;

    ////Debug for remain
//    int stop_line = rows + radius + (int)(space / 2) - img_remain_h_without_overlap;
//    cv::line(img, Point(0, stop_line), Point(cols, stop_line), cv::Scalar(255, 255, 255), 1);
//    int remain_line = rows - img_remain_h;
//    cv::line(img, Point(0, remain_line), Point(cols, remain_line), cv::Scalar(255, 255, 255), 1);
//    std::stringstream ss1;
//    ss1 << overLap;
//    String tmp_str1 = "overLap:" + ss1.str();
//    cv::putText(img, tmp_str1, Point(2, cols / 15), CV_FONT_HERSHEY_COMPLEX, radius*2.0 / CY_R_MAX, Scalar(0, 0, 0), 1);
//    std::stringstream ss2;
//    ss2 << img_remain_h;
//    String tmp_str2 = "h1:" + ss2.str();
//    cv::putText(img, tmp_str2, Point(2, cols / 9), CV_FONT_HERSHEY_COMPLEX, radius*2.0 / CY_R_MAX, Scalar(0, 0, 0), 1);
//    std::stringstream ss3;
//    ss3 << img_remain_h_without_overlap - radius - (int)(space / 2);
//    String tmp_str3 = "h2:" + ss3.str();
//    cv::putText(img, tmp_str3, Point(2, cols / 8), CV_FONT_HERSHEY_COMPLEX, radius*2.0 / CY_R_MAX, Scalar(0, 0, 0), 1);


    for(int j=0; j<numOfPoints; j++)
    {
        /// 坐标映射与输出
        std::cout << vec[j] << std::endl;
        /// 冲压结果画圆
        Point tmp_Point(vec[j]);
        tmp_Point.y -= overLap;
        tmp_Point.y += img_remain_h;
        cv::circle(img, tmp_Point, radius, cv::Scalar(255, 255, 255), 1);
        /// 顺序显示
        //modified by czh 20170829 to fit qt in linux
        //cv::putText(img, String(std::to_string(j+1)), tmp_Point-cv::Point(radius/5.0, -radius/5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(255, 0, 0), 1, cv::LINE_AA);
        std::stringstream ss;
        ss << j+1;
        cv::putText(img, cv::String(ss.str()), tmp_Point-cv::Point(radius/5.0, -radius/5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(255, 0, 0), 1);
    }
    img.copyTo(tmp_img);
    // Debug end

//    // Debug
//    ///结果显示 2
//    for(int j=0; j<numOfPoints; j++)
//    {
//        /// 坐标映射与输出
//        std::cout << vec[j] << std::endl;
//        /// 冲压结果画圆
//        cv::circle(img, vec[j], radius, cv::Scalar(255, 255, 255), 1);
//        /// 顺序显示
//        cv::putText(img, String(std::to_string(j+1)), vec[j]-cv::Point(radius/5.0, -radius/5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(255, 0, 0), 1, cv::LINE_AA);
//    }
//    // Debug end


    return numOfPoints;
}

/*
* @brief: 圆形主处理(不后退)--普通三角形排法（增加新料片顶部扫描）
* @param:
*		img:	opencv二值化图像
*		radius: 冲孔半径
*		dist:	随边间距
*		space:	冲孔间距
*		vec:	点序列
*       overLap:重叠区域纵向像素值（必须大于等于零）
*@return:
*		<0: 函数运行错误
*		other:	冲孔个数
*/
int cy_algorithm::chongyaFowardCircle(cv::Mat& img, int radius, int dist, int space, QVector<cv::Point> &vec, int overLap)
{
    Mat img_raw, imgEdge;
    unsigned int rows, cols;
    int numOfPoints;
    bool firstFrameLine = true;
    Mat cpartRemStore, cpartRemStoreRaw;   //opencv 很多图像操作都是对同一块内存的，为了存储中间过程的图像，要用copyTo开一块新的内存保留

    /// 图像有效性检测
    if (img.empty())
    {
        return -1;
    }
    img.copyTo(img_raw);

    /// Image concat
    static Mat img_remain, last_frameRaw, img_rawRaw;
    Mat img_cpartRem, img_cpartNow, img_cpartRemRaw,img_cpartNowRaw;
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    int img_remain_h = (int)img_raw.rows/3.0;   ///拼接高度，如果要保证半径为r绝对不出错，h>2r+1,否则小概率出错,当h=图像高度，绝对不出错，但是计算很大
//modified by czh 20170615
    int img_remain_h = (int)(radius+space)*2.2;
    int img_remain_h_limit = (int)(img_raw.rows);
    if(img_remain_h > img_remain_h_limit)
        return -1;
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(overLap>img_remain_h)
        img_remain_h = overLap;
    if(img_remain.empty())  //第一帧用空白填充连接部分
    {
        img_raw.copyTo(img_remain);
        img_raw.copyTo(last_frameRaw);
        img_remain = IMGBW_WITHE;
        last_frameRaw = IMGBW_WITHE;
        img_cpartRem = img_remain(Rect(0, img_remain.rows-img_remain_h, img_remain.cols, img_remain_h));
        img_cpartRemRaw = last_frameRaw(Rect(0, last_frameRaw.rows-img_remain_h, last_frameRaw.cols, img_remain_h));

        // 首帧后退处理pointPixForwardSort
        img_cpartRem.copyTo(cpartRemStore);
        img_cpartRemRaw.copyTo(cpartRemStoreRaw);

        img_cpartNow = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));
        img_cpartNowRaw = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));

//        vconcat(img_cpartRem, img_cpartNow, img_raw);
//        vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
        //modified by czh
        if (img_cpartRem.cols == img_cpartNow.cols)
        {
            vconcat(img_cpartRem, img_cpartNow, img_raw);
            vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
        }
        else
        {
//            img_cpartNow.copyTo(img_raw);
//            img_cpartNowRaw.copyTo(img_rawRaw);
            //modified by czh
            if (img_cpartRem.cols == img_cpartNow.cols)
            {
                vconcat(img_cpartRem, img_cpartNow, img_raw);
                vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
            }
            else
            {
                img_cpartNow.copyTo(img_raw);
                img_cpartNowRaw.copyTo(img_rawRaw);
            }
        }
    }
    else
    {
        img_cpartRem = img_remain(Rect(0, img_remain.rows-img_remain_h, img_remain.cols, img_remain_h));
        img_cpartRemRaw = last_frameRaw(Rect(0, last_frameRaw.rows-img_remain_h, last_frameRaw.cols, img_remain_h));
        // 首帧后退处理
        img_cpartRem.copyTo(cpartRemStore);
        img_cpartRemRaw.copyTo(cpartRemStoreRaw);

        img_cpartNow = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));
        img_cpartNowRaw = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));

        if (img_cpartRem.cols == img_cpartNow.cols)
        {
            vconcat(img_cpartRem, img_cpartNow, img_raw);
            vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
        }
        else
        {
            img_cpartNow.copyTo(img_raw);
            img_cpartNowRaw.copyTo(img_rawRaw);
        }
    }
    img_raw.copyTo(img_remain);
    img_raw.copyTo(last_frameRaw);

    // 首帧后退处理,检测第一帧
    firstFrameLine = true;
    for(int i=0; i<cpartRemStore.rows; i++)
    {
        Point rad;
        rad = findValueLine(cpartRemStore, IMGBW_BLACK, i);

        if (rad.x>=0 && rad.y>=0)
        {
            firstFrameLine = false;
            break;
        }
    }

    // Debug
    ///结果显示 1
    Mat img_remain_Debug;
    img_remain.copyTo(img_remain_Debug);
//    imshow("img_remain", img_remain);
    // Debug End

    /// 显示图像信息
    rows = img_raw.rows;
    cols = img_raw.cols;

    /// 随边处理
    Mat img_rawRawEdge = edgesbw(img_rawRaw);
    img_rawRawEdge.col(0) = IMGBW_WITHE;
    img_rawRawEdge.col(cols - 1) = IMGBW_WITHE;
    img_rawRaw = img_rawRaw | circleSub(img_rawRawEdge, dist);

    /// 随边与冲孔图像整合
    img_raw = img_raw | img_rawRaw;

    /// 得到整合图像边缘
    imgEdge = edgesbw(img_raw);

    /// 加边框边界，若不需要，移除以下四条语句
    imgEdge.row(0) = IMGBW_WITHE;
    imgEdge.row(rows - 1) = IMGBW_WITHE;
    imgEdge.col(0) = IMGBW_WITHE;
    imgEdge.col(cols - 1) = IMGBW_WITHE;

    /// 得到首个圆心区域
    img_raw = img_raw | circleSub(imgEdge, radius);

//    // Debug
//    imshow("centerArea", img_raw);
//    // Debug End

    /// 主处理循环，获得像素坐标点集合
    Point rad;
    vec.clear();
    int num = 0;
    static int dir = 0;
    int rowStart;
    static Point lastFramePoint = Point(cols, rows);

    /// 连接图像扫描首行值
    rowStart = img_remain_h-lastFramePoint.y+(radius+space*0.5f)*1.732f+1;
    if(rowStart < 0)
    {
        rowStart = 0;
    }

    /// 冲压处理
    //{
    static int fistLineCols = 0;
    //} //modified by Duan@20170616
    for (int i = rowStart; i < (int)rows;)   // Row scan
    {
        rad = findValueLine(img_raw, IMGBW_BLACK, i);

        if (rad.x<0 || rad.y<0)
        {
            i++;
            // 检测到整行白色，标志接下来将是新料片头部
            firstFrameLine = true;

        }
        else
        {
            int colStart=0;
            //{
            if(dir)
//                colStart = radius+ dist+1;
                colStart = fistLineCols%(2*radius+space) + 1;
            else
//                colStart = 2*radius+space*0.5f+ dist+1;
                colStart = radius+space*0.5f+ fistLineCols%(2*radius+space) + 1;
            //} //modified by Duan@20170616

            bool existValidPoint = false;
            int linePoints = 0;
            for(int j=colStart; j<(int)cols; j+=2*radius+space) //Col scan
            {
                if(img_raw.at<uchar>(i, j)==IMGBW_BLACK)
                {
                    existValidPoint = true;
                    /// 像素坐标数组
                    rad.x = j;
                    vec.append(rad);
                    num++;
                    linePoints++;

                    /// Image concat
                    if(!firstFrameLine)
                        plotc(img_remain, rad, radius);

                    /// If the points number is bigger than CY_maxStep then stop
                    if(num>=CY_MAXSTEP) break;
                }
            }
            /// Next Line
            if(existValidPoint)
            {
                // 首帧后退处理--扫描新料片顶部
                if(firstFrameLine)
                {
                    //{
                    ////////////////////////////////////////////
                    MRCYU mrpoints;
                    //在ScanRange内扫描，得到最大行冲压单元
                    ////////////////////////////////////////////////
                    int ScanRange = (int)(0.35f*(radius+space*0.5f));   ///扫描范围, 0.268f=2-sqrt(3),系数大于这个值可能会在最多相切与行冲点最多之间产生抉择，但是大于0.268f小于1的效果时比较好的;
                    ///////////////////////////////////////////////////
                    mrpoints.vec.clear();
                    mrpoints.maxNum = 0;
                    mrpoints.lineInd = i;

                    for(int r_index=i; (r_index<rows)&&(r_index<i+ScanRange); r_index++)// Scan Range
                    {
                        int fl_pointsNum = 0;
                        for(int j=0; j<(int)cols;j += 2*radius+space) //Col scan
                        {
                            if(img_raw.at<uchar>(r_index, j)==IMGBW_BLACK)
                            {
                                fl_pointsNum++;
                            }
                        }
                        //更新最大行冲压单元
                        if(fl_pointsNum>mrpoints.maxNum)
                        {
                            mrpoints.maxNum = fl_pointsNum;
                            mrpoints.lineInd = r_index;
                        }
                    }
                    i = mrpoints.lineInd;     /// 首行扫描跳转

                    //移除之前加入队列的料片头部冲点
                    for(int k=0; k<linePoints; k++)
                    {
                        vec.removeLast();
                        num--;
                    }
                    firstFrameLine = false;


                    Point fistLinePoint = findValueLine(img_raw, IMGBW_BLACK, i);
                    fistLineCols = fistLinePoint.x<0? fistLineCols:fistLinePoint.x;
                    dir=0;
                    //} //modified by Duan@20170711
                }
                else
                    i += (radius+space*0.5f)*1.732f;   //sqr(3)=1.732f

                dir = !dir;
            }
            else
                i += 1;
        }
    }

    ///得到点集有效个数
    numOfPoints = vec.size();

    ///更新LastPoint
    if(numOfPoints>0)
    {
        lastFramePoint = vec.last();
        lastFramePoint.y = rows-lastFramePoint.y;
    }
    else
        lastFramePoint = Point(cols, rows);

    ///冲压顺序排序（像素点）
    vec = pointPixForwardSort(vec);

    /// Image concat ,Point modify
    for(int j=0; j<numOfPoints; j++)
    {
        vec[j].y -= img_remain_h;   //remain concat modify
        vec[j].y += overLap;    // overlap modify
    }


    // Debug
    ///结果显示 1
    img = img_remain_Debug;
    for(int j=0; j<numOfPoints; j++)
    {
        /// 坐标映射与输出
        std::cout << vec[j] << std::endl;
        /// 冲压结果画圆
        Point tmp_Point(vec[j]);
        tmp_Point.y -= overLap;
        tmp_Point.y += img_remain_h;
        cv::circle(img, tmp_Point, radius, cv::Scalar(255, 255, 255), 1);
        /// 顺序显示
        //cv::putText(img, String(std::to_string(j+1)), tmp_Point-cv::Point(radius/5.0, -radius/5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(255, 0, 0), 1, cv::LINE_AA);
        //modified by czh 20170829 to fit qt in linux
        //cv::putText(img, String(std::to_string(j+1)), tmp_Point-cv::Point(radius/5.0, -radius/5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(255, 0, 0), 1, cv::LINE_AA);
        std::stringstream ss;
        ss << j+1;
        cv::putText(img, cv::String(ss.str()), tmp_Point-cv::Point(radius/5.0, -radius/5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(255, 0, 0), 1);
    }
    img.copyTo(tmp_img);
    // Debug end

//    // Debug
//    ///结果显示 2
//    for(int j=0; j<numOfPoints; j++)
//    {
//        /// 坐标映射与输出
//        std::cout << vec[j] << std::endl;
//        /// 冲压结果画圆
//        cv::circle(img, vec[j], radius, cv::Scalar(255, 255, 255), 1);
//        /// 顺序显示
//        cv::putText(img, String(std::to_string(j+1)), vec[j]-cv::Point(radius/5.0, -radius/5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(255, 0, 0), 1, cv::LINE_AA);
//    }
//    // Debug end


    return numOfPoints;
}


/*
* @brief: 六边形主处理(不后退)--随边智能横排
* @param:
*		img:	opencv二值化图像
*		radius: 冲孔半径
*		dist:	随边间距
*		space:	冲孔间距
*		vec:	点序列
*       overLap:重叠区域纵向像素值（必须大于等于零）
*@return:
*		<0: 函数运行错误
*		other:	冲孔个数
*/
int cy_algorithm::chongyaFowardPolySmartHorizontal(cv::Mat& img, int radius, int dist, int space, QVector<cv::Point> &vec, int overLap)
{
    Mat img_raw, imgEdge;
    unsigned int rows, cols;
    int numOfPoints;
    Mat cpartRemStore, cpartRemStoreRaw;   //opencv 很多图像操作都是对同一块内存的，为了存储中间过程的图像，要用copyTo开一块新的内存保留

    /// 图像有效性检测
    if (img.empty())
    {
        return -1;
    }
    img.copyTo(img_raw);

    /// Image concat
    static Mat img_remain, last_frameRaw, img_rawRaw;
    Mat img_cpartRem, img_cpartNow, img_cpartRemRaw,img_cpartNowRaw;
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    int img_remain_h = (int)img_raw.rows/3.0;   ///拼接高度，如果要保证半径为r绝对不出错，h>2r+1,否则小概率出错,当h=图像高度，绝对不出错，但是计算很大
    int img_remain_h = (int)(radius+space)*3.5; //拼接区域倍数要用外切圆半径的2.2，为了简化计算，直接用内切圆的3.5倍做拼接高度 modified by czh 20170711
    int img_remain_h_limit = (int)(img_raw.rows);
    if(img_remain_h > img_remain_h_limit)
        return -1;
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(overLap>img_remain_h)
        img_remain_h = overLap;
    if(img_remain.empty())  //第一帧用空白填充连接部分
    {
        img_raw.copyTo(img_remain);
        img_raw.copyTo(last_frameRaw);
        img_remain = IMGBW_WITHE;
        last_frameRaw = IMGBW_WITHE;
        img_cpartRem = img_remain(Rect(0, img_remain.rows-img_remain_h, img_remain.cols, img_remain_h));
        img_cpartRemRaw = last_frameRaw(Rect(0, last_frameRaw.rows-img_remain_h, last_frameRaw.cols, img_remain_h));

        // 首帧后退处理
        img_cpartRem.copyTo(cpartRemStore);
        img_cpartRemRaw.copyTo(cpartRemStoreRaw);

        img_cpartNow = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));
        img_cpartNowRaw = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));

        vconcat(img_cpartRem, img_cpartNow, img_raw);
        vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
    }
    else
    {
        img_cpartRem = img_remain(Rect(0, img_remain.rows-img_remain_h, img_remain.cols, img_remain_h));
        img_cpartRemRaw = last_frameRaw(Rect(0, last_frameRaw.rows-img_remain_h, last_frameRaw.cols, img_remain_h));
        // 首帧后退处理
        img_cpartRem.copyTo(cpartRemStore);
        img_cpartRemRaw.copyTo(cpartRemStoreRaw);

        img_cpartNow = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));
        img_cpartNowRaw = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));

        vconcat(img_cpartRem, img_cpartNow, img_raw);
        vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
    }
    img_raw.copyTo(img_remain);
    img_raw.copyTo(last_frameRaw);

    // Debug
    ///结果显示 1
    Mat img_remain_Debug;
    img_remain.copyTo(img_remain_Debug);
//    imshow("img_remain", img_remain);
    // Debug End

    /// 显示图像信息
    rows = img_raw.rows;
    cols = img_raw.cols;

    /// 随边处理
    Mat img_rawRawEdge = edgesbw(img_rawRaw);
    img_rawRawEdge.col(0) = IMGBW_WITHE;
    img_rawRawEdge.col(cols - 1) = IMGBW_WITHE;
    img_rawRaw = img_rawRaw | polySub(img_rawRawEdge, dist);

    /// 随边与冲孔图像整合
    img_raw = img_raw | img_rawRaw;

    /// 得到整合图像边缘
    imgEdge = edgesbw(img_raw);

    /// 加边框边界，若不需要，移除以下四条语句
    imgEdge.row(0) = IMGBW_WITHE;
    imgEdge.row(rows - 1) = IMGBW_WITHE;
    imgEdge.col(0) = IMGBW_WITHE;
    imgEdge.col(cols - 1) = IMGBW_WITHE;

    /// 得到首个圆心区域
    img_raw = img_raw | polySub(imgEdge, radius);

//    // Debug
//    imshow("centerArea", img_raw);
//    // Debug End

    /// 主处理循环，获得像素坐标点集合
    Point rad;
    vec.clear();
    int num = 0;
    int rowStart;

    ///减去上一帧最后两行得到顶部六边形区域
    static QVector<cv::Point> lastFramelastRVec;
    Point lastFramePoint = Point(cols, rows);
    if(lastFramelastRVec.length()>0)
    {
        lastFramePoint = lastFramelastRVec.last();
        for(int k=0; k<lastFramelastRVec.length(); k++)
        {
            lastFramelastRVec[k].y = img_remain_h-lastFramelastRVec[k].y;
            plotp(img_raw, lastFramelastRVec[k], 2*radius+space);
        }
    }
    // Debug
//    imshow("centerArea", img_raw);
    // Debug End

    /// 连接图像扫描首行值
    rowStart = img_remain_h-lastFramePoint.y+(radius+space*0.5)*1.732f+3;

    //{
    /// 补足前一帧最后一行冲孔
    int lastFrameLastrow = img_remain_h-lastFramePoint.y+4;
    if(lastFrameLastrow > 0)
    {
        for(int j=0; j<(int)cols;) //Col scan
        {
            if(img_raw.at<uchar>(lastFrameLastrow, j)==IMGBW_BLACK)
            {
                rad.x = j;
                rad.y = lastFrameLastrow+1;
                vec.append(rad);
                //modified by czh 20170711
                /// 拼接图冲孔
                plotp(img_remain, rad, radius);
                /// 得到新的圆心区域
                plotp(img_raw, rad, 2*radius+space);

                j += 2*radius+space;
            }
            else
                j += 1;
        }
    }
    //} Modified by Duan20170710

    if(rowStart < 0)
    {
        rowStart = 0;
    }

    /// 冲压处理
    MRCYU mrpoints;
    mrpoints.vec.clear();

    //{
    QVector<cv::Point> pointsVecBuf;
    pointsVecBuf.clear();
    //} Modified by Duan20170710

    for (int i = rowStart; i < (int)rows;)   // Row scan
    {
        rad = findValueLine(img_raw, IMGBW_BLACK, i);

        if (rad.x<0 || rad.y<0)
        {
            i++;
        }
        else
        {
            bool existValidPoint = false;
            int linePoints = 0;

            //在ScanRange内扫描，得到最大行冲压单元
            ////////////////////////////////////////////////
            int ScanRange = (int)(0.3f*(radius+space*0.5f));   ///扫描范围, 0.268f=2-sqrt(3),系数大于这个值可能会在最多相切与行冲点最多之间产生抉择，但是大于0.268f小于1的效果时比较好的;
            ///////////////////////////////////////////////////

            //{
            pointsVecBuf = mrpoints.vec;
            //} Modified by Duan20170710
            mrpoints.vec.clear();
            mrpoints.maxNum = mrpoints.vec.length();
            mrpoints.lineInd = i;

            for(int r_index=i; (r_index<rows)&&(r_index<i+ScanRange); r_index++)// Scan Range
            {
                QVector<cv::Point> tmpvec;
                tmpvec.clear();

//                // Debug
//                Mat tmp_raw_img;
//                img_raw.copyTo(tmp_raw_img);
//                // Debug end
                for(int j=0; j<(int)cols;) //Col scan
                {
                    if(img_raw.at<uchar>(r_index, j)==IMGBW_BLACK)
                    {
                        existValidPoint = true;
                        /// 像素坐标数组
                        rad.x = j;
                        rad.y = r_index;
                        tmpvec.append(rad);
                        j += 2*radius+space;

//                        // Debug
//                        circle(tmp_raw_img, rad, radius, cv::Scalar(0, 255, 0), 1);
//                        // Debug end
                    }
                    else
                    {
                        j += 1;
                    }
                }
//                // Debug end
//                cv::putText(tmp_raw_img, String(std::to_string(r_index)), cv::Point(20, 20), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(0, 255, 0), 1, cv::LINE_AA);
//                imshow("debug_img_raw", tmp_raw_img);
//                waitKey(0);
//                // Debug end

                //更新最大行冲压单元
                if(tmpvec.length()>mrpoints.maxNum)
                {
                    mrpoints.maxNum = tmpvec.length();
                    mrpoints.lineInd = r_index;
                    mrpoints.vec = tmpvec;
                }
                /// If the points number is bigger than CY_maxStep then stop
                if(num+mrpoints.maxNum>=CY_MAXSTEP) break;
            }

            ///以行最后一个点为起点反冲一次，避免大间距
            if(mrpoints.maxNum>0)
            {
                Point lineLastPoint = mrpoints.vec.last();
                Point resort_rad;
                QVector<cv::Point> tmpvec;
                for(int j=lineLastPoint.x; j>0; j -= 2*radius+space) //Col scan
                {
                    if(img_raw.at<uchar>(lineLastPoint.y, j)==IMGBW_BLACK)
                    {
                        /// 像素坐标数组
                        resort_rad.x = j;
                        resort_rad.y = lineLastPoint.y;
                        tmpvec.append(resort_rad);
                    }
                }
                //如果反冲数量不减少，则最大行冲压单元更新为反冲结果，以此避免大间距
                //if(tmpvec.length()>=mrpoints.maxNum)
                {
                    mrpoints.maxNum = tmpvec.length();
                    mrpoints.lineInd = lineLastPoint.y;
                    mrpoints.vec = tmpvec;
                }
            }
            ///更新特征量
            linePoints = mrpoints.maxNum;
            num += linePoints;
            i = mrpoints.lineInd;
            vec += mrpoints.vec;

            ///拼接图冲孔和当前行去圆形区域
            for(int j=0; j<mrpoints.maxNum; j++)
            {
                /// 拼接图冲孔
                plotp(img_remain, mrpoints.vec[j], radius);
                /// 得到新的圆心区域
                plotp(img_raw, mrpoints.vec[j], 2*radius+space);
            }

            /// 更新下一行
            if(existValidPoint)
            {
                i += (radius+space*0.5f)*1.732f;   //sqr(3)=1.732f
            }
            else
            {
                i += 1;
            }
        }
    }

    ///得到点集有效个数
    numOfPoints = vec.size();

    ///更新一帧最后一行冲孔
    if(numOfPoints>0)
    {
        //{
        lastFramelastRVec = pointsVecBuf + mrpoints.vec;
        //} Modified by Duan20170710
        for(int k=0; k<lastFramelastRVec.length(); k++)
        {
            lastFramelastRVec[k].y = rows-lastFramelastRVec[k].y;
        }
    }
    else
        lastFramelastRVec.clear();

    ///冲压顺序排序（像素点）
    vec = pointPixForwardSort(vec);

    /// 图像拼接与最后一行冲点坐标修正
    for(int j=0; j<numOfPoints; j++)
    {
        vec[j].y -= img_remain_h;   //remain concat modify
        vec[j].y += overLap;    // overlap modify
    }

    // Debug
    ///结果显示 1
    img = img_remain_Debug;
    for(int j=0; j<numOfPoints; j++)
    {
        /// 坐标映射与输出
        std::cout << vec[j] << std::endl;
        /// 冲压结果画圆
        Point tmp_Point(vec[j]);
        tmp_Point.y -= overLap;
        tmp_Point.y += img_remain_h;
        plotpEmpty(img, tmp_Point, radius);
        /// 顺序显示
        //cv::putText(img, String(std::to_string(j+1)), tmp_Point-cv::Point(radius/5.0, -radius/5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(255, 0, 0), 1, cv::LINE_AA);
        //modified by czh 20170829 to fit qt in linux
        //cv::putText(img, String(std::to_string(j+1)), tmp_Point-cv::Point(radius/5.0, -radius/5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(255, 0, 0), 1, cv::LINE_AA);
        std::stringstream ss;
        ss << j+1;
        cv::putText(img, cv::String(ss.str()), tmp_Point-cv::Point(radius/5.0, -radius/5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(255, 0, 0), 1);
    }
    img.copyTo(tmp_img);
    // Debug end

//    // Debug
//    ///结果显示 2
//    for(int j=0; j<numOfPoints; j++)
//    {
//        /// 坐标映射与输出
//        std::cout << vec[j] << std::endl;
//        /// 冲压结果画圆
//        cv::circle(img, vec[j], radius, cv::Scalar(255, 255, 255), 1);
//        /// 顺序显示
//        cv::putText(img, String(std::to_string(j+1)), vec[j]-cv::Point(radius/5.0, -radius/5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(255, 0, 0), 1, cv::LINE_AA);
//    }
//    // Debug end


    return numOfPoints;
}

/*
* @brief: 六边形主处理(不后退)--普通三角排法还是采用强行后退0.5r，未改
* @param:
*		img:	opencv二值化图像
*		radius: 冲孔半径
*		dist:	随边间距
*		space:	冲孔间距
*		vec:	点序列
*       overLap:重叠区域纵向像素值（必须大于等于零）
*@return:
*		<0: 函数运行错误
*		other:	冲孔个数
*/
int cy_algorithm::chongyaFowardPoly(cv::Mat& img, int radius, int dist, int space, QVector<cv::Point> &vec, int overLap)
{
    Mat img_raw, imgEdge;
    unsigned int rows, cols;
    int numOfPoints;
    bool firstFrameLine = true;
    Mat cpartRemStore, cpartRemStoreRaw;   //opencv 很多图像操作都是对同一块内存的，为了存储中间过程的图像，要用copyTo开一块新的内存保留

    /// 图像有效性检测
    if (img.empty())
    {
        return -1;
    }
    img.copyTo(img_raw);

    /// Image concat
    static Mat img_remain, last_frameRaw, img_rawRaw;
    Mat img_cpartRem, img_cpartNow, img_cpartRemRaw,img_cpartNowRaw;
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    int img_remain_h = (int)img_raw.rows/3.0;   ///拼接高度，如果要保证半径为r绝对不出错，h>2r+1,否则小概率出错,当h=图像高度，绝对不出错，但是计算很大
    //modified by czh 20170615
    int img_remain_h = (int)(radius+space)*2.2;
    int img_remain_h_limit = (int)(img_raw.rows);
    if(img_remain_h > img_remain_h_limit)
        return -1;
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(overLap>img_remain_h)
        img_remain_h = overLap;
    if(img_remain.empty())  //第一帧用空白填充连接部分
    {
        img_raw.copyTo(img_remain);
        img_raw.copyTo(last_frameRaw);
        img_remain = IMGBW_WITHE;
        last_frameRaw = IMGBW_WITHE;
        img_cpartRem = img_remain(Rect(0, img_remain.rows-img_remain_h, img_remain.cols, img_remain_h));
        img_cpartRemRaw = last_frameRaw(Rect(0, last_frameRaw.rows-img_remain_h, last_frameRaw.cols, img_remain_h));

        // 首帧后退处理
        img_cpartRem.copyTo(cpartRemStore);
        img_cpartRemRaw.copyTo(cpartRemStoreRaw);

        img_cpartNow = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));
        img_cpartNowRaw = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));

//        vconcat(img_cpartRem, img_cpartNow, img_raw);
//        vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
        //modified by czh
        if (img_cpartRem.cols == img_cpartNow.cols)
        {
            vconcat(img_cpartRem, img_cpartNow, img_raw);
            vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
        }
        else
        {
            img_cpartNow.copyTo(img_raw);
            img_cpartNowRaw.copyTo(img_rawRaw);
        }
    }
    else
    {
        img_cpartRem = img_remain(Rect(0, img_remain.rows-img_remain_h, img_remain.cols, img_remain_h));
        img_cpartRemRaw = last_frameRaw(Rect(0, last_frameRaw.rows-img_remain_h, last_frameRaw.cols, img_remain_h));
        // 首帧后退处理
        img_cpartRem.copyTo(cpartRemStore);
        img_cpartRemRaw.copyTo(cpartRemStoreRaw);

        img_cpartNow = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));
        img_cpartNowRaw = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));

//        vconcat(img_cpartRem, img_cpartNow, img_raw);
//        vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
        //modified by czh
        if (img_cpartRem.cols == img_cpartNow.cols)
        {
            vconcat(img_cpartRem, img_cpartNow, img_raw);
            vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
        }
        else
        {
            img_cpartNow.copyTo(img_raw);
            img_cpartNowRaw.copyTo(img_rawRaw);
        }
    }
    img_raw.copyTo(img_remain);
    img_raw.copyTo(last_frameRaw);

    // 首帧后退处理,检测第一帧
    firstFrameLine = true;
    for(int i=0; i<cpartRemStore.rows; i++)
    {
        Point rad;
        rad = findValueLine(cpartRemStore, IMGBW_BLACK, i);

        if (rad.x>=0 && rad.y>=0)
        {
            firstFrameLine = false;
            break;
        }
    }

    // Debug
//    ///结果显示 1
    Mat img_remain_Debug;
    img_remain.copyTo(img_remain_Debug);
//    imshow("img_remain", img_remain);
    // Debug End

    /// 显示图像信息
    rows = img_raw.rows;
    cols = img_raw.cols;

    /// 随边处理
    Mat img_rawRawEdge = edgesbw(img_rawRaw);
    img_rawRawEdge.col(0) = IMGBW_WITHE;
    img_rawRawEdge.col(cols - 1) = IMGBW_WITHE;
    img_rawRaw = img_rawRaw | polySub(img_rawRawEdge, dist);

    /// 随边与冲孔图像整合
    img_raw = img_raw | img_rawRaw;

    /// 得到整合图像边缘
    imgEdge = edgesbw(img_raw);

    /// 加边框边界，若不需要，移除以下四条语句
    imgEdge.row(0) = IMGBW_WITHE;
    imgEdge.row(rows - 1) = IMGBW_WITHE;
    imgEdge.col(0) = IMGBW_WITHE;
    imgEdge.col(cols - 1) = IMGBW_WITHE;

    /// 得到首个圆心区域
    img_raw = img_raw | polySub(imgEdge, radius);

//    // Debug
//    imshow("centerArea", img_raw);
//    // Debug End

    /// 主处理循环，获得像素坐标点集合
    Point rad;
    vec.clear();
    int num = 0;
    static int dir = 0;
    int rowStart;
    static Point lastFramePoint = Point(cols, rows);

    /// 连接图像扫描首行值
    rowStart = img_remain_h-lastFramePoint.y+(radius+space*0.5f)*1.732f+1;
    if(rowStart < 0)
    {
        rowStart = 0;
    }

    /// 冲压处理
    //{
    static int fistLineCols = 0;
    //} //modified by Duan@20170616
    for (int i = rowStart; i < (int)rows;)   // Row scan
    {
        rad = findValueLine(img_raw, IMGBW_BLACK, i);

        if (rad.x<0 || rad.y<0)
        {
            i++;
            // 检测到整行白色，标志接下来将是新料片头部
            firstFrameLine = true;
        }
        else
        {
            int colStart=0;
            //{
            if(dir)
//                colStart = radius+ dist+1;
                colStart = fistLineCols%(2*radius+space) + 1;
            else
//                colStart = 2*radius+space*0.5f+ dist+1;
                colStart = radius+space*0.5f+ fistLineCols%(2*radius+space) + 1;
            //} //modified by Duan@20170616

            bool existValidPoint = false;
            int linePoints = 0;
            for(int j=colStart; j<(int)cols; j+=2*radius+space) //Col scan
            {
                if(img_raw.at<uchar>(i, j)==IMGBW_BLACK)
                {
                    existValidPoint = true;
                    /// 像素坐标数组
                    rad.x = j;
                    vec.append(rad);
                    num++;
                    linePoints++;

                    /// Image concat
                    if(!firstFrameLine)
                        plotp(img_remain, rad, radius);

                    /// If the points number is bigger than CY_maxStep then stop
                    if(num>=CY_MAXSTEP) break;
                }
            }
            /// Next Line
            if(existValidPoint)
            {
                // 首帧后退处理
                if(firstFrameLine)
                {
                    ////////////////////////////////////////////
                    i += (int)radius/2;     /// 后退的像素值
                    ////////////////////////////////////////////
                    //移除之前加入队列的料片头部冲点
                    for(int k=0; k<linePoints; k++)
                    {
                        vec.removeLast();
                        num--;
                    }
                    firstFrameLine = false;
                    //{
                    Point fistLinePoint = findValueLine(img_raw, IMGBW_BLACK, i);
                    fistLineCols = fistLinePoint.x<0? fistLineCols:fistLinePoint.x;
                    dir=0;
                    //} //modified by Duan@20170616
                }
                else
                    i += (radius+space*0.5f)*1.732f;   //sqr(3)=1.732f

                dir = !dir;
            }
            else
                i += 1;
        }
    }

    ///得到点集有效个数
    numOfPoints = vec.size();

    ///更新LastPoint
    if(numOfPoints>0)
    {
        lastFramePoint = vec.last();
        lastFramePoint.y = rows-lastFramePoint.y;
    }
    else
        lastFramePoint = Point(cols, rows);

    ///冲压顺序排序（像素点）
    vec = pointPixForwardSort(vec);

    /// Image concat ,Point modify
    for(int j=0; j<numOfPoints; j++)
    {
        vec[j].y -= img_remain_h;   //remain concat modify
        vec[j].y += overLap;    // overlap modify
    }


    // Debug
    ///结果显示 1
    img = img_remain_Debug;
    for(int j=0; j<numOfPoints; j++)
    {
        /// 坐标映射与输出
        std::cout << vec[j] << std::endl;
        /// 冲压结果画圆
        Point tmp_Point(vec[j]);
        tmp_Point.y -= overLap;
        tmp_Point.y += img_remain_h;
        plotpEmpty(img, tmp_Point, radius);
        /// 顺序显示
        //cv::putText(img, String(std::to_string(j+1)), tmp_Point-cv::Point(radius/5.0, -radius/5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(255, 0, 0), 1, cv::LINE_AA);
        //modified by czh 20170829 to fit qt in linux
        //cv::putText(img, String(std::to_string(j+1)), tmp_Point-cv::Point(radius/5.0, -radius/5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(255, 0, 0), 1, cv::LINE_AA);
        std::stringstream ss;
        ss << j+1;
        cv::putText(img, cv::String(ss.str()), tmp_Point-cv::Point(radius/5.0, -radius/5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(255, 0, 0), 1);
    }
    img.copyTo(tmp_img);
    // Debug end

//    // Debug
//    ///结果显示 2
//    for(int j=0; j<numOfPoints; j++)
//    {
//        /// 坐标映射与输出
//        std::cout << vec[j] << std::endl;
//        /// 冲压结果画圆
//        cv::circle(img, vec[j], radius, cv::Scalar(255, 255, 255), 1);
//        /// 顺序显示
//        cv::putText(img, String(std::to_string(j+1)), vec[j]-cv::Point(radius/5.0, -radius/5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(255, 0, 0), 1, cv::LINE_AA);
//    }
//    // Debug end


    return numOfPoints;
}
/*
* @brief: 矩形主处理(不后退)--采用顶部扫描
* @param:
*		img:	opencv二值化图像
*		xradius: x方向冲孔半径
*       yradius: y方向冲孔半径
*		dist:	随边间距
*		space:	冲孔间距
*		vec:	点序列
*       overLap:重叠区域纵向像素值（必须大于等于零）
*@return:
*		<0: 函数运行错误
*		other:	冲孔个数
*/
/*
* 修改记录
* build180127:采用顶部扫描方式来处理新料片第一帧
*/
int cy_algorithm::chongyaFowardRect(cv::Mat& img, int xradius, int yradius, int dist, int space, QVector<cv::Point> &vec, int overLap)
{
    Mat img_raw, imgEdge;
    unsigned int rows, cols;
    int numOfPoints;
    bool firstFrameLine = true;
    Mat cpartRemStore, cpartRemStoreRaw;   //opencv 很多图像操作都是对同一块内存的，为了存储中间过程的图像，要用copyTo开一块新的内存保留

    /// 图像有效性检测
    if (img.empty())
    {
        return -1;
    }
    img.copyTo(img_raw);

    /// Image concat
    static Mat img_remain, last_frameRaw, img_rawRaw;
    Mat img_cpartRem, img_cpartNow, img_cpartRemRaw,img_cpartNowRaw;
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    int img_remain_h = (int)img_raw.rows/3.0;   ///拼接高度，如果要保证半径为r绝对不出错，h>2r+1,否则小概率出错,当h=图像高度，绝对不出错，但是计算很大
   //modified by czh 20170711
    int img_remain_h = (int)(yradius+space)*3;
    int img_remain_h_limit = (int)(img_raw.rows);
    if(img_remain_h > img_remain_h_limit)
        return -1;
    //modified end
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(overLap>img_remain_h)
        img_remain_h = overLap;
    if(img_remain.empty())  //第一帧用空白填充连接部分
    {
        img_raw.copyTo(img_remain);
        img_raw.copyTo(last_frameRaw);
        img_remain = IMGBW_WITHE;
        last_frameRaw = IMGBW_WITHE;
        img_cpartRem = img_remain(Rect(0, img_remain.rows-img_remain_h, img_remain.cols, img_remain_h));
        img_cpartRemRaw = last_frameRaw(Rect(0, last_frameRaw.rows-img_remain_h, last_frameRaw.cols, img_remain_h));

        // 首帧后退处理
        img_cpartRem.copyTo(cpartRemStore);
        img_cpartRemRaw.copyTo(cpartRemStoreRaw);

        img_cpartNow = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));
        img_cpartNowRaw = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));

//        vconcat(img_cpartRem, img_cpartNow, img_raw);
//        vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
        //modified by czh
        if (img_cpartRem.cols == img_cpartNow.cols)
        {
            vconcat(img_cpartRem, img_cpartNow, img_raw);
            vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
        }
        else
        {
            img_cpartNow.copyTo(img_raw);
            img_cpartNowRaw.copyTo(img_rawRaw);
        }
    }
    else
    {
        img_cpartRem = img_remain(Rect(0, img_remain.rows-img_remain_h, img_remain.cols, img_remain_h));
        img_cpartRemRaw = last_frameRaw(Rect(0, last_frameRaw.rows-img_remain_h, last_frameRaw.cols, img_remain_h));
        // 首帧后退处理
        img_cpartRem.copyTo(cpartRemStore);
        img_cpartRemRaw.copyTo(cpartRemStoreRaw);

        img_cpartNow = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));
        img_cpartNowRaw = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));

//        vconcat(img_cpartRem, img_cpartNow, img_raw);
//        vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
        //modified by czh
        if (img_cpartRem.cols == img_cpartNow.cols)
        {
            vconcat(img_cpartRem, img_cpartNow, img_raw);
            vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
        }
        else
        {
            img_cpartNow.copyTo(img_raw);
            img_cpartNowRaw.copyTo(img_rawRaw);
        }
    }
    img_raw.copyTo(img_remain);
    img_raw.copyTo(last_frameRaw);

    //// 首帧后退处理,检测第一帧
    //firstFrameLine = true;
    //for(int i=0; i<cpartRemStore.rows; i++)
    //{
    //    Point rad;
    //    rad = findValueLine(cpartRemStore, IMGBW_BLACK, i);

    //    if (rad.x>=0 && rad.y>=0)
    //    {
    //        firstFrameLine = false;
    //        break;
    //    }
    //}

    // Debug
//    ///结果显示 1
    Mat img_remain_Debug;
    img_remain.copyTo(img_remain_Debug);
//    imshow("img_remain", img_remain);
    // Debug End

    /// 显示图像信息
    rows = img_raw.rows;
    cols = img_raw.cols;

    /// 随边处理
    Mat img_rawRawEdge = edgesbw(img_rawRaw);
    img_rawRawEdge.col(0) = IMGBW_WITHE;
    img_rawRawEdge.col(cols - 1) = IMGBW_WITHE;
    img_rawRaw = img_rawRaw | polySub(img_rawRawEdge, dist);

    /// 随边与冲孔图像整合
    img_raw = img_raw | img_rawRaw;

    /// 得到整合图像边缘
    imgEdge = edgesbw(img_raw);

    /// 加边框边界，若不需要，移除以下四条语句
    imgEdge.row(0) = IMGBW_WITHE;
    imgEdge.row(rows - 1) = IMGBW_WITHE;
    imgEdge.col(0) = IMGBW_WITHE;
    imgEdge.col(cols - 1) = IMGBW_WITHE;

    /// 得到首个圆心区域
    img_raw = img_raw | rectSub(imgEdge, xradius, yradius);

 //   // Debug
 //   imshow("centerArea", img_raw);
    //imshow("img_remain", img_remain);
 //   // Debug End

    /// 主处理循环，获得像素坐标点集合
    Point rad;
    vec.clear();
    int num = 0;
    int rowStart;
    static Point lastFramePoint = Point(cols, rows);

    /// 连接图像扫描首行值
    rowStart = img_remain_h-lastFramePoint.y+2*yradius+space+1;
    if(rowStart < 0)
    {
        rowStart = 0;
    }

    /// 冲压处理
    MRCYU mrpoints;
    mrpoints.vec.clear();

    QVector<cv::Point> pointsVecBuf;
    pointsVecBuf.clear();

    bool new_tablet_flag = false;//新料片检测
    for (int i = rowStart; i < (int)rows;)   // Row scan
    {
        rad = findValueLine(img_raw, IMGBW_BLACK, i);

        //新料片检测
        Point rad_new_tablet_detection = findValueLine(img_remain, IMGBW_BLACK, i);
        if (rad_new_tablet_detection.x<0 || rad_new_tablet_detection.y<0)
        {
            new_tablet_flag = true;
            //line(img_remain_Debug, Point(0, i), Point(cols, i), cv::Scalar(0, 0, 0), 1);
            cv::putText(img_remain_Debug, cv::String("New_Tablet"), cv::Point(10, 10), CV_FONT_HERSHEY_COMPLEX, yradius*2.0 / CY_R_MAX, Scalar(0, 0, 0), 1);
        }


        if (rad.x<0 || rad.y<0)
        {
            i++;
        }
        else
        {
            int colStart=0;
            colStart = rad.x;

            bool existValidPoint = false;
            int linePoints = 0;
            for(int j=colStart; j<(int)cols; j+=2*xradius+space) //Col scan
            {
                if(img_raw.at<uchar>(i, j)==IMGBW_BLACK)
                {
                    existValidPoint = true;
                    /// 像素坐标数组
                    rad.x = j;
                    vec.append(rad);
                    num++;
                    linePoints++;

                    /// Image concat
                    if(!new_tablet_flag)
                        plotr(img_remain, rad, xradius, yradius);

                    /// If the points number is bigger than CY_maxStep then stop
                    if(num>=CY_MAXSTEP) break;
                }
            }
            /// Next Line
            if(existValidPoint)
            {
                // 首帧扫描后退处理
                if (new_tablet_flag)
                {
                    //扫描得到后退行
                    //////////////////////////////////////////////////
                    int ScanRange = (int)(0.5f*(yradius + space*0.5f));
                    //////////////////////////////////////////////////
                    int max_points = linePoints;
                    int max_points_line = i;
                    int scanline_points = 0;
                    for (int r_index = i; (r_index < rows) && (r_index < i + ScanRange); r_index++)
                    {
                        scanline_points = 0;
                        for (int j = 0; j < (int)cols; /**/) //Col scan
                        {
                            if (img_raw.at<uchar>(r_index, j) == IMGBW_BLACK)
                            {
                                scanline_points++;
                                j += 2 * xradius + space;
                            }
                            else
                            {
                                j += 1;
                            }
                        }
                        if (scanline_points > max_points)
                        {
                            max_points = scanline_points;
                            max_points_line = r_index;
                        }
                    }

                    ////////////////////////////////////////////
                    i = max_points_line;     /// 跳到扫描最多冲孔行
                    ////////////////////////////////////////////
                    //移除之前加入队列的料片头部冲点
                    for (int k = 0; k<linePoints; k++)
                    {
                        vec.removeLast();
                        num--;
                    }
                    new_tablet_flag = false;
                }
                else
                    i += 2*yradius+space;   //sqr(3)=1.732f
            }
            else
                i += 1;
        }
    }

    ///得到点集有效个数
    numOfPoints = vec.size();

    ///更新LastPoint
    if(numOfPoints>0)
    {
        lastFramePoint = vec.last();
        lastFramePoint.y = rows-lastFramePoint.y;
    }
    else
        lastFramePoint = Point(cols, rows);

    ///冲压顺序排序（像素点）
    vec = pointPixForwardSort(vec);

    /// Image concat ,Point modify
    for(int j=0; j<numOfPoints; j++)
    {
        vec[j].y -= img_remain_h;   //remain concat modify
        vec[j].y += overLap;    // overlap modify
    }


    // Debug
    ///结果显示 1
    img = img_remain_Debug;
    for(int j=0; j<numOfPoints; j++)
    {
        /// 坐标映射与输出
        std::cout << vec[j] << std::endl;
        /// 冲压结果画圆
        Point tmp_Point(vec[j]);
        tmp_Point.y -= overLap;
        tmp_Point.y += img_remain_h;
        plotrEmpty(img, tmp_Point, xradius, yradius);
        /// 顺序显示
        //cv::putText(img, String(std::to_string(j+1)), tmp_Point-cv::Point(xradius/5.0, -yradius/5.0), CV_FONT_HERSHEY_COMPLEX, (xradius+yradius)*1.0/CY_R_MAX, Scalar(255, 0, 0), 1, cv::LINE_AA);
        //modified by czh 20170829 to fit qt in linux
        //cv::putText(img, String(std::to_string(j+1)), tmp_Point-cv::Point(radius/5.0, -radius/5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(255, 0, 0), 1, cv::LINE_AA);
        std::stringstream ss;
        ss << j+1;
        cv::putText(img, cv::String(ss.str()), tmp_Point-cv::Point(xradius/5.0, -yradius/5.0), CV_FONT_HERSHEY_COMPLEX, (xradius+yradius)*1.0/CY_R_MAX, Scalar(255, 0, 0), 1);
    }
    img.copyTo(tmp_img);
    // Debug end

//    // Debug
//    ///结果显示 2
//    for(int j=0; j<numOfPoints; j++)
//    {
//        /// 坐标映射与输出
//        std::cout << vec[j] << std::endl;
//        /// 冲压结果画圆
//        cv::circle(img, vec[j], radius, cv::Scalar(255, 255, 255), 1);
//        /// 顺序显示
//        cv::putText(img, String(std::to_string(j+1)), vec[j]-cv::Point(radius/5.0, -radius/5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(255, 0, 0), 1, cv::LINE_AA);
//    }
//    // Debug end


    return numOfPoints;
}

/*
* @brief: 圆形主处理(不后退) W走向型冲孔---智能竖排
* @param:
*		img:	opencv二值化图像
*		radius: 冲孔半径
*		dist:	随边间距
*		space:	冲孔间距
*		vec:	点序列
*       overLap:重叠区域纵向像素值（必须大于等于零）
*@return:
*		<0: 函数运行错误
*		other:	冲孔个数
*/
/*修改记录：
 * build180123：修改竖排排孔BUG
 *
*/
int cy_algorithm::chongyaFowardCircle_w(cv::Mat& img, int radius, int dist, int space, QVector<cv::Point> &vec, int overLap)
{
    Mat img_raw, imgEdge;
    unsigned int rows, cols;
    int numOfPoints;
    Mat cpartRemStore, cpartRemStoreRaw;   //opencv 很多图像操作都是对同一块内存的，为了存储中间过程的图像，要用copyTo开一块新的内存保留

    const unsigned int BackScanRange = 0;           //
    const double new_tablet_scanRange_factor = 0.3; //料片头部扫描范围参数
    const double scanRange_factor = 0.22;           //料片非头部扫描范围参数

    /// 图像有效性检测
    if (img.empty())
    {
        return -1;
    }
    img.copyTo(img_raw);

    /// Image concat
    static Mat img_remain, last_frameRaw, img_rawRaw;
    Mat img_cpartRem, img_cpartNow, img_cpartRemRaw,img_cpartNowRaw;
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    int img_remain_h = (int)img_raw.rows/3.0;   ///拼接高度，如果要保证半径为r绝对不出错，h>2r+1,否则小概率出错,当h=图像高度，绝对不出错，但是计算很大
//modified by czh 20170615
    int img_remain_h = (int)(radius+space)*2.5;
    int img_remain_h_without_overlap = img_remain_h;
    int img_remain_h_limit = (int)(img_raw.rows);
    if(img_remain_h > img_remain_h_limit)
        return -1;
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(overLap>img_remain_h)
        img_remain_h = overLap;
    if(img_remain.empty())  //第一帧用空白填充连接部分
    {
        img_raw.copyTo(img_remain);
        img_raw.copyTo(last_frameRaw);
        img_remain = IMGBW_WITHE;
        last_frameRaw = IMGBW_WITHE;
        img_cpartRem = img_remain(Rect(0, img_remain.rows-img_remain_h, img_remain.cols, img_remain_h));
        img_cpartRemRaw = last_frameRaw(Rect(0, last_frameRaw.rows-img_remain_h, last_frameRaw.cols, img_remain_h));

        // 首帧后退处理pointPixForwardSort(x)
        img_cpartRem.copyTo(cpartRemStore);
        img_cpartRemRaw.copyTo(cpartRemStoreRaw);

        img_cpartNow = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));
        img_cpartNowRaw = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));

        if (img_cpartRem.cols == img_cpartNow.cols)
        {
            vconcat(img_cpartRem, img_cpartNow, img_raw);
            vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
        }
        else
        {
            if (img_cpartRem.cols == img_cpartNow.cols)
            {
                vconcat(img_cpartRem, img_cpartNow, img_raw);
                vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
            }
            else
            {
                img_cpartNow.copyTo(img_raw);
                img_cpartNowRaw.copyTo(img_rawRaw);
            }
        }
    }
    else
    {
        img_cpartRem = img_remain(Rect(0, img_remain.rows-img_remain_h, img_remain.cols, img_remain_h));
        img_cpartRemRaw = last_frameRaw(Rect(0, last_frameRaw.rows-img_remain_h, last_frameRaw.cols, img_remain_h));
        // 首帧后退处理(x)
        img_cpartRem.copyTo(cpartRemStore);
        img_cpartRemRaw.copyTo(cpartRemStoreRaw);

        img_cpartNow = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));
        img_cpartNowRaw = img_raw(Rect(0, 0+overLap, img_raw.cols, img_raw.rows-overLap));

        if (img_cpartRem.cols == img_cpartNow.cols)
        {
            vconcat(img_cpartRem, img_cpartNow, img_raw);
            vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
        }
        else
        {
            img_cpartNow.copyTo(img_raw);
            img_cpartNowRaw.copyTo(img_rawRaw);
        }
    }
    img_raw.copyTo(img_remain);
    img_raw.copyTo(last_frameRaw);

    // Debug
    ///结果显示 1
    Mat img_remain_Debug;
    img_remain.copyTo(img_remain_Debug);
//    imshow("img_remain", img_remain);
    // Debug End

    /// 显示图像信息
    rows = img_raw.rows;
    cols = img_raw.cols;

    /// 随边处理
    Mat img_rawRawEdge = edgesbw(img_rawRaw);
    img_rawRawEdge.col(0) = IMGBW_WITHE;
    img_rawRawEdge.col(cols - 1) = IMGBW_WITHE;
    img_rawRaw = img_rawRaw | circleSub(img_rawRawEdge, dist);

    /// 随边与冲孔图像整合
    img_raw = img_raw | img_rawRaw;

    /// 得到整合图像边缘
    imgEdge = edgesbw(img_raw);

    /// 加边框边界，若不需要，移除以下四条语句
    imgEdge.row(0) = IMGBW_WITHE;
    imgEdge.row(rows - 1) = IMGBW_WITHE;
    imgEdge.col(0) = IMGBW_WITHE;
    imgEdge.col(cols - 1) = IMGBW_WITHE;

    /// 得到首个圆心区域
    img_raw = img_raw | circleSub(imgEdge, radius);

    // Debug
//    imshow("centerArea", img_raw);
    // Debug End

    /// 主处理循环，获得像素坐标点集合
    Point rad;
    vec.clear();
    int num = 0;
    int rowStart;

    ///减去上一帧最后两行得到顶部圆心区域
    static QVector<cv::Point> lastFramelastRVec;
    Point lastFramePoint = Point(cols, rows);
    if(lastFramelastRVec.length()>0)
    {
        lastFramePoint = lastFramelastRVec.last();
        for(int k=0; k<lastFramelastRVec.length(); k++)
        {
            lastFramelastRVec[k].y = img_remain_h-lastFramelastRVec[k].y;
            plotc(img_raw, lastFramelastRVec[k], 2*radius+space);
        }
    }
    // Debug
//    imshow("centerArea", img_raw);
    // Debug End

    /// 连接图像扫描首行值
    rowStart = img_remain_h-lastFramePoint.y+(int)(radius+space*0.5f)+1;

    //{
    /// 补足前一帧最后一行冲孔
    int lastFrameLastrow = img_remain_h-lastFramePoint.y+2;
    if(lastFrameLastrow > 0)
    {
        for(int j=0; j<(int)cols;) //Col scan
        {
            if(img_raw.at<uchar>(lastFrameLastrow, j)==IMGBW_BLACK)
            {
                rad.x = j;
                rad.y = lastFrameLastrow;
                vec.append(rad);
                //modified by czh 20170711
                /// 拼接图冲孔
                plotc(img_remain, rad, radius);
                /// 得到新的圆心区域
                plotc(img_raw, rad, 2*radius+space);

                j += (int)((2*radius+space)*1.732f);
            }
            else
                j += 1;
        }
    }
    //} Modified by Duan20170710

    if(rowStart < 0)
    {
        rowStart = 0;
    }

    /// 冲压处理
    MRCYU mrpoints;
    mrpoints.vec.clear();

    //{
    QVector<cv::Point> pointsVecBuf;
    pointsVecBuf.clear();
    //} Modified by Duan20170710

    bool new_tablet_flag = false; // New tablet
    for (int i = rowStart; i < (int)rows;)   // Row scan
    {
        rad = findValueLine(img_raw, IMGBW_BLACK, i);

        // New tablet detection
        Point rad_new_tablet_detection = findValueLine(img_remain, IMGBW_BLACK, i);
        if (rad_new_tablet_detection.x<0 || rad_new_tablet_detection.y<0)
        {
            new_tablet_flag = true;
            //line(img_remain_Debug, Point(0, i), Point(cols, i), cv::Scalar(0, 0, 0), 1);
            //cv::putText(img_remain_Debug, cv::String("New_Tablet"), cv::Point(10, 10), CV_FONT_HERSHEY_COMPLEX, radius*2.0 / CY_R_MAX, Scalar(0, 0, 0), 1);
        }

        if (rad.x<0 || rad.y<0)
        {
            i++;
        }
        else
        {
            //Modified by Duan20180123
            if(rows-i+radius+(int)(space/2) < img_remain_h_without_overlap)
                break;

            bool existValidPoint = false;
            int linePoints = 0;

            //在ScanRange内扫描，得到最大行冲压单元
            ////////////////////////////////////////////////
            int ScanRange = (int)(scanRange_factor*(radius+space*0.5f));   ///扫描范围, 由于竖排横向距离大,扫描范围不宜过大,系数大于这个值可能会在最多相切与行冲点最多之间产生抉择
            ///////////////////////////////////////////////////

            // New tablet scan range
            if (new_tablet_flag)
            {
                ScanRange = (int)(new_tablet_scanRange_factor*(radius + space*0.5f));
            }

            //{
            pointsVecBuf = mrpoints.vec;
            //} Modified by Duan20170710
            mrpoints.vec.clear();
            mrpoints.maxNum = mrpoints.vec.length();
            mrpoints.lineInd = i;


            for(int r_index=((int)(i-BackScanRange)>0? (int)(i-BackScanRange):0); (r_index<rows)&&(r_index<i+ScanRange); r_index++)// Scan Range
            {
                QVector<cv::Point> tmpvec;
                tmpvec.clear();

//                // Debug
//                Mat tmp_raw_img;
//                img_raw.copyTo(tmp_raw_img);
//                // Debug end
                for(int j=0; j<(int)cols;) //Col scan
                {
                    if(img_raw.at<uchar>(r_index, j)==IMGBW_BLACK)
                    {
                        existValidPoint = true;
                        /// 像素坐标数组
                        rad.x = j;
                        rad.y = r_index;
                        tmpvec.append(rad);
                        j += (int)((2*radius+space)*1.732f+1);

//                        // Debug
//                        circle(tmp_raw_img, rad, radius, cv::Scalar(0, 255, 0), 1);
//                        // Debug end
                    }
                    else
                    {
                        j += 1;
                    }
                }
//                // Debug end
//                cv::putText(tmp_raw_img, String(std::to_string(r_index)), cv::Point(20, 20), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(0, 255, 0), 1, cv::LINE_AA);
//                imshow("debug_img_raw", tmp_raw_img);
//                waitKey(0);
//                // Debug end

                //更新最大行冲压单元
                if(tmpvec.length()>mrpoints.maxNum)
                {
                    mrpoints.maxNum = tmpvec.length();
                    mrpoints.lineInd = r_index;
                    mrpoints.vec = tmpvec;
                }
                else if (tmpvec.length()==mrpoints.maxNum \
                    && new_tablet_flag \
                    && mrpoints.vec.length()>0 \
                    &&(tmpvec.first().x < mrpoints.vec.first().x))
                {
                    mrpoints.maxNum = tmpvec.length();
                    mrpoints.lineInd = r_index;
                    mrpoints.vec = tmpvec;
                    // Debug
                    //line(img_remain_Debug, Point(0, tmpvec.last().y), Point(cols, tmpvec.last().y), cv::Scalar(0, 0, 0), 1);
                }

                /// If the points number is bigger than CY_maxStep then stop
                if(num+mrpoints.maxNum>=CY_MAXSTEP) break;
            }
            // New tablet process finished
            if (new_tablet_flag && (mrpoints.maxNum>0))
            {
                new_tablet_flag = false;
            }

            ///以行最后一个点为起点反冲一次，避免大间距
            if(mrpoints.maxNum>0)
            {
                Point lineLastPoint = mrpoints.vec.last();
                Point resort_rad;
                QVector<cv::Point> tmpvec;
                for(int j=lineLastPoint.x; j>0;) //Col scan
                {
                    if(img_raw.at<uchar>(lineLastPoint.y, j)==IMGBW_BLACK)
                    {
                        /// 像素坐标数组
                        resort_rad.x = j;
                        resort_rad.y = lineLastPoint.y;
                        tmpvec.append(resort_rad);
                        j -= (int)((2*radius+space)*1.732f+1);
                    }
                    else
                    {
                        j -= 1;
                    }
                }
                //如果反冲数量不减少，则最大行冲压单元更新为反冲结果，以此避免大间距
//                if(tmpvec.length()>=mrpoints.maxNum)
                {
                    mrpoints.maxNum = tmpvec.length();
                    mrpoints.lineInd = lineLastPoint.y;
                    mrpoints.vec = tmpvec;
                }
            }
            ///更新特征量
            linePoints = mrpoints.maxNum;
            num += linePoints;
            i = mrpoints.lineInd;
            vec += mrpoints.vec;

            ///拼接图冲孔和当前行去圆形区域
            for(int j=0; j<mrpoints.maxNum; j++)
            {
                /// 拼接图冲孔
                plotc(img_remain, mrpoints.vec[j], radius);
                /// 得到新的圆心区域
                plotc(img_raw, mrpoints.vec[j], 2*radius+space);
            }

            /// 更新下一行
            if(existValidPoint)
            {
                i += (int)(radius+space*0.5f);
            }
            else
            {
                i += 1;
            }
            //Modified by Duan20171228
            if(rows-i+radius+(int)(space/2) < img_remain_h_without_overlap)
                break;
        }
    }

    ///得到点集有效个数
    numOfPoints = vec.size();

    ///更新一帧最后一行冲孔
    if(numOfPoints>0)
    {
        //{
        lastFramelastRVec = pointsVecBuf + mrpoints.vec;
        //} Modified by Duan20170710
        for(int k=0; k<lastFramelastRVec.length(); k++)
        {
            lastFramelastRVec[k].y = rows-lastFramelastRVec[k].y;
        }
    }
    else
        lastFramelastRVec.clear();

    ///冲压顺序排序（像素点）
    vec = pointPixForwardSort(vec);

    /// 图像拼接与最后一行冲点坐标修正
    for(int j=0; j<numOfPoints; j++)
    {
        vec[j].y -= img_remain_h;   //remain concat modify
        vec[j].y += overLap;    // overlap modify
    }

    // Debug
    ///结果显示 1
    img = img_remain_Debug;

//    //Debug
//    int stop_line = rows+radius+(int)(space/2) - img_remain_h_without_overlap;
//    int remain_line = rows-img_remain_h;
//    cv::line(img, Point(0, stop_line), Point(cols, stop_line), cv::Scalar(255, 255, 255), 1);
//    cv::line(img, Point(0, remain_line), Point(cols, remain_line), cv::Scalar(255, 255, 255), 1);
//    //OverLap
//    std::stringstream ss1;
//    ss1 << overLap;
//    cv::String tmp_str1 = "overLap:"+ss1.str();
//    cv::putText(img, tmp_str1, Point(cols/10, rows/5), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, cv::Scalar(0, 0, 0), 1);
//    //Remain_h
//    std::stringstream ss2;
//    ss2 << img_remain_h;
//    cv::String tmp_str2 = "img_remain_h:"+ss2.str();
//    cv::putText(img, tmp_str2, Point(cols/10, rows/4), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, cv::Scalar(0, 0, 0), 1);
//    //Radius
//    std::stringstream ss3;
//    ss3 << radius+(int)(space/2);
//    cv::String tmp_str3 = "r+s/2:"+ss3.str();
//    cv::putText(img, tmp_str3, Point(cols/10, rows/3), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, cv::Scalar(0, 0, 0), 1);

    for(int j=0; j<numOfPoints; j++)
    {
        /// 坐标映射与输出
        std::cout << vec[j] << std::endl;
        /// 冲压结果画圆
        Point tmp_Point(vec[j]);
        tmp_Point.y -= overLap;
        tmp_Point.y += img_remain_h;
        cv::circle(img, tmp_Point, radius, cv::Scalar(255, 255, 255), 1);
        /// 顺序显示
        //cv::putText(img, String(std::to_string(j+1)), tmp_Point-cv::Point(radius/5.0, -radius/5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(255, 0, 0), 1, cv::LINE_AA);
        //modified by czh 20170829 to fit qt in linux
        //cv::putText(img, String(std::to_string(j+1)), tmp_Point-cv::Point(radius/5.0, -radius/5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(255, 0, 0), 1, cv::LINE_AA);
        std::stringstream ss;
        ss << j+1;
        cv::putText(img, cv::String(ss.str()), tmp_Point-cv::Point(radius/5.0, -radius/5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(255, 0, 0), 1);
    }
    img.copyTo(tmp_img);
    // Debug end

//    // Debug
//    ///结果显示 2
//    for(int j=0; j<numOfPoints; j++)
//    {
//        /// 坐标映射与输出
//        std::cout << vec[j] << std::endl;
//        /// 冲压结果画圆
//        cv::circle(img, vec[j], radius, cv::Scalar(255, 255, 255), 1);
//        /// 顺序显示
//        cv::putText(img, String(std::to_string(j+1)), vec[j]-cv::Point(radius/5.0, -radius/5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(255, 0, 0), 1, cv::LINE_AA);
//    }
//    // Debug end


    return numOfPoints;
}
#if 0
/*
* @brief: 圆形主处理(不后退) W走向型冲孔
* @param:
*		img:	opencv二值化图像
*		radius: 冲孔半径
*		dist:	随边间距
*		space:	冲孔间距
*		vec:	点序列
*       overLap:重叠区域纵向像素值（必须大于等于零）
*@return:
*		<0: 函数运行错误
*		other:	冲孔个数
*/
int cy_algorithm::chongyaFowardCircle_w(cv::Mat& img, int radius, int dist, int space, QVector<cv::Point> &vec, int overLap)
{
    Mat img_raw, imgEdge;
    unsigned int rows, cols;
    int numOfPoints;
    bool firstFrameLine = true;
    Mat cpartRemStore, cpartRemStoreRaw;   //opencv 很多图像操作都是对同一块内存的，为了存储中间过程的图像，要用copyTo开一块新的内存保留

                                           /// 图像有效性检测
    if (img.empty())
    {
        return -1;
    }
    img.copyTo(img_raw);

    /// Image concat
    static Mat img_remain, last_frameRaw, img_rawRaw;
    Mat img_cpartRem, img_cpartNow, img_cpartRemRaw, img_cpartNowRaw;
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	int img_remain_h = (int)img_raw.rows / 3.0;   ///拼接高度，如果要保证半径为r绝对不出错，h>2r+1,否则小概率出错,当h=图像高度，绝对不出错，但是计算很大
    int img_remain_h = (int)(radius+space)*2.2;
    int img_remain_h_limit = (int)(img_raw.rows);
    if(img_remain_h > img_remain_h_limit)
        return -1;
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (overLap > img_remain_h)
        img_remain_h = overLap;
    if (img_remain.empty())  //第一帧用空白填充连接部分
    {
        img_raw.copyTo(img_remain);
        img_raw.copyTo(last_frameRaw);
        img_remain = IMGBW_WITHE;
        last_frameRaw = IMGBW_WITHE;
        img_cpartRem = img_remain(Rect(0, img_remain.rows - img_remain_h, img_remain.cols, img_remain_h));
        img_cpartRemRaw = last_frameRaw(Rect(0, last_frameRaw.rows - img_remain_h, last_frameRaw.cols, img_remain_h));

        // 首帧后退处理
        img_cpartRem.copyTo(cpartRemStore);
        img_cpartRemRaw.copyTo(cpartRemStoreRaw);

        img_cpartNow = img_raw(Rect(0, 0 + overLap, img_raw.cols, img_raw.rows - overLap));
        img_cpartNowRaw = img_raw(Rect(0, 0 + overLap, img_raw.cols, img_raw.rows - overLap));

        vconcat(img_cpartRem, img_cpartNow, img_raw);
        vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
    }
    else
    {
        img_cpartRem = img_remain(Rect(0, img_remain.rows - img_remain_h, img_remain.cols, img_remain_h));
        img_cpartRemRaw = last_frameRaw(Rect(0, last_frameRaw.rows - img_remain_h, last_frameRaw.cols, img_remain_h));
        // 首帧后退处理
        img_cpartRem.copyTo(cpartRemStore);
        img_cpartRemRaw.copyTo(cpartRemStoreRaw);

        img_cpartNow = img_raw(Rect(0, 0 + overLap, img_raw.cols, img_raw.rows - overLap));
        img_cpartNowRaw = img_raw(Rect(0, 0 + overLap, img_raw.cols, img_raw.rows - overLap));

        vconcat(img_cpartRem, img_cpartNow, img_raw);
        vconcat(img_cpartRemRaw, img_cpartNowRaw, img_rawRaw);
    }
    img_raw.copyTo(img_remain);
    img_raw.copyTo(last_frameRaw);

    // 首帧后退处理,检测第一帧
    firstFrameLine = true;
    for (int i = 0; i < cpartRemStore.rows; i++)
    {
        Point rad;
        rad = findValueLine(cpartRemStore, IMGBW_BLACK, i);

        if (rad.x >= 0 && rad.y >= 0)
        {
            firstFrameLine = false;
            break;
        }
    }

    // Debug
    ///结果显示 1
//	Mat img_remain_Debug;
//	img_remain.copyTo(img_remain_Debug);
    //    imshow("img_remain", img_remain);
    // Debug End

    /// 显示图像信息
    rows = img_raw.rows;
    cols = img_raw.cols;

    /// 随边处理
    Mat img_rawRawEdge = edgesbw(img_rawRaw);
    img_rawRawEdge.col(0) = IMGBW_WITHE;
    img_rawRawEdge.col(cols - 1) = IMGBW_WITHE;
    img_rawRaw = img_rawRaw | circleSub(img_rawRawEdge, dist);

    /// 随边与冲孔图像整合
    img_raw = img_raw | img_rawRaw;

    /// 得到整合图像边缘
    imgEdge = edgesbw(img_raw);

    /// 加边框边界，若不需要，移除以下四条语句
    imgEdge.row(0) = IMGBW_WITHE;
    imgEdge.row(rows - 1) = IMGBW_WITHE;
    imgEdge.col(0) = IMGBW_WITHE;
    imgEdge.col(cols - 1) = IMGBW_WITHE;

    /// 得到首个圆心区域
    img_raw = img_raw | circleSub(imgEdge, radius);

    //    // Debug
    //    imshow("centerArea", img_raw);
    //    // Debug End

    /// 主处理循环，获得像素坐标点集合
    Point rad;
    vec.clear();
    int num = 0;
    static int dir = 0;
    int rowStart;
    static Point lastFramePoint = Point(cols, rows);

    /// 连接图像扫描首行值
    rowStart = img_remain_h - lastFramePoint.y + radius + space*0.5 + 1;
    if (rowStart < 0)
    {
        rowStart = 0;
    }

    /// 冲压处理
    //{
    static int fistLineCols = 0;
    //} //modified by Duan@20170616
    for (int i = rowStart; i < (int)rows;)   // Row scan
    {
        rad = findValueLine(img_raw, IMGBW_BLACK, i);

        if (rad.x < 0 || rad.y < 0)
        {
            i++;
            // 检测到整行白色，标志接下来将是新料片头部
            firstFrameLine = true;
        }
        else
        {
            int colStart = 0;
            //{
            int roundLenth=(int)((2*radius+space)*1.732f);
            if(dir)
//                colStart = radius+ dist+1;
                colStart = fistLineCols%(roundLenth) + 1;
            else
//                colStart = 2*radius+space*0.5f+ dist+1;
                colStart = (radius+space*0.5)*1.732f+ fistLineCols%(roundLenth) + 1;
            //} //modified by Duan@20170616

            bool existValidPoint = false;
            int linePoints = 0;
            for (int j = colStart; j < (int)cols; j += (2 * radius + space)*1.732f) //Col scan
            {
                if (img_raw.at<uchar>(i, j) == IMGBW_BLACK)
                {
                    existValidPoint = true;
                    /// 像素坐标数组
                    rad.x = j;
                    vec.append(rad);
                    num++;
                    linePoints++;

                    /// Image concat
                    if (!firstFrameLine)
                        plotc(img_remain, rad, radius);

                    /// If the points number is bigger than CY_maxStep then stop
                    if (num >= CY_MAXSTEP) break;
                }
            }
            /// Next Line
            if (existValidPoint)
            {
                // 首帧后退处理
                if (firstFrameLine)
                {
                    ////////////////////////////////////////////
                    i += (int)radius / 2;     /// 后退的像素值
                                              ////////////////////////////////////////////
                                              //移除之前加入队列的料片头部冲点
                    for (int k = 0; k < linePoints; k++)
                    {
                        vec.removeLast();
                        num--;
                    }
                    firstFrameLine = false;
                    //{
                    Point fistLinePoint = findValueLine(img_raw, IMGBW_BLACK, i);
                    fistLineCols = fistLinePoint.x<0? fistLineCols:fistLinePoint.x;
                    dir=0;
                    //} //modified by Duan@20170616
                }
                else
                    i += radius + space*0.5;   //sqr(3)=1.732f

                dir = !dir;
            }
            else
                i += 1;
        }
    }

    ///得到点集有效个数
    numOfPoints = vec.size();

    ///更新LastPoint
    if (numOfPoints > 0)
    {
        lastFramePoint = vec.last();
        lastFramePoint.y = rows - lastFramePoint.y;
    }
    else
        lastFramePoint = Point(cols, rows);

    ///冲压顺序排序（像素点）
    vec = pointPixForwardSort(vec);

    /// Image concat ,Point modify
    for (int j = 0; j < numOfPoints; j++)
    {
        vec[j].y -= img_remain_h;   //remain concat modify
        vec[j].y += overLap;    // overlap modify
    }


    // Debug
//	///结果显示 1
//	img = img_remain_Debug;
//	for (int j = 0; j < numOfPoints; j++)
//	{
//		/// 坐标映射与输出
//		std::cout << vec[j] << std::endl;
//		/// 冲压结果画圆
//		Point tmp_Point(vec[j]);
//		tmp_Point.y -= overLap;
//		tmp_Point.y += img_remain_h;
//		cv::circle(img, tmp_Point, radius, cv::Scalar(255, 255, 255), 1);
//		/// 顺序显示
//		cv::putText(img, String(std::to_string(j + 1)), tmp_Point - cv::Point(radius / 5.0, -radius / 5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0 / CY_R_MAX, Scalar(255, 0, 0), 1, cv::LINE_AA);
//	}
    // Debug end

    //    // Debug
    //    ///结果显示 2
    //    for(int j=0; j<numOfPoints; j++)
    //    {
    //        /// 坐标映射与输出
    //        std::cout << vec[j] << std::endl;
    //        /// 冲压结果画圆
    //        cv::circle(img, vec[j], radius, cv::Scalar(255, 255, 255), 1);
    //        /// 顺序显示
    //        cv::putText(img, String(std::to_string(j+1)), vec[j]-cv::Point(radius/5.0, -radius/5.0), CV_FONT_HERSHEY_COMPLEX, radius*2.0/CY_R_MAX, Scalar(255, 0, 0), 1, cv::LINE_AA);
    //    }
    //    // Debug end


    return numOfPoints;
}

#endif




////////////////Get Factor/////////////////////
using namespace std;
//S->L
bool my_sort_func(Vec2f i, Vec2f j)
{
    return i[0]<j[0];
}

double cy_algorithm::averageDif(std::vector<Vec2f> &v)
{
    if (v.size() < 2)
        return -1;

    double avr;
    unsigned int N = v.size();
    if (N % 2)
    {
        N -= 1;
    }
    double sum = 0;
    for (size_t i = N/2; i < N; i++)
    {
        sum += v[i][0];
    }
    for (size_t i = 0; i < N / 2; i++)
    {
        sum -= v[i][0];
    }
    avr = 4*sum / (N*N);

    return avr;
}

void cy_algorithm::removeOverlapLines(std::vector<Vec2f> &v)
{
    vector<double> vDiff;
    vector<Vec2f> vClass;
    if (v.size() < 2)
        return;

    // sort
    std::sort(v.begin(), v.end(), my_sort_func);

    // diff
    double diff;
    for (size_t i = 1; i < v.size(); i++)
    {
        diff = v[i][0] - v[i - 1][0];
        vDiff.insert(vDiff.end(), diff);
    }
    double maxDiff = *max_element(vDiff.begin(), vDiff.end());
    vDiff.insert(vDiff.begin(), maxDiff);

    //  find the cluster
    double avrDiff=0;
    for (size_t i = 0; i < vDiff.size(); i++)
    {
        avrDiff += vDiff[i] / vDiff.size();
    }
    //////////////////////////////////
    avrDiff *= 0.83;	// modify arverage(distinguish different cluster from lines, show be [0.25,0.94])
    //////////////////////////////////

    // classify lines
    int clusterNum = 0;
    float clusterSumTheta = 0;
    float clusterSumRho = 0;
    Vec2f aLine;
    for (int i = vDiff.size()-1; i>=0 ; i--)
    {
        clusterNum++;
        clusterSumRho += v[i][0];
        clusterSumTheta += v[i][1];
        if (vDiff[i] > avrDiff)
        {
            aLine[0] = clusterSumRho / clusterNum;
            aLine[1] = clusterSumTheta / clusterNum;
            vClass.insert(vClass.begin(), aLine);
            clusterNum = 0;
            clusterSumTheta = 0;
            clusterSumRho = 0;
        }
    }
    v = vClass;	// assign
}

/*
 * @brief: 得到比例因子
 * @param:
 *          @img        :   二值化的棋盘ROI
 *          @houghValue :   霍夫变换参数，范围[0,180]
 *                          但是此值太小时会导致线条太多，计算时间非常长，因此对外接口范围[100,180]，初始值为150即可
 *          @imgResult  :   得到的线条以红色的形式显示在imgResult上，要求：imgResult与img尺寸相同，但是为彩色图像（如果不是彩色红色显示不出来）
 * @return: Vec2f 包涵两个浮点值   Vec2f[0]:竖直方向(|)单个棋盘格像素平均值    Vec2f[1]:水平方向(——)单个棋盘格像素平均值
*/
Vec2f cy_algorithm::getFactor(cv::Mat& img, int houghValue, cv::Mat& imgResult)
{
    Mat srcImage, midImage;

    img.copyTo(srcImage);
    Canny(srcImage, midImage, 50, 200, 3);
//    imshow("canny", midImage);

//    Mat dstImage, tmpsrc, plotImg;
//    cvtColor(midImage, plotImg, CV_GRAY2BGR);
//    plotImg.copyTo(dstImage);
//    srcImage.copyTo(tmpsrc);

    /// Hough Transform
    vector<Vec2f> lines;
    HoughLines(midImage, lines, 1, CV_PI / 180, houghValue, 0, 0);

    /// Too many lines, return error
    #define MAX_LINE_NUM 100
    if(lines.size()>MAX_LINE_NUM)
        return Vec2f(-1,-1);

    /// Generate lines image
//    cout << "terminal Point:" << endl;
    for (size_t i = 0; i < lines.size(); i++)
    {
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2, pt0;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        int extendLen = 1000;
        pt0 = Point(x0, y0);
        pt1.x = cvRound(x0 + extendLen * (-b));
        pt1.y = cvRound(y0 + extendLen * (a));
        pt2.x = cvRound(x0 - extendLen * (-b));
        pt2.y = cvRound(y0 - extendLen * (a));
        line(imgResult, pt1, pt2, Scalar(55, 55, 255), 1, CV_AA);
//        putText(dstImage, to_string(i + 1), pt0+Point(5, 10), 1, 1, Scalar(255, 0, 0));
//        cout << "#0:\t" << pt0.x << ',' << pt0.y << endl;
//        cout << "#1:\t" << pt1.x << ',' << pt1.y << endl;
//        cout << "#2:\t" << pt2.x << ',' << pt2.y << endl;

    }

    /// Classify
    vector<Vec2f> linesH;
    vector<Vec2f> linesV;
    float Hdir = CV_PI / 2;
    float Vdir = 0;
    for (size_t i = 0; i < lines.size(); i++)
    {
        float theta = lines[i][1];
        cout << theta << endl;
        double Hdist, Vdist;
        Hdist = abs(Hdir-theta);
        Vdist = abs(Vdir-theta);
        if (Hdist < Vdist)	// Horical direction
        {
            linesH.insert(linesH.end(), lines[i]);
        }
        else // Vertical direction
        {
            linesV.insert(linesV.end(), lines[i]);
        }
    }
//    cout << endl << "# Classify Result #" << endl;
//    cout << lines.size() << endl;
//    cout << linesV.size() << endl;
//    cout << linesH.size() << endl;

    /// Remove overlap lines
    removeOverlapLines(linesV);
    removeOverlapLines(linesH);

    /// Average distance
    double avr_distV = averageDif(linesV);
    double avr_distH = averageDif(linesH);

    cout <<endl << "# Avr Distance # "<< endl;
    cout << "H avr Distance: "<< avr_distH << endl;
    cout << "V avr Distance: "<< avr_distV << endl;
//    imshow("tmp src", tmpsrc);
//    imshow("hough", dstImage);

    return Vec2f(avr_distV,avr_distH);
}

////////////实心形状//////////////

/*
 * @brief: 画一个实心圆
 * @param:
 *          img: 画图图像
 *          rad: 矩形中心坐标
 *          r : 半径
 * @return：
 *          img画好实心矩形的图形
*/
cv::Mat cy_algorithm::plotc(cv::Mat& img, cv::Point rad, int r)
{
    cv::circle(img, rad, r, cv::Scalar(255, 255, 255), -1);
    return img;
}

/*
 * @brief: 画一个实心矩形
 * @param:
 *          img: 画图图像
 *          rad: 矩形中心坐标
 *          xr : x方向半径
 *          yr : y方向半径
 * @return：
 *          img画好实心矩形的图形
*/
cv::Mat cy_algorithm::plotr(cv::Mat& img, cv::Point rad, int xr, int yr)
{
    cv::rectangle(img,cv::Rect(rad.x-xr, rad.y-yr, xr*2, yr*2),cv::Scalar(255, 255, 255), -1, 8); //线宽为-1，向内填充
    return img;
}

/*
 * @brief: 画一个实心六边形
 * @param:
 *          img: 画图图像
 *          rad: 矩形中心坐标
 *          r : 内切圆半径
 * @return：
 *          img画好实心矩形的图形
*/
cv::Mat cy_algorithm::plotp(cv::Mat& img, cv::Point rad, int r)
{
    double factor1 = tan(CV_PI/6);
    int h = (int)(factor1*r);
    double factor2 = 1/cos(CV_PI/6);
    int Ro = (int)(factor2*r);

    //定义Point类型数组，存放六边形的点
    cv::Point ppt[6] = {cv::Point(rad.x-r, rad.y+h), cv::Point(rad.x, rad.y+Ro), cv::Point(rad.x+r, rad.y+h), cv::Point(rad.x+r, rad.y-h), cv::Point(rad.x, rad.y-Ro), cv::Point(rad.x-r, rad.y-h)};
    const cv::Point* pts[1] = {ppt};//ppt类型为Point*，pts类型为Point**，需定义成const类型
    int npt[1] ={6};  //npt的类型即为int*
    cv::fillPoly(img, pts, npt, 1, cv::Scalar(255, 255, 255), 8);

    return img;
}



////////////空心形状//////////////

/*
 * @brief: 画一个空心圆
 * @param:
 *          img: 画图图像
 *          rad: 矩形中心坐标
 *          r : 半径
 * @return：
 *          img画好实心矩形的图形
*/
cv::Mat cy_algorithm::plotcEmpty(cv::Mat& img, cv::Point rad, int r)
{
    cv::circle(img, rad, r, cv::Scalar(255, 255, 255), 0);
    return img;
}

/*
 * @brief: 画一个空心矩形
 * @param:
 *          img: 画图图像
 *          rad: 矩形中心坐标
 *          xr : x方向半径
 *          yr : y方向半径
 * @return：
 *          img画好实心矩形的图形
*/
cv::Mat cy_algorithm::plotrEmpty(cv::Mat& img, cv::Point rad, int xr, int yr)
{
    cv::rectangle(img,cv::Rect(rad.x-xr, rad.y-yr, xr*2, yr*2),cv::Scalar(255, 255, 255), 0, 8); //线宽为-1，向内填充
    return img;
}

/*
 * @brief: 画一个空心六边形
 * @param:
 *          img: 画图图像
 *          rad: 矩形中心坐标
 *          r : 内切圆半径
 * @return：
 *          img画好实心矩形的图形
*/
cv::Mat cy_algorithm::plotpEmpty(cv::Mat& img, cv::Point rad, int r)
{
    double factor1 = tan(CV_PI/6);
    int h = (int)(factor1*r);
    double factor2 = 1/cos(CV_PI/6);
    int Ro = (int)(factor2*r);

    //定义Point类型数组，存放六边形的点
    cv::Point ppt[6] = {cv::Point(rad.x-r, rad.y+h), cv::Point(rad.x, rad.y+Ro), cv::Point(rad.x+r, rad.y+h), cv::Point(rad.x+r, rad.y-h), cv::Point(rad.x, rad.y-Ro), cv::Point(rad.x-r, rad.y-h)};
    const cv::Point* pts[1] = {ppt};//ppt类型为Point*，pts类型为Point**，需定义成const类型
    int npt[1] ={6};  //npt的类型即为int*
    cv::polylines(img, pts, npt, 1, true, cv::Scalar(255, 255, 255), 0, 8);

    return img;
}

