/**
* @file cy_preproc.h
* 冲压预处理
* 该文件源码描述了冲压变量ID，获取与设置冲压变量接口函数等
* @date 2018-02-02
*/

#ifndef __CHONGYA_H
#define __CHONGYA_H

#include <QVector>
#include <opencv2/opencv.hpp>

/**
* @brief 冲压变量ID枚举
*/ 
typedef enum
{
    _CY_bw_thresh  = 0x00,	//<二值化阈值
    _CY_r = 0x01,	//<冲压半径
    _CY_maxStep = 0x02,	//<最大冲孔步数
    _CY_delta = 0x03,	//<冲孔间距
    _CY_dist = 0x04,	//<随边间距
    _CY_img_invert = 0x05,	//<图像反色
    _CY_ROI_x = 0x06,	//<ROI左上原点坐标x
    _CY_ROI_y = 0x07,	//<ROI左上原点坐标y
    _CY_ROI_L = 0x08,	//<ROI宽度L
    _CY_ROI_H = 0x09	//<ROI高度H
} CY_VAR_NAME;

/**
* @brief 判定是否有效冲压变量ID
*/
#define IS_VAR_NAME(var) (((var) == _CY_bw_thresh) || ((var) == _CY_r) ||\
                          ((var) == _CY_maxStep)   || ((var) == _CY_delta) ||\
                          ((var) == _CY_dist)      || ((var) == _CY_img_invert) ||\
                          ((var) == _CY_ROI_x)     || ((var) == _CY_ROI_y)||\
                          ((var) == _CY_ROI_L)     || ((var) == _CY_ROI_H))

/**
* @class cy_preproc
* @brief 冲压预处理类
*/
class cy_preproc
{
public:
    explicit cy_preproc();
    cv::Mat rgb2bw(cv::Mat& img);
	void cyVarInit();

	/** 
	* @brief 获取像素坐标点实际值 
	* @return 冲孔坐标点实际值向量组
	*/
    inline const QVector<cv::Point_<double>>& get_voutf() const{
        return voutf;
    }

	/** 
	* @brief 获取冲压变量值
	* @param var 冲压变量ID
	* @return 冲压变量值
	*/
    inline const int& get(CY_VAR_NAME var) const{
        assert(IS_VAR_NAME(var));
        switch (var) {
        case _CY_bw_thresh: return CY_bw_thresh;break;
        case _CY_r: return CY_r;break;
        case _CY_maxStep: return CY_maxStep;break;
        case _CY_delta: return CY_delta;break;
        case _CY_dist: return CY_dist;break;
        case _CY_img_invert: return CY_img_invert;break;
        case _CY_ROI_x: return CY_ROI_x;break;
        case _CY_ROI_y: return CY_ROI_y;break;
        case _CY_ROI_L: return CY_ROI_L;break;
        case _CY_ROI_H: return CY_ROI_H;break;
        default:break;
        }
    }

	/**
	* @brief 设置冲压变量值
	* @param var 冲压变量ID
	* @param value 冲压变量值
	*/
    inline void set(CY_VAR_NAME var, int value){
        assert(IS_VAR_NAME(var));
        switch (var) {
        case _CY_bw_thresh: CY_bw_thresh = value;break;
        case _CY_r: CY_r = value;break;
        case _CY_maxStep: CY_maxStep = value;break;
        case _CY_delta: CY_delta = value;break;
        case _CY_dist: CY_dist = value;break;
        case _CY_img_invert: CY_img_invert = value;break;
        case _CY_ROI_x: CY_ROI_x = value;break;
        case _CY_ROI_y: CY_ROI_y = value;break;
        case _CY_ROI_L: CY_ROI_L = value;break;
        case _CY_ROI_H: CY_ROI_H = value;break;
        default:break;
        }
    }

    ///冲压变量
    int CY_bw_thresh;		//<灰度阈值
    int CY_r;				//<冲孔半径
    int CY_maxStep;         //<单次最大冲孔数量
    int CY_delta;			//<冲孔间距
    int CY_dist;			//<随边间距
    int CY_img_invert;		//<反色选择
    int CY_ROI_x;           //<检测区域左上坐标x
    int CY_ROI_y;           //<检测区域左上坐标y
    int CY_ROI_L;           //<检测区域长度L
    int CY_ROI_H;           //<检测区域高度H
	
	///预处理及坐标变量
    int CY_houghValue;      //<霍夫变换值
    QVector<cv::Point> vout;             //<像素坐标点集输出
    QVector<cv::Point_<double>> voutf;   //<实际坐标点集输出
};

#endif // !__CHONGYA_H
