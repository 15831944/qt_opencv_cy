#ifndef __CHONGYA_H
#define __CHONGYA_H

#include <opencv2/opencv.hpp>
#include <QVector>

// Var name enum
typedef enum
{
    _CY_bw_thresh  = 0x00,
    _CY_r = 0x01,
    _CY_maxStep = 0x02,
    _CY_delta = 0x03,
    _CY_dist = 0x04,
    _CY_img_invert = 0x05,
    _CY_ROI_x = 0x06,
    _CY_ROI_y = 0x07,
    _CY_ROI_L = 0x08,
    _CY_ROI_H = 0x09
} CY_VAR_NAME;
#define IS_VAR_NAME(var) (((var) == _CY_bw_thresh) || ((var) == _CY_r) ||\
                          ((var) == _CY_maxStep)   || ((var) == _CY_delta) ||\
                          ((var) == _CY_dist)      || ((var) == _CY_img_invert) ||\
                          ((var) == _CY_ROI_x)     || ((var) == _CY_ROI_y)||\
                          ((var) == _CY_ROI_L)     || ((var) == _CY_ROI_H))


class cy
{
public:
    explicit cy();
    cv::Mat chongya(cv::Mat& img);
    cv::Mat resize2bw(cv::Mat& img);

    /// Get functions
//    /* Pix vout, reserved*/
//    inline const QVector<cv::Point>& get_vout() const{
//        return vout;
//    }
    inline const QVector<cv::Point_<double>>& get_voutf() const{
        return voutf;
    }
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
    /// Set funstions
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

private:
    ///functions
    void cy::cyVarInit();
    cv::Point_<double> mapPoint(cv::Point pin, cv::Point pinMax, double x, double y, double L, double H);
    int cy::mapScale(double realScale, cv::Point pinMax, double L, double H);
    QVector<cv::Point> cy::pointPixSort(QVector<cv::Point> &vin);

    ///vars
    int CY_bw_thresh;		// 灰度阈值
    int CY_r;				// 冲孔半径
    int CY_maxStep;         // 单次最大冲孔数量
    int CY_delta;			// 冲孔间距
    int CY_dist;			// 随边间距
    int CY_img_invert;		// 反色选择
    int CY_ROI_x;           //检测区域左上坐标x
    int CY_ROI_y;           //检测区域左上坐标y
    int CY_ROI_L;           //检测区域长度L
    int CY_ROI_H;           //检测区域高度H
    QVector<cv::Point> vout;             //像素坐标点集输出
    QVector<cv::Point_<double>> voutf;   //实际坐标点集输出
};

#endif // !__CHONGYA_H
