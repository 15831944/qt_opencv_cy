#ifndef __GLOBALPARAM_H
#define __GLOBALPARAM_H

#include <opencv2/opencv.hpp>

#define CAM_INDEX 0

#define IMGBW_BLACK 0
#define IMGBW_WITHE 255
#define IMGBW_THRESH_B2W 200

//#define IMG_RESIZE //尺寸过大是否调整尺寸  --Not recommend now
#define CY_SHOW_PROC    //显示处理过程 --Can be use for debug


#define CY_bw_thresh_Max 255		// 灰度阈值最大值
#define CY_r_Max 100				// 冲孔半径最大值
#define CY_delta_Max 100			// 冲孔间距最大值
#define CY_dist_Max 100				// 随边间距最大值
#define CY_img_invert_Max 1		// 反色选择

#define CY_ROI_x_Max 1000	//检测区域左上坐标x最大值
#define CY_ROI_y_Max 1000	//检测区域左上坐标y最大值
#define CY_ROI_L_Max 1000	//检测区域长度L最大值
#define CY_ROI_H_Max 1000	//检测区域高度H最大值



/// Global parameter clarations
/// -Definition in chongya.cpp-
extern int CY_bw_thresh;		// 灰度阈值
extern int CY_r;				// 冲孔半径
extern int CY_maxStep;		// 单次最大冲孔数量
extern int CY_delta;			// 冲孔间距
extern int CY_dist;				// 随边间距
extern int CY_img_invert;		// 反色选择

extern int CY_ROI_x;	//检测区域左上坐标x
extern int CY_ROI_y;	//检测区域左上坐标y
extern int CY_ROI_L;	//检测区域长度L
extern int CY_ROI_H;	//检测区域高度H

//extern cv::Point_<double> RadArray[400+1];

#endif // !__GLOBALPARAM_H
