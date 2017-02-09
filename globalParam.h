#ifndef __GLOBALPARAM_H
#define __GLOBALPARAM_H

#include <opencv2/opencv.hpp>

#define CAM_INDEX 0             //相机index值
#define IMGBW_BLACK 0           //opencv黑色值
#define IMGBW_WITHE 255         //opencv白色值
#define IMGBW_THRESH_B2W 200    //四方向二值图边缘检测阈值

//#define IMG_RESIZE      //尺寸过大是否调整尺寸  --Not recommend now

#define CY_bw_thresh_Max 255		// 灰度阈值最大值
#define CY_r_Max 100				// 冲孔半径最大值
#define CY_delta_Max 100			// 冲孔间距最大值
#define CY_dist_Max 100				// 随边间距最大值
#define CY_img_invert_Max 1         // 反色选择最大值

#define CY_ROI_x_Max 1000	//检测区域左上坐标x最大值
#define CY_ROI_y_Max 1000	//检测区域左上坐标y最大值
#define CY_ROI_L_Max 1000	//检测区域长度L最大值
#define CY_ROI_H_Max 1000	//检测区域高度H最大值

#endif // !__GLOBALPARAM_H
