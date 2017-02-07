#include <iostream>
#include <opencv2/opencv.hpp>
#include "chongya.h"
#include "globalParam.h"

using namespace std;
using namespace cv;

const string TrackbarWindow = "Control";
const string TrackbarVorP = "Video/Proc";
const string TrackbarRorE = "Run/Exit";
const string TrackbarValueThresh = "Threshold"; //"灰度阈值"
const string TrackbarValueR = "R"; //"冲孔半径"
const string TrackbarValueDelta = "D_hole2hole"; //"冲孔间距"
const string TrackbarValueDist = "D_side2hole"; //"随边间距"
const string TrackbarValueInvert = "ImgInvert"; //"图像反色"
const string TrackbarValueX = "X"; //检测区域左上坐标x
const string TrackbarValueY = "Y"; //检测区域左上坐标y
const string TrackbarValueL = "L"; //检测区域长度L
const string TrackbarValueH = "H"; //检测区域高度H

Rect CY_rect(0,0,0,0); //检测区域

int TrackbarVorP_value = 0;
int TrackbarRorE_value = 0;

/*Function Declaration*/
void on_trackbar(int, void*);
void onMouse(int event, int x, int y, int flag, void *param);

int main(int argc, char* argv[])
{
	VideoCapture vid;
	Mat frame;
	Mat imgbw;

	vid.open(0);
	if (!vid.isOpened())
	{
		cout << "Can not open CAM!" << endl;
		return -1;
	}	
	
	namedWindow(TrackbarWindow,2);
	createTrackbar(TrackbarVorP, TrackbarWindow, &TrackbarVorP_value, 1, on_trackbar);
	createTrackbar(TrackbarRorE, TrackbarWindow, &TrackbarRorE_value, 1, on_trackbar);
	createTrackbar(TrackbarValueThresh, TrackbarWindow, &CY_bw_thresh, CY_bw_thresh_Max, on_trackbar);
	createTrackbar(TrackbarValueR, TrackbarWindow, &CY_r, CY_r_Max, on_trackbar);
	createTrackbar(TrackbarValueDelta, TrackbarWindow, &CY_delta, CY_delta_Max, on_trackbar);
	createTrackbar(TrackbarValueDist, TrackbarWindow, &CY_dist, CY_dist_Max, on_trackbar);
	createTrackbar(TrackbarValueInvert, TrackbarWindow, &CY_img_invert, CY_img_invert_Max, on_trackbar);

	createTrackbar(TrackbarValueX, TrackbarWindow, &CY_ROI_x, CY_ROI_x_Max, on_trackbar);
	createTrackbar(TrackbarValueY, TrackbarWindow, &CY_ROI_y, CY_ROI_y_Max, on_trackbar);
	createTrackbar(TrackbarValueL, TrackbarWindow, &CY_ROI_L, CY_ROI_L_Max, on_trackbar);
	createTrackbar(TrackbarValueH, TrackbarWindow, &CY_ROI_H, CY_ROI_H_Max, on_trackbar);


	namedWindow("Video");
	setMouseCallback("Video", onMouse);


	bool stop(false);
	while (!stop)
	{
		if (!vid.read(frame))
		{
			cout << "Can get a frame!" << endl;
			break;
		}
		
		//选择检测区域
		rectangle(frame, CY_rect, Scalar(0, 0, 255), 1);
		imshow("Video", frame);
		
		//截取检测区域
		if (CY_rect.width > 0 && CY_rect.height > 0 \
			&&CY_rect.width+CY_rect.x < frame.cols && CY_rect.height+CY_rect.y < frame.rows \
			&&CY_rect.x > 0 && CY_rect.y > 0\
			&&CY_rect.x < frame.cols && CY_rect.y < frame.rows)
		{
			frame = frame(CY_rect);
		}

		imgbw = resize2bw(frame);
		imshow("Imgbw", imgbw);

		if (waitKey(30) >= 0)
			stop = true;

		if (TrackbarVorP_value)
		{
			chongya(frame);
			TrackbarVorP_value = 0;
			createTrackbar(TrackbarVorP, TrackbarWindow, &TrackbarVorP_value, 1, on_trackbar);
		}
		if (TrackbarRorE_value)
		{
			stop = true;
		}
		
		waitKey(1);
	}
	
	
	vid.release();
	waitKey(0);
	return 0;
}

void on_trackbar(int, void*)
{
	return;
}

void onMouse(int event, int x, int y, int flag, void *param)
{

	switch (event)
	{
	case EVENT_LBUTTONDOWN:
		CY_rect.x = x;
		CY_rect.y = y;
		break;

	case EVENT_LBUTTONUP:
		CY_rect.width = x - CY_rect.x;
		CY_rect.height = y - CY_rect.y;
		break;

	case EVENT_MOUSEMOVE:
		if (flag & EVENT_FLAG_LBUTTON)
		{
			CY_rect.width = x - CY_rect.x;
			CY_rect.height = y - CY_rect.y;
		}
		break;
	default:
		break;
	}
}

