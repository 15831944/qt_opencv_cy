#include <iostream>
#include <QTimer>
#include "cy_performance.h"

using namespace std;

cy_performance::cy_performance(QObject *parent) : QObject(parent)
{
   perf_time = new QTime;
}

cy_performance::~cy_performance()
{
}

/**
* @brief 计时开始 
*/
void cy_performance::time_start(void)
{
    perf_time->start();
}

/**
* @brief 计时结束
* @return 计时时间
*/
int cy_performance::time_end(void)
{
    int time_diff = 0; //<时间差
    time_diff = perf_time->elapsed();
	//cout << time_diff << " ms" << endl;
    return time_diff;
}
