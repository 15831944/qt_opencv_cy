/**
* @file cy_performance.h
* 算法耗时计算
* 该文件源码定义了一个定时器的开始|结束函数，用以获得计时时间
* @date 2018-02-02
*/

#ifndef CY_PERFORMANCE_H
#define CY_PERFORMANCE_H

#include <QObject>
#include <QTime>

class cy_performance : public QObject
{
    Q_OBJECT
public:
    explicit cy_performance(QObject *parent = 0);
    ~cy_performance();

    void time_start(void);
    int time_end(void);

signals:

public slots:

private:
        QTime *perf_time;
};

#endif // CY_PERFORMANCE_H
