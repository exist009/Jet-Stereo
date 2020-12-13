#ifndef WORKER_H
#define WORKER_H

#include <QObject>

#include <camera.h>

class Worker : public QObject
{
    Q_OBJECT
public:
    explicit Worker(Sensor sensor, QObject *parent = nullptr);
    ~Worker();

signals:
    void finished();
    void frame(const cv::Mat &frame);

public slots:
    void process();
    void stop();

private:
    Camera *camera;
    Sensor sensor;
    volatile bool run;
};

#endif // WORKER_H
