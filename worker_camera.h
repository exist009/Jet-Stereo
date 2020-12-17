#ifndef WORKER_CAMERA_H
#define WORKER_CAMERA_H

#include <QObject>

#include <camera.h>

class Worker_camera : public QObject
{
    Q_OBJECT
public:
    explicit Worker_camera(Sensor sensor, QObject *parent = nullptr);
    ~Worker_camera();

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

#endif // WORKER_CAMERA_H
