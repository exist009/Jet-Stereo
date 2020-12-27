#ifndef CAMERA_H
#define CAMERA_H

#include <QObject>
#include <QImage>

#include <opencv2/opencv.hpp>
#include <utils.h>

class Camera : public QObject
{
    Q_OBJECT
public:
    explicit Camera(Sensor sensor, const Resolution &camera, const Resolution &display, qint32 framerate, QObject *parent = nullptr);
    ~Camera();

public:
    bool capture();

private:
    cv::VideoCapture sensor;

signals:
    void frame(const cv::Mat &frame);

public slots:
};

Q_DECLARE_METATYPE(cv::Mat)

#endif // CAMERA_H
