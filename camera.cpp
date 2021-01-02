#include "camera.h"

#include <QDebug>

Camera::Camera(Sensor sensor, const Resolution &camera, const Resolution &display, qint32 framerate, QObject *parent) : QObject(parent), index(0)
{
    this->sensor.open(Utility::Pipeline(sensor, camera, framerate, Flip::Rotate180, display).toStdString(), cv::CAP_GSTREAMER);

    if (!this->sensor.isOpened())
    {
        qDebug() << "Camera open error";
        return;
    }
}

Camera::~Camera()
{
    this->sensor.release();
}

bool Camera::capture()
{
    cv::Mat mat;

    if (!this->sensor.read(mat))
    {
        return false;
    }

    cv::cvtColor(mat, mat, cv::COLOR_BGR2RGB);

	emit frame(this->index++, mat);

    return true;
}
