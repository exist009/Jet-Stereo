#include "camera.h"

#include <QDebug>

Camera::Camera(Sensor sensor, const Resolution &camera, const Resolution &display, qint32 framerate, QObject *parent) : QObject(parent)
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

//    cv::Mat mat_gray;
//    cv::cvtColor(mat, mat_gray, cv::COLOR_BGR2GRAY);

    //emit frame(QImage(mat_gray.data, mat_gray.cols, mat_gray.rows, mat_gray.step, QImage::Format_Grayscale8));

//    std::vector<cv::Rect> object_list;
//    std::vector<int> detect_list;

//    this->cascade.detectMultiScale(mat_gray, object_list, detect_list, 1.3, 5);

//    for (const auto &object : object_list)
//    {
//        cv::rectangle(mat, object, cv::Scalar(255, 0, 0), 2);
//    }

    cv::cvtColor(mat, mat, cv::COLOR_BGR2RGB);

    emit frame(mat);

    return true;
}
