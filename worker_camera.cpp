#include "worker_camera.h"

#include <camera_sync.h>

#include <QThread>

Worker_camera::Worker_camera(Sensor sensor, QObject *parent) : QObject(parent), camera(nullptr), sensor(sensor), run(true) { }
Worker_camera::~Worker_camera() { }

void Worker_camera::process()
{
    Camera_sync::init_wait();

    this->camera = new Camera(this->sensor, { 320, 240 }, { 320, 240 }, 30);

    connect(this->camera, &Camera::frame, this, &Worker_camera::frame);

    while (this->run)
    {
        this->camera->capture();
        QThread::msleep(1000);
    }

    delete this->camera;

    emit finished();
}

void Worker_camera::stop()
{
    this->run = false;
}
