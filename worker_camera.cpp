#include "worker_camera.h"

#include <QThread>

Worker_camera::Worker_camera(Sensor sensor, QObject *parent) : QObject(parent), camera(nullptr), sensor(sensor), run(true) { }
Worker_camera::~Worker_camera() { }

void Worker_camera::process()
{
    this->camera = new Camera(this->sensor, { 320, 240 }, { 320, 240 }, 30);

    connect(this->camera, &Camera::frame, this, &Worker_camera::frame);

    while (this->run)
    {
        this->camera->capture();
    }

    delete this->camera;

    emit finished();
}

void Worker_camera::stop()
{
    this->run = false;
}
