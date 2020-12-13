#include "worker.h"

#include <QDebug>
#include <QThread>

Worker::Worker(Sensor sensor, QObject *parent)
    : QObject(parent), camera(nullptr), sensor(sensor), run(true) { }
Worker::~Worker() { }

void Worker::process()
{
    this->camera = new Camera(this->sensor, { 320, 240 }, { 320, 240 }, 30);

    connect(this->camera, &Camera::frame, this, &Worker::frame);

    while (this->run)
    {
        this->camera->capture();
    }

    delete this->camera;

    emit finished();
}

void Worker::stop()
{
    this->run = false;
}
