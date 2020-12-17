#include "worker_imu.h"

#include <QThread>

Worker_imu::Worker_imu(QObject *parent) : QObject(parent), device(nullptr), run(true) { }
Worker_imu::~Worker_imu() { }

void Worker_imu::process()
{
    this->device = new IMU::Device();

    while (this->run)
    {
        emit data(this->device->get_gyroscope(), this->device->get_accelerometer(), this->device->get_magnetometer());
        QThread::msleep(100);
    }

    delete this->device;

    emit finished();
}

void Worker_imu::stop()
{
    this->run = false;
}
