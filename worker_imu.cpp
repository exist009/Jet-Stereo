#include "worker_imu.h"

#include <QThread>

Worker_imu::Worker_imu(QObject *parent) : QObject(parent), device(nullptr), run(true) { }
Worker_imu::~Worker_imu() { }

void Worker_imu::process()
{
    this->device = new IMU::Device();

    while (this->run)
    {
		auto gyroscope = this->device->get_gyroscope();
		auto accelerometer = this->device->get_accelerometer();
		auto magnetometer = this->device->get_magnetometer();
		auto angle = this->device->get_angle(gyroscope, accelerometer, magnetometer);

        emit data(gyroscope, accelerometer, magnetometer, angle);
        QThread::msleep(100);
    }

    delete this->device;

    emit finished();
}

void Worker_imu::stop()
{
    this->run = false;
}
