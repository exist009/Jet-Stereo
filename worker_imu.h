#ifndef WORKER_IMU_H
#define WORKER_IMU_H

#include <QObject>

#include <imu.h>

class Worker_imu : public QObject
{
    Q_OBJECT
public:
    explicit Worker_imu(QObject *parent = nullptr);
    ~Worker_imu();

signals:
    void finished();
    void data(const IMU::Device::Gyroscope &gyroscope, const IMU::Device::Accelerometer &accelerometer, const IMU::Device::Magnetometer &magnetometer, const IMU::Device::Angle &angle);

public slots:
    void process();
    void stop();

private:
    IMU::Device *device;
    volatile bool run;
};

#endif // WORKER_IMU_H
