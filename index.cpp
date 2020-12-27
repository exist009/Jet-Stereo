#include "index.h"
#include "ui_index.h"

#include <QDebug>

#include <worker_camera.h>
#include <worker_imu.h>

#include <camera_sync.h>

Index::Index(QWidget *parent) : QWidget(parent), ui(new Ui::Index)
{
    ui->setupUi(this);

    qRegisterMetaType<cv::Mat>();

    qRegisterMetaType<IMU::Device::Gyroscope>();
    qRegisterMetaType<IMU::Device::Accelerometer>();
    qRegisterMetaType<IMU::Device::Magnetometer>();
	qRegisterMetaType<IMU::Device::Angle>();

    for (auto i = 0; i < 2; i++)
    {
        auto sensor = static_cast<Sensor>(i);
        auto worker_camera = new Worker_camera(sensor);
        auto thread_camera = new QThread;

        worker_camera->moveToThread(thread_camera);

        connect(thread_camera, &QThread::started, worker_camera, &Worker_camera::process);
        connect(worker_camera, &Worker_camera::finished, thread_camera, &QThread::quit, Qt::DirectConnection);
        connect(worker_camera, &Worker_camera::finished, worker_camera, &Worker_camera::deleteLater, Qt::DirectConnection);

        connect(worker_camera, &Worker_camera::frame, this, [&, sensor](const cv::Mat &frame)
        {
            // cv::imwrite("", frame);

            auto image = QImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);

            if (sensor == Sensor::Left) this->ui->sensor_0->setPixmap(QPixmap::fromImage(image));
            if (sensor == Sensor::Right) this->ui->sensor_1->setPixmap(QPixmap::fromImage(image));
        });

        connect(this, &Index::stop, worker_camera, &Worker_camera::stop, Qt::DirectConnection);

        this->thread_list.push_back(thread_camera);

        thread_camera->start();
    }

    Camera_sync::init();

    //! IMU

    auto worker_imu = new Worker_imu;
    auto thread_imu = new QThread;

    worker_imu->moveToThread(thread_imu);

    connect(thread_imu, &QThread::started, worker_imu, &Worker_imu::process);
    connect(worker_imu, &Worker_imu::finished, thread_imu, &QThread::quit, Qt::DirectConnection);
    connect(worker_imu, &Worker_imu::finished, worker_imu, &Worker_imu::deleteLater, Qt::DirectConnection);

    connect(worker_imu, &Worker_imu::data, this, [&](const IMU::Device::Gyroscope &gyroscope, const IMU::Device::Accelerometer &accelerometer, const IMU::Device::Magnetometer &magnetometer, const IMU::Device::Angle &angle)
    {
        this->ui->imu_gyroscope_x->setText(QString::number(gyroscope.x, 'f', 2));
        this->ui->imu_gyroscope_y->setText(QString::number(gyroscope.y, 'f', 2));
        this->ui->imu_gyroscope_z->setText(QString::number(gyroscope.z, 'f', 2));

        this->ui->imu_accelerometer_x->setText(QString::number(accelerometer.x, 'f', 2));
        this->ui->imu_accelerometer_y->setText(QString::number(accelerometer.y, 'f', 2));
        this->ui->imu_accelerometer_z->setText(QString::number(accelerometer.z, 'f', 2));

        this->ui->imu_magnetometer_x->setText(QString::number(magnetometer.x, 'f', 2));
        this->ui->imu_magnetometer_y->setText(QString::number(magnetometer.y, 'f', 2));
        this->ui->imu_magnetometer_z->setText(QString::number(magnetometer.z, 'f', 2));

        this->ui->imu_angle_yaw->setText(QString::number(angle.yaw, 'f', 0));
        this->ui->imu_angle_pitch->setText(QString::number(angle.pitch, 'f', 0));
        this->ui->imu_angle_roll->setText(QString::number(angle.roll, 'f', 0));
    });

    connect(this, &Index::stop, worker_imu, &Worker_imu::stop, Qt::DirectConnection);

    this->thread_list.push_back(thread_imu);

    thread_imu->start();
}

Index::~Index()
{
    emit stop();

    foreach (const auto &thread, this->thread_list)
    {
        thread->wait();
    }

    qDeleteAll(this->thread_list);

    delete ui;
}
