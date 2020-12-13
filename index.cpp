#include "index.h"
#include "ui_index.h"

#include <QDebug>

#include <opencv2/opencv.hpp>
#include <worker.h>

Q_DECLARE_METATYPE(cv::Mat)

Index::Index(QWidget *parent) : QWidget(parent), ui(new Ui::Index)
{
    ui->setupUi(this);

    qRegisterMetaType<cv::Mat>();

    for (auto i = 0; i < 2; i++)
    {
        auto sensor = static_cast<Sensor>(i);
        auto worker = new Worker(sensor);
        auto thread = new QThread;

        worker->moveToThread(thread);

        connect(thread, &QThread::started, worker, &Worker::process);
        connect(worker, &Worker::finished, thread, &QThread::quit, Qt::DirectConnection);
        connect(worker, &Worker::finished, worker, &Worker::deleteLater, Qt::DirectConnection);

        connect(worker, &Worker::frame, this, [&, sensor](const cv::Mat &frame)
        {
            auto image = QImage(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);

            if (sensor == Sensor::Left) this->ui->sensor_0->setPixmap(QPixmap::fromImage(image));
            if (sensor == Sensor::Right) this->ui->sensor_1->setPixmap(QPixmap::fromImage(image));
        });

        connect(this, &Index::stop, worker, &Worker::stop, Qt::DirectConnection);

        this->thread_list.push_back(thread);

        thread->start();
    }
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
