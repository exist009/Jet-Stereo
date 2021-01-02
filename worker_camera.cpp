#include "worker_camera.h"

#include <camera_sync.h>

#include <QDateTime>
#include <QThread>

Worker_camera::Worker_camera(Sensor sensor, QObject *parent) : QObject(parent), camera(nullptr), sensor(sensor), run(true) { }
Worker_camera::~Worker_camera() { }

void Worker_camera::process()
{
    this->camera = new Camera(this->sensor, { 320, 240 }, { 320, 240 }, 30);

    connect(this->camera, &Camera::frame, this, &Worker_camera::frame);

	auto previous_frame_time(QDateTime::currentMSecsSinceEpoch());
	auto current_frame_time(previous_frame_time);

    while (this->run)
    {
		Camera_sync::sync_wait(this->sensor);

        this->camera->capture();

		current_frame_time = QDateTime::currentMSecsSinceEpoch();

		emit fps(1000 / (current_frame_time - previous_frame_time));

		previous_frame_time = current_frame_time;

		Camera_sync::inc_index(this->sensor);
    }

    delete this->camera;

    emit finished();
}

void Worker_camera::stop()
{
    this->run = false;
}
