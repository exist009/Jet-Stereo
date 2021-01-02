#include "worker_camera.h"

#include <camera_sync.h>
#include <fps.h>

#include <QDateTime>
#include <QThread>

Worker_camera::Worker_camera(Sensor sensor, QObject *parent) : QObject(parent), camera(nullptr), sensor(sensor), run(true) { }
Worker_camera::~Worker_camera() { }

void Worker_camera::process()
{
	this->camera = new Camera(this->sensor, { 320, 240 }, { 320, 240 }, 30);

    connect(this->camera, &Camera::frame, this, &Worker_camera::frame);

	Fps fps;

    while (this->run)
    {
		Camera_sync::sync_wait(this->sensor);

        this->camera->capture();

		emit framerate(fps.get());

		Camera_sync::inc_index(this->sensor);
    }

    delete this->camera;

    emit finished();
}

void Worker_camera::stop()
{
    this->run = false;
}
