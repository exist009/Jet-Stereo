#ifndef CAMERA_SYNC_H
#define CAMERA_SYNC_H

#include <QMutex>
#include <QSemaphore>
#include <QWaitCondition>
#include <utils.h>

class Camera_sync
{
public:
	static Camera_sync &instance()
	{
		static Camera_sync instance;
		return instance;
	}

private:
	Camera_sync() {}
	Camera_sync(const Camera_sync &);
	Camera_sync &operator=(Camera_sync &);

public:
	static void init()
	{
		instance()._l_index = 0;
		instance()._r_index = 0;
	}

	static void inc_index(Sensor sensor)
	{
		QMutexLocker locker(&instance()._sync_locker);

		switch (sensor)
		{
			case Sensor::Left: instance()._l_index++; break;
			case Sensor::Right: instance()._r_index++; break;
		}

		instance()._condition.wakeAll();
	}

	static void sync_wait(Sensor sensor)
	{
		QMutexLocker locker(&instance()._sync_locker);

		qint64 a(instance()._l_index);
		qint64 b(instance()._r_index);

		if (sensor != Sensor::Left) std::swap(a, b);

		if (a > b)
		{
			instance()._condition.wait(&instance()._sync_locker);
		}
	}

private:
	qint64 _l_index;
	qint64 _r_index;
	QMutex _sync_locker;
	QWaitCondition _condition;

};

#endif // CAMERA_SYNC_H
