#ifndef CAMERA_SYNC_H
#define CAMERA_SYNC_H

#include <QSemaphore>

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
    static void init() { instance()._init_locker.release(2); }
    static void init_wait() { instance()._init_locker.acquire(); }

private:
    QSemaphore _init_locker;

};

#endif // CAMERA_SYNC_H
