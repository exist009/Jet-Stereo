#ifndef UTILS_H
#define UTILS_H

#include <QString>

enum Sensor : qint32 { Left, Right };

enum Flip : qint32
{
    None,               // Identity (no rotation)
    Clockwise,          // Rotate clockwise 90 degrees
    Rotate180,          // Rotate 180 degrees
    CounterClockwise,   // Rotate counter-clockwise 90 degrees
    HorizontalFlip,     // Flip horizontally
    VerticalFlip,       // Flip vertically
    UpperLeftDiagonal,  // Flip across upper left/lower right diagonal
    UpperRightDiagonal, // Flip across upper right/lower left diagonal
    Automatic           // Select flip method based on image-orientation tag
};

struct Resolution
{
    qint32 width;
    qint32 height;
};

namespace Utility
{
    inline auto Pipeline(Sensor sensor, const Resolution &camera, qint32 framerate, Flip flip, const Resolution &display) -> QString
    {
        return QString("nvarguscamerasrc sensor-id=%1 ! video/x-raw(memory:NVMM), width=(int)%2"
                       ", height=(int)%3, format=(string)NV12, framerate=(fraction)%4"
                       "/1 ! nvvidconv flip-method=%5 ! video/x-raw, width=(int)%6, height=(int)%7"
                       ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink")
                .arg(sensor)
                .arg(camera.width)
                .arg(camera.height)
                .arg(framerate)
                .arg(flip)
                .arg(display.width)
                .arg(display.height);
    }
}

#endif // UTILS_H
