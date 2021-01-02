#ifndef FPS_H
#define FPS_H

#include <QDateTime>

class Fps
{
public:
	Fps();
	~Fps();

public:
	qint32 get();

private:
	static const auto avg_max = 8;

private:
	qint64 previous;
	qint64 current;
	qint32 index;
	qint32 buffer[avg_max];
};

#endif // FPS_H
