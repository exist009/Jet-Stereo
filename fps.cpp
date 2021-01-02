#include "fps.h"

#include <QtMath>

Fps::Fps() : previous(QDateTime::currentMSecsSinceEpoch()), current(0), index(0), buffer{} { }

Fps::~Fps() {}

qint32 Fps::get()
{
	this->current = QDateTime::currentMSecsSinceEpoch();

	this->buffer[this->index] = static_cast<qint32>(1000 / (this->current - this->previous));

	this->previous = this->current;

	qreal fps(0);

	for (auto i = 0; i < this->avg_max; i++)
	{
		fps += this->buffer[i];
	}

	if (++this->index >= this->avg_max) this->index = 0;

	return qCeil(fps / this->avg_max);
}
