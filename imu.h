#ifndef IMU_H
#define IMU_H

#include <QObject>
#include <QVector>
#include <exception>

#include <utils.h>

namespace IMU
{
	struct i2c_exception : public std::exception
	{
        const char *what() const throw()
        {
            return "Failed to open I2C bus";
        }
	};

	class I2C
	{
		public:
			I2C(const QString &dev = "/dev/i2c-1");
			~I2C();

		public:
			quint8 read_1b(quint8 address, quint8 reg);
            qint16 read_2b(quint8 address, quint8 reg_low, quint8 reg_high);
			void write_1b(quint8 address, quint8 reg, quint8 value);

		private:
			qint32 fd;
	};

	class Device
	{
	public:
		Device();
		~Device();

	public:

		struct Gyroscope
		{
			float x;
			float y;
			float z;
		};

		struct Accelerometer
		{
			float x;
			float y;
			float z;
		};

		struct Magnetometer
		{
			float x;
			float y;
			float z;
		};

		struct Angle
		{
			float yaw;
			float pitch;
			float roll;
		};

	public:
		Gyroscope get_gyroscope();
		Accelerometer get_accelerometer();
		Magnetometer get_magnetometer();
		Angle get_angle(const Gyroscope &gyroscope, const Accelerometer &accelerometer, const Magnetometer &magnetometer);

	private:

		struct Sensor_data
		{
			Sensor_data() : x(0), y(0), z(0) {}

			qint16 x;
			qint16 y;
			qint16 z;
		};

		struct Average_data
		{
			Average_data() : index(0), buffer{} {}

			quint8 index;
            Sensor_data buffer[ICM20948::avg_max];
		};

		struct Angle_ratio
		{
			Angle_ratio() : q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f) {}

			float q0;
			float q1;
			float q2;
			float q3;
		};

	private:
		I2C *i2c;
		Sensor_data gyroscope_offset;
		Average_data average_gyroscope;
		Average_data average_accelerometer;
		Average_data average_magnetometer;
		Angle_ratio ratio;

	private:
		void init();

		bool check_icm20948();
		bool check_mag();

		void gyroscope_calibration();

		void read_secondary(quint8 address, quint8 reg, quint8 length, QVector<quint8> &buffer);
		void write_secondary(quint8 address, quint8 reg, quint8 data);

		Sensor_data read_gyroscope();
		Sensor_data read_accelerometer();
		Sensor_data read_magnetometer();

		void calculate_average(Average_data &average, const Sensor_data &value, Sensor_data &result);
	};
}

Q_DECLARE_METATYPE(IMU::Device::Gyroscope)
Q_DECLARE_METATYPE(IMU::Device::Accelerometer)
Q_DECLARE_METATYPE(IMU::Device::Magnetometer)
Q_DECLARE_METATYPE(IMU::Device::Angle)

#endif // IMU_H
