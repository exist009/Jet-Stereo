#include "imu.h"

#include <utils.h>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>

#include <QDebug>
#include <QThread>

IMU::I2C::I2C(const QString &dev) : fd(open(dev.toStdString().data(), O_RDWR))
{
	if (this->fd < 0)
	{
		throw i2c_exception();
	}
}

IMU::I2C::~I2C()
{
	close(this->fd);
}

quint8 IMU::I2C::read_1b(quint8 address, quint8 reg)
{
	if (ioctl(this->fd, I2C_SLAVE, address) < 0)
	{
		qDebug() << "Failed to acquire bus access and/or talk to slave";
		return 0;
	}

	quint8 result;

	write(this->fd, &reg, 1);
    read(this->fd, &result, 1);

	return result;
}

void IMU::I2C::write_1b(quint8 address, quint8 reg, quint8 value)
{
	if (ioctl(this->fd, I2C_SLAVE, address) < 0)
	{
		qDebug() << "Failed to acquire bus access and/or talk to slave";
		return;
	}

	auto buffer = new quint8[2];
	buffer[0] = reg;
	buffer[1] = value;

	write(this->fd, buffer, 2);

	delete[] buffer;
}

qint16 IMU::I2C::read_2b(quint8 address, quint8 reg_low, quint8 reg_high)
{
	return ((this->read_1b(address, reg_high) << 8) | this->read_1b(address, reg_low));
}

IMU::Device::Device()
{
	try
	{
		this->i2c = new I2C;
	}
	catch (i2c_exception &e)
	{
		qDebug() << e.what();
		return;
	}

	if (!this->check_icm20948())
	{
		qDebug() << "ICM20948 check failed";
		return;
	}

	this->init();
}

IMU::Device::~Device()
{
	delete this->i2c;
}

IMU::Device::Gyroscope IMU::Device::get_gyroscope()
{
	auto data = this->read_gyroscope();

	Gyroscope gyroscope;

	gyroscope.x = data.x / ICM20948::SSF::gyro_1000dps;
	gyroscope.y = data.y / ICM20948::SSF::gyro_1000dps;
	gyroscope.z = data.z / ICM20948::SSF::gyro_1000dps;

	return gyroscope;
}

IMU::Device::Accelerometer IMU::Device::get_accelerometer()
{
	auto data = this->read_accelerometer();

	Accelerometer accelerometer;

	accelerometer.x = data.x / ICM20948::SSF::accel_2g;
	accelerometer.y = data.y / ICM20948::SSF::accel_2g;
	accelerometer.z = data.z / ICM20948::SSF::accel_2g;

	return accelerometer;
}

IMU::Device::Magnetometer IMU::Device::get_magnetometer()
{
	auto data = this->read_magnetometer();

	Magnetometer magnetometer;

    magnetometer.x = data.x * ICM20948::SSF::mag_4900ut;
    magnetometer.y = data.y * ICM20948::SSF::mag_4900ut;
    magnetometer.z = data.z * ICM20948::SSF::mag_4900ut;

	return magnetometer;
}

void IMU::Device::init()
{
	//! User Bank 0 Register

	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_reg_bank_sel, ICM20948::Register::val_reg_bank_0);
	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_pwr_migmt_1,  ICM20948::Register::val_all_rge_reset);

	QThread::msleep(10);

	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_pwr_migmt_1,  ICM20948::Register::val_run_mode);

	//! User Bank 2 Register

	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_reg_bank_sel, ICM20948::Register::val_reg_bank_2);
	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_gyro_smplrt_div, 0x07);
	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_gyro_config_1, ICM20948::Register::val_bit_gyro_dlpcfg_6 | ICM20948::Register::val_bit_gyro_fs_1000dps | ICM20948::Register::val_bit_gyro_dlpf);
	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_accel_smplrt_div_2, 0x07);
	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_accel_config, ICM20948::Register::val_bit_accel_dlpcfg_6 | ICM20948::Register::val_bit_accel_fs_2g | ICM20948::Register::val_bit_accel_dlpf);

	//! User Bank 0 Register

	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_reg_bank_sel, ICM20948::Register::val_reg_bank_0);

	QThread::msleep(10);

	this->gyroscope_calibration();

	this->check_mag();

	this->write_secondary(ICM20948::Address::icm20948_ak09916 | ICM20948::Address::icm20948_ak09916_write, ICM20948::Register::add_mag_cntl2, ICM20948::Register::val_mag_mode_20hz);
}

bool IMU::Device::check_icm20948()
{
	return this->i2c->read_1b(ICM20948::Address::icm20948, ICM20948::Register::add_wia) == ICM20948::Register::val_wia;
}

bool IMU::Device::check_mag()
{
	QVector<quint8> buffer;
	this->read_secondary(ICM20948::Address::icm20948_ak09916 | ICM20948::Address::icm20948_ak09916_read, ICM20948::Register::add_mag_wia1, 2, buffer);

	return buffer[0] == ICM20948::Register::val_mag_wia1 && buffer[1] == ICM20948::Register::val_mag_wia2;
}

void IMU::Device::gyroscope_calibration()
{
	qint32 x_counter(0);
	qint32 y_counter(0);
	qint32 z_counter(0);

	for (qint32 i = 0; i < 32; i++)
	{
		auto data = this->read_gyroscope();

		x_counter += data.x;
		y_counter += data.y;
		z_counter += data.z;

		QThread::msleep(10);
	}

	this->gyroscope_offset.x = static_cast<qint16>(x_counter >> 5);
	this->gyroscope_offset.y = static_cast<qint16>(y_counter >> 5);
	this->gyroscope_offset.z = static_cast<qint16>(z_counter >> 5);
}

void IMU::Device::read_secondary(quint8 address, quint8 reg, quint8 length, QVector<quint8> &buffer)
{
	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_reg_bank_sel,  ICM20948::Register::val_reg_bank_3); //! Swtich Bank 3
	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_i2c_slv0_addr, address);
	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_i2c_slv0_reg,  reg);
	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_i2c_slv0_ctrl, ICM20948::Register::val_bit_slv0_en | length);

	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_reg_bank_sel, ICM20948::Register::val_reg_bank_0); //! Swtich Bank0

	auto tmp = this->i2c->read_1b(ICM20948::Address::icm20948, ICM20948::Register::add_user_ctrl);

	tmp |= ICM20948::Register::val_bit_i2c_mst_en;

	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_user_ctrl, tmp);

	QThread::msleep(5);

	tmp &= ~ICM20948::Register::val_bit_i2c_mst_en;

	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_user_ctrl, tmp);

	for (quint8 i = 0; i < length; i++)
	{
		buffer.push_back(this->i2c->read_1b(ICM20948::Address::icm20948, ICM20948::Register::add_ext_sens_data_00 + i));
	}

	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_reg_bank_sel, ICM20948::Register::val_reg_bank_3); //! Swtich Bank 3

	tmp = this->i2c->read_1b(ICM20948::Address::icm20948, ICM20948::Register::add_i2c_slv0_ctrl);
	tmp &= ~((ICM20948::Register::val_bit_i2c_mst_en) & (ICM20948::Register::val_bit_mask_len));

	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_i2c_slv0_ctrl,  tmp);
	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_reg_bank_sel, ICM20948::Register::val_reg_bank_0); //! Swtich Bank 0
}

void IMU::Device::write_secondary(quint8 address, quint8 reg, quint8 data)
{
	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_reg_bank_sel,  ICM20948::Register::val_reg_bank_3); //! Swtich Bank 3
	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_i2c_slv1_addr, address);
	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_i2c_slv1_reg,  reg);
	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_i2c_slv1_do,   data);
	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_i2c_slv1_ctrl, ICM20948::Register::val_bit_slv0_en | 1);

	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_reg_bank_sel, ICM20948::Register::val_reg_bank_0); //! Swtich Bank 0

	auto tmp = this->i2c->read_1b(ICM20948::Address::icm20948, ICM20948::Register::add_user_ctrl);

	tmp |= ICM20948::Register::val_bit_i2c_mst_en;

	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_user_ctrl, tmp);

	QThread::msleep(5);

	tmp &= ~ICM20948::Register::val_bit_i2c_mst_en;

	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_user_ctrl, tmp);

	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_reg_bank_sel, ICM20948::Register::val_reg_bank_3); //! Swtich Bank3

	tmp = this->i2c->read_1b(ICM20948::Address::icm20948, ICM20948::Register::add_i2c_slv0_ctrl);
	tmp &= ~((ICM20948::Register::val_bit_i2c_mst_en) & (ICM20948::Register::val_bit_mask_len));

	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_i2c_slv0_ctrl,  tmp);
	this->i2c->write_1b(ICM20948::Address::icm20948, ICM20948::Register::add_reg_bank_sel, ICM20948::Register::val_reg_bank_0); //! Swtich Bank 0
}

IMU::Device::Sensor_data IMU::Device::read_gyroscope()
{
	Sensor_data value;

    value.x = this->i2c->read_2b(ICM20948::Address::icm20948, ICM20948::Register::add_gyro_xout_l, ICM20948::Register::add_gyro_xout_h);
    value.y = this->i2c->read_2b(ICM20948::Address::icm20948, ICM20948::Register::add_gyro_yout_l, ICM20948::Register::add_gyro_yout_h);
    value.z = this->i2c->read_2b(ICM20948::Address::icm20948, ICM20948::Register::add_gyro_zout_l, ICM20948::Register::add_gyro_zout_h);

	Sensor_data result;

	this->calculate_average(this->average_gyroscope, value, result);

	result.x -= this->gyroscope_offset.x;
	result.y -= this->gyroscope_offset.y;
	result.z -= this->gyroscope_offset.z;

	return result;
}

IMU::Device::Sensor_data IMU::Device::read_accelerometer()
{
	Sensor_data value;

	value.x = this->i2c->read_2b(ICM20948::Address::icm20948, ICM20948::Register::add_accel_xout_l, ICM20948::Register::add_accel_xout_h);
	value.y = this->i2c->read_2b(ICM20948::Address::icm20948, ICM20948::Register::add_accel_yout_l, ICM20948::Register::add_accel_yout_h);
	value.z = this->i2c->read_2b(ICM20948::Address::icm20948, ICM20948::Register::add_accel_zout_l, ICM20948::Register::add_accel_zout_h);

	Sensor_data result;

	this->calculate_average(this->average_accelerometer, value, result);

	return result;
}

IMU::Device::Sensor_data IMU::Device::read_magnetometer()
{
	auto counter(20);

	QVector<quint8> tmp_buffer;

	while (counter)
	{
		QThread::msleep(10);

		this->read_secondary(ICM20948::Address::icm20948_ak09916 | ICM20948::Address::icm20948_ak09916_read, ICM20948::Register::add_mag_st2, 1, tmp_buffer);

		if (tmp_buffer[0] & 0x01) break;

		counter--;
	}

	Sensor_data value;
	QVector<quint8> value_buffer;

	if (counter)
	{
		this->read_secondary(ICM20948::Address::icm20948_ak09916 | ICM20948::Address::icm20948_ak09916_read, ICM20948::Register::add_mag_data, ICM20948::MAG::data_len, value_buffer);

		value.x = (static_cast<qint16>(value_buffer[1] << 8)) | value_buffer[0];
		value.y = (static_cast<qint16>(value_buffer[3] << 8)) | value_buffer[2];
		value.z = (static_cast<qint16>(value_buffer[5] << 8)) | value_buffer[4];
	}

	Sensor_data result;

	this->calculate_average(this->average_magnetometer, value, result);

	result.y *= -1;
	result.z *= -1;

	return result;
}

void IMU::Device::calculate_average(Average_data &average, const Sensor_data &value, Sensor_data &result)
{
	average.buffer[average.index].x = value.x;
	average.buffer[average.index].y = value.y;
	average.buffer[average.index].z = value.z;

	qint32 out_x(0);
	qint32 out_y(0);
	qint32 out_z(0);

	for (auto i = 0; i < 8; i++)
	{
		out_x += average.buffer[i].x;
		out_y += average.buffer[i].y;
		out_z += average.buffer[i].z;
	}

	out_x >>= 3;
	out_y >>= 3;
	out_z >>= 3;

	result.x = static_cast<qint16>(out_x);
	result.y = static_cast<qint16>(out_y);
	result.z = static_cast<qint16>(out_z);

	average.index++;
	average.index &= 0x07;
}
