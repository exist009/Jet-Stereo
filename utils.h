#ifndef UTILS_H
#define UTILS_H

#include <QString>

enum Sensor : qint32 { Left, Right };

enum Flip : qint32
{
    None,               //! Identity (no rotation)
    Clockwise,          //! Rotate clockwise 90 degrees
    Rotate180,          //! Rotate 180 degrees
    CounterClockwise,   //! Rotate counter-clockwise 90 degrees
    HorizontalFlip,     //! Flip horizontally
    VerticalFlip,       //! Flip vertically
    UpperLeftDiagonal,  //! Flip across upper left/lower right diagonal
    UpperRightDiagonal, //! Flip across upper right/lower left diagonal
    Automatic           //! Select flip method based on image-orientation tag
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

	inline auto rsqrt(float x) -> float //! Fast inverse square root
	{
		float half = 0.5f * x;
		float y = x;

		long i = *(long *)&y;
		i = 0x5F3759DF - (i >> 1);
		y = *(float *)&i;
		y = y * (1.5f - (half * y * y));

		return y;
	}

	inline auto sqr(float x) -> float
	{
		return x * x;
	}
}

namespace ICM20948
{
	namespace SSF //! Sensitivity Scale Factor
	{
		const float  gyro_250dps  = 131;   //! LSB/dps
		const float  gyro_500dps  = 65.5;  //! LSB/dps
		const float  gyro_1000dps = 32.8;  //! LSB/dps
		const float  gyro_2000dps = 16.4;  //! LSB/dps
        const float  accel_2g     = 16384; //! LSB/g
        const float  accel_4g     = 8192;  //! LSB/g
        const float  accel_8g     = 4096;  //! LSB/g
        const float  accel_16g    = 2048;  //! LSB/g
        const float  mag_4900ut   = 0.15;  //! uT/LSB
	}

	namespace Address
	{
		const quint8 icm20948               = 0x68;
		const quint8 icm20948_ak09916       = 0x0C;
		const quint8 icm20948_ak09916_read  = 0x80;
		const quint8 icm20948_ak09916_write = 0x00;
	}

	namespace Register
	{
		//! User Bank 0 Register

		const quint8 add_wia                 = 0x00;
		const quint8 val_wia                 = 0xEA;
		const quint8 add_user_ctrl           = 0x03;
		const quint8 val_bit_dmp_en          = 0x80;
		const quint8 val_bit_fifo_en         = 0x40;
		const quint8 val_bit_i2c_mst_en      = 0x20;
		const quint8 val_bit_i2c_if_dis      = 0x10;
		const quint8 val_bit_dmp_rst         = 0x08;
		const quint8 val_bit_diamond_dmp_rst = 0x04;
		const quint8 add_pwr_migmt_1         = 0x06;
		const quint8 val_all_rge_reset       = 0x80;
		const quint8 val_run_mode            = 0x01;  //! Non low-power mode
		const quint8 add_lp_config           = 0x05;
		const quint8 add_pwr_mgmt_1          = 0x06;
		const quint8 add_pwr_mgmt_2          = 0x07;
		const quint8 add_accel_xout_h        = 0x2D;
		const quint8 add_accel_xout_l        = 0x2E;
		const quint8 add_accel_yout_h        = 0x2F;
		const quint8 add_accel_yout_l        = 0x30;
		const quint8 add_accel_zout_h        = 0x31;
		const quint8 add_accel_zout_l        = 0x32;
		const quint8 add_gyro_xout_h         = 0x33;
		const quint8 add_gyro_xout_l         = 0x34;
		const quint8 add_gyro_yout_h         = 0x35;
		const quint8 add_gyro_yout_l         = 0x36;
		const quint8 add_gyro_zout_h         = 0x37;
		const quint8 add_gyro_zout_l         = 0x38;
		const quint8 add_ext_sens_data_00    = 0x3B;
		const quint8 add_reg_bank_sel        = 0x7F;
		const quint8 val_reg_bank_0          = 0x00;
		const quint8 val_reg_bank_1          = 0x10;
		const quint8 val_reg_bank_2          = 0x20;
		const quint8 val_reg_bank_3          = 0x30;

		//! User Bank 1-2 Register

		const quint8 add_gyro_smplrt_div     = 0x00;
		const quint8 add_gyro_config_1       = 0x01;
		const quint8 val_bit_gyro_dlpcfg_2   = 0x10; //! bit[5:3]
		const quint8 val_bit_gyro_dlpcfg_4   = 0x20; //! bit[5:3]
		const quint8 val_bit_gyro_dlpcfg_6   = 0x30; //! bit[5:3]
		const quint8 val_bit_gyro_fs_250dps  = 0x00; //! bit[2:1]
		const quint8 val_bit_gyro_fs_500dps  = 0x02; //! bit[2:1]
		const quint8 val_bit_gyro_fs_1000dps = 0x04; //! bit[2:1]
		const quint8 val_bit_gyro_fs_2000dps = 0x06; //! bit[2:1]
		const quint8 val_bit_gyro_dlpf       = 0x01; //! bit[0]
		const quint8 add_accel_smplrt_div_2  = 0x11;
		const quint8 add_accel_config        = 0x14;
		const quint8 val_bit_accel_dlpcfg_2  = 0x10; //! bit[5:3]
		const quint8 val_bit_accel_dlpcfg_4  = 0x20; //! bit[5:3]
		const quint8 val_bit_accel_dlpcfg_6  = 0x30; //! bit[5:3]
		const quint8 val_bit_accel_fs_2g     = 0x00; //! bit[2:1]
		const quint8 val_bit_accel_fs_4g     = 0x02; //! bit[2:1]
		const quint8 val_bit_accel_fs_8g     = 0x04; //! bit[2:1]
		const quint8 val_bit_accel_fs_16g    = 0x06; //! bit[2:1]
		const quint8 val_bit_accel_dlpf      = 0x01; //! bit[0]

		//! User Bank 3 Register

		const quint8 add_i2c_slv0_addr = 0x03;
		const quint8 add_i2c_slv0_reg  = 0x04;
		const quint8 add_i2c_slv0_ctrl = 0x05;
		const quint8 val_bit_slv0_en   = 0x80;
		const quint8 val_bit_mask_len  = 0x07;
		const quint8 add_i2c_slv0_do   = 0x06;
		const quint8 add_i2c_slv1_addr = 0x07;
		const quint8 add_i2c_slv1_reg  = 0x08;
		const quint8 add_i2c_slv1_ctrl = 0x09;
		const quint8 add_i2c_slv1_do   = 0x0A;

		//! MAG Register

		const quint8 add_mag_wia1       = 0x00;
		const quint8 val_mag_wia1       = 0x48;
		const quint8 add_mag_wia2       = 0x01;
		const quint8 val_mag_wia2       = 0x09;
		const quint8 add_mag_st2        = 0x10;
		const quint8 add_mag_data       = 0x11;
		const quint8 add_mag_cntl2      = 0x31;
		const quint8 val_mag_mode_pd    = 0x00;
		const quint8 val_mag_mode_sm    = 0x01;
		const quint8 val_mag_mode_10hz  = 0x02;
		const quint8 val_mag_mode_20hz  = 0x04;
		const quint8 val_mag_mode_50hz  = 0x05;
		const quint8 val_mag_mode_100hz = 0x08;
		const quint8 val_mag_mode_st    = 0x10;
	}

	namespace MAG
	{
		const qint32 data_len = 6;
	}

	const auto Kp = 4.50f; //! proportional gain governs rate of convergence to accelerometer/magnetometer
	const auto Ki = 1.0f; //! integral gain governs rate of convergence of gyroscope biases
	const auto hT = 0.024f;

    const auto avg_max = 8;
}

#endif // UTILS_H
