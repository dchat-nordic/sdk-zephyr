/*
 * Copyright (c) 2021, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/i2c.h>
#include <drivers/sensor.h>

#include "mpu9250.h"
#include "ak8963.h"

#include <logging/log.h>
LOG_MODULE_DECLARE(MPU9250, CONFIG_SENSOR_LOG_LEVEL);


void ak8963_convert_magn(struct sensor_value *val, int16_t raw_val,
			 int16_t scale)
{
	/* The sensor device returns 10^-9 Teslas after scaling.
	 * Scale adjusts for callibration data and units
	 * So sensor instance returns Gauss units
	 */

	int32_t scaled_val = raw_val * scale ;
	val->val1 = scaled_val / 1000000;
	val->val2 = scaled_val % 1000000;
}


static int ak8963_execute_rw(const struct device *dev, uint8_t reg, bool write)
{
	/* Instruct the MPU9250 to access over its external i2c bus
	 * given device register with given details
	 */
	const struct mpu9250_config *cfg = dev->config;
	struct mpu9250_data *drv_data = dev->data;
	uint8_t mode_bit = 0x00;
	uint8_t status;

	if (write == false) {
		mode_bit = I2C_READ_FLAG;
	}

	/* Set target i2c address */
	if (i2c_reg_write_byte(drv_data->i2c, cfg->i2c_addr,
			       MPU9250_REG_I2C_SLV4_ADDR,
			       AK8963_I2C_ADDR | mode_bit  )) {
		LOG_ERR("Failed to write i2c target slave address.");
		return -EIO;
	}

	/* Set target i2c register */
	if (i2c_reg_write_byte(drv_data->i2c, cfg->i2c_addr,
			       MPU9250_REG_I2C_SLV4_REG,
			       reg )) {
		LOG_ERR("Failed to write i2c target slave register.");
		return -EIO;
	}

	/* Initiate transfer  */
	if (i2c_reg_write_byte(drv_data->i2c, cfg->i2c_addr,
			       MPU9250_REG_I2C_SLV4_CTRL,
			       MPU9250_REG_I2C_SLV4_CTRL_VAL )) {
		LOG_ERR("Failed to initiate i2c slave transfer.");
		return -EIO;
	}

	/* Wait for a transfer to be ready */
	while (!(status & MPU9250_I2C_MST_STS_SLV4_DONE)) {
		if (i2c_reg_read_byte(drv_data->i2c, cfg->i2c_addr,
				      MPU9250_I2C_MST_STS, &status)) {
					LOG_ERR("Waiting for slave failed.");
					return -EIO;
				}

		k_msleep(10);
	}

	return 0;
}

static int ak8963_read_reg(const struct device *dev, uint8_t reg, uint8_t* data)
{
	const struct mpu9250_config *cfg = dev->config;
	struct mpu9250_data *drv_data = dev->data;

	/* Execute transfer */
	if (ak8963_execute_rw(dev, reg, false)) {
		LOG_ERR("Failed to prepare transfer.");
		return -EIO;
	}

	/* Read the result */
	if (i2c_reg_read_byte(drv_data->i2c, cfg->i2c_addr,
			      MPU9250_REG_I2C_SLV4_DI, data)) {
		LOG_ERR("Failed to read data from slave.");
		return -EIO;
	}

	return 0;
}

static int ak8963_write_reg(const struct device *dev, uint8_t reg, uint8_t data)
{
	const struct mpu9250_config *cfg = dev->config;
	struct mpu9250_data *drv_data = dev->data;

	/* Set the data to write */
	if (i2c_reg_write_byte(drv_data->i2c, cfg->i2c_addr,
			       MPU9250_REG_I2C_SLV4_DO, data )) {
		LOG_ERR("Failed to write data to slave.");
		return -EIO;
	}

	/* Execute transfer */
	if (ak8963_execute_rw(dev, reg, true)) {
		LOG_ERR("Failed to transfer write to slave.");
		return -EIO;
	}

	return 0;
}


static int ak8963_set_mode(const struct device *dev, uint8_t mode)
{
	if (ak8963_write_reg(dev, AK8963_REG_CNTL1, mode)) {
		LOG_ERR("Failed to set AK8963 mode.");
		return -EIO;
	}

	/* Wait for mode to change */
	k_sleep(K_MSEC(1));
	return 0;
}

static int16_t ak8963_calc_adj(int16_t val)
{

	/** Datasheet says the actual register value is in 16bit output max
	 *  value of 32760 that corresponds to 4912 uT flux, yielding factor
	 *  of 0.149938.
	 *
	 *  Now Zephyr unit is Gauss, and conversion is 1T = 10^4G
	 *  -> 0.1499 * 10^4 = 1499
	 *  So if we multiply with scaling with 1499 the unit is uG.
	 */
	return ((AK9863_SCALE_TO_UG * (val - 128)) / 256) + AK9863_SCALE_TO_UG;
}

static int ak8963_fetch_adj(const struct device *dev)
{
	/* Read magnetometer adjustment data from the AK8963 chip */
	struct mpu9250_data *drv_data = dev->data;
	uint8_t buf;

	/* Change to FUSE access mode to access adjustment registers */
	if (ak8963_set_mode(dev, AK8963_REG_CNTL1_FUSE_ROM_VAL)) {
		LOG_ERR("Failed to set chip in fuse access mode.");
		return -EIO;
	}

	if (ak8963_read_reg(dev, AK8963_REG_ADJ_DATA_X, &buf)) {
		LOG_ERR("Failed to read adjustment data.");
		return -EIO;
	}
	drv_data->magn_scale_x = ak8963_calc_adj(buf);

	if (ak8963_read_reg(dev, AK8963_REG_ADJ_DATA_Y, &buf)) {
		LOG_ERR("Failed to read adjustment data.");
		return -EIO;
	}
	drv_data->magn_scale_y = ak8963_calc_adj(buf);

	if (ak8963_read_reg(dev, AK8963_REG_ADJ_DATA_Z, &buf)) {
		LOG_ERR("Failed to read adjustment data.");
		return -EIO;
	}
	drv_data->magn_scale_z = ak8963_calc_adj(buf);

	/* Change back to the powerdown mode */
	if (ak8963_set_mode(dev, AK8963_REG_CNTL1_POWERDOWN_VAL)) {
		LOG_ERR("Failed to set chip in power down mode.");
		return -EIO;
	}

	LOG_DBG("Adjustment values %d %d %d", drv_data->magn_scale_x,
		drv_data->magn_scale_y, drv_data->magn_scale_z );

	return 0;
}

static int ak8963_reset(const struct device *dev)
{
	/* Reset the chip -> reset all settings. */
	if (ak8963_write_reg(dev, AK8963_REG_CNTL2,
			     AK8963_REG_CNTL2_RESET_VAL)) {
		LOG_ERR("Failed to reset AK8963.");
		return -EIO;
	}

	/* Wait for mode to change */
	k_msleep(1);

	return 0;
}

static int ak8963_init_master(const struct device *dev)
{
	const struct mpu9250_config *cfg = dev->config;
	struct mpu9250_data *drv_data = dev->data;

	/* Instruct MPU9250 to use its external I2C bus as master */
	if (i2c_reg_write_byte(drv_data->i2c, cfg->i2c_addr,
			       MPU9250_REG_USER_CTRL,
			       MPU9250_REG_USER_CTRL_I2C_MASTERMODE_VAL)) {
		LOG_ERR("Failed to set MPU9250 master i2c mode.");
		return -EIO;
	}

	/* Set MPU9250 I2C bus as 400kHz and issue interrupt at data ready. */
	if (i2c_reg_write_byte(drv_data->i2c, cfg->i2c_addr,
			       MPU9250_REG_I2C_MST_CTRL,
			       MPU9250_REG_I2C_MST_CTRL_WAIT_MAG_400KHZ_VAL)) {
		LOG_ERR("Failed to set MPU9250 master i2c speed.");
		return -EIO;
	}

	return 0;
}

static int ak8963_init_readout(const struct device *dev)
{
	const struct mpu9250_config *cfg = dev->config;
	struct mpu9250_data *drv_data = dev->data;

	/* Set target i2c address */
	if (i2c_reg_write_byte(drv_data->i2c, cfg->i2c_addr,
			       MPU9250_REG_I2C_SLV0_ADDR,
			       AK8963_I2C_ADDR | I2C_READ_FLAG)) {
		return -EIO;
	}

	/* Set target as data registers */
	if (i2c_reg_write_byte(drv_data->i2c, cfg->i2c_addr,
			       MPU9250_REG_I2C_SLV0_REG, AK8963_REG_DATA)) {
		return -EIO;
	}

	/* Initiate readout at sample rate */
	if (i2c_reg_write_byte(drv_data->i2c, cfg->i2c_addr,
			       MPU9250_REG_I2C_SLV0_CTRL,
			       MPU9250_REG_READOUT_CTRL_VAL)) {
		return -EIO;
	}

	return 0;
}

int ak8963_init(const struct device *dev)
{
	uint8_t buf;

	if (ak8963_init_master(dev)) {
		return -EIO;
	}

	if (ak8963_reset(dev)) {
		return -EIO;
	}

	/* First check that the chip says hello */
	if (ak8963_read_reg(dev, AK8963_REG_ID, &buf)) {
		LOG_ERR("Failed to read AK8963 chip id.");
		return -ENOTSUP;
	}

	if (buf != AK8963_REG_ID_VAL) {
		LOG_ERR("Invalid AK8963 chip id (0x%X).", buf);
		return -EIO;
	}

	/* Fetch calibration data */
	if (ak8963_fetch_adj(dev)) {
		return -EIO;
	}

	/* Set AK sample rate and resolution */
	if (ak8963_set_mode(dev, AK8963_REG_CNTL1_16BIT_100HZ_VAL)) {
		LOG_ERR("Failed set sample rate for AK8963.");
		return -EIO;
	}

	/* Init constant readouts at sample rate */
	if (ak8963_init_readout(dev)) {
		return -EIO;
	}

	return 0;
}
