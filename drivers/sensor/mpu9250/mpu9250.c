/*
 * Copyright (c) 2021, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT invensense_mpu9250

#include <drivers/i2c.h>
#include <init.h>
#include <sys/byteorder.h>
#include <drivers/sensor.h>
#include <logging/log.h>

#include "mpu9250.h"

#ifdef CONFIG_MPU9250_MAGN_EN
#include "ak8963.h"
#endif

LOG_MODULE_REGISTER(MPU9250, CONFIG_SENSOR_LOG_LEVEL);

/* see "Accelerometer Measurements" section from register map description */
static void mpu9250_convert_accel(struct sensor_value *val, int16_t raw_val,
				  uint16_t sensitivity_shift)
{
	int64_t conv_val;

	conv_val = ((int64_t)raw_val * SENSOR_G) >> sensitivity_shift;
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

/* see "Gyroscope Measurements" section from register map description */
static void mpu9250_convert_gyro(struct sensor_value *val, int16_t raw_val,
				 uint16_t sensitivity_x10)
{
	int64_t conv_val;

	conv_val = ((int64_t)raw_val * SENSOR_PI * 10) /
		   (sensitivity_x10 * 180U);
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

/* see "Temperature Measurement" section from register map description */
static inline void mpu9250_convert_temp(struct sensor_value *val,
					int16_t raw_val)
{
	val->val1 = raw_val / 340 + 36;
	val->val2 = ((int64_t)(raw_val % 340) * 1000000) / 340 + 530000;

	if (val->val2 < 0) {
		val->val1--;
		val->val2 += 1000000;
	} else if (val->val2 >= 1000000) {
		val->val1++;
		val->val2 -= 1000000;
	}
}

static int mpu9250_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct mpu9250_data *drv_data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		mpu9250_convert_accel(val, drv_data->accel_x,
				      drv_data->accel_sensitivity_shift);
		mpu9250_convert_accel(val + 1, drv_data->accel_y,
				      drv_data->accel_sensitivity_shift);
		mpu9250_convert_accel(val + 2, drv_data->accel_z,
				      drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_X:
		mpu9250_convert_accel(val, drv_data->accel_x,
				      drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		mpu9250_convert_accel(val, drv_data->accel_y,
				      drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		mpu9250_convert_accel(val, drv_data->accel_z,
				      drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		mpu9250_convert_gyro(val, drv_data->gyro_x,
				     drv_data->gyro_sensitivity_x10);
		mpu9250_convert_gyro(val + 1, drv_data->gyro_y,
				     drv_data->gyro_sensitivity_x10);
		mpu9250_convert_gyro(val + 2, drv_data->gyro_z,
				     drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_X:
		mpu9250_convert_gyro(val, drv_data->gyro_x,
				     drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_Y:
		mpu9250_convert_gyro(val, drv_data->gyro_y,
				     drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_Z:
		mpu9250_convert_gyro(val, drv_data->gyro_z,
				     drv_data->gyro_sensitivity_x10);
#ifdef CONFIG_MPU9250_MAGN_EN
	case SENSOR_CHAN_MAGN_XYZ:
		ak8963_convert_magn(val, drv_data->magn_x,
				    drv_data->magn_scale_x);
		ak8963_convert_magn(val + 1, drv_data->magn_y,
				    drv_data->magn_scale_y);
		ak8963_convert_magn(val + 2, drv_data->magn_z,
				    drv_data->magn_scale_z);
		break;
	case SENSOR_CHAN_MAGN_X:
		ak8963_convert_magn(val, drv_data->magn_x,
				    drv_data->magn_scale_x);
		break;
	case SENSOR_CHAN_MAGN_Y:
		ak8963_convert_magn(val, drv_data->magn_y,
				    drv_data->magn_scale_y);
		break;
	case SENSOR_CHAN_MAGN_Z:
		ak8963_convert_magn(val, drv_data->magn_z,
				    drv_data->magn_scale_z);
#endif
		break;
	default: /* chan == SENSOR_CHAN_DIE_TEMP */
		mpu9250_convert_temp(val, drv_data->temp);
	}

	return 0;
}

#ifdef CONFIG_MPU9250_MAGN_EN
#define MPU9250_READ_BUF_SIZE 10
#else
#define MPU9250_READ_BUF_SIZE 7
#endif

static int mpu9250_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	struct mpu9250_data *drv_data = dev->data;
	const struct mpu9250_config *cfg = dev->config;
	int16_t buf[MPU9250_READ_BUF_SIZE];

	if (i2c_burst_read(drv_data->i2c, cfg->i2c_addr,
			   MPU9250_REG_DATA_START, (uint8_t *)buf,
			   sizeof(buf))) {
		LOG_ERR("Failed to read data sample.");
		return -EIO;
	}

	drv_data->accel_x = sys_be16_to_cpu(buf[0]);
	drv_data->accel_y = sys_be16_to_cpu(buf[1]);
	drv_data->accel_z = sys_be16_to_cpu(buf[2]);
	drv_data->temp = sys_be16_to_cpu(buf[3]);
	drv_data->gyro_x = sys_be16_to_cpu(buf[4]);
	drv_data->gyro_y = sys_be16_to_cpu(buf[5]);
	drv_data->gyro_z = sys_be16_to_cpu(buf[6]);
#ifdef CONFIG_MPU9250_MAGN_EN
	drv_data->magn_x = sys_be16_to_cpu(buf[7]);
	drv_data->magn_y = sys_be16_to_cpu(buf[8]);
	drv_data->magn_z = sys_be16_to_cpu(buf[9]);
#endif

	return 0;
}

static const struct sensor_driver_api mpu9250_driver_api = {
#if CONFIG_MPU9250_TRIGGER
	.trigger_set = mpu9250_trigger_set,
#endif
	.sample_fetch = mpu9250_sample_fetch,
	.channel_get = mpu9250_channel_get,
};

int mpu9250_init(const struct device *dev)
{
	struct mpu9250_data *drv_data = dev->data;
	const struct mpu9250_config *cfg = dev->config;
	uint8_t id, i;

	drv_data->i2c = device_get_binding(cfg->i2c_label);
	if (drv_data->i2c == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			    cfg->i2c_label);
		return -EINVAL;
	}

	/* check chip ID */
	if (i2c_reg_read_byte(drv_data->i2c, cfg->i2c_addr,
			      MPU9250_REG_CHIP_ID, &id)) {
		LOG_ERR("Failed to read chip ID.");
		return -EIO;
	}

	if (id != MPU9250_CHIP_ID && id != MPU9250_CHIP_ID) {
		LOG_ERR("Invalid chip ID.");
		return -EINVAL;
	}

	/* wake up chip */
	if (i2c_reg_update_byte(drv_data->i2c, cfg->i2c_addr,
				MPU9250_REG_PWR_MGMT1, MPU9250_SLEEP_EN, 0)) {
		LOG_ERR("Failed to wake up chip.");
		return -EIO;
	}

	/* set accelerometer full-scale range */
	for (i = 0U; i < 4; i++) {
		if (BIT(i+1) == CONFIG_MPU9250_ACCEL_FS) {
			break;
		}
	}

	if (i == 4U) {
		LOG_ERR("Invalid value for accel full-scale range.");
		return -EINVAL;
	}

	if (i2c_reg_write_byte(drv_data->i2c, cfg->i2c_addr,
			       MPU9250_REG_ACCEL_CFG,
			       i << MPU9250_ACCEL_FS_SHIFT)) {
		LOG_ERR("Failed to write accel full-scale range.");
		return -EIO;
	}

	drv_data->accel_sensitivity_shift = 14 - i;

	/* set gyroscope full-scale range */
	for (i = 0U; i < 4; i++) {
		if (BIT(i) * 250 == CONFIG_MPU9250_GYRO_FS) {
			break;
		}
	}

	if (i == 4U) {
		LOG_ERR("Invalid value for gyro full-scale range.");
		return -EINVAL;
	}

	if (i2c_reg_write_byte(drv_data->i2c, cfg->i2c_addr,
			       MPU9250_REG_GYRO_CFG,
			       i << MPU9250_GYRO_FS_SHIFT)) {
		LOG_ERR("Failed to write gyro full-scale range.");
		return -EIO;
	}

	if (i2c_reg_write_byte(drv_data->i2c, cfg->i2c_addr,
			       MPU9250_REG_CONFIG,
			       CONFIG_MPU9250_GYRO_DLPF & MPU9250_DLPF_MASK)) {
		LOG_ERR("Failed to write gyro digital LPF settings.");
		return -EIO;
	}

	if (i2c_reg_write_byte(drv_data->i2c, cfg->i2c_addr,
			       MPU9250_REG_SR_DIV,
			       CONFIG_MPU9250_GYRO_ODR)) {
		LOG_ERR("Failed to write gyro ODR divider.");
		return -EIO;
	}

	drv_data->gyro_sensitivity_x10 = mpu9250_gyro_sensitivity_x10[i];

#ifdef CONFIG_MPU9250_TRIGGER
	if (mpu9250_init_interrupt(dev)) {
		LOG_DBG("Failed to initialize interrupts.");
		return -EIO;
	}
#endif

#ifdef CONFIG_MPU9250_MAGN_EN
	if (ak8963_init(dev)) {
		return -EIO;
	}
#endif

	return 0;
}

static struct mpu9250_data mpu9250_driver;
static const struct mpu9250_config mpu9250_cfg = {
	.i2c_label = DT_INST_BUS_LABEL(0),
	.i2c_addr = DT_INST_REG_ADDR(0),
#ifdef CONFIG_MPU9250_TRIGGER
	.int_pin = DT_INST_GPIO_PIN(0, irq_gpios),
	.int_flags = DT_INST_GPIO_FLAGS(0, irq_gpios),
	.int_label = DT_INST_GPIO_LABEL(0, irq_gpios),
#endif /* CONFIG_MPU9250_TRIGGER */
};

DEVICE_DT_INST_DEFINE(0, mpu9250_init, NULL,
		      &mpu9250_driver, &mpu9250_cfg,
		      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		      &mpu9250_driver_api);
