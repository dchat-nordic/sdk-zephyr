/*
 * Copyright (c) 2021, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <drivers/i2c.h>
#include <sys/util.h>
#include <kernel.h>
#include <drivers/sensor.h>

#include "mpu9250.h"

#include <logging/log.h>
LOG_MODULE_DECLARE(MPU9250, CONFIG_SENSOR_LOG_LEVEL);


int mpu9250_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler)
{
	struct mpu9250_data *drv_data = dev->data;
	const struct mpu9250_config *cfg = dev->config;

	if (trig->type != SENSOR_TRIG_DATA_READY) {
		return -ENOTSUP;
	}

	gpio_pin_interrupt_configure(drv_data->gpio, cfg->int_pin,
				     GPIO_INT_DISABLE);

	drv_data->data_ready_handler = handler;
	if (handler == NULL) {
		return 0;
	}

	drv_data->data_ready_trigger = *trig;

	gpio_pin_interrupt_configure(drv_data->gpio, cfg->int_pin,
				     GPIO_INT_EDGE_TO_ACTIVE);

	return 0;
}

static void mpu9250_gpio_callback(const struct device *dev,
				  struct gpio_callback *cb, uint32_t pins)
{
	struct mpu9250_data *drv_data =
		CONTAINER_OF(cb, struct mpu9250_data, gpio_cb);
	const struct mpu9250_config *cfg = drv_data->dev->config;

	ARG_UNUSED(pins);

	gpio_pin_interrupt_configure(drv_data->gpio, cfg->int_pin,
				     GPIO_INT_DISABLE);

#if defined(CONFIG_MPU9250_TRIGGER_OWN_THREAD)
	k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_MPU9250_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&drv_data->work);
#endif
}

static void mpu9250_thread_cb(const struct device *dev)
{
	struct mpu9250_data *drv_data = dev->data;
	const struct mpu9250_config *cfg = dev->config;

	if (drv_data->data_ready_handler != NULL) {
		drv_data->data_ready_handler(dev,
					     &drv_data->data_ready_trigger);
	}

	gpio_pin_interrupt_configure(drv_data->gpio, cfg->int_pin,
				     GPIO_INT_EDGE_TO_ACTIVE);

}

#ifdef CONFIG_MPU9250_TRIGGER_OWN_THREAD
static void mpu9250_thread(struct mpu9250_data *drv_data)
{
	while (1) {
		k_sem_take(&drv_data->gpio_sem, K_FOREVER);
		mpu9250_thread_cb(drv_data->dev);
	}
}
#endif

#ifdef CONFIG_MPU9250_TRIGGER_GLOBAL_THREAD
static void mpu9250_work_cb(struct k_work *work)
{
	struct mpu9250_data *drv_data =
		CONTAINER_OF(work, struct mpu9250_data, work);

	mpu9250_thread_cb(drv_data->dev);
}
#endif

int mpu9250_init_interrupt(const struct device *dev)
{
	struct mpu9250_data *drv_data = dev->data;
	const struct mpu9250_config *cfg = dev->config;

	/* setup data ready gpio interrupt */
	drv_data->gpio = device_get_binding(cfg->int_label);
	if (drv_data->gpio == NULL) {
		LOG_ERR("Failed to get pointer to %s device",
			    cfg->int_label);
		return -EINVAL;
	}

	drv_data->dev = dev;

	gpio_pin_configure(drv_data->gpio, cfg->int_pin,
			   GPIO_INPUT | cfg->int_flags);

	gpio_init_callback(&drv_data->gpio_cb,
			   mpu9250_gpio_callback,
			   BIT(cfg->int_pin));

	if (gpio_add_callback(drv_data->gpio, &drv_data->gpio_cb) < 0) {
		LOG_ERR("Failed to set gpio callback");
		return -EIO;
	}

	/* enable data ready interrupt */
	if (i2c_reg_write_byte(drv_data->i2c, cfg->i2c_addr,
			       MPU9250_REG_INT_EN, MPU9250_DRDY_EN) < 0) {
		LOG_ERR("Failed to enable data ready interrupt.");
		return -EIO;
	}

#if defined(CONFIG_MPU9250_TRIGGER_OWN_THREAD)
	k_sem_init(&drv_data->gpio_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&drv_data->thread, drv_data->thread_stack,
			CONFIG_MPU9250_THREAD_STACK_SIZE,
			(k_thread_entry_t)mpu9250_thread, drv_data,
			NULL, NULL, K_PRIO_COOP(CONFIG_MPU9250_THREAD_PRIORITY),
			0, K_NO_WAIT);
#elif defined(CONFIG_MPU9250_TRIGGER_GLOBAL_THREAD)
	drv_data->work.handler = mpu9250_work_cb;
#endif

	gpio_pin_interrupt_configure(drv_data->gpio, cfg->int_pin,
				     GPIO_INT_EDGE_TO_ACTIVE);

	return 0;
}
