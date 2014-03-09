/*
 * Device driver for DHT11 temperature and humidity sensor
 * Copyright (C) 2014 Charles Sluder 
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/time.h>

#define MODNAME "sr501"

static inline s64 get_sr501_time_ns(void)
{
	struct timespec ts;

	getnstimeofday(&ts);

	return timespec_to_ns(&ts);
}

struct sr501_private {
	int gpio;
	int value;
	int irq;
	int irq_enable;
	struct mutex mutex;
	struct work_struct nl_work;
};


static ssize_t sr501_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len);
static ssize_t sr501_show(struct device *dev, struct device_attribute *attr, char *buf);
irqreturn_t sr501_isr( int,  void * );
static ssize_t sr501_dev_create(struct class *class, struct class_attribute *attr, const char *buf, size_t len);

