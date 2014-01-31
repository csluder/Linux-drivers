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

#define MODNAME "dht11"

/*
 * For the beaglebone black:
 * Start plus falling edge up front for the prolog and 40 falling edges for data bits
 * The data starts with pulse 2. For a faster processor this may have to be changed
 * to 43 and 3 if the IRQ can be hooked fast enough to catch the first falling edge.
 * To handle this dynamically based on the width of the first pulse adds code to the ISR
 * or forces the code to timeout. For the beaglebone this extra code increases the number of 
 * of failures so I chose to statically set the number of edges.
 */
#define EXPECTED_IRQS 42
#define FIRST_DATA_PULSE 2

#define DATA_LOW  0
#define DATA_HIGH 1

/*
 * The sensor uses pulse width modulation to communicate with the host.
 * Host sends a 500 uS low pulse and then pulls the pin high
 * Sensor responds within 20 to 40 uS with an 80 uS low pulse
 * The sensor pulls the pin high for 80 uS then sends 40 data frames.
 * Each data frame consists of a 50 uS start pulse (low) followed by
 * a data pulse (high). A 0 is 26 to 28 uS and a 1 is 70 uS.
 * The DHT11 expects an 18 mS start signal.
 */

#ifdef DHT22
#define START_SIGNAL     500
#else
#define START_SIGNAL     18
#endif

#define START_XMIT_INTVL 50
#define INVALID_DATA -200


/*
 * The start interval plus data interval is ~77 mS for 0 and 130 mS for 1. With
 * the latency and jitter of the interrupt subsystem, using 110 as the dividing 
 * point decodes > 90% of the samples accurately.
 */
#define DATA_ONE_INTVL 110

static inline s64 get_dht11_time_ns(void)
{
	struct timespec ts;

	getnstimeofday(&ts);

	return timespec_to_ns(&ts);
}

struct dht11_private {
	int gpio;
	int temperature;
	int humidity;
	int irq;
	int bitIdx;
	unsigned long sampleTime;
	struct mutex mutex;
	s64 time[ EXPECTED_IRQS ];
	struct completion complete;
} dht11_private;


void dht11_decode( struct dht11_private* private );
int dht11_sample_sensor(struct dht11_private* private );
static ssize_t dht11_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len);
static ssize_t dht11_show(struct device *dev, struct device_attribute *attr, char *buf);
irqreturn_t dht11_isr( int,  void * );

