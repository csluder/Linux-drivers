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

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>  
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#include "dht11.h"

#define MODNAME "dht11"

static struct class* dht11_class;
struct device* dht11_device;


/*
 * Setup sysfs attributes
 */
struct dht11_attr {
        struct device_attribute attr;
	int value;
};

static struct dht11_attr dht11_config = {
	.attr.attr.name="GPIO",
	.attr.attr.mode = 0666,
	.attr.show = dht11_show,
	.attr.store = dht11_store,
	.value = 1,
};

static struct dht11_attr dht11_temp = {
	.attr.attr.name="temperature",
	.attr.attr.mode = 0444,
	.attr.show = dht11_show,
	.attr.store = dht11_store,
	.value = 2,
};

static struct dht11_attr dht11_humidity = {
	.attr.attr.name="humidity",
	.attr.attr.mode = 0444,
	.attr.show = dht11_show,
	.attr.store = dht11_store,
	.value = 3,
};

static struct dht11_attr dht11_sample = {
	.attr.attr.name="sample",
	.attr.attr.mode = 0666,
	.attr.show = dht11_show,
	.attr.store = dht11_store,
	.value = 4,
};



/*
 * sysfs show routine. Handles GPIO, temperature, and humidity
 */
static ssize_t dht11_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;

        struct dht11_attr *aPtr = container_of(attr, struct dht11_attr, attr);

	switch (aPtr->value)
	{
		case 1:
			return scnprintf(buf, PAGE_SIZE, "%d\n", dht11_private.gpio);
			break;
		case 2:
			/*
			 * Return temperature
			 *
			 * Only allow periodic sampling so as to not overburden the sensor.
			 * Arbitrarily chose 10 second sampling interval
			 */
			if ( (jiffies - dht11_private.sampleTime) >= (HZ * 10) ) {
				ret = mutex_lock_interruptible( &dht11_private.mutex );
				if (ret == -EINTR ) {
					return ret;
				}
				dht11_sample_sensor( &dht11_private );
				mutex_unlock( &dht11_private.mutex );
			}
			return scnprintf(buf, PAGE_SIZE, "%d\n", dht11_private.temperature);
			break;
		case 3:
			/*
			 * Return humidity
			 *
			 * Only allow periodic sampling so as to not overburden the sensor.
			 * Arbitrarily chose 10 second sampling interval
			 */
			if ( (jiffies - dht11_private.sampleTime) >= (HZ * 10) ) {
				ret = mutex_lock_interruptible( &dht11_private.mutex );
				if (ret == -EINTR ) {
					return ret;
				}
				dht11_sample_sensor( &dht11_private );
				mutex_unlock( &dht11_private.mutex );
			}
			return scnprintf(buf, PAGE_SIZE, "%d\n", dht11_private.humidity);
			break;
		case 4:
			return 0;
			break;
		default:
			return 0;
	}
}

/*
 * Data is decoded into four one byte data values and a CRC. The time is
 * sampled on the falling edge. On the Beaglebone this yeilds 42 samples
 * with sample 0 being the start time. The second sample is the Second falling
 * edge after switching to input mode. Apparently the beaglebone is two slow 
 * to catch the first edge. This may need to be tweeked for a faster processor.
 * i.e. EXPECTED_IRQS = 43 and first pulse = pulse 3.
 */
void dht11_decode( struct dht11_private* private )
{
	char data[5] = { 0, 0, 0, 0, 0 };
	int bitIdx = 0;
	int i;
	s64* dht11_time = &private->time[0];
	s64 last = dht11_time[FIRST_DATA_PULSE - 1];

	/*
         * Start decoding with bit two. The first pulse is 230 uS of prolog.
         */
	for ( i = FIRST_DATA_PULSE; i < EXPECTED_IRQS; i++ )
	{	
		int interval;
		
		interval = (int)(dht11_time[i] - last)/1000;

		/*
		 * Set the data bit to 0 and then OR ina 1 if the interval is
		 * greater than some predetermined point between the DATA_LOW
		 * pulse width and the DATA_HIGH pulse width.
		 */
		data [ bitIdx / 8 ] <<= 1;
		if ( interval >= DATA_ONE_INTVL )
		{
			data[ bitIdx / 8 ] |= DATA_HIGH;
		}

		bitIdx++;
		last = dht11_time[i];
	}

	/*
	 * If the checksum doesn't match then we return a value beyond the range of the sensor.
	 * The sample time is only updated for success. This allows the sample to be rerun 
	 * immediately if it failed. If it succeeds we only sample if the data is stale as
	 * determined by the elapsed time since the last succesful sample.
	 */
	if ( data[0] + data[2] == data[4] ) {
		private->humidity = data[0];
        	private->temperature = data[2];
		private->sampleTime = jiffies;
	} else {
		private->humidity = INVALID_DATA;
        	private->temperature = INVALID_DATA;
		dev_printk(KERN_WARNING, dht11_device, "Checksum did not match\n");
	}
}

/*
 * Send the data request signal to the sensor. Line should be pulled high by the pullup
 * at the start. Driving the line high causes it to bounce when the pin is switched to
 * input mode. (Dave Jenkins says the capacitor between Vcc and Gnd doesn't filter as well 
 * if it is charged when the cycle is started.)  Drive the pin low for the start signal 
 * duration and then switch the pin to input mode. The pullup resister should bring the line 
 * high if there is enough pullup voltage (The beaglebone works with a power supply, but not
 * when using USB power). The sensor will respond with data or the wait will timeout after 
 * two seconds.
 */
int dht11_sample_sensor(struct dht11_private* private )
{
	int ret;

	if ( gpio_is_valid(private->gpio) ) {

		/*
		 * Drive the line low
		 */
		ret = gpio_direction_output(private->gpio, 0);
		if ( ret == 0 ) {

			/*
			 * Hold the line low for the required duration
			 */
			msleep(START_SIGNAL);

			/**
			 * Stop driving the output and let the pullup resister pull the line high.
			 * The sensor should respond with an 80 uS low response.
			 */
			ret = gpio_direction_input( private->gpio );
			if (ret < 0) {
				dev_printk(KERN_ERR, dht11_device, "GPIO set input failed \n");
				return ret;
			} 

			/*
			 * Grab the start time. The data is decoded post sampling using
			 * the elapsed time between falling edges.
			 */
			private->time[ 0 ] = get_dht11_time_ns();
			private->bitIdx = 1;

			/*
			 * Hook the IRQ to the ISR for the dht11 part.  The request IRQ function
			 * and the gpio direction function checks the pin direction/IRQ assignment
			 * and returns an error if there is an IRQ assigned in output mode. 
			 * Therefore we have to request and free the IRQ every time the pin
			 * direction is switched. End result the first edge is lost, fortunately
			 * it is not needed.
			 */
			ret = request_irq( private->irq,
					   dht11_isr,
					   IRQF_NO_SUSPEND|IRQF_DISABLED|IRQF_TRIGGER_FALLING,
					   MODNAME,
					   (void *)private );

			if ( ret < 0 ) {
				gpio_free( private->gpio );
				dev_printk(KERN_ERR, dht11_device, "Failed to hook interrupt\n");
				return ret;
			}
				
			/*
			 * It takes about 5.5 ms if it doesn't miss an edge. The documents all
			 * say two seconds, so I'm assuming the device needs some quite time to
			 * itself before it can be sampled again. Wait 2 seconds and if it
			 * fails the device will be ready to sample again immediately.
			 */
			ret = wait_for_completion_killable_timeout(&private->complete, HZ * 2 );
			free_irq( dht11_private.irq, (void *)private );
			if ( (ret == 0) || (private->bitIdx != EXPECTED_IRQS) ) {
				dev_printk(KERN_WARNING, dht11_device, "Sample timed out\n");
				private->humidity = INVALID_DATA;
		        	private->temperature = INVALID_DATA;
				return -EINVAL;
			} 

			/*
			 * Decode the data and stash the results in the private data structure.
			 */
			dht11_decode( private );

		} else {
			dev_printk(KERN_ERR, dht11_device,
				 "Failed to set gpio direction %d %d \n", ret, dht11_private.gpio );
		}
	}

	return 0;
}


static ssize_t dht11_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	int ret;

        struct dht11_attr *aPtr = container_of(attr, struct dht11_attr, attr);
	switch (aPtr->value)
	{
		case 1:
			/*
			 * Request the gpio pin passed by the user.
			 *
			 * If the gpio is already assigned it needs to be freed
			 */
			if ( gpio_is_valid(dht11_private.gpio) )
			{
				ret = mutex_lock_interruptible( &dht11_private.mutex );
				if (ret == -EINTR ) {
					return ret;
				}
				gpio_free( dht11_private.gpio );
				mutex_unlock( &dht11_private.mutex );
			}

			sscanf( buf, "%d", &dht11_private.gpio );

			if ( dht11_private.gpio && gpio_is_valid(dht11_private.gpio) ){
				/*
				 * User passes in the gpio pin number the part is tied to
				 */
				ret = gpio_request( dht11_private.gpio, "dht11" );
				if ( ret < 0 ) {
					dht11_private.gpio = -1;
					dev_printk(KERN_ERR, dht11_device,
						"GPIO request for %d failed\n", dht11_private.gpio);
					return ret;
				}
		
				/*
				 * Don't care if it fails. Just trying to put it in a known state.
				 */
				gpio_direction_input( dht11_private.gpio );

				/*
				 * Find the IRQ mapping for this gpio pin
				 */
				dht11_private.irq = gpio_to_irq( dht11_private.gpio );
				if ( dht11_private.irq < 0 ) {
					gpio_free( dht11_private.gpio );
					dev_printk(KERN_ERR, dht11_device, 
						"Failed to get irq for gpio %d\n", dht11_private.gpio);
					return dht11_private.irq;
				}
			} else {
				return -EINVAL;
			}
			break;

		case 4:
			/*
			 * Normal case is to sample on the read periodically. This is a
			 * forced sample for those times when waiting just will not do.
			 */
			ret = mutex_lock_interruptible( &dht11_private.mutex );
			if (ret == -EINTR ) {
				return ret;
			}

			ret = dht11_sample_sensor( &dht11_private );
			mutex_unlock( &dht11_private.mutex );
			if (ret < 0 ) {
				return ret;
			}
			break;

		default:
			return -EINVAL;
	}
	return sizeof(int);
}

/*
 * This needs to be fast or the data gets munged during the first four pulses.
 */
irqreturn_t
dht11_isr( int irq, void* data )
{
	struct dht11_private* private = data;


	private->time[ private->bitIdx++ ] = get_dht11_time_ns();

	if ( private->bitIdx >= (EXPECTED_IRQS) ) {
		complete( &private->complete );
	}

	return IRQ_HANDLED;
}

 

static int __init dht11_start(void)
{


	dht11_private.sampleTime = 0;
	dht11_private.gpio = -1;
	dht11_private.irq = 0;

	mutex_init( &dht11_private.mutex );

	/*
	 * Create sysfs entries
	 */

	dht11_class = class_create( THIS_MODULE, "hsss_sensors" );
	if ( IS_ERR(dht11_class) ) {
		dev_printk( KERN_ERR, dht11_device, "Call to class_create failed\n" );
	}
	
	dht11_device =  device_create( dht11_class, NULL, MKDEV(0,0), NULL, "dht11" );
	if ( IS_ERR(dht11_device) ) {
		dev_printk( KERN_ERR, dht11_device, "Call to class_device_create failed\n" );
	}

	dev_printk(KERN_INFO, dht11_device, "Loading DHT11 module...\n");

	/*
	 * /sys/class/hsss_sensors/dht11/config
	 * /sys/class/hsss_sensors/dht11/temperature
	 * /sys/class/hsss_sensors/dht11/humidity
	 * /sys/class/hsss_sensors/dht11/sample
         */
	device_create_file( dht11_device, &dht11_config.attr );
	device_create_file( dht11_device, &dht11_temp.attr );
	device_create_file( dht11_device, &dht11_humidity.attr );
	device_create_file( dht11_device, &dht11_sample.attr );
	
	/**
	 * Completion variable to synchronize isr with data sampling interface.
	 */
	init_completion( &dht11_private.complete );
	
	return 0;
}

static void __exit dht11_end(void)
{
	

	dev_printk(KERN_INFO, dht11_device, "Removing DHT11 module\n");

	 mutex_lock( &dht11_private.mutex );

	device_remove_file( dht11_device, &dht11_config.attr );
	device_remove_file( dht11_device, &dht11_temp.attr );
	device_remove_file( dht11_device, &dht11_humidity.attr );
	device_remove_file( dht11_device, &dht11_sample.attr );

	device_destroy( dht11_class, MKDEV(0,0) );

	class_destroy( dht11_class );

	gpio_free( dht11_private.gpio );

	mutex_unlock( &dht11_private.mutex );
}


module_init(dht11_start);
module_exit(dht11_end);

MODULE_AUTHOR("Charles Sluder");
MODULE_DESCRIPTION("DHTxx humidity and temperature sensor");
MODULE_LICENSE("GPL v2");
