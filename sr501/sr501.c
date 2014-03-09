/*
 * Device driver for DHT11 value and humidity sensor
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
#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>

#include "sr501.h"

#define MODNAME "sr501"
#define MAX_PAYLOAD 256
#define NETLINK_SENSOR 31

/*
 * Netlink socket for broadcasting interrupt events to user apps
 */
struct sock *sr501_nl_sk = NULL;

/*
 * Workqueue for ISR bottom half
 */
static struct workqueue_struct *sr501_workqueue;

/*
 * sysfs interface used by app to create an instance of the
 *  motion sensor device
 */
static struct class_attribute sr501_class_attrs[] = {
        __ATTR(sr501_gpio, 0200, NULL, sr501_dev_create),
        __ATTR_NULL,
};

/*
 * sysfs device class for the motion sensor
 */
static struct class sr501_class = {
        .name =         "sr501",
        .owner =        THIS_MODULE,

        .class_attrs =  sr501_class_attrs,
};



/*
 * Setup sysfs attributes
 */
struct sr501_attr {
        struct device_attribute attr;
	int value;
};


static struct sr501_attr sr501_value = {
	.attr.attr.name="value",
	.attr.attr.mode = 0444,
	.attr.show = sr501_show,
	.attr.store = sr501_store,
	.value = 1,
};

static struct sr501_attr sr501_enable = {
	.attr.attr.name="enable",
	.attr.attr.mode = 0666,
	.attr.show = sr501_show,
	.attr.store = sr501_store,
	.value = 2,
};


/*
 * sysfs  read interface.
 */
static ssize_t sr501_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	const struct sr501_private  *sr501_data = dev_get_drvdata(dev);

        struct sr501_attr *aPtr = container_of(attr, struct sr501_attr, attr);

	switch (aPtr->value)
	{
		case 1:
			/*
			 * Return value of PIR sensor
			 */
			return scnprintf(buf, PAGE_SIZE, "%d\n", gpio_get_value(sr501_data->gpio));
			break;
		case 2:
			/*
			 * Return IRQ enabled status
			 */
			return scnprintf(buf, PAGE_SIZE, "%d\n", sr501_data->irq_enable);
			break;
		default:
			return 0;
	}
}


/*
 * The sysfs write interface.
 */
static ssize_t 
sr501_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	int ret;
	int enable;
	struct sr501_private  *sr501_data = dev_get_drvdata(dev);

	sscanf( buf, "%d", &enable );

	/*
	 * If this is the first time enabling the interrupt, request the IRQ.
	 */
	if ( (enable == 1) && (sr501_data->irq_enable == -1) ) 
	{
		/*
		 * Set the gpio pin direction for the motion sensor data line to 
		 * input.
		 */
		gpio_direction_input( sr501_data->gpio );

		/*
		 * Find the IRQ mapping for this gpio pin
		 */
		sr501_data->irq = gpio_to_irq( sr501_data->gpio );
		if ( sr501_data->irq < 0 ) {
			dev_printk(KERN_ERR, dev, 
				"Failed to get irq for gpio %d\n", sr501_data->gpio);
		}

		/*
		 * Configure the interrupt to trigger on the rising edge of the sensor 
	         * data line signal.
		 */
		ret = request_irq( sr501_data->irq,
				   sr501_isr,
				   IRQF_TRIGGER_RISING,
				   MODNAME,
			   	   (void *)sr501_data );

		sr501_data->irq_enable = 1;

		if ( ret < 0 ) {
			dev_printk(KERN_ERR, dev, "Failed to hook interrupt\n");
			return ret;
		}

	}

	/*
	 * If The enable request is different from the current interrupt state, swith
	 * to the requested state otherwise ignore the request.
	 */
	if (enable != sr501_data->irq_enable ) {
		if ( enable == 1 ) {
			enable_irq( sr501_data->irq );
			sr501_data->irq_enable = 1;
		} else {
			disable_irq( sr501_data->irq );
			sr501_data->irq_enable = 0;
		}
	}

	return sizeof(int);
}

/*
 * Handle 3 volt input from motion sensor.  Queue the work structure for this device to the 
 * workqueue for the module. The workque handler will broadcast a message to the user 
 * applications on a netlink socket.
 */
irqreturn_t
sr501_isr( int irq, void* data )
{
	struct sr501_private *sr501_data = data;

        queue_work(sr501_workqueue, &sr501_data->nl_work);

	return IRQ_HANDLED;
}

/*
 * This is the workqueue handler for the motion sensor interrupt. It broadcasts a message
 * with the gpio number and a message indicating the sensor generated an event.
 */
void
sr501_nl_work(struct work_struct *work)
{
	struct sk_buff *skb = NULL;
	struct nlmsghdr *nlh;
	char *data;

	/*
	 * The address of the work structure is in the device private data structure.
	 * Use the address of the work struct to find the address of the private data 
	 * structure.
	 */
        struct sr501_private *sr501_data = container_of(work, struct sr501_private, nl_work);

        skb = nlmsg_new(256, GFP_ATOMIC);
        if (!skb) {
                return;
        }


	nlh = nlmsg_put(skb, 0, 0, NLMSG_DONE, 256, 0);
	if (!nlh) {
	        kfree_skb(skb);
	        return;
	}
	
	/*
	 * Fill in the payload with the gpio pin and the message.
	 */
	data = nlmsg_data(nlh);
	
	data[0] = sr501_data->gpio;
	memcpy(&data[1], "Inimicum ad portas. Para bellum!", 33);
	
	/*
	 * Distribution group 1
	 */
	NETLINK_CB(skb).dst_group = 1;
	
	netlink_broadcast(sr501_nl_sk, skb, 0, 1, GFP_ATOMIC);

        return;
}


/*
 * Create and initialize a device and data structure for each motion sensor.
 */
static ssize_t
sr501_dev_create(struct class *class,
                 struct class_attribute *attr,
                 const char *buf, 
                 size_t len)

{
	int gpio;
	struct sr501_private *sr501_data;
	struct device *sr501_device;

	sscanf( buf, "%d", &gpio );

	if ( gpio_is_valid( gpio ) ){
		int ret;

		ret = gpio_request( gpio, "sr501" );
		if ( ret < 0 ) {
			printk("GPIO request for %d failed\n", gpio);
			return ret;
		}

		/*
		 * This will be the private per device structure
		 */
		sr501_data = kmalloc( sizeof(struct sr501_private), GFP_KERNEL );
		if ( sr501_data == NULL ) {
			return -ENOMEM;
		}

		/*
		 * Create the device and an entry in the sysfs class for this device.
		 * The file created will have the gpio pin number appended.
		 */
		sr501_device =  device_create( class, NULL, MKDEV(0, gpio), sr501_data, "sr501_%u", gpio );

		if ( IS_ERR(sr501_device) ) {
			printk( "Call to class_device_create failed\n" );
		}

		/*
		 * Successful. Initialize the per device private data structure
		 */
		sr501_data->gpio = gpio;
		sr501_data->irq_enable = -1;
		sr501_data->irq = -1;
		mutex_init( &sr501_data->mutex );
		INIT_WORK(&sr501_data->nl_work, sr501_nl_work);

		/*
		 * Provide sysfs interfaces to enable interrupts and read the sensor value.
		 *
		 * /sys/class/sr501sr501_n/value
		 * /sys/class/sr501sr501_n/enable
       		 */
		device_create_file( sr501_device, &sr501_value.attr );
		device_create_file( sr501_device, &sr501_enable.attr );
	}

	return sizeof(int);
}
 

/*
 * Process netlink messages from user applications
 */
static void 
nl_data_ready (struct sk_buff *skb)
{

}

static int __init sr501_start(void)
{
	int status;

	/*
	 * Receive message handler for the netlink socket
	 */
	struct netlink_kernel_cfg cfg = {
		.input  = nl_data_ready,
	};


	/*
	 * Cannot send netlink messages from ISR since it can sometimes deadlock
	 * with send and recv message system calls if they occur in the same context.
	 * Create a work queue to send the messages in a thread context.
	 */
	sr501_workqueue = create_workqueue("sr501_netlink");
	if (sr501_workqueue == NULL) {
		return -ENOMEM;
	}

	/*
	 * Create sysfs class entry
	 */
	status = class_register(&sr501_class);
        if (status < 0) {
		destroy_workqueue(sr501_workqueue);
                return status;
	}

	
	/*
	 * Create a netlink socket to pass data to user app.
	 */
 	sr501_nl_sk = netlink_kernel_create(&init_net, NETLINK_SENSOR, &cfg);
	if ( sr501_nl_sk <= 0 ) {
		destroy_workqueue(sr501_workqueue);
		class_destroy( &sr501_class );
		return (int)sr501_nl_sk;
	}



	return 0;
}

/*
 *  Find the sr501 devices. Called by class_find_device.
 */
static int sr501_match(struct device *dev, const void *data)
{
	return !strncmp(dev_name(dev), (const char *)data, 5);
}

/*
 * Cleanup everything created by the driver
 */
static void __exit sr501_end(void)
{
	struct device *dev;

	netlink_kernel_release(sr501_nl_sk);
	flush_workqueue(sr501_workqueue);
	destroy_workqueue(sr501_workqueue);

	/**
	 * Cleanup the devices that have been created
	 */
	while ( 1 == 1 )
	{
		struct sr501_private  *sr501_data;
		dev = class_find_device(&sr501_class, NULL, "sr501", sr501_match);
		if ( dev == 0 ) {
			break;
		}

		sr501_data = dev_get_drvdata(dev);
		device_remove_file( dev, &sr501_value.attr );
		device_remove_file( dev, &sr501_enable.attr );
		if (sr501_data->gpio >= 0) {
			free_irq( sr501_data->irq, (void *)sr501_data );
		}
		gpio_free( sr501_data->gpio );
		device_destroy( &sr501_class, MKDEV(0, sr501_data->gpio) );
		kfree(sr501_data);
	}

	class_destroy( &sr501_class );
}


module_init(sr501_start);
module_exit(sr501_end);

MODULE_AUTHOR("Charles Sluder");
MODULE_DESCRIPTION("HC-SR501 PIR motion sensor");
MODULE_LICENSE("GPL v2");
