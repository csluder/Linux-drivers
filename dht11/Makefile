# If KERNELRELEASE is defined then called by kbuild
# else called from cmdline.

ifneq ($(KERNELRELEASE),)
	obj-m = dht11.o
else
	KERNEL_SOURCE := /opt/poky/1.5.1/sysroots/armv7a-vfp-neon-poky-linux-gnueabi/usr/src/linux
	PWD := `pwd`

default:
	$(MAKE) -C $(KERNEL_SOURCE) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNEL_SOURCE) M=$(PWD) clean
endif
