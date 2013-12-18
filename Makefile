DRIVER = rf1101
KERNELDIR=/home/gwang/raspi_tool/kernel-3.8
CROSS_COMPILE=arm-linux-gnueabi-
#KERNELDIR=/lib/modules/3.6.11+/kernel
ifneq ($(KERNELRELEASE),)
    obj-m := $(DRIVER).o
else
    PWD := $(shell pwd)

default:
ifeq ($(strip $(KERNELDIR)),)
	$(error "KERNELDIR is undefined!")
else
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules 
endif


clean:
	rm -rf *~ *.ko *.o *.mod.c modules.order Module.symvers .$(DRIVER)* .tmp_versions

endif

