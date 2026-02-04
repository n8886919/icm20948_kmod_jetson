obj-m := inv_icm20948.o
inv_icm20948-objs := inv_icm20948_core.o inv_icm20948_i2c.o inv_icm20948_gyro.o inv_icm20948_power.o inv_icm20948_temp.o

KDIR ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
