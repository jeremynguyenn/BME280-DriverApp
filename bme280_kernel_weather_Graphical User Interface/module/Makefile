obj-m += bme280.o
bme280-objs := module.o bme280_info_mapp.o bme280.o bme280_regs_mapp.o
CFLAGS_bme280.o := -Wall

all:
    make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
    make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean