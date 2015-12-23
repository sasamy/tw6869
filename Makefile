ifneq ($(KERNELRELEASE),)
obj-m := tw6869.o
tw6869-y := tw6869-core.o tw6869-video.o tw6869-audio.o

else
KDIR ?= /lib/modules/`uname -r`/build

default:
	$(MAKE) -C $(KDIR) M=$$PWD
clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean
endif
