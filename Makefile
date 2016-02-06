ifneq ($(KERNELRELEASE),)
obj-m := tw6869.o
tw6869-y := tw6869-core.o tw6869-video.o tw6869-audio.o

else
KDIR ?= /lib/modules/`uname -r`/build

default:
	sed -i 's/^.*FSL_G_CHIP_IDENT.*$$/#undef FSL_G_CHIP_IDENT/g' tw6869.h
	sed -i 's/^.*FSL_QUERYBUF.*$$/#undef FSL_QUERYBUF/g' tw6869.h
	$(MAKE) -C $(KDIR) M=$$PWD

fsl:
	sed -i 's/^.*FSL_G_CHIP_IDENT.*$$/#define FSL_G_CHIP_IDENT/g' tw6869.h
	sed -i 's/^.*FSL_QUERYBUF.*$$/#define FSL_QUERYBUF/g' tw6869.h
	$(MAKE) -C $(KDIR) M=$$PWD

install:
	$(MAKE) -C $(KDIR) M=$$PWD modules_install
	depmod -a

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean
endif
