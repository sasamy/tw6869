tw6869/tw6865 V4L2 driver

Freescale/NXP i.MX6 - primary platform on which the driver is tested, more details can be found here:

https://community.freescale.com/thread/319973

To use NXP userptr DMA the additional patch for Linux kernel is required:

https://github.com/sasamy/mxc-v4l2-tvin-tw6869-vdi/blob/master/rel_imx_3.14.52_1.1.0_ga-nxp-userptr-dma.patch


Generic installation instructions:

$ make

$ sudo make install