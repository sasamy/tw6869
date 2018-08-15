# tw6869/tw6865 V4L2 driver

Freescale/NXP i.MX6 - primary platform on which the driver is tested, more details can be found here:

https://community.freescale.com/thread/319973

## Generic installation instructions:

$ make

$ sudo make install

## Note: on i.MX6 linux 4.9+, you must disable MSI on the kernel command line

pci=nomsi

https://community.nxp.com/message/1021557?commentID=1021557#comment-1021557
