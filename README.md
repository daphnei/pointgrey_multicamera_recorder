# Camera polling server for synchronized video recording

This repo contains a ROS node for recording simeoultaneously from several Pointgrey Cameras, with aligned timestamps.
It is corrently a work in progress. All code has been tested on four Grasshopper cameras.

## Installation dependencies:

- The bgp package (which itself has gtsam and other dependencies, see documentation there)
 
    git clone https://github.com/berndpfrommer/bgp.git

- The flea3 driver:

    git clone https://github.com/KumarRobotics/flea3.git

## Testing extrinsic calibration:

- Record a bag with images (just a few seconds should be plenty enough):

    # launch camera polling
    roslaunch poll_cameras cam_poll.launch 
    # in another shell record into bag, kill record after a few seconds
    rosbag record -O /data/extrinsic_calib_2017-04-20.bag /poll_cameras/cam0/image_raw /poll_cameras/cam1/image_raw /poll_cameras/cam2/image_raw /poll_cameras/cam3/image_raw

- Run the extrinsic calibration on the bag

    roslaunch poll_cameras extrinsic_calibration.launch bag_file:=/data/extrinsic_calib_2017-04-20.bag 


## Getting maximum USB3 performance:
To achieve full performance, each camera needs to be served by a
dedicated 5Gbits/sec host controller.

Use the lspci command to see what controllers you have, and to what pci bus they are connected:

    lspci -tv
    -[0000:00]-+-00.0  Intel Corporation Sky Lake Host Bridge/DRAM Registers
               +-01.0-[01]----00.0  Samsung Electronics Co Ltd Device a804
               +-02.0  Intel Corporation Sky Lake Integrated Graphics
               +-14.0  Intel Corporation Sunrise Point-H USB 3.0 xHCI Controller
               +-14.2  Intel Corporation Sunrise Point-H Thermal subsystem
               +-16.0  Intel Corporation Sunrise Point-H CSME HECI #1
               +-17.0  Intel Corporation SATA Controller [RAID mode]
               +-1c.0-[02-04]----00.0-[03-04]----02.0-[04]----00.0  Renesas Technology Corp. uPD720201 USB 3.0 Host Controller
               +-1c.4-[05]----00.0  Samsung Electronics Co Ltd Device a804
               +-1d.0-[06-0b]----00.0-[07-0b]--+-01.0-[08]----00.0  Renesas Technology Corp. uPD720202 USB 3.0 Host Controller
               |                               +-02.0-[09]----00.0  Renesas Technology Corp. uPD720202 USB 3.0 Host Controller
               |                               +-03.0-[0a]----00.0  Renesas Technology Corp. uPD720202 USB 3.0 Host Controller
               |                               \-04.0-[0b]----00.0  Renesas Technology Corp. uPD720202 USB 3.0 Host Controller
               +-1f.0  Intel Corporation Sunrise Point-H LPC Controller
               +-1f.2  Intel Corporation Sunrise Point-H PMC
               +-1f.3  Intel Corporation Sunrise Point-H HD Audio
               +-1f.4  Intel Corporation Sunrise Point-H SMBus
                \-1f.6  Intel Corporation Ethernet Connection (2) I219-LM


In this example there are 5 usb controllers:

- 14.0  Intel Corporation Sunrise Point-H USB 3.0 xHCI Controller, that's the built-in controller on the main board. Unfortunately, all USB3.0 ports on the motherboard share this controller, so you can only run a single camera off of the main board.
- 1c.0  Renesas Technology Corp. uPD720201 USB 3.0 Host Controller, that's an old-style PCI card (not PCIe!) that also just has a single USB 3.0 controller.
- 1d.0   Renesas Technology Corp. uPD720202 USB 3.0 Host Controller, and you see 4 of them because each port on this card has its own host controller.


Now let's look at what devices there are:

    lsusb -v | grep -i 'point grey' | grep -i bus

    Bus 006 Device 003: ID 1e10:3300 Point Grey Research, Inc. 
    Bus 004 Device 003: ID 1e10:3300 Point Grey Research, Inc. 
    Bus 004 Device 002: ID 1e10:3300 Point Grey Research, Inc. 
    Bus 002 Device 002: ID 1e10:3300 Point Grey Research, Inc. 

But which camera is which? 

    lsusb -v -d 1e10:3300 | grep -i serial
    iSerial                 3 01040A0C
    iSerial                 3 01040A05
    iSerial                 3 01040A10
    iSerial                 3 01040A0B

Now use a hex-to-decimal converter on the last column to get the pointgrey serial numbers so now you know which device is on what bus. Unfortunately, that is the usb bus number, not the pci bus, so we have to find the mapping between the two:

    ls -l /sys/bus/usb/devices/usb*
    lrwxrwxrwx 1 root root 0 Apr 15 18:40 /sys/bus/usb/devices/usb1 -> ../../../devices/pci0000:00/0000:00:14.0/usb1
    lrwxrwxrwx 1 root root 0 Apr 15 18:40 /sys/bus/usb/devices/usb10 -> ../../../devices/pci0000:00/0000:00:1d.0/0000:06:00.0/0000:07:03.0/0000:0a:00.0/usb10
    lrwxrwxrwx 1 root root 0 Apr 15 18:40 /sys/bus/usb/devices/usb11 -> ../../../devices/pci0000:00/0000:00:1d.0/0000:06:00.0/0000:07:04.0/0000:0b:00.0/usb11
    lrwxrwxrwx 1 root root 0 Apr 15 18:40 /sys/bus/usb/devices/usb12 -> ../../../devices/pci0000:00/0000:00:1d.0/0000:06:00.0/0000:07:04.0/0000:0b:00.0/usb12
    lrwxrwxrwx 1 root root 0 Apr 15 18:40 /sys/bus/usb/devices/usb2 -> ../../../devices/pci0000:00/0000:00:14.0/usb2
    lrwxrwxrwx 1 root root 0 Apr 15 18:40 /sys/bus/usb/devices/usb3 -> ../../../devices/pci0000:00/0000:00:1c.0/0000:02:00.0/0000:03:02.0/0000:04:00.0/usb3
    lrwxrwxrwx 1 root root 0 Apr 15 18:40 /sys/bus/usb/devices/usb4 -> ../../../devices/pci0000:00/0000:00:1c.0/0000:02:00.0/0000:03:02.0/0000:04:00.0/usb4
    lrwxrwxrwx 1 root root 0 Apr 15 18:40 /sys/bus/usb/devices/usb5 -> ../../../devices/pci0000:00/0000:00:1d.0/0000:06:00.0/0000:07:01.0/0000:08:00.0/usb5
    lrwxrwxrwx 1 root root 0 Apr 15 18:40 /sys/bus/usb/devices/usb6 -> ../../../devices/pci0000:00/0000:00:1d.0/0000:06:00.0/0000:07:01.0/0000:08:00.0/usb6
    lrwxrwxrwx 1 root root 0 Apr 15 18:40 /sys/bus/usb/devices/usb7 -> ../../../devices/pci0000:00/0000:00:1d.0/0000:06:00.0/0000:07:02.0/0000:09:00.0/usb7
    lrwxrwxrwx 1 root root 0 Apr 15 18:40 /sys/bus/usb/devices/usb8 -> ../../../devices/pci0000:00/0000:00:1d.0/0000:06:00.0/0000:07:02.0/0000:09:00.0/usb8
    lrwxrwxrwx 1 root root 0 Apr 15 18:40 /sys/bus/usb/devices/usb9 -> ../../../devices/pci0000:00/0000:00:1d.0/0000:06:00.0/0000:07:03.0/0000:0a:00.0/usb9


Here usb bus 004 maps to pci bus 1c, which we know to be the old style PCI card, and in fact, there are two cameras connected to this single bus, so we cannot get full performance. We could have seen that already from a simple lsusb -t:

    lsusb -t
    /:  Bus 12.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/1p, 5000M
    /:  Bus 11.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/1p, 480M
    /:  Bus 10.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/1p, 5000M
    /:  Bus 09.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/1p, 480M
        |__ Port 1: Dev 2, If 0, Class=Human Interface Device, Driver=usbhid, 1.5M
    /:  Bus 08.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/1p, 5000M
    /:  Bus 07.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/1p, 480M
        |__ Port 1: Dev 2, If 0, Class=Human Interface Device, Driver=usbhid, 1.5M
    /:  Bus 06.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/1p, 5000M
        |__ Port 1: Dev 3, If 0, Class=Miscellaneous Device, Driver=, 5000M
        |__ Port 1: Dev 3, If 1, Class=Miscellaneous Device, Driver=, 5000M
        |__ Port 1: Dev 3, If 2, Class=Miscellaneous Device, Driver=, 5000M
    /:  Bus 05.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/1p, 480M
    /:  Bus 04.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/4p, 5000M
        |__ Port 1: Dev 2, If 0, Class=Miscellaneous Device, Driver=, 5000M
        |__ Port 1: Dev 2, If 1, Class=Miscellaneous Device, Driver=, 5000M
        |__ Port 1: Dev 2, If 2, Class=Miscellaneous Device, Driver=, 5000M
        |__ Port 4: Dev 3, If 1, Class=Miscellaneous Device, Driver=, 5000M
        |__ Port 4: Dev 3, If 2, Class=Miscellaneous Device, Driver=, 5000M
        |__ Port 4: Dev 3, If 0, Class=Miscellaneous Device, Driver=, 5000M
    /:  Bus 03.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/4p, 480M
    /:  Bus 02.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/10p, 5000M
        |__ Port 6: Dev 2, If 0, Class=Miscellaneous Device, Driver=, 5000M
        |__ Port 6: Dev 2, If 1, Class=Miscellaneous Device, Driver=, 5000M
        |__ Port 6: Dev 2, If 2, Class=Miscellaneous Device, Driver=, 5000M
    /:  Bus 01.Port 1: Dev 1, Class=root_hub, Driver=xhci_hcd/16p, 480M

There are two devices (Dev 2 and Dev 3, each with 3 interfaces) connected to bus 04, no good!
