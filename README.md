# motu-avb
Linux USB driver for the MOTU AVB series interfaces

## Module parameters:
samplerate: set the samplerate (its currently fixed at module load) default: 44100

midi: set 1 for devices that have a midi port, 0 for the ones that don't

vendor: 0 = use class compliant mode (24 channels in/out), 1 = vendor mode (64/32/24)

Important: vendor mode requires to patch and recompile the kernel!

it is recommended to set the parameters in the file /etc/modprobe.d/alsa-base.conf, e.g.

	options motu samplerate=44100 midi=1 vendor=0 

You may also make linux load the module during boot to prevent the alsa usb audio driver to take control of your device.
This is done by adding motu to file /etc/modules-load.d/modules.conf

## Preparations

In case you use a dual boot with windows, make sure you temporarily disable secure boot in the BIOS for installation
of the driver.

Install dkms and the kernel source of your running kernel, then

##Build

	sudo cp -r motu-avb-usb /usr/src/motu-avb-usb-1.0
	sudo dkms add motu-avb-usb/1.0
	sudo dkms build motu-avb-usb/1.0
	sudo dkms install motu-avb-usb/1.0

If you want to use vendor mode, make sure you have curl installed, then connect the device through ethernet and execute the curl command

	 curl  --data 'json={"value":"USB2"}' <ip address of the device>/datastore/host/mode

