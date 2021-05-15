# motu-avb
Linux USB driver for the MOTU AVB series interfaces

## Kernel parameters:
samplerate: set the samplerate (its currently fixed at module load) default: 44100

midi: set 1 for devices that have a midi port, 0 for the ones that don't

vendor: 0 = use class compliant mode (24 channels in/out), 1 = vendor mode (64/32/24)

## Build
Get the kernel source of your running kernel

Copy the files \<kernel-src\>/sound/usb/usbaudio.h and \<kernel-src\>/sound/usb/midi.h from the kernel source over the files provided in this directory.

Make sure your kernel headers are install, then

    make 
    sudo make install

If you want to use vendor mode, make sure you have curl installed, then connect the device through ethernet and execute the curl command

	 curl  --data 'json={"value":"USB2"}' <ip address of the device>/datastore/host/mode

