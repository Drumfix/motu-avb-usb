# motu-avb
Linux USB driver for the MOTU AVB series interfaces

## Kernel parameters:
samplerate: set the samplerate (its currently fixed at module load) default: 44100

midi: set 1 for devices that have a midi port, 0 for the ones that don't

vendor: 0 = use class compliant mode (24 channels in/out), 1 = vendor mode (64/32/24)

## Build
Install the kernel source of your running kernel.

The makefile expects the kernel source to be in /lib/modules/<kernel-version>/source.
	
If that is not the case set the environment variable KDIRS to point to the location of your kernel source.

Then execute

    make 
    sudo make install

If you want to use vendor mode, make sure you have curl installed, then connect the device through ethernet and execute the curl command

	 curl  --data 'json={"value":"USB2"}' <ip address of the device>/datastore/host/mode

