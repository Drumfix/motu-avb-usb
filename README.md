# motu-avb
Linux USB driver for the MOTU AVB series interfaces

## Kernel parameters:
samplerate: set the samplerate (its currently fixed at module load) default: 44100
midi: set 1 for devices that have a midi port, 0 for the ones that don't
vendor: 0 = use class compliant mode (24 channels in/out), 1 = vendor mode (64/32/24)

## Build
Make sure your kernel headers are install, then

    make 
    sudo make install
