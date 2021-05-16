obj-m += motu.o

# Default to running kernel's build directory if KDIR not set externally
KDIR ?= "/lib/modules/$(shell uname -r)/build"
KDIRS ?= "/lib/modules/$(shell uname -r)/source"

all: copy_h_files
	$(MAKE) -C "$(KDIR)" M="$(CURDIR)" modules

install:
	$(MAKE) -C "$(KDIR)" M="$(CURDIR)" modules_install
	depmod -A

clean:
	$(MAKE) -C "$(KDIR)" M="$(CURDIR)" clean

show:
	@echo $(KDIR)

HEADER_FILE_USBAUDIO := $(KDIRS)/sound/usb/usbaudio.h
HEADER_FILE_MIDI := $(KDIRS)/sound/usb/midi.h
copy_h_files:
	@echo "Copying required header files from kernel sources..."
	@if [ -f $(HEADER_FILE_USBAUDIO) ] ; then cp -v $(HEADER_FILE_USBAUDIO) $(CURDIR) ; else echo "Could not locate required header file from kernel sources: $(HEADER_FILE_USBAUDIO)" ; fi
	@if [ -f $(HEADER_FILE_MIDI) ] ; then cp -v $(HEADER_FILE_MIDI) $(CURDIR) ; else echo "Could not locate required header file from kernel sources: $(HEADER_FILE_MIDI)" ; fi
