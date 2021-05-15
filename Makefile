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

copy_h_files:
	cp -v $(KDIRS)/sound/usb/usbaudio.h $(CURDIR)
	cp -v $(KDIRS)/sound/usb/midi.h $(CURDIR)
