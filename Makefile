ifneq ($(KERNELVERSION),)
obj-m	:= ds3231_drv.o


else
KDIR	:= /lib/modules/$(shell uname -r)/build
PWD	:= $(shell pwd)
BUILD_DIR ?= $(PWD)/build
BUILD_DIR_MAKEFILE ?= $(BUILD_DIR)/Makefile

default: ds3231_drv

$(BUILD_DIR):
	mkdir -p "$@"

$(BUILD_DIR_MAKEFILE): $(BUILD_DIR)
	touch "$@"

ds3231_drv: $(BUILD_DIR_MAKEFILE)
	@echo "Building module ..."
	@(make -C $(KDIR) M=$(BUILD_DIR) src=$(PWD) modules)

clean:
	-rm -f *.o *.ko .*.cmd .*.flags *.mod.c Module.symvers modules.order
	-rm -rf build/
	-rm -rf .tmp_versions
	-rm -rf *~

reload: ds3231_drv
	-sudo rmmod ds3231_drv
	sudo insmod build/ds3231_drv.ko

test: 
	@bash -c "cd $(BUILD_DIR); /home/TreiberTests/all.sh $(TEST)"

endif
