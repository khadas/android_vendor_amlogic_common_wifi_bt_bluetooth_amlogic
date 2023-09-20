KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(PWD)
###############################################################################
# AMLBT
###############################################################################
AMLBT_W1U_MOD_NAME = w1u_bt
AMLBT_W2_MOD_NAME = w2_bt

AMLBT_W1U_CFILES := \
	aml_bt_usb.c

AMLBT_W2_CFILES := \
	aml_bt_usb_w2.c
$(AMLBT_W1U_MOD_NAME)-objs := $(AMLBT_W1U_CFILES:.c=.o)
$(AMLBT_W2_MOD_NAME)-objs := $(AMLBT_W2_CFILES:.c=.o)

###############################################################################
# Common
###############################################################################
obj-m := $(AMLBT_W1U_MOD_NAME).o
obj-m += $(AMLBT_W2_MOD_NAME).o

EXTRA_CFLAGS += -Wno-pointer-to-int-cast

all:
	make -C $(KERNEL_SRC) M=$(M) KBUILD_EXTRA_SYMBOLS=$(EXTRA_SYMBOLS_PATH_USB_BT) modules
modules_install:
	@$(MAKE) INSTALL_MOD_STRIP=1 M=$(M) -C $(KERNEL_SRC) modules_install
	mkdir -p ${OUT_DIR}/../vendor_lib/modules
	cd ${OUT_DIR}/$(M)/; find -name "*.ko" -exec cp {} ${OUT_DIR}/../vendor_lib/modules/ \;

usb:
	make -C $(KERNEL_SRC) M=$(PWD) $(AMLBT_W1U_MOD_NAME).ko
	make -C $(KERNEL_SRC) M=$(PWD) $(AMLBT_W2_MOD_NAME).ko

clean:
	make -C $(KERNEL_SRC) M=$(PWD) clean



