LOCAL_PATH := $(call my-dir)

ifneq ($(BOARD_HAVE_BLUETOOTH_AMLOGIC),)

include $(CLEAR_VARS)

# androidT and above use packages/modules/Bluetooth/system path
ifeq ($(filter-out 33 34, $(strip $(PLATFORM_SDK_VERSION))), )
BDROID_DIR := $(TOP_DIR)packages/modules/Bluetooth/system
else
BDROID_DIR := $(TOP_DIR)system/bt
endif

LOCAL_SRC_FILES := \
        src/bt_vendor_aml.c \
        src/hardware.c \
        src/userial_vendor.c \
        src/upio.c \
        src/conf.c \
        src/FallthroughBTA.cpp \
        src/sysbridge.cpp

LOCAL_C_INCLUDES += \
        $(LOCAL_PATH)/include \
        $(BDROID_DIR)/hci/include \
        $(TOP)/vendor/amlogic/frameworks/services \
        $(TOP)/$(BOARD_AML_VENDOR_PATH)/frameworks/services \
        $(TOP_DIR)vendor/amlogic/common/wifi_bt/bluetooth/common/include

LOCAL_SHARED_LIBRARIES := \
        libcutils \
        liblog \
        libbinder \
        libsystemcontrolservice \
        libutils \
        libdl
LOCAL_CFLAGS += -DANDROID_PLATFORM_SDK_VERSION=$(PLATFORM_SDK_VERSION)  -DUSE_SYS_WRITE_SERVICE=1

LOCAL_CFLAGS +=  -Wno-unused-parameter -Wno-unused-variable -Wno-unused-function -Wno-unneeded-internal-declaration -Wno-implicit-function-declaration


ifeq ($(shell test $(PLATFORM_SDK_VERSION) -ge 26 && echo OK),OK)
LOCAL_C_INCLUDES += \
       $(BDROID_DIR)/device/include

LOCAL_CFLAGS += -DO_AMLOGIC
endif

ifeq ($(BOARD_HAVE_BLUETOOTH_MULTIBT),true)
	LOCAL_MODULE := libbt-vendor_aml
else
	LOCAL_MODULE := libbt-vendor
endif

LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_OWNER := amlogic

ifeq ($(shell test $(PLATFORM_SDK_VERSION) -ge 26 && echo OK),OK)
LOCAL_PROPRIETARY_MODULE := true
endif

LOCAL_LICENSE_KINDS := SPDX-license-identifier-Apache-2.0 legacy_proprietary
LOCAL_LICENSE_CONDITIONS := notice proprietary by_exception_only

include $(LOCAL_PATH)/vnd_buildcfg.mk

include $(BUILD_SHARED_LIBRARY)


endif # BOARD_HAVE_BLUETOOTH_AMLOGIC
