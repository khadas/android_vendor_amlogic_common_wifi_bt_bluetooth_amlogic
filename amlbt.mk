# RELEASE

BOARD_HAVE_BLUETOOTH_AMLOGIC := true
AML_BLUETOOTH_LPM_ENABLE := true

PRODUCT_COPY_FILES += $(call find-copy-subdir-files,*,vendor/amlogic/common/wifi_bt/bluetooth/amlogic/config,$(TARGET_COPY_OUT_VENDOR)/firmware)
PRODUCT_COPY_FILES += $(call find-copy-subdir-files,*,vendor/amlogic/common/wifi_bt/bluetooth/amlogic/firmware,$(TARGET_COPY_OUT_VENDOR)/firmware)

PRODUCT_COPY_FILES += frameworks/native/data/etc/android.hardware.bluetooth.xml:$(TARGET_COPY_OUT_VENDOR)/etc/permissions/android.hardware.bluetooth.xml \
       frameworks/native/data/etc/android.hardware.bluetooth_le.xml:$(TARGET_COPY_OUT_VENDOR)/etc/permissions/android.hardware.bluetooth_le.xml

