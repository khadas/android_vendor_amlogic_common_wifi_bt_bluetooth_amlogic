# RELEASE

BOARD_HAVE_BLUETOOTH_AMLOGIC := true
AML_BLUETOOTH_LPM_ENABLE := true

PRODUCT_COPY_FILES += \
  vendor/amlogic/common/wifi_bt/bluetooth/amlogic/config/aml_bt.conf:$(TARGET_COPY_OUT_VENDOR)/firmware/aml_bt.conf

PRODUCT_PACKAGES += w1_bt_fw_uart.bin w1u_bt_fw_uart.bin w1u_bt_fw_usb.bin w2_bt_fw_uart.bin w2_bt_fw_usb.bin

PRODUCT_COPY_FILES += frameworks/native/data/etc/android.hardware.bluetooth.xml:$(TARGET_COPY_OUT_VENDOR)/etc/permissions/android.hardware.bluetooth.xml \
       frameworks/native/data/etc/android.hardware.bluetooth_le.xml:$(TARGET_COPY_OUT_VENDOR)/etc/permissions/android.hardware.bluetooth_le.xml

