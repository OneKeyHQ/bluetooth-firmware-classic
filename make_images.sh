#!/bin/sh

PATH_UTILS=./utils
PATH_KEY=temp.pk
PATH_OUTPUT_DIR=artifacts_signed
PATH_OUTPUT_FACTORY_HEX=$PATH_OUTPUT_DIR/factory.hex
PATH_OUTPUT_FACTORY_BIN=$PATH_OUTPUT_DIR/factory.bin
PATH_OUTPUT_ZIP=$PATH_OUTPUT_DIR/ota.zip
PATH_OUTPUT_OTA_BIN=$PATH_OUTPUT_DIR/ota.bin
PATH_BL_CONFIG=bl_config.hex
PATH_BOOTLOADER=artifacts/dfu.hex
PATH_APP=artifacts/app.hex
PATH_SD=ble-firmware/components/softdevice/s132/hex/s132_nrf52_7.0.1_softdevice.hex

rm -rf $PATH_OUTPUT_DIR
mkdir $PATH_OUTPUT_DIR

chmod +x $PATH_UTILS/*

##### SETUP #####

# Check if nrf5sdk-tools is already installed
if $PATH_UTILS/nrfutil list 2>/dev/null | grep -q "nrf5sdk-tools"; then
    echo "nrf5sdk-tools already installed, skipping install"
else
    echo "Installing nrf5sdk-tools..."
    # Clean up any corrupted bootstrap cache that might cause issues
    rm -rf ~/.nrfutil/bootstrap 2>/dev/null || true
    # Try to install nrf5sdk-tools
    # Note: This may fail if the bootstrap URL is unavailable (410 Gone error)
    # In that case, the tools might still work if they're built into nrfutil
    if ! $PATH_UTILS/nrfutil install nrf5sdk-tools 2>&1; then
        echo "Warning: nrfutil install failed (likely due to unavailable bootstrap URL)"
        echo "Checking if nrf5sdk-tools commands are available anyway..."
        # Test if the commands we need are available
        if $PATH_UTILS/nrfutil settings --help >/dev/null 2>&1 && \
           $PATH_UTILS/nrfutil pkg --help >/dev/null 2>&1; then
            echo "nrf5sdk-tools commands appear to be available, continuing..."
        else
            echo "Error: Required nrfutil commands not available"
            exit 1
        fi
    fi
fi

echo "$BT_SIG_PK" > $PATH_KEY
unset BT_SIG_PK

##### FACTORY #####

# gen hex
$PATH_UTILS/nrfutil settings generate --family NRF52 --application $PATH_APP --application-version 3 --bootloader-version 2 --bl-settings-version 2 --app-boot-validation VALIDATE_ECDSA_P256_SHA256 --key-file $PATH_KEY $PATH_BL_CONFIG
$PATH_UTILS/mergehex --merge $PATH_BOOTLOADER $PATH_BL_CONFIG $PATH_SD $PATH_APP --output $PATH_OUTPUT_FACTORY_HEX
rm $PATH_BL_CONFIG

# gen bin
objcopy -I ihex -O binary --gap-fill=0xFF $PATH_OUTPUT_FACTORY_HEX $PATH_OUTPUT_FACTORY_BIN.tmp
dd if=$PATH_OUTPUT_FACTORY_BIN.tmp of=$PATH_OUTPUT_FACTORY_BIN bs=512K count=1
rm $PATH_OUTPUT_FACTORY_BIN.tmp 

##### OTA #####

# sign app
$PATH_UTILS/nrfutil pkg generate --application $PATH_APP --application-version 3 --hw-version 52 --sd-req 0xCB --app-boot-validation VALIDATE_ECDSA_P256_SHA256 --key-file $PATH_KEY $PATH_OUTPUT_ZIP
rm $PATH_KEY
$PATH_UTILS/ota_to_onekey_bin.py $PATH_OUTPUT_ZIP $PATH_OUTPUT_OTA_BIN