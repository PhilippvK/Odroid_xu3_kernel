# installs driver libraries on device
DEST_DIR=/system/lib/modules/
SRC_DIR=$(pwd)/../../../out/target/product/odroidxu3/system/lib/modules
ADB=adb

if [ "$#" -eq 1 ]; then
	OPTIONAL_ANDROID_ID="$1"
fi

MODULES=($(find . -name "*.ko"))

echo "${MODULES[*]}"

$ADB $OPTIONAL_ANDROID_ID root
$ADB $OPTIONAL_ANDROID_ID wait-for-device
$ADB $OPTIONAL_ANDROID_ID shell mount -o rw,remount /system
$ADB $OPTIONAL_ANDROID_ID shell chmod 777 $DEST_DIR
$ADB $OPTIONAL_ANDROID_ID push ${MODULES[*]} $DEST_DIR

# Push sys_logger to device if enabled
test -f drivers/cpufreq/sys_logger.ko && $ADB $OPTIONAL_ANDROID_ID push drivers/cpufreq/sys_logger.ko /data/local/tmp/sys_logger.ko
