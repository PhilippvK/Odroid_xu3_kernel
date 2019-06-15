#!/bin/bash

if [ $# -eq 0 ]
then
    echo "Please specify board (odroidxu3 ;) )"
    exit 0
fi

CPU_JOB_NUM=$(nproc --all)

KERNEL_CROSS_COMPILE_PATH="/opt/toolchains/arm-eabi-4.8/bin/arm-eabi-"
echo "cross compiler path: $KERNEL_CROSS_COMPILE_PATH"

PRODUCT_BOARD=$1

function check_exit()
{
    if [ $? != 0 ]
    then
        exit $?
    fi
}

function build_kernel()
{

    echo "make -j$CPU_JOB_NUM ARCH=arm CROSS_COMPILE=$KERNEL_CROSS_COMPILE_PATH ANDROID_MAJOR_VERSION=7 ANDROID_VERSION=1 CONFIG_DEBUG_SECTION_MISMATCH=y"
    echo
    make -j$CPU_JOB_NUM ARCH=arm CROSS_COMPILE=$KERNEL_CROSS_COMPILE_PATH ANDROID_MAJOR_VERSION=7 ANDROID_VERSION=1 CONFIG_DEBUG_SECTION_MISMATCH=y
    check_exit
}

build_kernel

echo success!!

exit 0
