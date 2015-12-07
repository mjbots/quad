#!/bin/bash
# wrapper for kernel make that sets all variables for cross-compiling

set -e

mydir=$(dirname $(readlink -f "$0"))

kversion=linux-4.3
KERNEL_SRC=/usr/src/kernel-odroid/${kversion}
if [[ ! -d "$KERNEL_SRC" ]]; then
    KERNEL_SRC=/usr/src/${kversion}
fi

# Where the files will be build
export KBUILD_OUTPUT=/usr/src/kernel-odroid/${kversion}-build

out_path=/usr/src/kernel-odroid/${kversion}-install
export INSTALL_PATH=${out_path}/boot
export INSTALL_MOD_PATH=${out_path}

export KCONFIG_CONFIG=${mydir}/config-${kversion}
export LOCALVERSION="-mjmech1"
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabi-
export CORES=4

export INSTALL_FW_PATH=${out_path}/lib/firmware-${kversion}
export INSTALLKERNEL=nonexistent-file

# LOADADDR is obtained from u-boot command line, see
#   strings /media/boot/boot.scr
export LOADADDR=40008000

if ! test -d "$KERNEL_SRC"; then
    echo kernel source not found
    exit 1
fi

mkdir -p ${out_path}
mkdir -p $INSTALL_PATH

cd $KERNEL_SRC
exec make INSTALL_FW_PATH=$INSTALL_FW_PATH INSTALLKERNEL=$INSTALLKERNEL "$@"
