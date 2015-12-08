Compiling mainline kernel for odroid

-----------------------------
Motivation:

The linux kernel in the odroid distribution we are using is based on 3.8.13.30
kernel modified by hardkernel. This has two problems:
(1) there is no 'build' directory, so it is impossible to build out-of-tree
drivers.
(2) some wireless drivers are too old (such as rt2800usb)

There are some instructions on how to partially rebuild a kernel to fix (1),
but if we are going to make our own kernel, we migh t as well go to mainline.

Stock kernel info:
uname: 3.8.13.30 #1 SMP PREEMPT Fri Sep 4 23:45:57 BRT 2015 armv7l armv7l armv7l GNU/Linux
vermagic (from modinfo): 3.8.13.30 SMP preempt mod_unload ARMv7 p2v8
ABI ('file' on module): 32-bit LSB relocatable, ARM, EABI5 version 1 (SYSV)
source: git clone https://github.com/hardkernel/linux.git -b odroid-3.8.y

Inspiration:
http://rglinuxtech.com/?p=1622 (and other posts on this blog)

Instructions:
https://mescanef.net/blog/2014/12/custom-kernel-compilation-for-odroid-u2-u3-on-host-running-fedora-linux/
http://rtisat.blogspot.com/search/label/odroid-u3

-----------------------------
Preparation:
possibly "mainline kernels need an updated u-boot to successfully boot."
http://rtisat.blogspot.com/2015/04/a-little-progress.html

I updated the u-boot via odroid-utility.sh

luckily, initrd does not seem to contain any kernel modules.
check with:
dd bs=64 skip=1 if=/media/boot/uInitrd  | zcat | cpio --list | less

New kernel needs new u-boot, which can be updated per blog post above.
Run the following on the odroid:
    apt-get install device-tree-compiler
    git clone https://github.com/tobiasjakobi/u-boot
    cd u-boot
    make odroid_config
    make
    # commands and constants are from /usr/local/bin/kernel_update.sh
    echo 0 | sudo tee /sys/block/mmcblk0boot0/force_ro
    sudo dd iflag=dsync oflag=dsync if=./u-boot-dtb.bin of=/dev/mmcblk0boot0 seek=62

-----------------------------
Process:
We do a cross-compile on a x86_64 machine.

(1) download the kernel, prepare the build dir
 sudo apt-get install gcc-4.7-arm-linux-gnueabi gcc-arm-linux-gnueabi
 # the kernels are extracted as root, and thus read-only. make the build dir.
 sudo install -d -o $UID /usr/src/kernel-odroid

(1a) get the stock kernel. note
 cd /tmp
 wget https://cdn.kernel.org/pub/linux/kernel/v4.x/linux-4.3.tar.xz
 sudo tar xJf linux-4.3.tar.xz -C /usr/src
  get the odroid kernel

(2) configure kernel:
note: always use 'kmake.sh' instead of make in kernel dir, or you many get
options related to x86 kernel. It also move .config file location inside
the tree, so 'git diff' becomes more helpful.

use previous configuration with new things set as modules:
 rm tools/odroid/kernel/config-linux-4.3
 KCONFIG_ALLCONFIG=$PWD/tools/odroid/kernel/config-hardkernel-3.8.13.30 \
     tools/odroid/kernel/kmake.sh linux-4.3 allmodconfig
OR use previous configuration with new things set as default:
 cp tools/odroid/kernel/config-hardkernel-3.8.13.30 tools/odroid/kernel/config-linux-4.3
 tools/odroid/kernel/kmake.sh linux-4.3 olddefconfig

the enter/exit menunconfig to inspect it by hand and update the config:
 tools/odroid/kernel/kmake.sh linux-4.3 menuconfig
 # pay attention to options with '(NEW)'
 # make sure to clear CONFIG_EXTRA_FIRMWARE if using mainline kernels
 # make sure to CONFIG_MODULE_COMPRESS is not set

(3) build and copy modules
 time tools/odroid/kernel/kmake.sh linux-4.3 -j4 zImage modules
 tools/odroid/kernel/kmake.sh linux-4.3 modules_install firmware_install zinstall

 # Check version
 tools/odroid/kernel/kmake.sh linux-4.3 -s kernelrelease
 # optionally delete old modules
 rm /usr/src/kernel-odroid/linux-4.3-install/boot/*.old
 ssh odroid@odroid-mjmech rm -rf /lib/modules/4.3.0-mjmech1 /lib/firmware-linux-4.3
 # copy all files over (this will not activate them yet)
 rsync -av --rsync-path='sudo rsync' /usr/src/kernel-odroid/linux-4.3-install/ odroid@odroid-mjmech:/

 # copy to /boot partition
 odroid@odroid-mjmech# sudo cp /boot/exynos4412-odroidu3.dtb /media/boot/
 odroid@odroid-mjmech# sudo cp /boot/vmlinuz-4.3.0-mjmech1 /media/boot/zImage-4.3
 sudo mkimage -T script -d kernel-4.3.script kernel-4.3.scr

(4) activate the new kernel
 odroid@odroid-mjmech# sudo cp kernel-4.3.scr boot.scr
 odroid@odroid-mjmech# sudo reboot


--------
recovery procedure
- connect serial port, open at 115200
- press any key during countdown
- run the original boot script (old bootloader)
fatload mmc 0:1 40008000 boot-auto_edid.scr
run bootscript
# if the bootloader has been updated, the last step will fail; in this case, run:
bootz  0x40008000 0x42000000

- once booted, restore original script:
odroid@odroid-mjmech$ cd /media/boot
odroid@odroid-mjmech$ sudo cp boot-auto_edid.scr boot.scr
odroid@odroid-mjmech$ sudo /usr/local/bin/odroid-utility.sh
# select 'update bootloader'

- examine the current config (or just see the boot.scr file on original image)
Exynos4412 # printenv bootcmd
bootcmd=    cfgload; mmc rescan 0:1; mmc rescan 0:2;  if run loadbootscript_1;     then run bootscript;  else     if run loadbootscript_2;  |
Exynos4412 # cfgload
do_fat_cfgload : cmd = fatload mmc 0:1 0x41000000 boot.ini
reading boot.ini
** Unable to read file boot.ini **
Exynos4412 # printenv loadbootscript_1
loadbootscript_1=echo >>> Load Boot Script from mmc 0:1 <<<;fatload mmc 0:1 40008000 boot.scr
Exynos4412 # run loadbootscript_1
>>> Load Boot Script from mmc 0:1 <<<
reading boot.scr
Warning : Reads a file that is smaller than the cluster size.
380 bytes read in 34 ms (10.7 KiB/s)
Exynos4412 # printenv bootscript
bootscript=source 40008000
Exynos4412 # md 40008000 75
40008000: 56190527 379fda4f 3e2d7d52 3c010000    '..VO..7R}->...<
40008010: 00000000 00000000 39df7f05 00060205    ...........9....
40008020: 746f6f62 7263732e 726f6620 77205820    boot.scr for X w
40008030: 20687469 494d4448 74756120 72702d6f    ith HDMI auto-pr
40008040: 34010000 00000000 65746573 6920766e    ...4....setenv i
40008050: 7274696e 69685f64 22206867 66667830    nitrd_high "0xff

- run stored commands using different kernel name
setenv initrd_high "0xffffffff"
setenv fdt_high "0xffffffff"
setenv bootcmd "fatload mmc 0:1 0x40008000 zImage.0; fatload mmc 0:1 0x42000000 uInitrd; bootm 0x40008000 0x42000000"
setenv edid_load "drm_kms_helper.edid_firmware=edid/1024x768.bin"
setenv bootargs "console=tty1 console=ttySAC1,115200n8 ${edid_load} root=UUID=e139ce78-9841-40fe-8823-96a304a09859 rootwait ro  mem=2047M"
boot
