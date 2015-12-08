Compiling new odroid kernel
(modified version, from hardkernel)

-----------------------------
Motivation:

The linux kernel in the odroid distribution we are using is based on 3.8.13.30
kernel modified by hardkernel. This has two problems:
(1) there is no 'build' directory, so it is impossible to build out-of-tree
drivers.
(2) some wireless drivers are too old (such as rt2800usb)

I wanted to do mainline version but I could not get it to work. So here are
the instructions to use patched kernel.

Stock kernel info:
uname: 3.8.13.30 #1 SMP PREEMPT Fri Sep 4 23:45:57 BRT 2015 armv7l armv7l armv7l GNU/Linux
vermagic (from modinfo): 3.8.13.30 SMP preempt mod_unload ARMv7 p2v8
ABI ('file' on module): 32-bit LSB relocatable, ARM, EABI5 version 1 (SYSV)
source: git clone https://github.com/hardkernel/linux.git -b odroid-3.8.y

Instructions:
https://mescanef.net/blog/2014/12/custom-kernel-compilation-for-odroid-u2-u3-on-host-running-fedora-linux/

https://github.com/tobiasjakobi/linux-odroid/archive/odroid-4.3.y.zip

-----------------------------
Preparation:
If you had custom u-boot for mainline kernel, downgrade it with
/usr/local/sbin/odroid-utility.sh
-----------------------------
Process:
We do a cross-compile on a x86_64 machine.

(1) download the kernel, prepare the build dir
 sudo apt-get install gcc-4.7-arm-linux-gnueabi gcc-arm-linux-gnueabi u-boot-tools
 # the kernels are extracted as root, and thus read-only. make the build dir.
 sudo install -d -o $UID /usr/src/kernel-odroid

(1a) get the hardkernel's custom kernel:
 cd /tmp
 wget https://github.com/tobiasjakobi/linux-odroid/archive/odroid-4.3.y.tar.gz
 sudo tar xf odroid-4.3.y -C /usr/src/
 sudo mv /usr/src/linux-odroid-odroid-4.3.y /usr/src/odroid-4.3.y
 # note: 4.3 does not compile with an error:
 #    /usr/src/odroid-4.3.y/drivers/gpu/arm/mali/Kbuild:37: *** Runtime PM is incompatible with non-GPL license.  Stop.

 # patch kernel to remove broken check (I suspect it is not compatible with out-of-tree building)
 sudo nano -w /usr/src/odroid-4.3.y/drivers/gpu/arm/mali/Kbuild
 # replace
 #  ifeq ($(wildcard $(src)/linux/license/gpl/*),)
 # with
 #  ifeq ($(wildcard $(src)/linux/license/gpl/*),NO)

(1b) to get earlier hardkernel default config
 cd /tmp
 wget https://github.com/hardkernel/linux/archive/odroid-3.14.y-linaro.tar.gz
 sudo tar xf odroid-3.14.y-linaro -C /usr/src/
 sudo mv /usr/src/linux-odroid-3.14.y-linaro /usr/src/odroid-3.14.y-ln

(2) configure kernel:
note: always use 'kmake.sh' instead of make in kernel dir, or you many get
options related to x86 kernel. It also move .config file location inside
the tree, so 'git diff' becomes more helpful.

(2a) newer kernel
 tools/odroid/kernel/kmake.sh odroid-4.3.y exynos_defconfig
 tools/odroid/kernel/kmake.sh odroid-4.3.y menuconfig
 # pay attention to options with '(NEW)'
 # make sure to clear CONFIG_EXTRA_FIRMWARE if using mainline kernels
 # make sure to CONFIG_MODULE_COMPRESS is not set

(2b) older kernel

(3) build and copy kernel
 time tools/odroid/kernel/kmake.sh odroid-4.3.y -j4 zImage modules dtbs
 tools/odroid/kernel/kmake.sh odroid-4.3.y modules_install firmware_install zinstall

 # copy all files over to main storage
 rsync -av --rsync-path='sudo rsync' /usr/src/kernel-odroid/odroid-4.3.y-install/ odroid@odroid-mjmech:/

 # copy to /boot partition, on mjmech run:
   cd /media/boot/
   sudo cp /boot/k-4.3.0-mjodroid* .
   head k-4.3.0-mjodroid.script
   # run line from comments
   sudo mkimage -T script -d k-4.3.0-mjodroid.script k-4.3.0-mjodroid.scr

(4) activate the new kernel
 odroid@odroid-mjmech# sudo cp k-4.3.0-mjodroid.scr boot.scr
 odroid@odroid-mjmech# sudo reboot
 (or reboot once and choo

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
