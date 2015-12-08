#!/bin/bash
set -e

kversion="$1"
image="$2"
system_map="$3"
target="$4"
if [[ "$target" == "" ]]; then
    echo do not run this, run 'kmake.sh .. zinstall' instead
    exit 1
fi

echo making odroid files
img_out=k-$kversion-$(basename $image)
dtb_out=k-$kversion-dtb
map_out=k-$kversion-$(basename $system_map)
script_out=k-$kversion.script
# do the dtb first so it fails fast

if [[ "$img_out" == *-uImage ]]; then
    boot_word="bootm"
else
    boot_word="bootz"
fi

cat >$target/$script_out <<EOF
# this script needs to be wrapped on odroid:
#   mkimage -T script -d $script_out k-$kversion.scr
# the resulting scr may be copied over boot.scr to activate

setenv initrd_high "0xffffffff"
setenv fdt_high "0xffffffff"
setenv load_image "fatload mmc 0:1 0x40008000 $img_out; run load_misc"
setenv load_misc "fatload mmc 0:1 0x42000000 uInitrd; fatload mmc 0:1 0x4f000000 $dtb_out"
setenv bootcmd "run load_image; $boot_word 40008000 42000000 4f000000"
setenv bootargs "console=tty1 console=ttySAC1,115200n8 root=LABEL=rootfs panic=5 rootwait earlyprintk ro mem=2047M smsc95xx.turbo_mode=N"
boot

EOF


(set -x;
    cp arch/arm/boot/dts/exynos4412-odroidu3.dtb $target/$dtb_out
    cp $image $target/$img_out
    cp $system_map $target/$map_out
    cd $target && ls -lh k-$kversion* )
