#!/bin/bash

if ! test -d /sys/devices/platform/odroidu2-fan; then
    echo Error: this script is for odroid only
    echo Try:
    echo tools/odroid/push-tree.sh -H odroid-mjmech sudo tools/odroid/setup-system.sh
    exit 1
fi


if [[ `id -u` != 0 ]]; then
    echo This script needs to be run as root
    exit 1
fi

set -e

cd $(dirname $(readlink -f "$0"))/../..
echo Using settings from $PWD @ $(hostname)

# packages install
if ! ./install-packages.sh --system --test; then
    ./install-packages.sh --system -y
fi


# general config
update-alternatives --set editor /bin/nano

for group in adm video dialout plugdev netdev i2c; do
    if ! id odroid | fgrep -q "($group)"; then
        gpasswd -a odroid $group;
    fi
done

if locale 2>&1 >/dev/null | grep 'Cannot set'; then
    locale-gen en_US.UTF-8
fi

# assert some stuff
if [[ $(date +%Z) != "UTC" ]]; then
    # does not happen in default image
    echo Timezone $(date +%Z) is bad, run 'sudo dpkg-reconfigure tzdata'
    exit 1
fi

if pgrep NetworkManager; then
    echo Somehow network-manager got installed. remove it
    exit 1
fi

# helper to override system files with backup
backup_path=/var/system-backups/setup-$(date +%F-%H%M%S)
copy_stdin_to() {
    local target="$1"
    local mode="$2"
    local tempname=/tmp/new-system-file
    cat >$tempname
    if [[ -f "$target" ]]; then
        if cmp -s "$tempname" "$target"; then
            echo Up to date: $target
            rm $tempname
            return
        fi
        if ! test -d $backup_path; then
            echo Will back up files to $backup_path
            mkdir -p $backup_path
        fi
        rsync --relative -a $target $backup_path
        echo Updating: $target
    else
        echo Creating: $target
    fi
    cp $tempname $target
    rm $tempname
    if [[ "$mode" != "" ]]; then
        chmod $mode $target
    fi
}

# Enable ifplugd support for wired ethernet
copy_stdin_to /etc/default/ifplugd <<EOF
INTERFACES="eth0"
HOTPLUG_INTERFACES="eth0"
ARGS="-q -f -u0 -d10 -w -I"
SUSPEND_ACTION="stop"
EOF

copy_stdin_to /etc/network/interfaces.d/eth0 <<EOF
# managed by ifplugd
allow-hotplug eth0
iface eth0 inet dhcp
EOF

# run our setup script fo rprimary link cards
copy_stdin_to /etc/network/interfaces.d/wlinkX <<EOF
auto wlink0
iface wlink0 inet manual
        # running root script from user's homedir is icky, but
        # that user has passwordless sudo anyway
        up /home/odroid/mjmech-clean/tools/setup_wifi_link.py up-boot
EOF

# Enable wpa_supplicant for any additional wifi cards
copy_stdin_to /etc/network/interfaces.d/wlan0 <<EOF
auto wlan0
allow-hotplug wlan0
iface wlan0 inet manual
        # there seems to be some sort of race on startup, so
        # sleep for some time.
        pre-up sleep 5
        wpa-driver wext
        wpa-roam /etc/wpa_supplicant/wpa_supplicant.conf
        post-up iw dev wlan0 power_save off || true

iface default inet dhcp
EOF

# disable persistent net generator so any model of wlan card appears as wlan0
echo -n | copy_stdin_to /etc/udev/rules.d/75-persistent-net-generator.rules

# add our rules to use dual-band wifi cards as comm link
# to get info:
#  udevadm info --query=all --attribute-walk --path=/sys/class/net/wlan0
# (note we add line-break support here because udev does not have it)
perl -0pe 's/\\\s*/ /g' <<EOF | \
    copy_stdin_to /etc/udev/rules.d/70-persistent-net.rules
SUBSYSTEM=="net", ACTION=="add", DRIVERS=="rtl8812au", ATTR{dev_id}=="0x0", \
     ATTR{type}=="1", NAME="wlink%n"
SUBSYSTEM=="net", ACTION=="add", DRIVERS=="rt2800usb", ATTR{dev_id}=="0x0", \
     ATTR{type}=="1", NAME="wlink%n"
EOF

# Add wifi config file, but do not override wifi passwords
if ! test -f /etc/wpa_supplicant/wpa_supplicant.conf; then
    copy_stdin_to /etc/wpa_supplicant/wpa_supplicant.conf <<EOF
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev

network={
 ssid="NETWORK"
 psk="PASSWORD"
}

EOF
fi

# update ralink firmware
# file from windows installer, version 5.1.24.0 from 10/26/2015
# version listing is at http://www.mediatek.com/en/downloads1/downloads/
# actual downloading is easier elsewhere -- no need to give email
# run exe in wine, and while it is sitting at the license screen, extract
# the .cab file from temp dir with unshield 1.3.

# Update ralink firmware to version from
# per advice in http://raspberrypi.stackexchange.com/questions/15153/i-get-wifi-timouts-with-the-rt2800usb-driver
# Version 33 is from DPO_RT5572_LinuxSTA_2.6.1.3_20121022.tar.bz2
# Version 29 is from standard repos
copy_stdin_to /lib/firmware/rt2870.bin < tools/odroid/rt2870.bin.29

# prevent kernel complains for rt2800usb-based wifi cards
copy_stdin_to /etc/modprobe.d/rt2800usb.conf <<EOF
#options rt2800usb nohwcrypt=1
EOF

if ! test -f /etc/init/networking.conf.orig; then
    copy_stdin_to /etc/init/networking.conf.orig </etc/init/networking.conf
fi

# Normally, upstart requires all statically configured interfaces to be up
# during boot. This is a problem if wlan0 is requested to be up, but the adapter
# is not present. So ignore it.
# See /etc/network/if-up.d/upstart for details.

# (we carefully patch the file because 'unmount nfs before reboot'
# functionality is useful)
export net_code="    mkdir /run/network/static-network-up-emitted "
net_code+=" && initctl emit --no-wait static-network-up"
net_code+=" && logger network was not up, continuing boot anyway"
perl -p -e 's/(ifup -a)/$1\n$ENV{"net_code"}\n/' \
   /etc/init/networking.conf.orig | copy_stdin_to /etc/init/networking.conf

# disable IPv6  (to prevent extra packets)
copy_stdin_to /etc/sysctl.d/60-mjmech.conf <<EOF
net.ipv6.conf.all.disable_ipv6 = 1
EOF

# one logfile is enough
copy_stdin_to /etc/rsyslog.d/50-default.conf <<EOF
*.*                   /var/log/syslog
*.emerg                                :omusrmsg:*
EOF

# Do not update package indices periodically
copy_stdin_to /etc/apt/apt.conf.d/10periodic <<EOF
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Download-Upgradeable-Packages "0";
APT::Periodic::AutocleanInterval "0";
Apt::Periodic::Enable "0";
EOF

# Do not backup aptitude package states or passwords, do not auto-update
echo -n | copy_stdin_to /etc/cron.daily/aptitude
echo -n | copy_stdin_to /etc/cron.daily/dpkg
echo -n | copy_stdin_to /etc/cron.daily/passwd
echo -n | copy_stdin_to /etc/cron.daily/update-notifier-common
echo -n | copy_stdin_to /etc/cron.weekly/update-notifier-common
echo -n | copy_stdin_to /etc/cron.weekly/apt-xapian-index

# add out-of-tree wifi driver
#test -d /usr/src/8812au-4.2.2 || \
#    git clone https://github.com/gnab/rtl8812au.git /usr/src/8812au-4.2.2

# TODO theamk: actually build the driver. This requires getting the kernel
# sources somehow -- we seem to get it from some weird autobuilder?
# http://builder.mdrjr.net/kernel-3.8/LATEST/
# test -d /var/lib/dkms/8812au/4.2.2/source || dkms add -m 8812au -v 4.2.2
# test -d XXX || dkms build -m 8812au -v 4.2.2
# test -f XXX || dkms install -m 8812au -v 4.2.2
# also see http://forum.odroid.com/viewtopic.php?f=52&t=1674 on how
# to build your own kernel

copy_stdin_to /etc/default/locale <<EOF
LANG="en_US.UTF-8"
EOF

copy_stdin_to /etc/modules <<EOF
i2c-dev
EOF

# TODO theamk: fix hwclock (/var/log/upstart/hwclock-save.log)
# TODO theamk: run hwclock after successful ntpdate ('sudo start hwclock-save')

# TODO theamk: install xinetd with basic services?

# TODO theamk: read-only root (with auto-remount on SW update)
# https://github.com/theamk/misc-tools/blob/master/docs/debian-readonly-root.txt
# TODO theamk: make sure read-only root has proper /var/log/upstart/mountall.log
# (this should contain fsck entry for rootfs)

# TODO theamk: if filesystem is not readonly, at least rotate syslog and
# /var/log/upstart on each reboot / shutdown

# TODO theamk: setup ALSA?

# TODO theamk: setup log drive?
