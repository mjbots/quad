#!/bin/bash

if ! test -d /sys/devices/platform/odroidu2-fan; then
    echo Error: this script is for odroid only
fi


if [[ `id -u` != 0 ]]; then
    echo This script needs to be run as root
    exit 1
fi

set -e

cd $(dirname $(readlink -f "$0"))/../..
echo Using settings from $PWD @ $(hostname)

# general packages install
./install-packages.sh -y

# extra packages
apt-get install ifplugd iw wireless-tools wpasupplicant git

# general config
update-alternatives --set editor /bin/nano
if ! id odroid | grep -q adm; then gpasswd -a odroid adm; fi

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
}

# Enable ifplugd support
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

# Enale wpa_supplicant
copy_stdin_to /etc/network/interfaces.d/wlan0 <<EOF

auto wlan0
allow-hotplug wlan0
iface wlan0 inet manual
        wpa-driver wext
        wpa-roam /etc/wpa_supplicant/wpa_supplicant.conf
        # disable power management for better responsivity
        # post-up iwconfig wlan0 power off

iface default inet dhcp

EOF

# disable persisstent net so any model of wlan card appears as wlan0
echo -n | copy_stdin_to /etc/udev/rules.d/75-persistent-net-generator.rules
echo -n | copy_stdin_to /etc/udev/rules.d/70-persistent-net.rules

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

# TODO theamk: read-only root?
# https://github.com/theamk/misc-tools/blob/master/docs/debian-readonly-root.txt
