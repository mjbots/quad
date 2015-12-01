#!/bin/bash
set -e
if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    cat <<EOF
This script detects supporterd wlan cards, and if one is found,
creates a new namespace and moves it into that namespace.

It will create namespace 'robotlink', and will create interface
'wlan_r' there.

It is useful on desktops, to ensure the network manager does not mess
with the card.
EOF
    exit 1
fi

# delete namespace, if found
# (this needs to be done before device scan)
if test -e /var/run/netns/robotlink; then
    echo Namespace found, deleting
    ip netns del robotlink
    sleep 1
fi

# detect phy of device in question
main_phy_name=""
for phy_path in /sys/class/ieee80211/phy*; do
    dev_path="$(readlink -f $phy_path/device)"
    usb_id="$(paste -d : $dev_path/../{idVendor,idProduct} 2>/dev/null)" || true
    phy_name="$(basename $phy_path)"
    if [[ "$usb_id" == 148f:3572 ]]; then
        echo Found wireless phy $phy_name: ALFA with ralink chipset
        main_phy_name="$phy_name"
    else
        echo "Ignoring wireless phy $phy_name: usb_id [$usb_id]"
    fi
done

if [[ "$main_phy_name" == "" ]]; then
    echo Wireless adapter not found >&2
    exit 1
fi
echo

in_netns() { ip netns exec robotlink "$@"; }

# Create our namespace
ip netns add robotlink

# setup lo inside
in_netns ip addr add 127.0.0.1/8 scope host dev lo
in_netns ip link set dev lo up

# setup link to outside -- this way, we will not need root access to
# send/receive packets.
# TODO theamk

# Move our phy to namespace.
# 'iw' needs namespace PID, so do this little trick:
read ns_pid < <(in_netns sh -c 'echo $$ && sleep 2')
iw phy $main_phy_name set netns $ns_pid

# unfortunately, the command below does not work..
#in_netns iw phy $main_phy_name set name phy99
