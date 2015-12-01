#!/bin/bash
# A script to set up the radio. should be run as root, in namespace.
set -e

# We are supposed to be in a namespace, so expect exactly
# one phy.
phyname="$(cd /sys/class/ieee80211 && ls -d phy*)"
if ! test -f "/sys/class/ieee80211/$phyname/index"; then
    # both empty string and two space-separated args will fail.
    echo "Fatal: need exactly 1 wifi phy, have [$phyname]"
    exit 1
fi

cleanup_all() {
    # tear down everything
    local devname
    for devname in $(cd /sys/class/ieee80211/$phyname/device/net && ls); do
        (set -x;
            ip link set dev $devname down;
            iw dev $devname del;
        )
    done
    echo Cleanup complete
}

create_interfaces() {
    # general setup, create ibss interface
    (set -x;
        iw reg set US
        echo 1 | tee /sys/class/ieee80211/phy*/rfkill*/state >/dev/null
        iw phy $phyname interface add wlan_r type ibss
        ip link set dev wlan_r up
    )
}


maybe_create_interfaces() {
    if [[ "$(basename $(readlink -f /sys/class/net/wlan_r/phy80211) 2>/dev/null)" \
        == "$phyname" ]]; then
        echo Looks like wifi interface is good
    else
        cleanup_all
        create_interfaces
    fi
}

join_ibss() {
    local FREQ SSID BSSID RATE IPADDR
    local devname=wlan_r

    if ip link show $devname | grep -q 'UP'; then
        # We are already connected to something or trying to
        (set -x;
            ! iw dev $devname ibss leave
            ip link set dev $devname down
        )
    fi

    # Make IP from last 2 digits of MAC
    IPADDR=10.89.$(perl -lne \
        'm/(..):(..)$/ && print(hex $1, ".", hex $2)' \
        /sys/class/net/$devname/address)

    #FREQ=2452   # channel 9
    #FREQ=5200   # channel 40
    FREQ=5765   # channel 153

    SSID=MjmechTelemetry
    BSSID=00:C6:0B:F0:F0:F0
    RATE=24    # Mbps, 6/9/12/18/24/36/48/54
    # yes,.
    #WEP_KEY=
    (set -x;
        iw dev $devname set type ibss
        ip link set dev $devname up
        iw dev $devname ibss join "$SSID" "$FREQ" HT20 fixed-freq $BSSID \
            basic-rates $RATE mcast-rate $RATE
        ip addr change ${IPADDR}/16 dev $devname
        ip route add 239.89.0.0/16 dev $devname
    )
}

case "$1" in
    down)
        cleanup_all
        ;;
    reload)
        cleanup_all
        create_interfaces
        join_ibss
        ;;
    up)
        maybe_create_interfaces
        join_ibss
        ;;
    mon)
        maybe_create_interfaces
        (set -x;
            iw phy $phyname interface add mon_r type monitor
            #flags fcsfail,control,otherbss
            ip link set dev mon_r up
        )
        ;;
    scan)
        maybe_create_interfaces
        (set -x;
            iw dev wlan_r scan
        )
        ;;
    *)
        cat <<EOF
Invalid command. Commands:
  down
  up
  mon
  reload

Sample: set up link
 sudo ./tools/move_wifi_to_namespace.sh
 sudo ip netns exec robotlink ./tools/setup_wifi_link.sh up

Sample: monitor traffic
 sudo ip netns exec robotlink ./tools/setup_wifi_link.sh mon
 sudo ip netns exec robotlink tcpdump -i mon_r -n -l -vv

Sample: describe device
 sudo ip netns exec robotlink iw list

EOF
        exit 1
        ;;
esac
