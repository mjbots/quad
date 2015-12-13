#!/usr/bin/python
"""
This script sets up the radio. It will automatically detect the device to work
on.

Basic commands:
  up down info

Other commands:
  reinit mon scan

Sample: monitor traffic
 sudo ./tools/setup_wifi_link.py mon
 sudo tcpdump -i mon_r -n -l -vv
 sudo horst -i mon_r      # 'Stats' shows bitrates
"""

import sys
import os
import optparse
import errno
import glob
import subprocess
import re

#FREQ=2452   # channel 9
FREQ=5200   # channel 40
#FREQ=5765   # channel 153

SSID  = "MjmechTelemetry"
BSSID = "00:C6:0B:F0:F0:F0"
RATE  = 24    # Mbps, 6/9/12/18/24/36/48/54
# yes, we can only do WEP or client-side. WPA has too much state.
#WEP_KEY=

# Increase beacon interval. This may be a bad idea if the other side needs
# beacons, however as we are hardcoding our configuration on both ends we
# should not need beacons at all.
BEACON_INTERVAL = 2000

MCAST_NET = '239.89.108.0/24'
IP_PREFIX = '10.89'
# Give well-known devices short IP addresses
FIXED_IP_MAP = {
    '36:e6:6a:0e:97:b1': '10.89.0.10',  # mjmech-odroid
}

class WifiSetup(object):
    def __init__(self, opts):
        self._opts = opts
        self._verbose = opts.verbose

        self.phyname = None
        self.ifname = None
        self.os_driver = None

        self._find_interface()

    def debug(self, level, msg):
        if level <= self._verbose:
            print '-%s %s' % ('-' * level, msg)

    def _linkbase(self, name):
        try:
            rv = os.path.basename(os.readlink(name))
        except OSError as e:
            if e.errno != errno.ENOENT:
                raise
            rv = None
        self.debug(4, 'Linkbase %r => %r' % (name, rv))
        return rv

    def _find_interface(self, keep_phyname=False):
        # (ifname, phyname, os_driver) tuples
        candidates = list()

        self.debug(1, 'Looking for wifi cards')
        # First check for new-style extensions (nl80211)
        for phydir in sorted(glob.glob('/sys/class/ieee80211/*')):
            driver = self._linkbase(phydir + '/device/driver')
            subsystem = self._linkbase(phydir + '/device/subsystem')
            phyname = os.path.basename(phydir)
            self.debug(2, 'nl80211 phy %r has driver %r, subsystem %r' %
                       (phyname, driver, subsystem))
            if subsystem != 'usb':
                self.debug(2, ' skipping this phy -- not usb based')
                continue
            # Any 802.11nl USB card should work
            candidates.append((None, phyname, driver))

        # check for legacy network cards (wext)
        wired_ifs = list()
        for ifdir in sorted(glob.glob('/sys/class/net/*')):
            ifname = os.path.basename(ifdir)
            if not os.path.exists(ifdir + '/wireless'):
                wired_ifs.append(ifname)
                continue

            phybase = self._linkbase(ifdir + '/phy80211')
            driver = self._linkbase(ifdir + '/device/driver')
            subsystem = self._linkbase(ifdir + '/device/subsystem')

            self.debug(2, 'wireless if %r has driver %r, subsystem %r' %
                       (ifname, driver, subsystem))

            if subsystem != 'usb':
                self.debug(2, ' skipping this interface -- not usb')
                continue

            if ifname.startswith('mon'):
                self.debug(2, ' skipping this interface -- looks monitorish')
                continue

            if (None, phybase, driver) in candidates:
                self.debug(2, ' this will be a primary if for phy %r' %
                           (phybase))
                candidates.remove((None, phybase, driver))

            candidates.append((ifname, phybase, driver))

        self.debug(2, 'skipped wired interfaces %r' % (wired_ifs, ))

        if len(candidates) == 0:
            raise Exception('no wireless card found')

        if len(candidates) > 1:
            raise Exception('too many wireless cards found: %r' % (
                candidates, ))
            # TODO theamk handle opts.interface

        self.debug(1, 'Choosing interface %r' % (candidates[0], ))

        old_phyname = self.phyname
        self.ifname, self.phyname, self.os_driver = candidates[0]
        if keep_phyname:
            assert self.phyname == old_phyname, \
                'reselected a different why, this was not supposed to happen'


    def print_info(self):
        print 'WL_IFNAME=%s' % self.ifname
        print 'WL_PHYNAME=%s' % (self.phyname or '')
        print 'WL_DRIVER=%s' % (self.os_driver)
        if_ip = self._desired_station_ip()
        print 'WL_IP=%s' % if_ip

    def _nl_get_interface_type(self):
        out = self._check_output('iw dev %s info' % self.ifname)
        assert out.startswith('Interface %s\n' % self.ifname)
        mm = re.search('^\ttype ([a-zA-Z]+)$', out, re.MULTILINE)
        assert mm, 'invalid info output: %r' % (out, )
        result = mm.group(1)
        self.debug(2, 'Interface %r has type %r' % (self.ifname, result))
        return result

    def bring_interface_up(self, force_reinit=False):
        if self.phyname is None:
            self._bring_interface_up_legacy(force_reinit=force_reinit)
            return

        # Destroy old interfaces one by one until we see ibss one.
        count = 0
        while self.ifname is not None:
            itype = self._nl_get_interface_type()
            if itype == 'IBSS' and not force_reinit:
                break
            force_reinit = False
            self.debug(0, 'Destroying interface %r because it is of '
                       'wrong type %r' % (self.ifname, itype))
            self._exec('ip link set dev %s down' % self.ifname)
            self._exec('iw dev %s del' % self.ifname)
            self._find_interface(keep_phyname=True)

            count += 1
            assert count <= 10

        if self.ifname is None:
            # We have a bare phy. Create a new interface.
            IFNAME = 'wlan_r'
            self.debug(0, 'Creating new interface %r on %r' % (
                IFNAME, self.phyname))
            self._exec('iw phy %s interface add %s type ibss' % (
                self.phyname, IFNAME))
            self._find_interface(keep_phyname=True)

            assert self.ifname is not None
            assert self._nl_get_interface_type() == 'IBSS'

        reg = self._check_output('iw reg get')
        if not reg.startswith('country US'):
            self.debug(0, 'setting country to US')
            self._exec('iw reg set US')

        for rfkpath in glob.glob(
                '/sys/class/ieee80211/%s/rfkill*'% self.phyname):
            rfkabs = os.path.realpath(rfkpath)
            with open(rfkabs + '/state', 'r') as f:
                state = f.read(1024).strip()
            if state != '1':
                self.debug(0, 'Disabling rfkill %r' % os.path.basename(rfkabs))
                self._write_to_file(rfkabs + '/state', '1')

        self.debug(2, 'Configuring ibss link')

        #iw dev $devname set type ibss

        self.bring_interface_down()

        # bring it up before we can join
        self._exec('ip link set dev %s up' % self.ifname)

        # join ibss
        cmd = ['iw', 'dev', self.ifname, 'ibss', 'join', SSID, str(FREQ),
               'HT20', 'fixed-freq', BSSID]
        cmd += ['beacon-interval', str(BEACON_INTERVAL)]
        cmd += ['basic-rates', str(RATE), 'mcast-rate', str(RATE)]
        self._exec(cmd)

        # Do not crash if power saving is not supported
        self._exec('iw dev %s set power_save off' % self.ifname,
                   ok_codes=[161])

        # set ip
        self._setup_ip_addressing()
        self.debug(0, 'success')

    def _bring_interface_up_legacy(self, force_reinit=False):
        # the commands below were tested for rtl8812au driver only
        self._exec('ip link set dev %s up' % self.ifname)
        self._exec('iwconfig %s mode ad-hoc' % self.ifname)
        cmd = ['iwconfig', self.ifname, "essid", SSID, "freq", str(FREQ * 1000),
               'rate', '%.1fM' % RATE]
        self._exec(cmd)
        self._exec('iwconfig %s ap %s' % (self.ifname, BSSID))

        # if self.os_driver == 'rtl8812au':
        #     # I am not sure what extactly 'mcast2uni' does, but it breaks
        #     # multicast TX when we are in AP mode.
        #     for param, val in (('rtw_mc2u_disable', 1), ):
        #         self._write_to_file('/sys/module/8812au/parameters/' + param,
        #                             str(val))

        #self._exec('iwconfig %s power off' % self.ifname)
        #self._exec('iwconfig %s commit' % self.ifname)
        self._setup_ip_addressing()


    def bring_interface_down(self):
        link_str = self._check_output(['iw', 'dev', self.ifname, 'link'])
        if link_str == 'Not connected.\n':
            self.debug(2, 'no ibss link')
        else:
            self.debug(1, 'Breaking ibss link %r' % (link_str.split('\n')))
            self._exec('iw dev %s ibss leave' % self.ifname)

        self._exec('ip link set dev %s down' % self.ifname)
        self.debug(1, 'Interface %r is now down' % self.ifname)

    def add_mon_interface(self):
        if self.phyname is None:
            raise NotImplementedError()
        self._exec('iw phy %s interface add mon_r type monitor'
                   % self.phyname)
        #flags fcsfail,control,otherbss
        self._exec('ip link set dev mon_r up')

    def _desired_station_ip(self):
        # Make IP from last 2 digits of MAC of eth0 interface (this way,
        # changing wifi card will leav IP alone)
        with open('/sys/class/net/eth0/address', 'r') as f:
            base_mac = f.read().strip()
        self.debug(2, 'Calculating station IP for MAC %r' % base_mac)

        if_ip = FIXED_IP_MAP.get(base_mac)
        if if_ip:
            return if_ip

        base_mac_int = [int(x, 16) for x in base_mac.split(':')]
        if_ip = IP_PREFIX + '.%d.%d' % (base_mac_int[-2], base_mac_int[-1])
        return if_ip

    def _setup_ip_addressing(self):

        if_ip = self._desired_station_ip()
        self.debug(0, 'Setting ip address to %s' % if_ip)

        self._exec('ip addr flush dev %s' % (self.ifname))
        self._exec('ip addr replace %s/16 dev %s' % (if_ip, self.ifname))
        self._exec('ip route add %s dev %s' % (MCAST_NET, self.ifname))
        iv6f = '/proc/sys/net/ipv6/conf/%s/disable_ipv6' % self.ifname
        if os.path.exists(iv6f):
            self._write_to_file(iv6f, '1')
        else:
            self.debug(2, 'Looks like this interface has no IPv6')

    def _exec(self, cmd, ok_codes=None):
        self.debug(2, 'Executing: %r' % (cmd, ))
        if self._opts.dry_run:
            print 'DRY-RUN: exec %r' % (cmd, )
            return

        try:
            subprocess.check_call(cmd, shell=isinstance(cmd, str))
        except subprocess.CalledProcessError as e:
            if ok_codes and e.returncode in ok_codes:
                self.debug(2, 'command returned %r, but this is ok' % (
                    e.returncode))
                return
            raise

    def _check_output(self, cmd):
        self.debug(2, 'Executing for results: %r' % (cmd, ))
        return subprocess.check_output(
            cmd, shell=isinstance(cmd, str))

    def _write_to_file(self, fname, data):
        self.debug(2, 'Writing to %r: %r' % (fname, data))
        if self._opts.dry_run:
            print 'DRY-RUN: echo %r > %s' % (data, fname)
            return
        with open(fname, 'w') as f:
            print >>f, data

def main():
    parser = optparse.OptionParser(usage='%prog command',
                                   description=__doc__)
    parser.format_description = lambda _: parser.description.lstrip()
    parser.add_option('-i', '--interface',
                      help='force wifi interface/phy to work on')
    parser.add_option('-v', '--verbose', action='count', default=0,
                      help='Print more info')
    parser.add_option('-n', '--dry-run', action='store_true',
                      help='Do not actually change anything')
    opts, args = parser.parse_args()

    if len(args) == 0:
        parser.error('command missing')

    ws = WifiSetup(opts)
    if args[0] == 'info':
        ws.print_info()
    elif args[0] == 'up-boot':
        # TODO theamk enable
        pass
    elif args[0] == 'up':
        ws.bring_interface_up()
    elif args[0] == 'reinit':
        ws.bring_interface_up(force_reinit=True)
    elif args[0] == 'down':
        ws.bring_interface_down()
    elif args[0] == 'mon':
        ws.add_mon_interface()
    else:
        parser.error('invalid command')

if __name__ == '__main__':
    sys.exit(main())

"""
    scan)
        maybe_create_interfaces
        (set -x;
            iw dev wlan_r scan
        )
"""
