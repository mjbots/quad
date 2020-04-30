#!/usr/bin/python3 -B

# Copyright 2018-2020 Josh Pieper.  All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Sets up a raspberry pi 3b+ for use as an mjmech computer.

The base operating system should already be installed.  Either a
vanilla raspbian can be used, or the PREEMPT_RT variant from
https://github.com/guysoft/RealtimePi is possible.

It is intended to be run as root, like:

sudo ./setup-system.py

"""

import os
import shlex
import shutil
import subprocess
import time

ORIG_SUFFIX = time.strftime(".orig-%Y%m%d-%H%M%S")


def run(*args, **kwargs):
    print('run: ' + args[0])
    subprocess.check_call(*args, shell=True, **kwargs)


def ensure_keyword_present(filename, keyword, value):
    '''Ensure the given keyword is present in a one-line file'''

    current_content = [
        x.strip() for x in open(filename, encoding='utf-8').readlines()]

    assert len(current_content) == 1


    new_item = "{}={}".format(keyword, value)

    items = [x.strip() for x in current_content[0].split(' ')]

    present = [x for x in items if x == keyword or x.startswith(keyword + '=')]
    if len(present):
        new_items = [x if not (x == keyword or x.startswith(keyword + '=')) else
                     new_item for x in items]
    else:
        new_items = items + [new_item]

    if new_items == items:
        return

    print('ensure_keyword_present({}): Adding: {}={}'.format(
        filename, keyword, value))

    with open(filename, 'w', encoding='utf-8') as f:
        f.write(' '.join(new_items) + '\n')


def ensure_present(filename, line):
    '''Ensure the given line is present in the named file.'''

    current_content = [
        x.strip() for x in open(filename, encoding='utf-8').readlines()]
    if line.strip() in current_content:
        # Yes, the line is already present there
        return

    shutil.copy(filename, filename + ORIG_SUFFIX)

    print('ensure_present({}): Adding: {}'.format(filename, line))

    # Nope, we need to add it.
    with open(filename, 'a', encoding='utf-8') as f:
        f.write(line + '\n')


def ensure_contents(filename, contents):
    '''Ensure the given file has exactly the given contents'''

    if os.path.exists(filename):
        existing = open(filename, encoding='utf-8').read()
        if existing == contents:
            return

        shutil.copy(filename, filename + ORIG_SUFFIX)

    print('ensure_contents({}): Updating'.format(filename))

    with open(filename, 'w', encoding='utf-8') as f:
        f.write(contents)


def set_config_var(name, value):
    '''Set the given variable in /boot/config.txt'''
    contents = open('/boot/config.txt', encoding='utf-8').readlines()

    new_value = '{}={}'.format(name, value)

    maybe_value = [x for x in contents
                   if x.startswith('{}='.format(name))]
    if len(maybe_value) == 1 and maybe_value[0].strip() == new_value:
        return

    new_contents = ([
        x for x in contents
        if not x.startswith('{}='.format(name))] +
        [new_value + '\n'])

    shutil.copy('/boot/config.txt', '/boot/config.txt' + ORIG_SUFFIX)

    print('set_config_var({})={}'.format(name, value))

    open('/boot/config.txt', 'w', encoding='utf-8').write(
        ''.join([x for x in new_contents]))


def main():
    if os.getuid() != 0:
        raise RuntimeError('must be run as root')

    # Some useful utilities
    run('apt install --yes socat setserial screen')

    # Things necessary to be an AP
    run('apt install --yes hostapd dnsmasq')

    # P1 Camera - Yes
    run('raspi-config nonint do_camera 0')

    # P2 SSH - Yes
    run('raspi-config nonint do_ssh 0')

    # P6 Serial
    #  Login shell - No
    #  Serial enabled - Yes
    #
    # NOTE: The version of raspi-config we have now doesn't support
    # enabling the UART from noninteractive mode.
    run('raspi-config nonint do_serial 1')

    # This we have to manually enable the UART once it is done.
    set_config_var('enable_uart', '1')

    # We have US keyboards!
    ensure_contents('/etc/default/keyboard', '''
XKBMODEL="pc105"
XKBLAYOUT="us"
XKBVARIANT=""
XKBOPTIONS=""
BACKSPACE="guess"
''')


    # Switch to use the PL011 UART
    #  https://www.raspberrypi.org/documentation/configuration/uart.md
    ensure_present('/boot/config.txt', 'dtoverlay=pi3-disable-bt')

    ensure_keyword_present('/boot/cmdline.txt', 'isolcpus', '1,2,3')

    ensure_contents('/etc/network/interfaces',
                    '''
 # interfaces(5) file used by ifup(8) and ifdown(8)

# Please note that this file is written to be used with dhcpcd
# For static IP, consult /etc/dhcpcd.conf and 'man dhcpcd.conf'

# Include files from /etc/network/interfaces.d:
source-directory /etc/network/interfaces.d

auto eth0
iface eth0 inet static
    address 192.168.17.47
    netmask 255.255.255.0
    network 192.168.17.0
    broadcast 192.168.17.255
    # gateway 192.168.17.1
    dns-nameservers 8.8.8.8 8.8.4.4
    post-up ip route add 239.89.108.0/24 dev eth0

allow-hotplug wlan0
iface wlan0 inet static
    address 192.168.16.47
    netmask 255.255.255.0
    network 192.168.16.0
    broadcast 192.168.16.255
    post-up iw dev wlan0 set power_save off || true
    post-up iw dev wlan0 set retry long 1 || true
    post-up ip route add 239.89.108.0/24 dev wlan0
''')

    MAC = subprocess.check_output(
        "iw dev wlan0 info | grep addr | perl -pe 's/.*addr //; s/://g'",
        shell=True).decode('utf-8').strip()

    ensure_contents('/etc/hostapd/hostapd.conf',
                    '''
country_code=US

interface=wlan0
driver=nl80211
ssid=MjMech-{MAC}
hw_mode=a
ieee80211n=1
require_ht=1
ieee80211ac=1
require_vht=1
ieee80211d=1
ieee80211h=0

ht_capab=[HT40+][SHORT-GI-20][DSSS_CK-40][MAX-AMSDU-3839]
vht_capab=[MAX-MDPU-3895][SHORT-GI-80][SU-BEAMFORMEE]

vht_oper_chwidth=1
channel=36
vht_oper_centr_freq_seg0_idx=42

wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0

wpa=2
wpa_passphrase=WalkingRobots
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
'''.format(MAC=MAC))

    ensure_present('/etc/default/hostapd',
                   'DAEMON_CONF="/etc/hostapd/hostapd.conf"')

    ensure_contents('/etc/dnsmasq.conf',
                    '''
interface=wlan0
dhcp-range=192.168.16.100,192.168.16.150,255.255.255.0,24h
''')

    ensure_present('/etc/dhcpcd.conf', 'denyinterfaces wlan0')

    run('systemctl unmask hostapd')
    run('systemctl enable hostapd')
    run('systemctl start hostapd')


if __name__ == '__main__':
    main()
