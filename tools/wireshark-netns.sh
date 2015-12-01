#!/bin/bash

# run wireshark in a network namespace, as non-root user
# we override dumpcap binary for that
# unfortunately, it is hardcoded to be in the same dir as wireshark

set -e

if [[ "$whatever" != "1" ]]; then
    echo THIS SCRIPT IS BROKEN
    exit 1
fi

if [[ `id -u` == 0 ]]; then
    echo Error: do not run this as root
    exit 1
fi

orig_shark=/usr/bin/wireshark
orig_dumpcap=/usr/bin/dumpcap
$orig_shark --version > >(head -n 1)
(set -x;
    sudo $orig_dumpcap -v
) > >(head -n 1)

work_dir=/tmp/wirshark-netns
new_shark=$work_dir/wireshark

mkdir -p $work_dir
ln -fs $orig_shark $new_shark
cat >$work_dir/dumpcap <<EOF
#!/bin/sh
umask 000
exec sudo ip netns exec robotlink $orig_dumpcap -g "\$@"
EOF
chmod a+x $work_dir/dumpcap

exec $new_shark "$@"
