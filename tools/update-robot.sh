#!/bin/bash

if [[ "$1" == "--help" || "$1" == "-h" ]]; then
    cat <<EOF
A script to update/recomplie robot software.

Any arguments are passed to a remote scons.
EOF
    exit 1
fi

set -e

cd $(dirname $(dirname $(readlink -f $0)))

# make sure we can compile our copy
# (set -x; scons -j8 mech/build-x86_64/mech_warfare)

set -x
exec tools/odroid/push-tree.sh scons "$@" mech/build-armv7l/mech_warfare
