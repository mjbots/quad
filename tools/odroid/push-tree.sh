#!/bin/bash

set -e
cd $(dirname $(readlink -f "$0"))/../..
echo Source: $PWD

dry_run=""
if [[ "$1" == "--dry-run" || "$1" == "-n" ]]; then
    dry_run="--dry-run"
    shift
fi

if [[ "$1" == "" || "$1" == "-h" ]]; then
    cat <<EOF
Usage: $0 [--dry-run] hostname [remote-command]

This script transfers code files (the ones not ignored by git) to a remote host.
This both propagates file deletions and allows incremental rebuilds to work.

Warning: any existing files will be overriden

A remote-command, if specified, will be executed if push was successfull
EOF
    exit 1
fi

if find . -name '#*' | grep .; then
    echo Warning -- unsaved emacs files found
    echo Will continue in 10 seconds
    sleep 10
fi

git describe --dirty --all --always > _version.txt

host="$1"
shift

(set -x; rsync --exclude-from=.gitignore -vrtlE -e ssh --delete $dry_run \
    --exclude=hw/ --exclude=docs/ --exclude=video-ui/ --exclude=.git \
    --exclude=old/ --exclude=core --exclude=*'*.core' \
     -- . "odroid@$host:mjmech-clean")
EC=$?

rm -f _version.txt
if [[ $EC != 0 ]]; then
    echo Push failed
    exit $EC
fi
echo Push success
if [[ "$1" != "" ]]; then
    (set -x;
        ssh "odroid@$host" -t cd mjmech-clean '&&' "$*"
    )
fi
