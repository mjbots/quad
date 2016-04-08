#!/bin/bash

set -e
cd $(dirname $(readlink -f "$0"))/../..
echo Source: $PWD

dry_run=""
do_push="yes"

# Use wlan IP unless specified otherwise
# to use wired, prefix with: REMOTE=odroid-mjmech
: ${REMOTE:=10.89.0.10}

while [[ "$#" != 0 ]]; do
    case "$1" in
        --dry-run|-n)
            dry_run="--dry-run"
            shift
            ;;
        --no-push|-P)
            do_push=""
            shift
            ;;
        --host|-H)
            shift
            REMOTE="$1"
            shift
            ;;
        *)
            break
            ;;
    esac
done

if [[ "$1" == "-h" || "$1" == "--help" || "$REMOTE" == "" ]]; then
    cat <<EOF
Usage: $0 [--dry-run|-n] [--no-push|-P] [-H hostname] [remote-command]

Unless --no-push is specitifed, this script transfers code files (the ones not
ignored by git) to a remote host.  This both propagates file deletions and
allows incremental rebuilds to work.

Warning: any existing files will be overriden

A remote-command, if specified, will be executed if push was successfull
EOF
    exit 1
fi

# ping the robot
if ! ping -c 1 -i 0.2 -w 30 $REMOTE; then
	echo could not ping
	exit 1
fi

if [[ "$do_push" == "yes" ]]; then
    echo Pushing code to $REMOTE

    if find . -name '#*' | grep .; then
        echo Warning -- unsaved emacs files found
        echo Will continue in 10 seconds
        sleep 10
    fi

    git describe --dirty --all --always > _version.txt

    (set -x; rsync --exclude-from=.gitignore -vrtlE -e ssh --delete $dry_run \
        --exclude=hw/ --exclude=docs/ --exclude=video-ui/ --exclude=.git \
        --exclude=old/ --exclude=core --exclude=*'*.core' \
        -- . "odroid@$REMOTE:mjmech-clean")
    EC=$?

    rm -f _version.txt
    if [[ $EC != 0 ]]; then
        echo Push failed
        exit $EC
    fi
    echo Push success
fi

if [[ "$1" != "" ]]; then
    (set -x;
        ssh "odroid@$REMOTE" -t cd mjmech-clean '&&' "$*"
    )
fi
