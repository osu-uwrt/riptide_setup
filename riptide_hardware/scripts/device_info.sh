#!/bin/bash

if [[ -z $1 ]]; then
    echo "You must specify the device path as a command-line argument. Example:"
    echo "./device_info.sh /dev/ttyACM0"
    exit 1
fi

udevadm info -a -p $(udevadm info -q path -n $1)