#!/bin/bash

# Update /etc/hosts File
# Add appropriate lines to the hosts if they do not exist
if ! grep -Fq "baycat" /etc/hosts; then
    echo -e "192.168.1.19\tbaycat\n$(cat /etc/hosts)" > /etc/hosts
fi

if ! grep -Fq "venus" /etc/hosts; then
    echo -e "192.168.1.21\tvenus\n$(cat /etc/hosts)" > /etc/hosts
fi

if ! grep -Fq "jetson" /etc/hosts; then
    echo -e "192.168.1.69\tjetson\n$(cat /etc/hosts)" > /etc/hosts
fi

if ! grep -Fq "uwrt-dvl" /etc/hosts; then
    echo -e "192.168.1.212\tuwrt-dvl\n$(cat /etc/hosts)" > /etc/hosts
fi