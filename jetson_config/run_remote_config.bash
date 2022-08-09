#!/bin/bash

ADDRESS=$1
if [ $# -eq 2 ]; then 
    USERNAME=$2
else
    USERNAME="ros"
fi

echo "Connecting to ${1}"

# test the connection
ping $ADDRESS -c 2 > /dev/null
HAS_CONNECTION=$?
# DO NOT SEPARATE THE TWO ABOVE LINES! the ping command sets the value for $?

if [ $HAS_CONNECTION -gt 0 ]; then
    echo "Failed to connect to $ADDRESS"
    echo "Make sure network is setup properly then re-run this script"
    exit
fi

# Determine if we need to configure the sshkey for this user
ls -al ~/.ssh | grep "sshkey_${ADDRESS}" > /dev/null
HAS_SSH_KEY=$?
# DO NOT SEPARATE THE TWO ABOVE LINES! the ls command sets the value for $?

if [ $HAS_SSH_KEY -eq 1 ]; then
    echo "Configuring passwordless login for this user"
    ./relationship.sh $ADDRESS $USERNAME

else
    echo "Passwordless login already configured for this user"

fi


echo "Transferring setup scripts to target"

# create the directory on the remote
ssh $USERNAME@$ADDRESS "mkdir -p ~/osu-uwrt/riptide_setup"

# rsync the config scripts over
rsync -vrzc --delete --exclude="**/.git/" --exclude="**/.vscode/" ~/osu-uwrt/riptide_setup/jetson_config $USERNAME@$ADDRESS:~/osu-uwrt/riptide_setup

echo "Running target installation"