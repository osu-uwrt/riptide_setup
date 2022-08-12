# the goal of this script is to create, install and configure service units for auto starting our code

# template ROS Service
'''
[Unit]
Description=Service to bring up a ros network 
After=network.target

[Service]
Type=simple
ExecStart=/bin/bash /home/coalman321/capstone_ws/deployer/remote_computer/jetson_entrypoint.sh
StandardOutput=inherit
StandardError=inherit
Restart=on-failure
User=coalman321


[Install]
WantedBy=multi-user.target
'''