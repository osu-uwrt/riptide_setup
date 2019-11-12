---
description: How to setup the jetson
---

# jetson\_ssd\_readme

Mount SSD on startup:

'sudo mount /dev/nvme0n1 /home/ros/ssd' added to /etc/rc.local

Remap ros user home directory to /home/ros/ssd:

'/home/ros' changed to '/home/ros/ssd' in /etc/passwd

