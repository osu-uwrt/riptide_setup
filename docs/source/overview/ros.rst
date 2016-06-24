Robot Operating System
======================

The team has elected to build most of its software using Robot Operating System (ROS), a modular framework for collaborate robotic software development. The modularity of ROS eases efforts to break the software system into manageable projects throughout the school year and then integrate them into the full system as they are completed. The collaborative aspect of ROS encourages many researchers and corporations to provide open source modules (called packages in ROS) which implement a variety of algorithms and device drivers. By taking advantage of this pre-existing software, the team saves significant development time and gains the ability to learn from the work of experts. The team has created a number of its own packages to operate its AUV. The most significant packages are Autonomy, Estimation, Navigation and Vision, as seen in Fig 16.

.. image:: /_static/package_stack.png

The Autonomy package runs the mission by creating a sequence of desired states throughout the competition. The Estimation package continuously analyzes feedback to determine the vehicles current state. To travel throughout the course, the Navigation package resolves discrepancies between the desired and current states to create thruster commands. The Vision package handles the vehicle’s primary sensors, the forward-facing stereo camera system and the downward-facing monocular camera system.

A number of open source packages were used in conjunction with those created by the team, such as the imu_3dm_gx4 packages [1].

A number of smaller packages were also created to support those listed above. A serial interface package enables communication with the team’s custom electronics system and, by extension, the remaining sensors and actuators. A Teleoperation package was created for initial control testing and vehicle demonstration. The Description package provides a vehicle model which can be accessed by other software to determine the vehicles properties when making dynamic calculations.
