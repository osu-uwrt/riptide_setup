# Clone riptide_setup into correct directory
git clone https://github.com/osu-uwrt/riptide_gazebo ~/osu-uwrt/riptide_software/src/riptide_gazebo

# Run rosdep install again to make sure we have required packages (otherwise we usually get an error about missing uuv_descriptions)
./install_rosdeps.sh

# catkin build needs built from inside the catkin workspace
cd ~/osu-uwrt/riptide_software

# Build (riptide_gazebo should be the only one that needs builded if setup.sh was just run)
catkin build