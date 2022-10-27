~/osu-uwrt/riptide_setup/scripts/jetson_config/config_target/date_set.bash $1

scp ~/osu-uwrt/riptide_setup/scripts/ros_scripts/build_remote.sh ros@${1}:~/osu-uwrt

echo "xfering robot code"
rsync -vrzc --delete --exclude="**/.git/" --exclude="riptide_gazebo/" --exclude="**/.vscode/" --exclude="riptide_rqt_plugins" --exclude="riptide_vision/weights/" ~/osu-uwrt/riptide_software/src ros@$1:~/osu-uwrt/riptide_software

if [ $# -eq 2 ]; then 
    if [ $2 == "--deps" ]; then 
        echo "xfering dependencies"

        rsync -vrzc --delete --exclude="**/.git/" --exclude="riptide_gazebo/" --exclude="**/.vscode/" --exclude="riptide_rqt_plugins" --exclude="riptide_vision/weights/" ~/osu-uwrt/dependencies/src ros@$1:~/osu-uwrt/dependencies

        echo "Running a build on dependencies"
        ssh ros@${1} 'cd ~/osu-uwrt && ./build_remote.sh ./dependencies'
    fi
    if [ $2 == "--build" ]; then
        echo "Running a build on riptide_software"
        ssh ros@${1} 'cd ~/osu-uwrt && ./build_remote.sh ./riptide_software'
    fi
fi
