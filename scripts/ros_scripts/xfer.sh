~/osu-uwrt/riptide_setup/scripts/dates_scripts/date_set.sh $1

rsync -vrzc --delete --exclude="**/.git/" --exclude="riptide_gazebo/" --exclude="riptide_vision/weights/" ~/osu-uwrt/riptide_software/src ros@$1:~/osu-uwrt/riptide_software
ssh ros@$1 'find ~/osu-uwrt/riptide_software/src/ -type f -iname "*.py" -exec chmod +x {} \;'
