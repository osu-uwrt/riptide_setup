#ssh ros@riptide 'rm -rf ~/osu-uwrt/riptide_software/src'

~/osu-uwrt/riptide_software/src/riptide_utilities/dates_scripts/date_set-riptide.sh

rsync -vrzc --delete --exclude=".*" --exclude=".*/" ~/osu-uwrt/riptide_software/src ros@riptide:~/osu-uwrt/riptide_software
ssh ros@riptide 'chmod 700 ~/osu-uwrt/riptide_software/src/riptide_utilities/*.sh'
ssh ros@riptide 'find ~/osu-uwrt/riptide_software/src/ -type f -iname "*.py" -exec chmod +x {} \;'
