#ssh ros@baycat 'rm -rf ~/osu-uwrt/riptide_software/src'

~/osu-uwrt/riptide_software/src/riptide_utilities/dates_scripts/date_set-baycat.sh

rsync -vrzc --delete --exclude=".*" --exclude=".*/" ~/osu-uwrt/riptide_software/src ros@baycat:~/osu-uwrt/riptide_software
ssh ros@baycat 'chmod 700 ~/osu-uwrt/riptide_software/src/riptide_utilities/*.sh'
ssh ros@baycat 'find ~/osu-uwrt/riptide_software/src/ -type f -iname "*.py" -exec chmod +x {} \;'
