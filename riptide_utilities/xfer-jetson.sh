#ssh ros@riptide 'rm -rf ~/osu-uwrt/riptide_software/src'

dates_scripts/date_set-jetson.sh

rsync -tvrz --exclude=".*" --exclude=".*/" ~/osu-uwrt/riptide_software/src ros@jetson:~/osu-uwrt/riptide_software
ssh ros@jetson 'chmod 700 ~/osu-uwrt/riptide_software/src/riptide_utilities/*.sh'
