ssh ros@jetson '~/osu-uwrt/riptide_software/src/riptide_utilities/shutdown_scripts/shutdown_on_jetson.sh'
ssh ros@baycat "sudo shutdown -h now"
ssh ros@riptide "sudo shutdown -h now"
