#!/usr/bin/bash

#
# UWRT Matlab Install Script
# usage: ./install_matlab.bash
# this script expects to be executed in its own directory. DONT call it from another directory

echo
echo "[INFO] Installing Matlab"
echo

MATLAB_INSTALL_LOCATION=~/osu-uwrt/matlab/MATLAB
mkdir -p $MATLAB_INSTALL_LOCATION #create target folder

#need to ask use to download the installer because we probably shouldnt host it on github 
#because mathworks wouldnt be too happy about that. So we'll do that here
download_fail=1
while [ $download_fail -eq 1 ]
do
    echo
    echo
    echo "Checksum of MATLAB Installer could not be verified. Please download the MATLAB installer!"
    echo "You can find the UWRT offline installer on Teams in Software -> Files -> MATLAB -> uwrt_matlab_installer.zip."
    echo
    echo "Please download the file to your \"Downloads\" folder, then press enter to continue:"
    read

    installer_name=uwrt_matlab_installer.zip

    if [ -f "$HOME/Downloads/$installer_name" ]
    then
        #found file, check its sum. if the sum is good then while will break
        echo "Found installer, checking checksum"
        echo "$(cat ./uwrt_matlab_installer.zip.sha256) $HOME/Downloads/$installer_name" | sha256sum --check --status
        download_fail=$? #download fail will be set to the result of the previous command
    else
        echo "Installer not found in Downloads directory. Perhaps it has a different name?"
        echo "Please enter the name of the installer in the Downloads directory (currently $installer_name)"
        read installer_name
    fi
done

#extract installer
echo "Found MATLAB installer at $HOME/Downloads/$installer_name"
cp  $MATLAB_INSTALL_LOCATION
echo "Extracting installer"
unzip -q ~/Downloads/$installer_name -d $MATLAB_INSTALL_LOCATION

#run installer
#TODO: document the process: ui/install/product_installer_ui/bundle.index.js
echo "Running installer"
$MATLAB_INSTALL_LOCATION/uwrt_matlab_installer/install

#remove installer
echo "Removing installer"
rm -rf $MATLAB_INSTALL_LOCATION/uwrt_matlab_installer

#post-installation tasks (install python, configure matlab)
./install_python39.bash $MATLAB_INSTALL_LOCATION
./configure_matlab.bash $MATLAB_INSTALL_LOCATION /usr/local/bin/python3.9
