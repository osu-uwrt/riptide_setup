# Setup Acoustics
tar -zxf ~/osu-uwrt/riptide_software/src/riptide_hardware/resources/acoustics/FrontPanel-Ubuntu16.04LTS-x64-5.0.1.tgz
mv FrontPanel-Ubuntu16.04LTS-x64-5.0.1 ~
cd ~/FrontPanel-Ubuntu16.04LTS-x64-5.0.1
chmod +x install.sh
sudo install.sh
sudo cp API/libokFrontPanel.so /usr/lib/
rm -rf ~/FrontPanel-Ubuntu16.04LTS-x64-5.0.1

echo "Acoustics dependencies setup\n"