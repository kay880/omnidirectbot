#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  coreboard"
echo "rplidar usb connection as /dev/coreboard , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy coreboard.rules to  /etc/udev/rules.d/"
echo "`rospack find omnidirectbot`/scripts/coreboard.rules"
sudo cp `rospack find omnidirectbot`/scripts/coreboard.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
sudo udevadm control --reload && sudo udevadm trigger
echo "finish "
