#!/bin/bash

echo "delete remap the device serial port(ttyUSBX) to  coreboard"
echo "sudo rm   /etc/udev/rules.d/coreboard.rules"
sudo rm   /etc/udev/rules.d/coreboard.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"
