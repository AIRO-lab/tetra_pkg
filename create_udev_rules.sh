#!/bin/bash

echo " "
sudo cp 25-name-video-devices.rules  /etc/udev/rules.d
echo " "
echo "Restart udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "Created"
