#!/bin/sh

#########################################
### Assign symlinks to serial devices ###
#########################################

sudo echo 'ACTION=="add", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", GROUP:="dialout", SYMLINK+="arduino"
ACTION=="add", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="lidar"' > ./99-local.rules
sudo mv 99-local.rules /etc/udev/rules.d

reboot