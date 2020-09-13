#!/bin/sh
# In case of green or orange cube download different image and change the filename here

PASSWORD="TjSDBkAu"
FILENAME="ArduCopter.4.0.1.CubeSolo.apj"
TARGET="root@10.1.1.10"

# Accept SSH fingerprint
sshpass -p $PASSWORD ssh -o "StrictHostKeyChecking no" $TARGET

echo "Uploading Frimware $FILENAME"
sshpass -p $PASSWORD scp $FILENAME $TARGET:/firmware

echo "Rebooting the copter"
sshpass -p $PASSWORD ssh $TARGET "reboot"

echo "Restart in progress, the process takes about 45 seconds, you will hear the Cube click and reboot."