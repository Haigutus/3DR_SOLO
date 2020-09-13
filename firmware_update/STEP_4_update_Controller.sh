PASSWORD="TjSDBkAu"
FILENAME_1="3dr-controller.tar.gz"
FILENAME_2="3dr-controller.tar.gz.md5"
TARGET="root@10.1.1.1"

# Accept SSH fingerprint
sshpass -p $PASSWORD ssh -o "StrictHostKeyChecking no" $TARGET

echo "Preparing Controller for update"
sshpass -p $PASSWORD ssh $TARGET "sololink_config --update-prepare sololink"

echo "Uploading $FILENAME_1"
sshpass -p $PASSWORD scp $FILENAME_1 $TARGET:/log/updates

echo "Uploading $FILENAME_2"
sshpass -p $PASSWORD scp $FILENAME_2 $TARGET:/log/updates

echo "Executing update and reboot"
sshpass -p $PASSWORD ssh $TARGET "sololink_config --update-apply sololink --reset"

echo "Update in progress. the process takes about 5 min. You will need to pair again Copter and Controller after update"