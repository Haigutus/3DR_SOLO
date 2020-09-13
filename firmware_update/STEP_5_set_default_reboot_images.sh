# Do this only after you have verified that new firmware is good. 
# If you have issues with firmware you can now still reset to 3DR factory image.
# After running this script the facotry images are replaced with ones you uploaded in previous steps.

PASSWORD="TjSDBkAu"
TARGET_COPTER="root@10.1.1.10"
TARGET_CONTROLLER="root@10.1.1.1"

echo "Setting current firmware as default images. Factory reset will use them for reset. In 3DR terminology 'golden image'."

echo "Setting default image for Copter"
sshpass -p $PASSWORD ssh $TARGET_COPTER "sololink_config --make-golden"

echo "Setting default image for Controller"
sshpass -p $PASSWORD ssh $TARGET_CONTROLLER "sololink_config --make-golden"


echo "Done. No further action required"