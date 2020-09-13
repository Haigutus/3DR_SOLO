
REPOSITIRY="https://github.com/OpenSolo/OpenSolo"
RELEASE="OpenSolo-4.0.0"
FILES=(
	ArduCopter.4.0.1.CubeSolo.apj
	3dr-controller.tar.gz
	3dr-controller.tar.gz.md5
	3dr-solo.tar.gz
	3dr-solo.tar.gz.md5
	)


echo "Start Download from OpenSolo GIT."
echo "Computer needs to be connected to Internet."
echo "OS needs to have installed 'wget'"

for FILE in "${FILES[@]}"; do
    echo "Downloading $FILE"
    wget $REPOSITIRY/releases/download/$RELEASE/$FILE
done

echo "STEP 1 Done - Download complete. Connect your computer now to Solo wify and run STEP 2"