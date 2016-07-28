#!/bin/sh
md5sum READY/system/chainfire/SuperSU.apk | awk '{print $1}' > READY/system/chainfire/SuperSU.apk.md5;
echo "Done!";
cat READY/system/chainfire/SuperSU.apk.md5;
