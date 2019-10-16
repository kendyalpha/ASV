#!/bin/bash
COPYPATH="./"
rsync -av --progress ../../ASV ${COPYPATH} --exclude build --exclude Build --exclude BUILD \
--exclude build --exclude Build --exclude BUILD --exclude tools --exclude .git \
--exclude '*.csv'  --exclude '*.db' --exclude matlab 
tar -cvzf ./ASV.tar.gz "${COPYPATH}ASV"
sudo rm -r ./ASV