#!/bin/bash
rsync -av --progress ../ASV ~/ --exclude build --exclude Build --exclude BUILD \
--exclude build --exclude Build --exclude BUILD --exclude '*.db'  --exclude .git \
--exclude '*.csv' 
tar -cvzf ~/ASV.tar.gz ~/ASV