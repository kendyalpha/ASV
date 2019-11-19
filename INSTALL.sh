#!/bin/bash

sudo apt update
sudo apt install build-essential cmake sqlite3 libsqlite3-dev rsync \
	catkin python3 python3-pip vim sshfs cutecom

# install GeographicLib
cd third_party/
tar xvf GeographicLib-1.50.tar.gz
cd GeographicLib-1.50
sudo mkdir -m 777 BUILD
cd BUILD
cmake ..
sudo make 
sudo make install

# install SqliteModernCpp
cd ../../sqlite_modern_cpp
./configure && make && sudo make install

# install serial
cd ../serial
make
sudo make install	

# install boost 1.71
cd ../
tar xvf boost_1_71_0.tar.gz 
cd boost_1_71_0
./bootstrap.sh --prefix=/usr
sudo ./b2 install

#### install mosek9   ###
cd ../mosek
tar xvf mosektoolslinux64x86.tar.bz2
sudo cp -rf mosek/ /opt/
sudo cp -rf /opt/mosek/9.0/tools/platform/linux64x86/bin/libmosek64.* /usr/lib/

echo '# environmental variable for mosek9' >> ~/.bashrc
echo 'PATH=/opt/mosek/9.0/tools/platform/linux64x86/bin:$PATH' >> ~/.bashrc
sudo source ~/.bashrc

sudo mkdir -m 777 ~/mosek
sudo mv mosek.lic ~/mosek/

