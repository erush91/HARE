#!/usr/bin/env bash
###################################################################################
# Script to setup an interface on BATMAN-adv                                      #
# To run the script type:                                                         # 
# ./setupScript.sh ip_address mtu frequency                                       #
# Examples:                                                                       #
# All 3 arguments specified:                                                      #
#     ./setupScript.sh 192.168.100.4 1560 2417                                    #
# No mtu specified:                                                               #
#     ./setupScript.sh 192.168.100.4 " " 2417                                     #
# No arguments:(the default values of  192.168.100.1, 1500, 2412 will be assigned)#
#     ./setupScript.sh                                                            #
###################################################################################

# Script arguments
 
# ip address
ARG1=${1:-192.168.100.52} # Address for ME
# mtu 
ARG2=${2:-1560}
# frequency
ARG3=${3:-2412}

#Stopping network manager is necessary if trying to setup
#on a machine's native wireless card.  No necessary for wifi adapters 
#sudo systemctl stop NetworkManager 
#sudo service networking stop  #for Raspberry Pis
#
#get wifi adapter's logical name (for VM and machine with no wireless card)
tmpVar=$(iw dev | awk '$1=="Interface"{print $2}')

#get the top device name returned.  Assumes the top listing is what is needed
arr=($(echo $tmpVar | tr " " "\n"))
#devID=$arr
devID=wlx000f020202da
#devID=wlx00198671e13c # good carl, used on gene_surface in Hidee Mine 
#devID=wlx0013ef202488 # bad inconsistent odroid
#devID=wlx00198671d7ec # bad managed carl

#bring down ip link in order to set up ip address, mtu and mode
sudo ip link set $devID down

echo "Device ID: $devID"

#set ip address (Diane edit added /24)
sudo ip addr add $ARG1/24 dev $devID

#set mtu
sudo ip link set $devID mtu $ARG2

#set mode to ad-hoc
sudo iw dev $devID set type ibss

#bring the device online
sudo ip link set $devID up

#join network with specified frequency, cell address and HT20
sudo iw dev $devID ibss join MARBLE $ARG3 HT20 fixed-freq 02:0f:00:73:09:07

#check to see if the interface exsists and delete it if it does
tmpVar2=$(ip link show | grep bat0)

if [ "{$tmpVar2}" != "" ];
then
   sudo ip link delete bat0
fi


#bring up BATMAN-adv
sudo ip link add name bat0 type batadv

#add the interface
# 3/13/19 edit do not set as master
#sudo ip link set dev $devID master bat0

# 3/13/19 use batctl to add interface
sudo batctl if add $devID
sudo ip link set up dev bat0 

# diane edit, 3/13/19 added /24
sudo ip addr add $ARG1/24 dev bat0

# edit, 3/13/19 do not flush
#sudo ip addr flush dev $devID







