1. Install batctl

sudo apt install batctl

2. Make sure /etc/hosts file includes correct batman-specific IP addresses

e.g.

127.0.0.1	localhost
192.168.100.51	odroid
192.168.100.53	gene_surface

3. Do not edit setupBatman.sh

4. Edit setupBatman.sh Line 18: Specify static IP address of local machine (you get to choose)

e.g. 

ARG1=${1:-192.168.100.53}

5. Edit setupBatman.sh Line 35: Specify WiFi device ID (find from ifconfig)

e.g. wlx001986513db2

wlx001986513db2: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        ether 00:19:86:51:3d:b2  txqueuelen 1000  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

5. Run batman

sudo ./startBatman.sh
