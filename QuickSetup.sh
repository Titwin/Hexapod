#!/bin/bash

# define Script helper functions and macros
PURPLE='\033[1;35m'
GREEN='\033[1;32m'
NC='\033[0m' # No Color

function printStepHeader {
	let "step+=1"
	echo -e "${step}/${stepTotal} : ${PURPLE}$1${NC}"
}
step=0
stepTotal=$(($(grep -o "printStepHeader" QuickSetup.sh | wc -l)-1))



# Update and upgrade OS
printStepHeader "Update and upgrade"
sudo apt-get update -y
sudo apt-get upgrade -y



# UART configuration
printStepHeader "UART configuration"
sudo mv /boot/config.txt /boot/config.txt.orig
sudo cp OSConfig/config.txt /boot/config.txt

# Allow SSH connection via root login
printStepHeader "Allow SSH connection via root login"
sudo mv /etc/ssh/sshd_config /etc/ssh/sshd_config.orig
sudo cp OSConfig/sshd_config /etc/ssh/sshd_config


# WIFI HOTSPOT
# prepare to configure Wifi hotspot
printStepHeader "Prepare to configure Wifi hotspot"
sudo apt-get install dnsmasq hostapd -y
sudo systemctl stop dnsmasq
sudo systemctl stop hostapd

# change dhcpcd configuration
printStepHeader "Change dhcpcd configuration"
sudo mv /etc/dhcpcd.conf /etc/dhcpcd.conf.orig
sudo cp OSConfig/dhcpcd.conf /etc/dhcpcd.conf
sudo systemctl daemon-reload
sudo service dhcpcd restart

# change dnsmasq configuration
printStepHeader "Change dnsmasq configuration"
sudo mv /etc/dnsmasq.conf /etc/dnsmasq.conf.orig
sudo cp OSConfig/dnsmasq.conf /etc/dnsmasq.conf

# change or create Wifi properties
ssid=$(hostname)_hotspot
printStepHeader "Change or create Wifi properties, hotspot ssid = '${ssid}'"
sudo mv /etc/hostapd/hostapd.conf /etc/hostapd/hostapd.conf.orig
sed -i "s/hostname/$(hostname)/g" OSConfig/hostapd.conf
sudo cp OSConfig/hostapd.conf /etc/hostapd/hostapd.conf
sudo mv /etc/default/hostapd /etc/default/hostapd.orig
sudo cp OSConfig/hostapd /etc/default/hostapd

# restarting services
printStepHeader "Restarting services"
sudo systemctl unmask hostapd.service
sudo systemctl start hostapd
sudo systemctl start dnsmasq

# share eth0 internet on wifi (acess point)
printStepHeader "Share eth0 internet on wifi (to create acess point)"
sudo mv /etc/sysctl.conf /etc/sysctl.conf.orig
sudo cp OSConfig/sysctl.conf /etc/sysctl.conf
sudo iptables -t nat -A  POSTROUTING -o eth0 -j MASQUERADE
sudo sh -c "iptables-save > /etc/iptables.ipv4.nat"

# add route via Thibault-SED-PC (aka 192.168.4.2)
sudo ip route add default via 192.168.4.2


# Finished !!
echo -e "I ${GREEN}Finished !! Now restart your Raspberry to takes modification.${NC}"
