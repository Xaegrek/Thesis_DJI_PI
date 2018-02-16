#!/bin/bash

service network-manager stop
ifconfig wlan0 down
iwconfig wlan0 mode ad-hoc
iwconfig wlan0 essid "xaegrek"
ifconfig wlan0 up

ifconfig wlan0 156.0.0.156 netmask 255.255.255.0 broadcast 156.0.0.255
