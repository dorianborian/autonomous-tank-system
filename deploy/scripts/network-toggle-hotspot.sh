#!/bin/bash
# Switch the Jetson's WiFi to broadcast its own hotspot.
# This drops the current home WiFi client connection while active.
#
# Requires two NetworkManager connection profiles to already exist:
#   - a client profile for your normal home WiFi (HOME_CONN below)
#   - a hotspot/AP profile for the robot's own network (HOTSPOT below)
# Create the hotspot profile once with something like:
#   sudo nmcli connection add type wifi ifname <IFACE> con-name <HOTSPOT> \
#     autoconnect no ssid <YOUR_SSID>
#   sudo nmcli connection modify <HOTSPOT> 802-11-wireless.mode ap \
#     802-11-wireless.band bg ipv4.method shared \
#     wifi-sec.key-mgmt wpa-psk wifi-sec.psk <YOUR_PASSWORD>
set -e

IFACE="<YOUR_WIFI_INTERFACE>"       # e.g. `nmcli device status` to find it
HOTSPOT="<YOUR_HOTSPOT_CONN_NAME>"  # NetworkManager connection profile name
HOME_CONN="<YOUR_HOME_WIFI_CONN_NAME>"

echo "Switching $IFACE to hotspot mode ($HOTSPOT)..."
sudo nmcli connection down "$HOME_CONN" >/dev/null 2>&1 || true
sudo nmcli connection up "$HOTSPOT"

sleep 2
echo ""
echo "Hotspot active. Details:"
nmcli -f GENERAL.STATE,IP4.ADDRESS device show "$IFACE" | grep -E "STATE|ADDRESS"
echo ""
echo "Connect to the hotspot with the SSID/password configured in the"
echo "'$HOTSPOT' NetworkManager profile (see the header of this script)."

# ODIN's vendor SDK binds a socket to the interface's IP at process startup
# and does not rebind if that IP changes later -- restart the stack so it
# picks up the hotspot's IP cleanly.
echo ""
echo "Restarting ODIN/behavior stack to rebind to the new IP..."
sudo systemctl restart robot-odin.service
sleep 8
sudo systemctl restart robot-behavior.service
echo "Done."
