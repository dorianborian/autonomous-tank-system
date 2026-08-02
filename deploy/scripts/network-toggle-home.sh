#!/bin/bash
# Switch the Jetson's WiFi back to the normal home network client connection.
set -e

IFACE="<YOUR_WIFI_INTERFACE>"
HOTSPOT="<YOUR_HOTSPOT_CONN_NAME>"
HOME_CONN="<YOUR_HOME_WIFI_CONN_NAME>"

echo "Switching $IFACE back to home WiFi ($HOME_CONN)..."
sudo nmcli connection down "$HOTSPOT" >/dev/null 2>&1 || true
sudo nmcli connection up "$HOME_CONN"

sleep 2
echo ""
echo "Home WiFi active. Details:"
nmcli -f GENERAL.STATE,IP4.ADDRESS device show "$IFACE" | grep -E "STATE|ADDRESS"

# See network-toggle-hotspot.sh: ODIN's SDK needs a restart to rebind to
# whatever IP the interface lands on.
echo ""
echo "Restarting ODIN/behavior stack to rebind to the new IP..."
sudo systemctl restart robot-odin.service
sleep 8
sudo systemctl restart robot-behavior.service
echo "Done."
