#!/bin/bash
# Diagnose why the FRDM-MCXA153's ttyACM port doesn't appear (or drops) on a
# Raspberry Pi. Run it with the board UNPLUGGED, plug the board in when told,
# leave it for 30 s, then send the whole output (also saved to the log file).
#
#   bash pi_usb_diag.sh

LOG="$HOME/mcxa153_usb_diag_$(date +%Y%m%d_%H%M%S).log"
exec > >(tee "$LOG") 2>&1

section() { echo; echo "=== $1 ==="; }

section "system"
cat /proc/device-tree/model 2>/dev/null; echo
grep PRETTY_NAME /etc/os-release
uname -r
id

section "power (throttled=0x0 is good; bit 0 / bit 16 = under-voltage)"
vcgencmd get_throttled 2>/dev/null || echo "vcgencmd not available"

section "services that grab serial ports"
for s in ModemManager brltty gpsd; do
    printf '%-14s %s\n' "$s" "$(systemctl is-active "$s" 2>/dev/null)"
done

section "USB devices before plug-in"
lsusb

section "kernel log while plugging in"
echo ">>> Plug the board in NOW. Watching for 30 seconds..."
if journalctl -k -n 0 >/dev/null 2>&1; then
    timeout 30 journalctl -k -f -n 0 --no-pager
else
    timeout 30 sudo dmesg -w --since now 2>/dev/null || timeout 30 sudo dmesg -w
fi

section "USB devices after plug-in"
lsusb
lsusb -t

section "serial ports"
ls -l /dev/ttyACM* /dev/ttyUSB* 2>&1
ls -l /dev/serial/by-id/ 2>&1

section "who has the ports open"
sudo fuser -v /dev/ttyACM* 2>&1 || true

section "power after plug-in"
vcgencmd get_throttled 2>/dev/null

echo
echo "Saved to $LOG"
