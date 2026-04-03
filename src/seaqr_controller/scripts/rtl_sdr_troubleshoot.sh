#!/bin/bash

echo "=== RTL-SDR Troubleshooting Script ==="
echo

echo "1. Checking for existing dump1090 processes..."
ps aux | grep dump1090 | grep -v grep
if [ $? -eq 0 ]; then
    echo "Found existing dump1090 processes. Killing them..."
    pkill -f dump1090
    sleep 2
else
    echo "No existing dump1090 processes found."
fi
echo

echo "2. Checking RTL-SDR device availability..."
rtl_test -t
echo

echo "3. Checking USB devices..."
lsusb | grep -i rtl
echo

echo "4. Checking for RTL-SDR kernel modules..."
lsmod | grep rtl
echo

echo "5. Checking device permissions..."
ls -la /dev/bus/usb/*/00*
echo

echo "6. Testing RTL-SDR with rtl_test..."
timeout 10s rtl_test -t
echo

echo "7. Checking if dump1090 is available..."
which dump1090-fa
if [ $? -eq 0 ]; then
    echo "dump1090-fa found at: $(which dump1090-fa)"
    echo "Testing dump1090 startup..."
    timeout 5s dump1090-fa --help | head -5
else
    echo "dump1090-fa not found in PATH"
fi
echo

echo "=== Troubleshooting Complete ==="
echo "If you see 'Device or resource busy' errors, try:"
echo "1. Unplug and replug the RTL-SDR device"
echo "2. Run: sudo modprobe -r rtl2832"
echo "3. Run: sudo modprobe rtl2832"
echo "4. Or reboot the system"
