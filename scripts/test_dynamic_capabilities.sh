#!/bin/bash

# Test script for dynamic power and channel querying
# Usage: ./test_dynamic_capabilities.sh

set -e

echo "Testing WPANUSB Dynamic Capabilities"
echo "====================================="

# Check if module is loaded
if ! lsmod | grep -q wpanusb; then
    echo "Error: wpanusb module not loaded"
    echo "Run: sudo ./modprobe.sh"
    exit 1
fi

# Find wpanusb device
PHY=$(iwpan phy | grep wpan_phy | cut -d' ' -f2)

if [ -z "$PHY" ]; then
    echo "Error: No wpanusb device found"
    exit 1
fi

echo "Found wpanusb device: $PHY"
echo

# Test 1: Check supported power levels
echo "1. Testing supported power levels:"
echo "   Querying power capabilities..."
iwpan phy $PHY info | grep -A 20 "Supported TX power"
echo

# Test 2: Check supported channels
echo "2. Testing supported channels:"
echo "   Querying channel capabilities..."
iwpan phy $PHY info | grep -A 10 "Supported channels"
echo

# Test 3: Test power level setting
echo "3. Testing power level configuration:"
CURRENT_POWER=$(iwpan phy $PHY info | grep "current TX power" | awk '{print $4}')
echo "   Current power: $CURRENT_POWER"

# Try setting different power levels
for power in 0 -100 -500 -1000; do
    echo "   Setting power to: ${power} mBm"
    if iwpan phy $PHY set txpower $power 2>/dev/null; then
        echo "   ✓ Power set successfully"
    else
        echo "   ✗ Failed to set power (may not be supported)"
    fi
done
echo

# Test 4: Test channel configuration
echo "4. Testing channel configuration:"
CURRENT_CHANNEL=$(iwpan phy $PHY info | grep "current channel" | awk '{print $3}')
echo "   Current channel: $CURRENT_CHANNEL"

# Try setting different channels
for channel in 11 15 20 26; do
    echo "   Setting channel to: $channel"
    if iwpan phy $PHY set channel 0 $channel 2>/dev/null; then
        echo "   ✓ Channel set successfully"
    else
        echo "   ✗ Failed to set channel (may not be supported)"
    fi
done
echo

# Test 5: Check for multi-page support
echo "5. Testing multi-page channel support:"
for page in 0 1 2; do
    echo "   Checking page $page support..."
    if iwpan phy $PHY info | grep -q "page $page"; then
        echo "   ✓ Page $page supported"
        iwpan phy $PHY info | grep "page $page" | head -1
    else
        echo "   ✗ Page $page not supported"
    fi
done
echo

# Test 6: Verify dynamic vs static behavior
echo "6. Verifying dynamic capability detection:"
echo "   Checking kernel log for capability queries..."
if dmesg | tail -20 | grep -q "Queried.*power levels"; then
    echo "   ✓ Dynamic power level querying detected"
else
    echo "   ⚠ No dynamic power querying detected (using defaults)"
fi

if dmesg | tail -20 | grep -q "channels supported"; then
    echo "   ✓ Dynamic channel querying detected"
else
    echo "   ⚠ No dynamic channel querying detected (using defaults)"
fi
echo

echo "Testing completed!"
echo "Check 'dmesg | tail -50' for detailed kernel messages"
