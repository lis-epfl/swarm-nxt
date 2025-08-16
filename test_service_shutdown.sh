#!/bin/bash
# Test script to verify the systemd service shutdown fix
# Run this script on the drone or host computer after deploying the changes

set -e

echo "🔧 Testing systemd service shutdown behavior..."
echo "This script will test if the ros_packages.service shuts down cleanly."
echo ""

# Check if service exists
if ! systemctl list-unit-files | grep -q "ros_packages.service"; then
    echo "❌ ros_packages.service not found. Make sure the service is installed."
    exit 1
fi

echo "📋 Current service status:"
systemctl status ros_packages.service --no-pager -l || true
echo ""

# Check if service is running
if systemctl is-active --quiet ros_packages.service; then
    echo "🔄 Service is currently running. Testing shutdown..."
    
    # Record start time
    START_TIME=$(date +%s)
    
    # Attempt to stop the service
    echo "⏹️  Stopping ros_packages.service..."
    systemctl stop ros_packages.service
    
    # Record end time
    END_TIME=$(date +%s)
    DURATION=$((END_TIME - START_TIME))
    
    echo "✅ Service stopped successfully!"
    echo "⏱️  Shutdown took ${DURATION} seconds"
    
    if [ $DURATION -lt 35 ]; then
        echo "🎉 SUCCESS: Shutdown completed within expected time (< 35 seconds)"
    else
        echo "⚠️  WARNING: Shutdown took longer than expected (${DURATION}s >= 35s)"
        echo "   This might indicate the fix needs further adjustment."
    fi
    
else
    echo "🔄 Service is not running. Starting it first..."
    systemctl start ros_packages.service
    
    echo "⏳ Waiting 10 seconds for service to fully start..."
    sleep 10
    
    if systemctl is-active --quiet ros_packages.service; then
        echo "✅ Service started successfully. Now testing shutdown..."
        
        # Record start time
        START_TIME=$(date +%s)
        
        # Attempt to stop the service
        echo "⏹️  Stopping ros_packages.service..."
        systemctl stop ros_packages.service
        
        # Record end time
        END_TIME=$(date +%s)
        DURATION=$((END_TIME - START_TIME))
        
        echo "✅ Service stopped successfully!"
        echo "⏱️  Shutdown took ${DURATION} seconds"
        
        if [ $DURATION -lt 35 ]; then
            echo "🎉 SUCCESS: Shutdown completed within expected time (< 35 seconds)"
        else
            echo "⚠️  WARNING: Shutdown took longer than expected (${DURATION}s >= 35s)"
            echo "   This might indicate the fix needs further adjustment."
        fi
    else
        echo "❌ Failed to start the service. Check logs with:"
        echo "   journalctl -u ros_packages.service -f"
        exit 1
    fi
fi

echo ""
echo "📊 Recent service logs:"
journalctl -u ros_packages.service --no-pager -l --since "5 minutes ago" | tail -10

echo ""
echo "🔍 To monitor service behavior in real-time, use:"
echo "   journalctl -u ros_packages.service -f"
echo ""
echo "✨ Test completed!"