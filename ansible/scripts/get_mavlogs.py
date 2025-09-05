#!/usr/bin/env python3
"""
MAVLink log collection script for swarm-nxt drones.
Uses pymavlink to download flight logs from the flight controller.
"""

from pymavlink import mavutil
from loguru import logger
import os
import sys
import time
from datetime import datetime
import signal
import threading

# Configure logging
os.makedirs("{{ drone_base_path }}/logs/scripts/get_mavlogs", exist_ok=True)
logger.add(f"{{ drone_base_path }}/logs/scripts/get_mavlogs/{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")

# Global shutdown event
shutdown_event = threading.Event()

def signal_handler(signum, frame):
    """Handle shutdown signals gracefully"""
    logger.info(f"Received signal {signum}, initiating shutdown")
    shutdown_event.set()

def get_mavlink_logs(connection_string, log_dir):
    """
    Download available MAVLink logs from flight controller
    
    Args:
        connection_string (str): MAVLink connection string (e.g., "/dev/ttyTHS1:921600")
        log_dir (str): Directory to save downloaded logs
    
    Returns:
        bool: True if successful, False otherwise
    """
    try:
        logger.info(f"Connecting to flight controller at {connection_string}")
        conn = mavutil.mavlink_connection(connection_string)
        
        # Wait for heartbeat to confirm connection
        logger.info("Waiting for heartbeat...")
        conn.wait_heartbeat(timeout=10)
        logger.info(f"Connected to system {conn.target_system}, component {conn.target_component}")
        
        # Request log list
        logger.info("Requesting log list...")
        conn.mav.log_request_list_send(
            conn.target_system,
            conn.target_component,
            0,  # start
            0xffff  # end (all logs)
        )
        
        # Collect log entries
        log_entries = []
        start_time = time.time()
        timeout = 10  # seconds
        
        while time.time() - start_time < timeout and not shutdown_event.is_set():
            msg = conn.recv_match(type=['LOG_ENTRY'], timeout=1)
            if msg:
                if msg.get_type() == 'LOG_ENTRY':
                    log_entries.append(msg)
                    logger.info(f"Found log {msg.id}: {msg.num_logs} total logs, size: {msg.size} bytes")
        
        if not log_entries:
            logger.warning("No logs found on flight controller")
            return True  # Not an error, just no logs available
        
        logger.info(f"Found {len(log_entries)} log entries")
        
        # Download each log
        for log_entry in log_entries:
            if shutdown_event.is_set():
                break
                
            log_id = log_entry.id
            log_size = log_entry.size
            
            if log_size == 0:
                logger.warning(f"Skipping log {log_id} (size is 0)")
                continue
            
            logger.info(f"Downloading log {log_id} ({log_size} bytes)...")
            
            # Create output filename
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            log_filename = os.path.join(log_dir, f"mavlink_log_{log_id}_{timestamp}.bin")
            
            # Request log data
            conn.mav.log_request_data_send(
                conn.target_system,
                conn.target_component,
                log_id,
                0,  # start offset
                log_size  # count (entire log)
            )
            
            # Collect log data
            log_data = bytearray()
            bytes_received = 0
            data_timeout = 30  # seconds
            data_start_time = time.time()
            
            while bytes_received < log_size and not shutdown_event.is_set():
                if time.time() - data_start_time > data_timeout:
                    logger.error(f"Timeout waiting for log data for log {log_id}")
                    break
                
                msg = conn.recv_match(type=['LOG_DATA'], timeout=2)
                if msg and msg.get_type() == 'LOG_DATA' and msg.id == log_id:
                    # Append the data
                    data_chunk = msg.data[:msg.count]  # Only use the valid bytes
                    log_data.extend(data_chunk)
                    bytes_received += msg.count
                    
                    if bytes_received % 1024 == 0:  # Log progress every KB
                        logger.debug(f"Received {bytes_received}/{log_size} bytes for log {log_id}")
            
            if bytes_received == log_size:
                # Save log to file
                with open(log_filename, 'wb') as f:
                    f.write(log_data)
                logger.info(f"Successfully downloaded log {log_id} to {log_filename}")
            else:
                logger.error(f"Incomplete download for log {log_id}: {bytes_received}/{log_size} bytes")
        
        conn.close()
        return True
        
    except Exception as e:
        logger.error(f"Error downloading MAVLink logs: {e}")
        return False

def main():
    """Main function"""
    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Configuration
    connection_string = "{{ drone_fcu_url }}"
    log_dir = "{{ drone_base_path }}/logs/latest/mavlink"
    
    # Create log directory
    os.makedirs(log_dir, exist_ok=True)
    
    logger.info("Starting MAVLink log collection")
    logger.info(f"Connection: {connection_string}")
    logger.info(f"Log directory: {log_dir}")
    
    # Download logs
    success = get_mavlink_logs(connection_string, log_dir)
    
    if success:
        logger.info("MAVLink log collection completed successfully")
        sys.exit(0)
    else:
        logger.error("MAVLink log collection failed")
        sys.exit(1)

if __name__ == "__main__":
    main()