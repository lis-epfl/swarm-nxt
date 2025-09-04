from pymavlink import mavutil
import yaml
from loguru import logger
import os
from datetime import datetime
import threading
import queue
import signal
import sys
import struct

os.makedirs("{{ drone_base_path }}/logs/scripts/ensure_params", exist_ok=True)
logger.add(f"{{ drone_base_path }}/logs/ensure_params/{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")

shutdown_event = threading.Event()
param_queue = queue.Queue()

class MAVLinkParamConverter:
    @staticmethod
    def to_float(actual_value, param_type):
        """Convert actual value to MAVLink float representation"""
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
            packed = struct.pack('B', int(actual_value)) + b'\x00\x00\x00'
        elif param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
            packed = struct.pack('b', int(actual_value)) + b'\x00\x00\x00'
        elif param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
            packed = struct.pack('H', int(actual_value)) + b'\x00\x00'
        elif param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
            packed = struct.pack('h', int(actual_value)) + b'\x00\x00'
        elif param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
            packed = struct.pack('I', int(actual_value))
        elif param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
            packed = struct.pack('i', int(actual_value))
        elif param_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
            return float(actual_value)
        else:
            return float(actual_value)

        return struct.unpack('f', packed)[0]

    @staticmethod
    def from_float(param_value_float, param_type):
        """Convert MAVLink float representation to actual value"""
        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
            return param_value_float

        packed_float = struct.pack('f', param_value_float)

        if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
            return struct.unpack('B', packed_float[:1])[0]
        elif param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
            return struct.unpack('b', packed_float[:1])[0]
        elif param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
            return struct.unpack('H', packed_float[:2])[0]
        elif param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
            return struct.unpack('h', packed_float[:2])[0]
        elif param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
            return struct.unpack('I', packed_float)[0]
        elif param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
            return struct.unpack('i', packed_float)[0]
        else:
            return param_value_float

def listen_for_params(conn: mavutil.mavserial):
    logger.info("Parameter listener thread started")
    while not shutdown_event.is_set():
        try:
            msg = conn.recv_match(type="PARAM_VALUE", timeout=1, blocking=True)
            if msg:
                param_queue.put(msg)
        except Exception as e:
            logger.error(f"Error in param listener: {e}")
            shutdown_event.set()
            break
    logger.info("Parameter listener thread shutting down")

def signal_handler(signum, frame):
    logger.info(f"Received signal {signum}, initiating shutdown")
    shutdown_event.set()

def process_param_message(msg, target_param=None):
    recv_name = msg.param_id
    recv_value = MAVLinkParamConverter.from_float(msg.param_value, msg.param_type)
    real_params[recv_name] = recv_value
    
    if target_param and recv_name == target_param:
        return True
    return False

def send_param_message(connection, param_id, param_value, param_type):
    """
    Send a parameter set message with proper type conversion
    
    Args:
        connection: MAVLink connection object
        param_id (str): Parameter ID/name
        param_value: Python native type value (int, float, etc.)
        param_type: MAVLink parameter type constant
    """
    logger.info(f"Updating parameter {param_id} -> {param_value}...") 
    # Convert the native Python value to MAVLink float representation
    param_value_float = MAVLinkParamConverter.to_float(param_value, param_type)
    
    # Ensure param_id is properly encoded
    if isinstance(param_id, str):
        param_id_bytes = param_id.encode('utf-8')[:16]
    else:
        param_id_bytes = param_id
    
    # Send the parameter set message
    connection.mav.param_set_send(
        connection.target_system,
        connection.target_component,
        param_id_bytes,
        param_value_float,
        param_type
    )

def get_and_check_params(conn, wanted_params, change_enabled):
    mismatch = False
    for k, v in wanted_params.items():
        if shutdown_event.is_set():
            break
            
        logger.info(f"Looking for param {k}")
        retry_count = 3
        
        
        found = False
        while retry_count > 0 and not shutdown_event.is_set():
            try:
                conn.mav.param_request_read_send(
                    conn.target_system,
                    conn.target_component,
                    bytes(k, 'utf-8'),
                    -1
                )
                msg = param_queue.get(timeout=3)
                logger.debug(msg)
                if process_param_message(msg, k):
                    logger.info(f"Found param {k} = {real_params[k]}")
                    if real_params[k] != v:
                        logger.warning(f"Param {k} mismatch: expected {v}, got {real_params[k]}")
                        mismatch = True

                        if change_enabled: 
                            send_param_message(conn, k, v, msg.param_type) 

                    found = True
                    break
            except queue.Empty:
                retry_count -= 1
                logger.warning(f"Timeout waiting for param {k}, retries left: {retry_count}")
        
        if not found and not shutdown_event.is_set():
            logger.error(f"Failed to read param {k}")
            shutdown_event.set()
        
        return mismatch

conn = mavutil.mavlink_connection("/dev/ttyTHS1", baud=921600)
with open("{{ drone_ros_path }}/config/params.yaml", "r") as f:
    wanted_params = yaml.safe_load(f)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


listener_thread = threading.Thread(target=listen_for_params, args=(conn,))
listener_thread.daemon = True
listener_thread.start()

real_params = {}
changed = False


# first get and check 
changed = get_and_check_params(conn, wanted_params, True)
shutdown_event.set()
listener_thread.join(timeout=2)
logger.info("Shutdown complete")

if changed: 
    exit(10)
else:
    exit(0)
