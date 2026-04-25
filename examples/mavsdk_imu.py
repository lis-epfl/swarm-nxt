import asyncio
import time
from collections import deque
from mavsdk import System

# Define a helper function to compute frequency over a window of last 500 measurements
def compute_frequency(window):
    if len(window) < 2:
        return 0  # Not enough data points to calculate frequency
    # Get the time difference between the first and last timestamps in the window
    time_diff = window[-1] - window[0]
    # Return frequency (in Hz), which is the number of messages per second
    return len(window) / time_diff if time_diff > 0 else 0

async def main():
    # Connect to the flight controller (adjust port and baudrate)
    drone = System()
    await drone.connect(system_address="serial:///dev/ttyTHS1:921600")  # Adjust the port and baudrate

    print("Waiting for the drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

# Create a deque to store timestamps for the last 500 attitude messages
    attitude_window = deque(maxlen=500)
    imu_window = deque(maxlen=500)

    # Subscribe to high-res IMU data
    print("Subscribing to HIGHRES_IMU messages...")
    async for imu in drone.telemetry.imu():
        current_time = time.time()
        imu_window.append(current_time)  # Store timestamp

        # Compute and display the frequency over the last 500 messages
        imu_frequency = compute_frequency(imu_window)
        print(f"Time (us): {imu.timestamp_us}")
        print(f"X Acceleration (m/s^2): {imu.acceleration_frd.forward_m_s2}")
        print(f"Y Acceleration (m/s^2): {imu.acceleration_frd.right_m_s2}")
        print(f"Z Acceleration (m/s^2): {imu.acceleration_frd.down_m_s2}")
        print(f"X Gyro (rad/s): {imu.angular_velocity_frd.forward_rad_s}")
        print(f"Y Gyro (rad/s): {imu.angular_velocity_frd.right_rad_s}")
        print(f"Z Gyro (rad/s): {imu.angular_velocity_frd.down_rad_s}")
        print(f"X Mag (gauss): {imu.magnetic_field_frd.forward_gauss}")
        print(f"Y Mag (gauss): {imu.magnetic_field_frd.right_gauss}")
        print(f"Z Mag (gauss): {imu.magnetic_field_frd.down_gauss}")
        print(f"Temperature (°C): {imu.temperature_degc}")
        print(f"HIGHRES_IMU message frequency: {imu_frequency:.2f} Hz")


if __name__ == "__main__":
    asyncio.run(main())
