import subprocess
import argparse

if __name__ == "__main__":
    DEFAULT_HOST_HOSTNAME = "nxt_host.local"
    DEFAULT_PORT = "14550"
    DEFAULT_DOMIAN_ID = 1

    parser = argparse.ArgumentParser("Launnch Mavros on Target")
    parser.add_argument("--host", default=DEFAULT_HOST_HOSTNAME)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--domain-id", default=DEFAULT_DOMIAN_ID)
    parser.add_argument("--serial-port", default="/dev/ttyTHS1")
    parser.add_argument("--baud-rate", default="921600")

    args = parser.parse_args()

    subprocess.run(
        [
            "source",
            "/opt/ros/humble/setup.bash",
            "&&",
            "ros2",
            "launch",
            "px4.launch",
            f'fcu_url:="{args.serial_port}:{args.baud_rate}"',
            f'gcs_url:="udp://@{args.host}:{args.port}"'
        ],
        env={
            "ROS_DOMAIN_ID": args.domain_id
        }
    )
