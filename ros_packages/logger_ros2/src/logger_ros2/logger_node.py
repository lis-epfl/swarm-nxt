import rclpy
from rclpy.node import Node
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions
from rclpy.qos import QoSProfile


class LoggerNode(Node):
    def __init__(self):
        super().__init__("logger_node")
        self.writer = None
        self.topics_to_log = []
        self.is_logging = False

    def start_logging(self, topics):
        if self.is_logging:
            self.get_logger().warn("Logging is already in progress.")
            return

        self.topics_to_log = topics
        storage_options = StorageOptions(uri="my_bag", storage_id="mcap")
        converter_options = ConverterOptions()
        self.writer = SequentialWriter()
        self.writer.open(storage_options, converter_options)

        for topic in self.topics_to_log:
            self.writer.create_topic(topic, "std_msgs/msg/String", QoSProfile(depth=10))

        self.is_logging = True
        self.get_logger().info("Started logging topics: {}".format(self.topics_to_log))

    def stop_logging(self):
        if not self.is_logging:
            self.get_logger().warn("Logging is not in progress.")
            return

        self.writer.close()
        self.is_logging = False
        self.get_logger().info("Stopped logging.")

    def log_message(self, topic, message):
        if self.is_logging and topic in self.topics_to_log:
            self.writer.write(topic, message)
            self.get_logger().info("Logged message to topic: {}".format(topic))
        else:
            self.get_logger().warn(
                "Cannot log message. Logging is not active or topic is not being logged."
            )
