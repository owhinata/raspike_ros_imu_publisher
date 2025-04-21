import rclpy
import time
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from raspike_uros_msg.msg import SpikeDevStatusMessage


class ImuPublisher(Node):
    def __init__(self):
        super().__init__("raspike_ros_imu_publisher")
        qos_profile = QoSProfile(depth=10, reliability=2)

        self.publisher = self.create_publisher(Imu, "imu", qos_profile)
        self.status_subscription = self.create_subscription(
            SpikeDevStatusMessage, "spike_device_status",
            self.status_on_subscribe, qos_profile)

    def status_on_subscribe(self, status):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_frame"
        msg.angular_velocity.x = np.deg2rad(status.angular_velocity[0]).item()
        msg.angular_velocity.y = np.deg2rad(status.angular_velocity[1]).item()
        msg.angular_velocity.z = np.deg2rad(status.angular_velocity[2]).item()
        msg.linear_acceleration.x = status.linear_acceleration[0].item() * 1e-3
        msg.linear_acceleration.y = status.linear_acceleration[1].item() * 1e-3
        msg.linear_acceleration.z = status.linear_acceleration[2].item() * 1e-3
        msg.angular_velocity_covariance = [
            0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        msg.linear_acceleration_covariance = [
            0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        msg.orientation_covariance = [
            -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
