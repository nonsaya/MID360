import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time


class GlimExtnavBridge(Node):
    def __init__(self):
        super().__init__('glim_extnav_bridge')

        # Parameters
        self.declare_parameter('glim_namespace', '/glim_ros')
        self.declare_parameter('use_corrected', False)
        self.declare_parameter('publish_rate_hz', 15.0)
        self.declare_parameter('odom_child_frame_id', '')  # override if needed

        glim_ns = self.get_parameter('glim_namespace').get_parameter_value().string_value
        use_corrected = self.get_parameter('use_corrected').get_parameter_value().bool_value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.override_child = self.get_parameter('odom_child_frame_id').get_parameter_value().string_value

        # Source topics from GLIM
        odom_topic = f"{glim_ns}/odom_corrected" if use_corrected else f"{glim_ns}/odom"
        pose_topic = f"{glim_ns}/pose"  # optional, not strictly needed

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        self.latest_odom = None

        self.sub_odom = self.create_subscription(Odometry, odom_topic, self._on_odom, qos)
        self.pub_mavros_odom = self.create_publisher(Odometry, '/mavros/odometry/in', 10)

        # Timer to throttle to desired rate
        period = 1.0 / max(1e-3, self.publish_rate_hz)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(f"Subscribing: {odom_topic}")
        self.get_logger().info("Publishing -> /mavros/odometry/in")

    def _on_odom(self, msg: Odometry):
        if self.override_child:
            msg.child_frame_id = self.override_child
        self.latest_odom = msg

    def _on_timer(self):
        if self.latest_odom is None:
            return
        # Note: MAVROS expects ODOMETRY in local frame. GLIM odom frame_id is typically 'odom'.
        # We forward as-is. Ensure timestamps are current if needed.
        self.pub_mavros_odom.publish(self.latest_odom)


def main():
    rclpy.init()
    node = GlimExtnavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


