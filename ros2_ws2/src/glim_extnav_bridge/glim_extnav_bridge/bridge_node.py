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
        # Timestamp handling
        # restamp_source: 'none' (as-is), 'arrival' (use arrival callback time), 'now' (use current clock now)
        self.declare_parameter('restamp_source', 'none')
        # reject message if older than this [ms] relative to node clock (only when restamp='none')
        self.declare_parameter('reject_older_than_ms', 200.0)
        # publish immediately on callback if rate allows
        self.declare_parameter('publish_immediately', True)

        glim_ns = self.get_parameter('glim_namespace').get_parameter_value().string_value
        use_corrected = self.get_parameter('use_corrected').get_parameter_value().bool_value
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.override_child = self.get_parameter('odom_child_frame_id').get_parameter_value().string_value
        self.restamp_source = self.get_parameter('restamp_source').get_parameter_value().string_value
        self.reject_older_than_ms = self.get_parameter('reject_older_than_ms').get_parameter_value().double_value
        self.publish_immediately = self.get_parameter('publish_immediately').get_parameter_value().bool_value

        # Source topics from GLIM
        odom_topic = f"{glim_ns}/odom_corrected" if use_corrected else f"{glim_ns}/odom"
        pose_topic = f"{glim_ns}/pose"  # optional, not strictly needed

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        self.latest_odom = None
        self.last_published_stamp = None
        self.last_publish_time = self.get_clock().now()

        self.sub_odom = self.create_subscription(Odometry, odom_topic, self._on_odom, qos)
        self.pub_mavros_odom = self.create_publisher(Odometry, '/mavros/odometry/in', 10)

        # Timer to throttle to desired rate
        period = 1.0 / max(1e-3, self.publish_rate_hz)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(f"Subscribing: {odom_topic}")
        self.get_logger().info("Publishing -> /mavros/odometry/in")

    def _on_odom(self, msg: Odometry):
        # Copy to avoid mutating shared instance unexpectedly
        odom = Odometry()
        odom.header = msg.header
        odom.child_frame_id = msg.child_frame_id
        odom.pose = msg.pose
        odom.twist = msg.twist

        if self.override_child:
            odom.child_frame_id = self.override_child

        # Timestamp policy
        now = self.get_clock().now()
        if self.restamp_source == 'arrival':
            odom.header.stamp = now.to_msg()
        elif self.restamp_source == 'now':
            odom.header.stamp = now.to_msg()
        else:
            # Reject too old messages only when using source timestamps
            dt_ms = (now - rclpy.time.Time.from_msg(odom.header.stamp)).nanoseconds / 1e6
            if self.reject_older_than_ms > 0.0 and dt_ms > self.reject_older_than_ms:
                self.get_logger().warn(
                    f"Drop old odom: age={dt_ms:.1f} ms > {self.reject_older_than_ms:.1f} ms")
                return

        self.latest_odom = odom

        if self.publish_immediately:
            self._maybe_publish(now)

    def _on_timer(self):
        self._maybe_publish(self.get_clock().now())

    def _maybe_publish(self, now):
        if self.latest_odom is None:
            return
        # Enforce publish rate
        min_period = 1.0 / max(1e-6, self.publish_rate_hz)
        if (now - self.last_publish_time).nanoseconds < min_period * 1e9:
            return

        # Deduplicate by stamp if unchanged
        if self.last_published_stamp is not None:
            if (self.latest_odom.header.stamp.sec == self.last_published_stamp.sec and
                self.latest_odom.header.stamp.nanosec == self.last_published_stamp.nanosec):
                return

        self.pub_mavros_odom.publish(self.latest_odom)
        self.last_publish_time = now
        self.last_published_stamp = self.latest_odom.header.stamp


def main():
    rclpy.init()
    node = GlimExtnavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


