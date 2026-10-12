from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_srvs.srv import Trigger
from ardupilot_msgs.msg import Status
from ardupilot_msgs.srv import ArmMotors, ModeSwitch, Takeoff
from .flight_state import FlightState
from .handlers import HANDLERS
class FlightTestNode(Node):
    GUIDED_MODE = 4
    LAND_MODE = 9
    TAKEOFF_ALTITUDE = 5.0 
    HOVER_DURATION = 30.0

    DELTA_X = 4.3
    DELTA_Y = -0.1
    TARGET_TOLERANCE = 0.25
    MAX_SPEED = 1.5
    MAX_YAW_RATE = 0.6
    YAW_TOLERANCE = 0.25

    def __init__(self):
        super().__init__("flight_test")
        self.state = FlightState.WAITING_FOR_SYSTEM
        self.request_in_progress = False
        self.status = None
        self.odometry = None
        self.hover_started_at = None

        self.start_x = None
        self.start_y = None
        self.target_x = None
        self.target_y = None

        self._handlers = HANDLERS
        self.status_subscription = self.create_subscription(
            Status,
            "/ap/v1/status",
            self.status_callback,
            10,
        )
        self.odometry_subscription = self.create_subscription(
            Odometry,
            "/odometry",
            self.odometry_callback,
            10,
        )
        self.prearm_client = self.create_client(
            Trigger,
            "/ap/v1/prearm_check",
        )
        self.mode_client = self.create_client(
            ModeSwitch,
            "/ap/v1/mode_switch",
        )
        self.arm_client = self.create_client(
            ArmMotors,
            "/ap/v1/arm_motors",
        )
        self.takeoff_client = self.create_client(
            Takeoff,
            "/ap/v1/experimental/takeoff",
        )

        cmd_vel_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10,
        )
        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped, 
            "/ap/v1/cmd_vel", 
            cmd_vel_qos,
        )

        self.timer = self.create_timer(0.2, self.control_loop)
        self.get_logger().info("Flight test node started")
        self.get_logger().info("Waiting for ArduPilot and ROS services...")
    def status_callback(self, message):
        self.status = message
    def odometry_callback(self, message):
        self.odometry = message
    def services_ready(self):
        return (
            self.prearm_client.service_is_ready()
            and self.mode_client.service_is_ready()
            and self.arm_client.service_is_ready()
            and self.takeoff_client.service_is_ready()
        )
    def control_loop(self):
        handler = self._handlers.get(self.state)
        if handler is not None:
            handler.handle(self)
    def call_mode_switch(self, mode, callback):
        self.request_in_progress = True
        request = ModeSwitch.Request()
        request.mode = mode
        future = self.mode_client.call_async(request)
        future.add_done_callback(callback)
        self.get_logger().info(f"Requesting flight mode {mode}")
    def fail(self, message):
        self.get_logger().error(message)
        self.state = FlightState.ERROR