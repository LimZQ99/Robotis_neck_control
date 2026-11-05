
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS


class NeckServoController(Node):

    # XM430-W350-T Control Table
    ADDR_TORQUE_ENABLE = 64
    ADDR_OPERATING_MODE = 11
    ADDR_GOAL_POSITION = 116
    ADDR_PROFILE_VELOCITY = 112
    ADDR_PRESENT_POSITION = 132

    # Servo configuration
    DXL_ID = 1
    BAUDRATE = 115200
    PORT = "/dev/ttyUSB0"

    # Position configuration
    HOME_TICK = 2048
    DIRECTION = 1

    # Angle safety limits (degrees relative to home)
    MIN_ANGLE = -180.0
    MAX_ANGLE = 180.0

    # Encoder configuration
    TICKS_PER_REVOLUTION = 4096

    # Joint configuration
    JOINT_NAME = "head_joint"

    # Publishing frequency
    PUBLISH_RATE = 50.0

    def __init__(self):
        super().__init__("neck_servo_controller")

        self.port = PortHandler(self.PORT)
        self.packet = PacketHandler(2.0)

        self.connected = False
        self.torque_enabled = False

        if not self.port.openPort():
            raise RuntimeError("Failed to open serial port")

        self.connected = True

        try:
            if not self.port.setBaudRate(self.BAUDRATE):
                raise RuntimeError("Failed to set baud rate")

            model, result, error = self.packet.ping(
                self.port, self.DXL_ID
            )

            self.check_result(result, error)

            self.get_logger().info(
                f"Dynamixel connected, model={model}"
            )

            # Disable torque before changing operating mode
            self.write1(self.ADDR_TORQUE_ENABLE, 0)

            # Position Control Mode
            self.write1(self.ADDR_OPERATING_MODE, 3)

            # Set profile velocity
            self.write4(self.ADDR_PROFILE_VELOCITY, 30)

            # Enable torque
            self.write1(self.ADDR_TORQUE_ENABLE, 1)
            self.torque_enabled = True

            # Subscribe to neck angle commands
            self.subscription = self.create_subscription(
                Float32,
                "/neck_angle",
                self.command_callback,
                10
            )

            # Publish actual joint states
            self.joint_state_pub = self.create_publisher(
                JointState,
                "/joint_states",
                10
            )

            # Timer for reading actual servo position
            self.joint_state_timer = self.create_timer(
                1.0 / self.PUBLISH_RATE,
                self.publish_joint_state
            )

            self.get_logger().info(
                "Neck controller ready.\n"
                "Listening: /neck_angle (Float32)\n"
                "Publishing: /joint_states (JointState)\n"
                f"Joint name: {self.JOINT_NAME}\n"
                f"Publishing rate: {self.PUBLISH_RATE} Hz"
            )

        except Exception:
            self.shutdown()
            raise

    def check_result(self, result, error):

        if result != COMM_SUCCESS:
            raise RuntimeError(
                self.packet.getTxRxResult(result)
            )

        if error != 0:
            raise RuntimeError(
                self.packet.getRxPacketError(error)
            )

    def write1(self, address, value):

        result, error = self.packet.write1ByteTxRx(
            self.port, self.DXL_ID, address, value
        )

        self.check_result(result, error)

    def write4(self, address, value):

        result, error = self.packet.write4ByteTxRx(
            self.port, self.DXL_ID, address, value
        )

        self.check_result(result, error)

    def read4(self, address):

        value, result, error = self.packet.read4ByteTxRx(
            self.port, self.DXL_ID, address
        )

        self.check_result(result, error)

        return value

    def angle_to_tick(self, angle):

        tick = self.HOME_TICK + round(
            self.DIRECTION * angle *
            self.TICKS_PER_REVOLUTION / 360.0
        )

        if not 0 <= tick <= 4095:
            raise ValueError(
                f"Target tick {tick} is outside position range"
            )

        return tick

    def tick_to_angle(self, tick):

        # Convert encoder ticks into degrees relative to home
        angle = (
            (tick - self.HOME_TICK)
            * 360.0 / self.TICKS_PER_REVOLUTION
        )

        angle *= self.DIRECTION

        return angle

    def command_callback(self, msg):

        angle = float(msg.data)

        if not self.MIN_ANGLE <= angle <= self.MAX_ANGLE:
            self.get_logger().warn(
                f"Angle {angle} is outside allowed limits "
                f"({self.MIN_ANGLE} to {self.MAX_ANGLE})"
            )
            return

        try:
            goal_tick = self.angle_to_tick(angle)

            self.write4(
                self.ADDR_GOAL_POSITION,
                goal_tick
            )

            self.get_logger().info(
                f"Moving neck: "
                f"angle={angle:.2f} deg, "
                f"tick={goal_tick}"
            )

        except Exception as e:
            self.get_logger().error(
                f"Failed to move neck: {e}"
            )

    def publish_joint_state(self):

        try:
            # Read actual servo position
            present_tick = self.read4(
                self.ADDR_PRESENT_POSITION
            )

            # Convert ticks to degrees
            angle_deg = self.tick_to_angle(present_tick)

            # Convert degrees to radians for ROS
            angle_rad = angle_deg * 3.141592653589793 / 180.0

            # Create JointState message
            msg = JointState()

            msg.header.stamp = self.get_clock().now().to_msg()

            msg.name = [self.JOINT_NAME]
            msg.position = [angle_rad]

            # Velocity and effort are not measured here
            msg.velocity = []
            msg.effort = []

            self.joint_state_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(
                f"Failed to read/publish joint state: {e}",
                throttle_duration_sec=2.0
            )

    def shutdown(self):

        if self.connected:

            if self.torque_enabled:
                try:
                    self.write1(
                        self.ADDR_TORQUE_ENABLE, 0
                    )

                except Exception as e:
                    self.get_logger().error(str(e))

                self.torque_enabled = False

            self.port.closePort()
            self.connected = False


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:
        node = NeckServoController()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(f"Controller error: {e}")

    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
