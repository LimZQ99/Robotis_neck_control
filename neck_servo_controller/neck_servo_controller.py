
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS


class NeckServoController(Node):

    # XM430-W350-T control table
    ADDR_TORQUE_ENABLE = 64
    ADDR_OPERATING_MODE = 11
    ADDR_GOAL_POSITION = 116
    ADDR_PROFILE_VELOCITY = 112

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

            # Subscribe to angle values
            self.subscription = self.create_subscription(
                Float32,
                "/neck_angle",
                self.command_callback,
                10
            )

            self.get_logger().info(
                "Neck controller ready. "
                "Listening to /neck_angle (Float32)"
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

    def angle_to_tick(self, angle):

        tick = self.HOME_TICK + round(
            self.DIRECTION * angle * 4096 / 360
        )

        if not 0 <= tick <= 4095:
            raise ValueError(
                f"Target tick {tick} is outside position range"
            )

        return tick

    def command_callback(self, msg):

        angle = float(msg.data)

        # Check angle limits
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

    def shutdown(self):

        if self.connected:

            if self.torque_enabled:
                try:
                    self.write1(self.ADDR_TORQUE_ENABLE, 0)

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
