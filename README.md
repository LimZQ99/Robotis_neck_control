# neck_servo_controller

ROS 2 node that drives a robot neck with a single Robotis Dynamixel **XM430-W350-T** servo. It subscribes to `/neck_angle` and moves the servo to a preset angle.

## Hardware

- Dynamixel XM430-W350-T, ID `1`, Protocol 2.0
- USB-to-Dynamixel adapter (e.g. U2D2) on `/dev/ttyUSB0`
- Baud rate `57600`
- External servo power supply (check the XM430 datasheet for voltage)

## Dependencies

- ROS 2 (`rclpy`, `std_msgs`)
- [Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) for Python:

```bash
pip install dynamixel-sdk
# or: sudo apt install ros-$ROS_DISTRO-dynamixel-sdk
```

Serial port access:

```bash
sudo usermod -aG dialout $USER   # log out and back in afterwards
```

## Build

From your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/LimZQ99/Robotis_neck_control.git neck_servo_controller
cd ~/ros2_ws
colcon build --packages-select neck_servo_controller
source install/setup.bash
```

## Usage

Check the servo responds:

```bash
python3 test_connection.py
```

Start the controller:

```bash
ros2 run neck_servo_controller neck_servo_controller
```

Send a command (the message type is `std_msgs/String`, not a number):

```bash
ros2 topic pub --once /neck_angle std_msgs/msg/String "{data: '30'}"
```

### Commands

| Command | Angle from home | Goal tick |
|---------|-----------------|-----------|
| `"0"`   | 0°              | 2048      |
| `"30"`  | 30°             | 2389      |
| `"45"`  | 45°             | 2560      |
| `"90"`  | 15°             | 2219      |

Unknown commands are logged as a warning and ignored.

> Note: `"90"` currently maps to **15°**, not 90°. Change `PRESET_ANGLES` if that is not intended.

## Configuration

All settings are class constants in `neck_servo_controller/neck_servo_controller.py`:

| Constant           | Default        | Meaning                                         |
|--------------------|----------------|-------------------------------------------------|
| `PORT`             | `/dev/ttyUSB0` | Serial device                                   |
| `BAUDRATE`         | `57600`        | Must match the servo's configured baud rate     |
| `DXL_ID`           | `1`            | Servo ID                                        |
| `HOME_TICK`        | `2048`         | Position (0–4095) treated as 0°                 |
| `DIRECTION`        | `1`            | Set to `-1` to reverse rotation                 |
| `PRESET_ANGLES`    | see above      | Command string → angle in degrees               |
| Profile velocity | `30` | Movement speed, hardcoded in `__init__`      |

Angles are converted as `tick = HOME_TICK + DIRECTION * angle * 4096 / 360`. Targets outside 0–4095 are rejected.

## Behaviour

On startup the node pings the servo, sets **position control mode** (operating mode 3), sets profile velocity, and enables torque. On shutdown (Ctrl+C) it disables torque and closes the port, so the neck will go limp.

## Troubleshooting

- **`Failed to open serial port`**: wrong `PORT`, adapter not plugged in, or missing `dialout` permission. Under WSL2, attach the USB device with `usbipd`.
- **Ping fails / `[TxRxResult] There is no status packet!`**: baud rate or ID mismatch, or servo unpowered. Use Dynamixel Wizard 2.0 to check the servo's settings.
- `test_connection.py` uses baud `115200` while the node uses `57600`; make them match your servo.

## Tests

```bash
colcon test --packages-select neck_servo_controller
colcon test-result --verbose
```

These are the default ament checks (copyright, flake8, pep257).
