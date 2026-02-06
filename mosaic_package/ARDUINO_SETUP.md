# Arduino ROS2 Integration Guide

This guide explains how to connect your Arduino rotary sensor to ROS2.

## Setup Steps

### 1. Arduino Setup

1. **Upload the sketch**: Open `arduino_sketch_example.ino` in Arduino IDE and upload it to your Arduino
2. **Adjust pin configuration**: If your rotary sensor is connected to a different pin, modify `ROTARY_PIN` in the sketch
3. **Calibrate angle range**: Adjust `MIN_ANGLE` and `MAX_ANGLE` based on your sensor's range
4. **Verify connection**: Open Serial Monitor in Arduino IDE (9600 baud) to see if angle values are being sent

### 2. ROS2 Node Setup

The `arduino_reader` node reads angle data from Arduino and publishes it to `/arduino/rotary_angle` topic.

#### Check Arduino Port

Your Arduino is currently connected at: `/dev/ttyACM0`

To verify:
```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

#### Source ROS2 Environment

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

#### Run the Arduino Reader Node

**Basic usage** (uses default port `/dev/ttyACM0`):
```bash
ros2 run mosaic_package arduino_reader
```

**Custom serial port**:
```bash
ros2 run mosaic_package arduino_reader --ros-args -p serial_port:=/dev/ttyUSB0
```

**Custom baud rate**:
```bash
ros2 run mosaic_package arduino_reader --ros-args -p baud_rate:=115200
```

**Custom publish rate** (Hz):
```bash
ros2 run mosaic_package arduino_reader --ros-args -p publish_rate:=30.0
```

### 3. Verify Data Flow

In a new terminal, listen to the angle data:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /arduino/rotary_angle
```

You should see Float32 messages with the current angle:
```
data: 45.5
---
data: 46.2
---
```

### 4. Visualize the Data (Optional)

Plot the angle data in real-time:

```bash
ros2 run rqt_plot rqt_plot /arduino/rotary_angle/data
```

### 5. Integration with Other Nodes

Other nodes in your mosaic_package can subscribe to `/arduino/rotary_angle`:

```python
from std_msgs.msg import Float32

class YourNode(Node):
    def __init__(self):
        super().__init__('your_node')
        self.subscription = self.create_subscription(
            Float32,
            '/arduino/rotary_angle',
            self.angle_callback,
            10
        )

    def angle_callback(self, msg):
        angle = msg.data
        self.get_logger().info(f'Received angle: {angle}°')
        # Your logic here
```

## Troubleshooting

### Permission Denied Error

If you get a permission error when accessing the serial port:

```bash
sudo usermod -a -G dialout $USER
```

Then log out and log back in for the changes to take effect.

### Arduino Not Found

If the node can't find the Arduino, check available ports:

```bash
python3 -c "import serial.tools.list_ports; [print(p.device, '-', p.description) for p in serial.tools.list_ports.comports()]"
```

### Parsing Errors

If the node can't parse the angle data, check:
1. Arduino is sending just the number (e.g., "45.5")
2. Baud rate matches between Arduino sketch and ROS2 node (default: 9600)
3. Serial Monitor in Arduino IDE shows clean numeric output

## Node Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | `/dev/ttyACM0` | Arduino serial port path |
| `baud_rate` | `9600` | Serial communication baud rate |
| `timeout` | `1.0` | Serial read timeout in seconds |
| `publish_rate` | `20.0` | Rate to publish angle data (Hz) |
