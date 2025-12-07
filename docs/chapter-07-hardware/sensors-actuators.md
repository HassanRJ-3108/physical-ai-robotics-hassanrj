---
sidebar_position: 4
---

# Sensors and Actuators

Sensors and actuators are the fundamental components that allow robots to perceive and interact with the world. Choosing the right sensors and actuators for your application is crucial for success.

## Sensors: The Senses of the Robot

Sensors are the "eyes, ears, and skin" of a robot, allowing it to perceive its environment and its own internal state.

### Vision (Cameras)

Cameras are one of the most versatile sensors in robotics, providing rich information about the world.

**Types**:
- **USB Webcams**: Inexpensive, easy to use, good for prototyping
- **CSI Cameras**: Direct connection to Jetson/Raspberry Pi, low latency
- **Industrial Cameras**: High quality, reliable, expensive
- **Stereo Cameras**: Two cameras for depth perception

**Popular Models**:
- Logitech C920/C930e (USB webcam)
- Raspberry Pi Camera Module V2 (CSI)
- Intel RealSense D435i (RGB-D camera)
- ZED 2 (Stereo camera)

**Considerations**:
- Resolution (720p, 1080p, 4K)
- Frame rate (30fps, 60fps, 120fps)
- Field of view (narrow, wide, fisheye)
- Low-light performance
- Interface (USB, CSI, Ethernet)

### Depth Sensors (LiDAR, Depth Cameras)

LiDAR and depth cameras provide 3D information about the environment, essential for navigation and obstacle avoidance.

**2D LiDAR**:
- Scans in a single plane
- Good for indoor navigation
- Examples: RPLIDAR A1/A2, Hokuyo URG-04LX
- Price: $100-$1000

**3D LiDAR**:
- Full 3D point clouds
- Outdoor navigation, mapping
- Examples: Velodyne VLP-16, Ouster OS1
- Price: $4000-$100,000

**Depth Cameras**:
- RGB + Depth information
- Shorter range than LiDAR (0.5-10m)
- Examples: Intel RealSense D435i, Kinect Azure
- Price: $200-$400

### Inertia (IMUs)

Inertial Measurement Units measure the robot's orientation and motion.

**Components**:
- 3-axis accelerometer
- 3-axis gyroscope
- 3-axis magnetometer (9-DOF IMU)

**Popular Models**:
- MPU-6050 (6-DOF, budget)
- BNO055 (9-DOF, good fusion)
- VectorNav VN-100 (high-end)

**Applications**:
- Balance control
- Dead reckoning
- Sensor fusion with GPS/cameras
- Vibration monitoring

### Position (GPS, Encoders)

**GPS**:
- Outdoor localization
- Accuracy: 2-10m (consumer), cm-level (RTK)
- Examples: u-blox NEO-M8N, ZED-F9P (RTK)

**Encoders**:
- Measure motor/wheel rotation
- Types: Incremental, absolute, magnetic
- Essential for odometry

### Force and Touch (Tactile Sensors)

**Force/Torque Sensors**:
- Measure forces at robot joints or end-effectors
- Enable force-controlled manipulation
- Examples: ATI Mini40, Robotiq FT 300

**Tactile Arrays**:
- Distributed pressure sensing
- Detect contact location and force
- Research-grade, expensive

## Actuators: The Muscles of the Robot

Actuators convert energy into physical motion.

### Electric Motors

The most common type of actuator in robotics.

**DC Motors**:
- Simple speed control
- Require gearbox for torque
- Inexpensive
- Examples: Pololu micro motors

**Brushless DC Motors (BLDC)**:
- More efficient than brushed
- Longer lifespan
- Higher performance
- Require ESC (Electronic Speed Controller)
- Examples: DJI motors, T-Motor

**Servo Motors**:
- Built-in position control
- Easy to use
- Limited rotation (typically 180°) or continuous
- Examples: Dynamixel, Hitec, Futaba

**Stepper Motors**:
- Precise positioning
- No feedback sensor needed
- Lower speed than DC motors
- Examples: NEMA 17, NEMA 23

### Hydraulic and Pneumatic Actuators

**Hydraulic**:
- Very high force
- Used in heavy-duty applications
- Complex, expensive
- Examples: Boston Dynamics Atlas uses hydraulics

**Pneumatic**:
- Fast actuation
- Compliant, safe
- Requires air compressor
- Examples: Soft robotic grippers

### Motor Controllers

Motors require controllers to operate:

**H-Bridge**: For DC motor direction control

**ESC**: For brushless motors

**Servo Controller**: For multiple servos (e.g., Maestro)

**Motor Driver ICs**: L298N, DRV8825, TB6612FNG

## Selection Criteria

### For Sensors

1. **Range**: How far does it need to sense?
2. **Accuracy**: How precise must measurements be?
3. **Update Rate**: How fast does data need to refresh?
4. **Environment**: Indoor/outdoor, lighting, weather
5. **Interface**: USB, I2C, SPI, Ethernet
6. **Power**: Consumption and supply voltage
7. **Cost**: Budget constraints

### For Actuators

1. **Torque/Force**: How much force is needed?
2. **Speed**: How fast must it move?
3. **Precision**: How accurate must positioning be?
4. **Size/Weight**: Physical constraints
5. **Power**: Voltage and current requirements
6. **Control**: Position, velocity, or torque control
7. **Cost**: Budget and quantity needed

## Integration Tips

**Start Simple**: Begin with basic sensors and actuators

**Test Individually**: Verify each component works before integration

**Power Carefully**: Separate power for motors and electronics

**Shield Cables**: Reduce electromagnetic interference

**Document**: Keep track of wiring and configurations

Choosing the right sensors and actuators is fundamental to building effective robots. Start with your application requirements and select components that meet your needs within your budget.
