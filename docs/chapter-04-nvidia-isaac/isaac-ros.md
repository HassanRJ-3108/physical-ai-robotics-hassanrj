---
sidebar_position: 2
---

# Isaac ROS

Isaac ROS is a collection of GPU-accelerated ROS 2 packages that provide a significant performance boost for common robotics tasks. These packages are designed to be a drop-in replacement for their standard ROS 2 counterparts, making it easy to accelerate existing ROS 2 applications.

## Perception Packages

Isaac ROS provides a suite of packages for robot perception:

**Visual SLAM**
- High-performance package for visual simultaneous localization and mapping
- Uses stereo cameras and IMU
- Estimates robot pose and builds environment maps

**Depth Estimation**
- Estimates depth from stereo camera pairs
- GPU-accelerated for real-time performance

**Object Detection**
- Detects and localizes objects using deep neural networks
- Supports popular models like YOLO, SSD
- Optimized for NVIDIA GPUs

## Navigation Packages

Isaac ROS also includes packages for robot navigation:

**Proximity Segmentation**
- Segments point clouds into ground and non-ground points
- Useful for obstacle detection
- Real-time performance

**AprilTag Navigation**
- Navigate using AprilTag visual fiducial markers
- Accurate localization
- Easy to deploy

## Performance Benefits

Isaac ROS packages provide:
- **10-100x speedup** over CPU implementations
- **Real-time performance** for complex algorithms
- **Lower latency** for time-critical applications
- **Reduced power consumption** compared to CPU-only solutions

Isaac ROS makes high-performance robotics accessible to everyone.
