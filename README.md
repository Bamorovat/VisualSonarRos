# Visual Sonar ROS v2.0

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)](https://github.com/Bamorovat/VisualSonarRos)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS Version](https://img.shields.io/badge/ROS-Kinetic%20%7C%20Melodic%20%7C%20Noetic-blue.svg)](http://wiki.ros.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-3.0%2B-green.svg)](https://opencv.org/)
[![C++](https://img.shields.io/badge/C%2B%2B-14-blue.svg)](https://en.cppreference.com/w/cpp/14)
[![Platform](https://img.shields.io/badge/Platform-Ubuntu%2016.04%2B-orange.svg)](https://ubuntu.com/)
[![CMake](https://img.shields.io/badge/CMake-3.0.2%2B-red.svg)](https://cmake.org/)
[![Algorithms](https://img.shields.io/badge/Edge%20Detection-6%20Algorithms-purple.svg)](#-multiple-edge-detection-algorithms)
[![Real-time](https://img.shields.io/badge/Performance-19.6%20FPS-brightgreen.svg)](#-performance-optimization)
[![Research](https://img.shields.io/badge/Research-IEEE%20Papers-blue.svg)](#-scientific-background)
[![Stars](https://img.shields.io/github/stars/Bamorovat/VisualSonarRos?style=social)](https://github.com/Bamorovat/VisualSonarRos/stargazers)
[![Forks](https://img.shields.io/github/forks/Bamorovat/VisualSonarRos?style=social)](https://github.com/Bamorovat/VisualSonarRos/network/members)

## Advanced Mobile Robot Navigation Using Omnidirectional Vision and Virtual Sonar

**Author:** [Mohammad Hossein Bamorovat Abadi](https://bamorovat.com)  
**Email:** m.bamorovvat@gmail.com  
**Website:** https://bamorovat.com  
**Project Page:** https://bamorovat.com/projects/visual-sonar.html  
**Version:** 2.0.0  

---

## **Overview**

Visual Sonar ROS is a sophisticated navigation system for mobile robots that uses **omnidirectional vision** to simulate **sonar-like obstacle detection**. Instead of relying on physical sonar sensors, this system processes camera images with advanced computer vision algorithms to detect obstacles and compute safe navigation paths.

### **Scientific Background**

This implementation is based on peer-reviewed research published in multiple IEEE conferences:

1. **Bamorovat Abadi, M.H., Asghari Oskoei, M.** *"Effects of Mirrors in Mobile Robot Navigation Based on Omnidirectional Vision."* 8th International Conference, ICIRA 2015, Portsmouth, UK, Springer International Publishing, 2015. [[**PDF**]](https://bamorovat.files.wordpress.com/2017/01/intelligentroboticsandapplications.pdf) [[**BibTex**]](https://bamorovat.wordpress.com/bibtex1/)

2. **Bamorovat Abadi, M.H., Asghari Oskoei, M., Fakharian, A.** *"Mobile robot navigation using sonar vision algorithm applied to omnidirectional vision."* AI & Robotics (IRANOPEN), 2015, IEEE, pp. 1-6, 2015. [[**PDF**]](https://bamorovat.files.wordpress.com/2017/01/the-7th-robocup-iranopen-international-symposium-and-the-5th-joint-conference-of-ai-robotics.pdf) [[**BibTex**]](https://bamorovat.wordpress.com/bibtex2/)

3. **Bamorovat Abadi, M.H., Asghari Oskoei, M., Fakharian, A.** *"Side Sonar Vision Applied to Omni-directional Images to Navigate Mobile Robots."* 16th Conference on Fuzzy Systems and 14th Conference on Intelligent Systems, IEEE, 2017. [[**PDF**]](https://bamorovat.files.wordpress.com/2017/08/side-sonar-vision-applied-to-omni-directional-images-to-navigate.pdf) [[**BibTex**]](https://bamorovat.wordpress.com/bibtext3/)

### **Demo Video**
Watch our robot in action: **[Robot Navigation Demo](https://youtu.be/JKRwDcHyVbo)**

---

## **Key Features (New in v2.0)**

### **Multiple Edge Detection Algorithms**
- **Sobel** - Robust general-purpose edge detection
- **Canny** - Precise edges with excellent noise handling  
- **Laplacian** - Fast detection for high-contrast scenarios
- **Scharr** - Improved rotation invariance over Sobel
- **Roberts** - Simple and fast for basic edge detection
- **Prewitt** - Alternative gradient operator

### **Advanced Processing Pipeline**
- **Light Reflection Elimination** - HSV-based analysis to reduce false positives
- **Virtual Sonar Beams** - Configurable number and positioning (default: 24 beams)
- **Graduated Speed Control** - Dynamic velocity adjustment based on obstacle proximity
- **Safety Mechanisms** - Emergency stop and velocity limiting

### **Modern Architecture**
- **Object-Oriented Design** - Clean, maintainable C++14 code
- **Namespace Organization** - Proper code organization and conflict prevention
- **Parameter Server Integration** - Runtime configuration via ROS parameters
- **Comprehensive Error Handling** - Robust operation with graceful degradation
- **Performance Monitoring** - Real-time statistics and diagnostics

### **Professional Features**
- **Real-time Visualization** - Debug images with navigation vectors and statistics
- **Multiple Configuration Scenarios** - Optimized presets for indoor/outdoor/low-light conditions
- **Thread-Safe Operations** - Proper concurrent processing
- **Memory Efficient** - Optimized resource usage with RAII principles

---

## **Requirements**

### **System Requirements**
- **Operating System:** Ubuntu 16.04+ (tested on 16.04, 18.04, 20.04)
- **ROS Versions:** Kinetic, Melodic, or Noetic
- **Compiler:** GCC 5.4+ with C++14 support
- **Memory:** Minimum 2GB RAM (4GB+ recommended)
- **Storage:** ~100MB for package installation

### **Dependencies**
- **ROS Core Packages:**
  - `roscpp` - ROS C++ client library
  - `sensor_msgs` - Standard sensor message definitions
  - `geometry_msgs` - Geometry-related message definitions
  - `cv_bridge` - OpenCV-ROS image conversion
  - `image_transport` - Efficient image publishing/subscribing

- **Computer Vision:**
  - **OpenCV 3.0+** - Image processing and computer vision
  - **OpenCV Contrib** (optional) - Additional algorithms

- **Build System:**
  - **CMake 3.0.2+** - Modern build system
  - **catkin** - ROS build tools

---

## **Installation**

> [!IMPORTANT]
> Ensure you have Ubuntu 16.04+ and at least 4GB RAM for optimal performance.

### **1. Prerequisites**

> [!WARNING]
> ROS installation requires sudo privileges and internet connection. This process may take 30-60 minutes.

**Install ROS (if not already installed):**
```bash
# For Ubuntu 18.04 (ROS Melodic)
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install ros-melodic-desktop-full
```

**Initialize ROS Environment:**
```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo rosdep init
rosdep update
```

### **2. Create Catkin Workspace**
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### **3. Clone and Build Visual Sonar ROS**

> [!TIP]
> Use `git clone --depth 1` for faster download if you don't need the full commit history.

```bash
cd ~/catkin_ws/src
git clone https://github.com/Bamorovat/VisualSonarRos.git visual_sonar_ros
cd ~/catkin_ws
catkin_make
```

> [!NOTE]
> Build time typically takes 2-5 minutes depending on your system specifications.

### **4. Install Additional Dependencies**
```bash
# Install OpenCV (if not included with ROS)
sudo apt install libopencv-dev libopencv-contrib-dev

# Install other dependencies
rosdep install --anonymous --from-paths src --ignore-src --rosdistro melodic -y
```

---

## 🎮 **Usage**

> [!IMPORTANT]
> Always ensure a camera is connected and publishing images before starting the Visual Sonar node.

### **Basic Operation**

**1. Start ROS Core:**
```bash
roscore
```

**2. Launch Camera Node (Example with USB camera):**

> [!TIP]
> Test your camera first with `lsusb` and `ls /dev/video*` to verify it's detected.

```bash
# Install usb_cam if needed
sudo apt install ros-melodic-usb-cam

# Launch camera node
roslaunch usb_cam usb_cam-test.launch
```

**3. Run Visual Sonar Node:**
```bash
rosrun visual_sonar_ros visual_sonar_ros_node
```

> [!NOTE]
> The node will automatically start processing images and publishing navigation commands.

### **Advanced Usage with Parameters**

> [!TIP]
> Use `rosparam list` to see all available parameters and `rosparam get /visual_sonar/param_name` to check current values.

**Launch with custom parameters:**
```bash
rosrun visual_sonar_ros visual_sonar_ros_node \
  _num_sonars:=24 \
  _first_sonar:=10 \
  _last_sonar:=27 \
  _start_point:=60 \
  _edge_algorithm:=canny \
  _debug_mode:=true
```

> [!WARNING]
> Debug mode significantly increases CPU usage and should only be enabled for development.

---

## 🔧 **Configuration**

### **Core Parameters**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `num_sonars` | int | 24 | Number of virtual sonar beams |
| `first_sonar` | int | 10 | Index of first active sonar |
| `last_sonar` | int | 27 | Index of last active sonar |
| `start_point` | int | 60 | Starting radius for sonar beams |
| `max_range` | double | 300.0 | Maximum sonar detection range |
| `edge_algorithm` | string | "canny" | Edge detection algorithm |
| `debug_mode` | bool | false | Enable debug visualization |

### **Edge Detection Algorithms**

> [!TIP]
> Start with `canny` for general use, then switch to `laplacian` for better performance or `sobel` for noisy environments.

| Algorithm | Best For | Parameters |
|-----------|----------|------------|
| `canny` | General use, precise edges | `low_threshold: 50`, `high_threshold: 150` |
| `sobel` | Robust, noise-tolerant | `kernel_size: 3`, `scale: 1.0` |
| `laplacian` | High contrast scenarios | `kernel_size: 3` |
| `scharr` | Rotation invariant | `scale: 1.0`, `delta: 0.0` |
| `roberts` | Fast, simple edges | `threshold: 128` |
| `prewitt` | Alternative gradient | `threshold: 128` |

> [!NOTE]
> Algorithm performance varies with lighting conditions. Test different algorithms in your specific environment.

### **Navigation Parameters**

> [!CAUTION]
> Always test navigation parameters in a safe environment before deploying on actual robots.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_linear_vel` | double | 0.5 | Maximum forward velocity (m/s) |
| `max_angular_vel` | double | 1.0 | Maximum angular velocity (rad/s) |
| `safety_distance` | double | 0.3 | Minimum obstacle distance (m) |
| `emergency_stop_distance` | double | 0.15 | Emergency stop threshold (m) |
| `speed_reduction_factor` | double | 0.8 | Speed reduction near obstacles |

> [!IMPORTANT]
> The `emergency_stop_distance` should always be less than `safety_distance` for proper safety operation.

### **Light Reflection Parameters**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `hue_tolerance` | int | 15 | HSV hue tolerance for reflection detection |
| `saturation_threshold` | int | 30 | Minimum saturation for real obstacles |
| `value_threshold` | int | 50 | Minimum brightness for obstacle detection |
| `enable_hsv_analysis` | bool | true | Enable HSV-based reflection filtering |

---

## **ROS Interface**

### **Subscribed Topics**

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Input camera images |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera calibration data |

### **Published Topics**

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Navigation commands |
| `/visual_sonar/debug_image` | `sensor_msgs/Image` | Debug visualization |
| `/visual_sonar/sonar_data` | `sensor_msgs/LaserScan` | Virtual sonar measurements |
| `/visual_sonar/statistics` | `std_msgs/String` | Performance statistics |

### **Services**

| Service | Type | Description |
|---------|------|-------------|
| `/visual_sonar/set_algorithm` | `visual_sonar_ros/SetAlgorithm` | Change edge detection algorithm |
| `/visual_sonar/emergency_stop` | `std_srvs/Trigger` | Emergency stop command |
| `/visual_sonar/reset_navigation` | `std_srvs/Trigger` | Reset navigation state |

---

## **Testing and Validation**

### **Unit Tests**
```bash
# Run all tests
cd ~/catkin_ws
catkin_make run_tests visual_sonar_ros

# Run specific test categories
rostest visual_sonar_ros edge_detection_test.launch
rostest visual_sonar_ros navigation_test.launch
```


## **Troubleshooting**

> [!TIP]
> Enable debug mode first to visualize what the system is seeing: `_debug_mode:=true`

### **Common Issues**

**1. No Image Received**

> [!WARNING]
> Check camera permissions before proceeding. USB cameras often need proper permissions.

```bash
# Check if camera topic is publishing
rostopic list | grep camera
rostopic hz /camera/image_raw

# Verify camera permissions
ls -la /dev/video*
sudo chmod 666 /dev/video0
```

**2. High CPU Usage**

> [!NOTE]
> CPU usage above 80% may cause frame drops and degraded performance.

- Reduce image resolution in camera launch file
- Use faster edge detection algorithm (laplacian/roberts)
- Decrease number of sonar beams
- Enable debug mode only when needed

**3. Navigation Oscillations**

> [!CAUTION]
> Oscillating behavior can damage robot motors. Stop the robot and adjust parameters immediately.

- Increase safety distance parameter
- Reduce maximum velocities
- Adjust speed reduction factor
- Check for light reflections causing false obstacles

**4. Memory Leaks**

> [!TIP]
> Use `htop` or `rosrun rqt_top rqt_top` to monitor memory usage in real-time.

- Monitor with: `rosrun rqt_top rqt_top`
- Check debug images are properly released
- Verify OpenCV Mat objects are not accumulating

### **Debug Mode**

> [!IMPORTANT]
> Debug mode creates additional ROS topics and significantly increases resource usage.

Enable comprehensive debugging:
```bash
rosrun visual_sonar_ros visual_sonar_ros_node _debug_mode:=true _log_level:=DEBUG
```

Debug output includes:
- **Processing Statistics** - Frame rate, timing, memory usage
- **Algorithm Performance** - Edge detection quality, reflection analysis
- **Navigation Decisions** - Velocity commands, obstacle distances
- **Visual Overlays** - Sonar beams, detected edges, navigation vectors

> [!TIP]
> View debug images with: `rosrun image_view image_view image:=/visual_sonar/debug_image`

---

## **Performance Optimization**

### **Real-time Performance Tips**

**1. Camera Configuration:**
```bash
# Optimize camera parameters for performance
rosrun dynamic_reconfigure dynparam set /usb_cam_node image_width 320
rosrun dynamic_reconfigure dynparam set /usb_cam_node image_height 240
rosrun dynamic_reconfigure dynparam set /usb_cam_node framerate 15
```

**2. Algorithm Selection:**
- **Indoor environments:** Use `canny` or `sobel`
- **Outdoor bright conditions:** Use `laplacian` or `roberts`
- **Low-light conditions:** Use `scharr` or `sobel`
- **High-speed navigation:** Use `roberts` or `laplacian`

**3. Parameter Tuning:**

> [!NOTE]
> These configurations are starting points. Fine-tune based on your specific robot and environment.

```yaml
# Optimized parameters for different scenarios
indoor_config:
  edge_algorithm: "canny"
  num_sonars: 24
  max_range: 250.0
  debug_mode: false

outdoor_config:
  edge_algorithm: "laplacian" 
  num_sonars: 18
  max_range: 400.0
  enable_hsv_analysis: true

high_speed_config:
  edge_algorithm: "roberts"
  num_sonars: 16
  max_range: 300.0
  max_linear_vel: 0.8
```

> [!CAUTION]
> High-speed configuration requires extensive testing and safety measures.

---

## **Algorithm Details**

### **Visual Sonar Principle**

The Visual Sonar algorithm converts omnidirectional images into distance measurements similar to sonar sensors:

1. **Image Preprocessing:** Apply edge detection to identify potential obstacles
2. **Beam Simulation:** Cast virtual rays from robot center at regular angular intervals
3. **Distance Calculation:** For each beam, find the first significant edge representing an obstacle
4. **Light Reflection Filtering:** Use HSV analysis to distinguish real obstacles from reflections
5. **Navigation Vector Computation:** Convert distance measurements to velocity commands

### **Mathematical Foundation**

**Sonar Beam Equations:**
```
Beam_angle(i) = (2π × i) / num_sonars
Beam_x(r, i) = r × cos(Beam_angle(i))
Beam_y(r, i) = r × sin(Beam_angle(i))
Distance(i) = min(r) where Edge_detected(Beam_x(r,i), Beam_y(r,i)) = true
```

**Navigation Vector Calculation:**
```
Linear_velocity = max_vel × (1 - obstacle_proximity_factor)
Angular_velocity = weighted_sum(beam_distances × beam_angles) / total_weight
Emergency_stop = any(beam_distance < emergency_threshold)
```

### **Edge Detection Comparison**

**Sobel Operator:**
- Gradient-based edge detection
- Good noise tolerance
- Computationally efficient
- Formula: `G = √(Gx² + Gy²)`

**Canny Algorithm:**
- Multi-stage edge detection
- Excellent precision with hysteresis thresholding
- More computationally intensive
- Steps: Gaussian blur → Gradient → Non-maximum suppression → Hysteresis

**Laplacian Method:**
- Second-derivative edge detection
- Very fast processing
- Sensitive to noise
- Formula: `∇²f = ∂²f/∂x² + ∂²f/∂y²`

---

## **Use Cases and Applications**

### **Autonomous Navigation**
- **Warehouse Robots:** Navigate between shelves and avoid obstacles
- **Service Robots:** Indoor navigation in offices, hospitals, homes
- **Agricultural Robots:** Crop field navigation with row detection
- **Security Patrol:** Autonomous surveillance with obstacle avoidance

### **Research Applications**
- **Vision-Based SLAM:** Integration with simultaneous localization and mapping
- **Multi-Robot Systems:** Cooperative navigation with visual sonar
- **Human-Robot Interaction:** Safe navigation around people
- **Outdoor Exploration:** Terrain mapping and obstacle detection

### **Educational Projects**
- **Computer Vision Learning:** Practical implementation of edge detection algorithms
- **ROS Development:** Understanding message passing and node communication
- **Robot Control:** Real-time sensor processing and actuator control
- **Algorithm Comparison:** Empirical evaluation of different approaches

---

## **Citation**

If you use **Visual Sonar ROS** in your research, please cite the original papers:

```bibtex
@inproceedings{bamorovat2015effects,
  title={Effects of Mirrors in Mobile Robot Navigation Based on Omnidirectional Vision},
  author={Bamorovat Abadi, Mohammad Hossein and Asghari Oskoei, Mohsen},
  booktitle={Intelligent Robotics and Applications: 8th International Conference, ICIRA 2015},
  pages={1--12},
  year={2015},
  publisher={Springer International Publishing},
  address={Portsmouth, UK}
}

@inproceedings{bamorovat2015mobile,
  title={Mobile robot navigation using sonar vision algorithm applied to omnidirectional vision},
  author={Bamorovat Abadi, Mohammad Hossein and Asghari Oskoei, Mohsen and Fakharian, Ahmad},
  booktitle={AI \& Robotics (IRANOPEN), 2015},
  pages={1--6},
  year={2015},
  organization={IEEE}
}

@inproceedings{bamorovat2017side,
  title={Side Sonar Vision Applied to Omni-directional Images to Navigate Mobile Robots},
  author={Bamorovat Abadi, Mohammad Hossein and Asghari Oskoei, Mohsen and Fakharian, Ahmad},
  booktitle={16th Conference on Fuzzy Systems and 14th Conference on Intelligent Systems},
  year={2017},
  organization={IEEE}
}
```

---

## **Contributing**

We welcome contributions! Please see our [Contributing Guidelines](CONTRIBUTING.md) for details.

### **Development Setup**

> [!TIP]
> Always create a feature branch for your contributions to keep the main branch clean.

```bash
# Fork the repository and clone your fork
git clone https://github.com/YOUR_USERNAME/VisualSonarRos.git
cd VisualSonarRos

# Create development branch
git checkout -b feature/your-feature-name

# Make changes and run tests
catkin_make run_tests

# Submit pull request
```

> [!NOTE]
> Please ensure all tests pass before submitting a pull request.

### **Code Style**
- Follow [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
- Use meaningful variable names and comprehensive comments  
- Include unit tests for new functionality
- Update documentation for API changes

---

## **License**

This project is licensed under the **MIT License** 

## **Support and Contact**

- **Author:** Mohammad Hossein Bamorovat Abadi
- **Email:** m.bamorovvat@gmail.com
- **Website:** https://bamorovat.com
- **Project Page:** https://bamorovat.com/projects/visual-sonar.html
- **Research Group:** https://bamorovat.wordpress.com
- **Issues:** [GitHub Issues](https://github.com/Bamorovat/VisualSonarRos/issues)
- **Discussions:** [GitHub Discussions](https://github.com/Bamorovat/VisualSonarRos/discussions)

---

## **Version History**

### **v2.0.0** (2024)
- **Major Rewrite:** Complete modernization to C++14 standards
- **Multi-Algorithm Support:** 6 edge detection algorithms (Sobel, Canny, Laplacian, Scharr, Roberts, Prewitt)
- **Enhanced Safety:** Emergency stop mechanisms and graduated speed control
- **Advanced Light Filtering:** HSV-based reflection analysis
- **Modern Architecture:** Object-oriented design with namespace organization
- **ROS Integration:** Parameter server support and comprehensive diagnostics
- **Performance Optimization:** Memory efficient with real-time processing
- **Critical Bug Fix:** Fixed XOR power calculation error in navigation
- **Complete Documentation:** Comprehensive README with examples and troubleshooting

### **v1.0.0** (2015-2017)
- **Initial Release:** Basic visual sonar implementation
- **Core Features:** Sobel edge detection and basic navigation
- **Research Foundation:** Based on published IEEE papers
- **ROS Support:** Basic ROS node implementation

---

**Star this repository if Visual Sonar ROS helps your project!**

**For the latest updates and research, visit [bamorovat.com](https://bamorovat.com)**
