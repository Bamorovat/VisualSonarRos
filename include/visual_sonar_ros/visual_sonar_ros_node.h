/**
 * @file visual_sonar_ros_node.h
 * @brief Header file for the main Visual Sonar ROS node
 * 
 * This header defines the main ROS node class for Visual Sonar processing,
 * integrating all components including edge detection, visual sonar processing,
 * navigation control, and ROS communication for autonomous robot navigation.
 * 
 * @author Mohammad Hossein Bamorovat Abadi
 * @email m.bamorovvat@gmail.com
 * @webpage https://bamorovat.com
 * @date 2016-2025
 * @version 2.0
 * 
 * @license MIT License
 * 
 * Copyright (c) 2016-2025 Mohammad Hossein Bamorovat Abadi
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef VISUAL_SONAR_ROS_NODE_H
#define VISUAL_SONAR_ROS_NODE_H

// Standard C++ includes
#include <memory>
#include <string>
#include <chrono>

// ROS includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <dynamic_reconfigure/server.h>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// Local includes
#include "Visual_Sonar.h"
#include "edge_detection.h"
#include "Navigation.h"

namespace visual_sonar_ros {

/**
 * @struct NodeConfig
 * @brief Configuration parameters for the Visual Sonar ROS node
 */
struct NodeConfig {
    // ROS topics
    std::string input_image_topic;      ///< Input camera image topic
    std::string output_velocity_topic;  ///< Output velocity command topic
    std::string debug_image_topic;      ///< Debug visualization topic
    std::string status_topic;           ///< Node status topic
    
    // Processing parameters
    std::string edge_detection_method;  ///< Edge detection method name
    int processing_rate;                ///< Maximum processing rate (Hz)
    bool enable_visualization;          ///< Enable debug visualization
    bool enable_diagnostics;            ///< Enable diagnostic publishing
    
    // Safety parameters
    double max_linear_velocity;         ///< Maximum allowed linear velocity
    double max_angular_velocity;        ///< Maximum allowed angular velocity
    double safety_timeout;              ///< Safety timeout (seconds)
    bool enable_safety_stop;            ///< Enable emergency stop functionality
    
    /**
     * @brief Default constructor with standard parameters
     */
    NodeConfig()
        : input_image_topic("camera/image")
        , output_velocity_topic("/mobile_base/commands/velocity")  
        , debug_image_topic("visual_sonar/debug_image")
        , status_topic("visual_sonar/status")
        , edge_detection_method("sobel")
        , processing_rate(30)
        , enable_visualization(true)
        , enable_diagnostics(false)
        , max_linear_velocity(0.3)
        , max_angular_velocity(1.0)
        , safety_timeout(1.0)
        , enable_safety_stop(true) {}
};

/**
 * @struct ProcessingStats
 * @brief Statistics for performance monitoring
 */
struct ProcessingStats {
    double avg_processing_time;     ///< Average processing time (ms)
    double max_processing_time;     ///< Maximum processing time (ms)
    int frames_processed;           ///< Total frames processed
    int frames_dropped;             ///< Frames dropped due to processing lag
    std::chrono::steady_clock::time_point last_frame_time;
    
    /**
     * @brief Default constructor
     */
    ProcessingStats()
        : avg_processing_time(0.0)
        , max_processing_time(0.0)
        , frames_processed(0)
        , frames_dropped(0)
        , last_frame_time(std::chrono::steady_clock::now()) {}
        
    /**
     * @brief Reset statistics
     */
    void reset() {
        avg_processing_time = 0.0;
        max_processing_time = 0.0;
        frames_processed = 0;
        frames_dropped = 0;
        last_frame_time = std::chrono::steady_clock::now();
    }
};

/**
 * @class VisualSonarNode
 * @brief Main ROS node class for Visual Sonar processing
 * 
 * This class integrates all Visual Sonar components and provides a complete
 * ROS interface for autonomous robot navigation using omnidirectional vision.
 */
class VisualSonarNode {
public:
    /**
     * @brief Constructor
     * @param nh ROS node handle
     * @param pnh Private node handle for parameters
     */
    explicit VisualSonarNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    
    /**
     * @brief Destructor
     */
    ~VisualSonarNode();
    
    /**
     * @brief Initialize the node with configuration
     * @param config Node configuration parameters
     * @return true if initialization was successful, false otherwise
     */
    bool initialize(const NodeConfig& config);
    
    /**
     * @brief Start the node processing
     * @return true if started successfully, false otherwise
     */
    bool start();
    
    /**
     * @brief Stop the node processing
     */
    void stop();
    
    /**
     * @brief Check if the node is running
     * @return true if running, false otherwise
     */
    bool isRunning() const { return is_running_; }
    
    /**
     * @brief Get current processing statistics
     * @return Copy of current processing statistics
     */
    ProcessingStats getProcessingStats() const { return stats_; }
    
    /**
     * @brief Emergency stop - immediately halt robot movement
     */
    void emergencyStop();

private:
    // ROS node handles
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    // ROS communication
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher velocity_pub_;
    ros::Publisher debug_image_pub_;
    ros::Publisher status_pub_;
    ros::Timer safety_timer_;
    ros::Timer stats_timer_;
    
    // Processing components
    std::unique_ptr<SonarConfig> sonar_config_;
    std::unique_ptr<EdgeDetectionConfig> edge_config_;
    std::unique_ptr<NavigationConfig> nav_config_;
    
    // Node configuration
    NodeConfig node_config_;
    
    // State variables
    bool is_initialized_;
    bool is_running_;
    bool emergency_stop_active_;
    std::chrono::steady_clock::time_point last_image_time_;
    
    // Processing statistics
    ProcessingStats stats_;
    mutable std::mutex stats_mutex_;
    
    // Current processing results
    geometry_msgs::Twist current_velocity_;
    cv::Mat last_debug_image_;
    
    /**
     * @brief Load parameters from ROS parameter server
     * @return true if parameters loaded successfully, false otherwise
     */
    bool loadParameters();
    
    /**
     * @brief Setup ROS publishers and subscribers
     * @return true if setup was successful, false otherwise
     */
    bool setupRosInterface();
    
    /**
     * @brief Initialize processing components with loaded parameters
     * @return true if initialization was successful, false otherwise
     */
    bool initializeProcessingComponents();
    
    /**
     * @brief Main image processing callback
     * @param msg Input image message
     */
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    
    /**
     * @brief Process a single frame through the Visual Sonar pipeline
     * @param input_image Input BGR image from camera
     * @param output_velocity Output velocity commands
     * @param debug_image Output debug visualization image
     * @return true if processing was successful, false otherwise
     */
    bool processFrame(const cv::Mat& input_image, 
                     geometry_msgs::Twist& output_velocity,
                     cv::Mat& debug_image);
    
    /**
     * @brief Safety timer callback - monitors for communication timeouts
     * @param event Timer event information
     */
    void safetyTimerCallback(const ros::TimerEvent& event);
    
    /**
     * @brief Statistics timer callback - publishes performance statistics
     * @param event Timer event information  
     */
    void statsTimerCallback(const ros::TimerEvent& event);
    
    /**
     * @brief Publish velocity commands with safety checks
     * @param velocity Velocity command to publish
     */
    void publishVelocity(const geometry_msgs::Twist& velocity);
    
    /**
     * @brief Publish debug visualization image
     * @param debug_image Debug image to publish
     */
    void publishDebugImage(const cv::Mat& debug_image);
    
    /**
     * @brief Publish node status information
     */
    void publishStatus();
    
    /**
     * @brief Update processing statistics
     * @param processing_time Processing time for current frame (ms)
     */
    void updateStatistics(double processing_time);
    
    /**
     * @brief Validate velocity commands for safety
     * @param velocity Velocity to validate and potentially modify
     * @return true if velocity is safe, false if modified for safety
     */
    bool validateVelocity(geometry_msgs::Twist& velocity);
    
    /**
     * @brief Create debug visualization image
     * @param input_image Original input image
     * @param edge_image Edge detection result
     * @param sonar_result Visual sonar processing result
     * @param nav_result Navigation computation result
     * @param output_image Output debug visualization
     * @return true if visualization was created successfully
     */
    bool createDebugVisualization(const cv::Mat& input_image,
                                 const cv::Mat& edge_image,
                                 const SonarResult& sonar_result,
                                 const NavigationResult& nav_result,
                                 cv::Mat& output_image);
    
    /**
     * @brief Handle processing errors gracefully
     * @param error_message Description of the error
     * @param stop_robot Whether to stop robot movement
     */
    void handleProcessingError(const std::string& error_message, bool stop_robot = true);
};

/**
 * @brief Load node configuration from ROS parameters
 * @param nh ROS node handle
 * @param config Output configuration structure
 * @return true if configuration loaded successfully, false otherwise
 */
bool loadNodeConfig(ros::NodeHandle& nh, NodeConfig& config);

/**
 * @brief Validate node configuration parameters
 * @param config Configuration to validate
 * @return true if configuration is valid, false otherwise
 */
bool validateNodeConfig(const NodeConfig& config);

/**
 * @brief Convert string to EdgeDetectionMethod enum
 * @param method_name Method name string
 * @return Corresponding EdgeDetectionMethod enum value
 */
EdgeDetectionMethod stringToEdgeDetectionMethod(const std::string& method_name);

} // namespace visual_sonar_ros

#endif // VISUAL_SONAR_ROS_NODE_H
