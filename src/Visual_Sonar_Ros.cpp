/**
 * @file Visual_Sonar_Ros.cpp
 * @brief Implementation of the main Visual Sonar ROS node
 * 
 * This file implements the complete Visual Sonar ROS node, integrating all
 * components including edge detection, visual sonar processing, navigation
 * control, and ROS communication for autonomous robot navigation using
 * omnidirectional vision.
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

// Standard C++ includes
#include <iostream>
#include <memory>
#include <chrono>
#include <mutex>
#include <algorithm>

// ROS includes
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Odometry.h>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// Local includes  
#include "visual_sonar_ros_node.h"
#include "Visual_Sonar.h"
#include "edge_detection.h"
#include "Navigation.h"

namespace visual_sonar_ros {

/**
 * @brief VisualSonarNode constructor
 */
VisualSonarNode::VisualSonarNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , it_(nh)
    , is_initialized_(false)
    , is_running_(false)
    , emergency_stop_active_(false)
    , last_image_time_(std::chrono::steady_clock::now()) {
    
    ROS_INFO("Visual Sonar ROS Node v2.0 - Initializing...");
}

/**
 * @brief VisualSonarNode destructor
 */
VisualSonarNode::~VisualSonarNode() {
    stop();
    ROS_INFO("Visual Sonar ROS Node - Shutdown complete");
}

/**
 * @brief Initialize the node with configuration
 */
bool VisualSonarNode::initialize(const NodeConfig& config) {
    node_config_ = config;
    
    if (!validateNodeConfig(node_config_)) {
        ROS_ERROR("Invalid node configuration");
        return false;
    }
    
    if (!loadParameters()) {
        ROS_ERROR("Failed to load parameters");
        return false;
    }
    
    if (!setupRosInterface()) {
        ROS_ERROR("Failed to setup ROS interface");
        return false;
    }
    
    if (!initializeProcessingComponents()) {
        ROS_ERROR("Failed to initialize processing components");
        return false;
    }
    
    is_initialized_ = true;
    ROS_INFO("Visual Sonar Node initialized successfully");
    return true;
}

/**
 * @brief Load parameters from ROS parameter server
 */
bool VisualSonarNode::loadParameters() {
    // Visual Sonar parameters
    int sonar_number, first_sonar_degree, last_sonar_degree, first_pixel;
    pnh_.param<int>("sonar_number", sonar_number, 24);
    pnh_.param<int>("first_sonar_degree", first_sonar_degree, 10);
    pnh_.param<int>("last_sonar_degree", last_sonar_degree, 27);
    pnh_.param<int>("first_pixel", first_pixel, 65);
    
    sonar_config_ = std::make_unique<SonarConfig>(sonar_number, first_sonar_degree, last_sonar_degree, first_pixel);
    
    // Edge detection parameters
    std::string edge_method;
    int edge_threshold;
    pnh_.param<std::string>("edge_detection_method", edge_method, "sobel");
    pnh_.param<int>("edge_threshold", edge_threshold, 27);
    
    edge_config_ = std::make_unique<EdgeDetectionConfig>(stringToEdgeDetectionMethod(edge_method), edge_threshold);
    
    // Navigation parameters
    nav_config_ = std::make_unique<NavigationConfig>();
    pnh_.param<float>("max_linear_velocity", nav_config_->max_linear_velocity, 0.3f);
    pnh_.param<float>("max_angular_velocity", nav_config_->max_angular_velocity, 1.0f);
    pnh_.param<float>("straight_threshold", nav_config_->straight_threshold, 2.0f);
    pnh_.param<float>("angular_scale_factor", nav_config_->angular_scale_factor, 50.0f);
    
    // Update node config from parameters
    pnh_.param<std::string>("input_image_topic", node_config_.input_image_topic, "camera/image");
    pnh_.param<std::string>("output_velocity_topic", node_config_.output_velocity_topic, "/mobile_base/commands/velocity");
    pnh_.param<bool>("enable_visualization", node_config_.enable_visualization, true);
    pnh_.param<bool>("enable_diagnostics", node_config_.enable_diagnostics, false);
    pnh_.param<double>("safety_timeout", node_config_.safety_timeout, 1.0);
    
    ROS_INFO("Parameters loaded successfully");
    return true;
}

/**
 * @brief Setup ROS publishers and subscribers
 */
bool VisualSonarNode::setupRosInterface() {
    // Setup velocity publisher
    velocity_pub_ = nh_.advertise<geometry_msgs::Twist>(node_config_.output_velocity_topic, 1);
    
    // Setup debug image publisher if visualization is enabled
    if (node_config_.enable_visualization) {
        debug_image_pub_ = nh_.advertise<sensor_msgs::Image>(node_config_.debug_image_topic, 1);
    }
    
    // Setup status publisher if diagnostics are enabled
    if (node_config_.enable_diagnostics) {
        status_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(node_config_.status_topic, 1);
    }
    
    // Setup image subscriber
    image_sub_ = it_.subscribe(node_config_.input_image_topic, 1, 
                              &VisualSonarNode::imageCallback, this);
    
    // Setup safety timer
    if (node_config_.enable_safety_stop) {
        safety_timer_ = nh_.createTimer(ros::Duration(node_config_.safety_timeout),
                                       &VisualSonarNode::safetyTimerCallback, this);
    }
    
    // Setup statistics timer if diagnostics are enabled
    if (node_config_.enable_diagnostics) {
        stats_timer_ = nh_.createTimer(ros::Duration(1.0),
                                      &VisualSonarNode::statsTimerCallback, this);
    }
    
    ROS_INFO("ROS interface setup complete");
    return true;
}

/**
 * @brief Initialize processing components
 */
bool VisualSonarNode::initializeProcessingComponents() {
    // Validate configurations
    if (!validateSonarConfig(*sonar_config_)) {
        ROS_ERROR("Invalid sonar configuration");
        return false;
    }
    
    if (!validateEdgeDetectionConfig(*edge_config_)) {
        ROS_ERROR("Invalid edge detection configuration");
        return false;
    }
    
    if (!validateNavigationConfig(*nav_config_)) {
        ROS_ERROR("Invalid navigation configuration");
        return false;
    }
    
    // Initialize OpenCV windows if visualization is enabled
    if (node_config_.enable_visualization) {
        cv::startWindowThread();
    }
    
    ROS_INFO("Processing components initialized");
    return true;
}

/**
 * @brief Start the node processing
 */
bool VisualSonarNode::start() {
    if (!is_initialized_) {
        ROS_ERROR("Node not initialized - call initialize() first");
        return false;
    }
    
    is_running_ = true;
    emergency_stop_active_ = false;
    stats_.reset();
    
    ROS_INFO("Visual Sonar Node started - Ready for processing");
    return true;
}

/**
 * @brief Stop the node processing
 */
void VisualSonarNode::stop() {
    if (is_running_) {
        is_running_ = false;
        
        // Send stop command
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        velocity_pub_.publish(stop_cmd);
        
        ROS_INFO("Visual Sonar Node stopped");
    }
}

/**
 * @brief Emergency stop - immediately halt robot movement
 */
void VisualSonarNode::emergencyStop() {
    emergency_stop_active_ = true;
    
    geometry_msgs::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    velocity_pub_.publish(stop_cmd);
    
    ROS_WARN("EMERGENCY STOP ACTIVATED");
}

/**
 * @brief Main image processing callback
 */
void VisualSonarNode::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    if (!is_running_ || emergency_stop_active_) {
        return;
    }
    
    auto start_time = std::chrono::steady_clock::now();
    last_image_time_ = start_time;
    
    try {
        // Convert ROS image to OpenCV format
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            handleProcessingError("Image conversion failed", false);
            return;
        }
        
        // Process the frame
        geometry_msgs::Twist output_velocity;
        cv::Mat debug_image;
        
        if (processFrame(cv_ptr->image, output_velocity, debug_image)) {
            // Publish results
            publishVelocity(output_velocity);
            
            if (node_config_.enable_visualization && !debug_image.empty()) {
                publishDebugImage(debug_image);
            }
            
            // Update statistics
            auto end_time = std::chrono::steady_clock::now();
            auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            updateStatistics(static_cast<double>(processing_time));
            
        } else {
            handleProcessingError("Frame processing failed", true);
        }
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in image callback: %s", e.what());
        handleProcessingError("Unexpected exception in processing", true);
    }
}

/**
 * @brief Process a single frame through the Visual Sonar pipeline
 */
bool VisualSonarNode::processFrame(const cv::Mat& input_image, 
                                  geometry_msgs::Twist& output_velocity,
                                  cv::Mat& debug_image) {
    
    // Validate input image
    if (input_image.empty()) {
        ROS_ERROR("Input image is empty");
        return false;
    }
    
    // 1. Edge Detection
    EdgeDetectionResult edge_result;
    if (!detectEdges(input_image, *edge_config_, edge_result)) {
        ROS_ERROR("Edge detection failed: %s", edge_result.error_message.c_str());
        return false;
    }
    
    // 2. Visual Sonar Processing
    SonarResult sonar_result;
    if (!processVisualSonar(input_image, edge_result.edge_image, *sonar_config_, sonar_result)) {
        ROS_ERROR("Visual sonar processing failed");
        return false;
    }
    
    // 3. Navigation Computation
    NavigationResult nav_result;
    if (!computeNavigationCommands(sonar_result.x_vector_sum, sonar_result.y_vector_sum, 
                                  *nav_config_, nav_result)) {
        ROS_ERROR("Navigation computation failed");
        return false;
    }
    
    // 4. Create output velocity command
    output_velocity.linear.x = nav_result.linear_velocity;
    output_velocity.linear.y = 0.0;
    output_velocity.linear.z = 0.0;
    output_velocity.angular.x = 0.0;
    output_velocity.angular.y = 0.0;
    output_velocity.angular.z = nav_result.angular_velocity;
    
    // 5. Create debug visualization if enabled
    if (node_config_.enable_visualization) {
        createDebugVisualization(input_image, edge_result.edge_image, sonar_result, nav_result, debug_image);
    }
    
    return true;
}

/**
 * @brief Publish velocity commands with safety checks
 */
void VisualSonarNode::publishVelocity(const geometry_msgs::Twist& velocity) {
    if (emergency_stop_active_) {
        geometry_msgs::Twist stop_cmd;
        velocity_pub_.publish(stop_cmd);
        return;
    }
    
    geometry_msgs::Twist safe_velocity = velocity;
    
    // Apply safety validation
    if (!validateVelocity(safe_velocity)) {
        ROS_WARN_THROTTLE(1.0, "Velocity command modified for safety");
    }
    
    // Publish the command
    velocity_pub_.publish(safe_velocity);
    current_velocity_ = safe_velocity;
}

/**
 * @brief Validate velocity commands for safety
 */
bool VisualSonarNode::validateVelocity(geometry_msgs::Twist& velocity) {
    bool modified = false;
    
    // Limit linear velocity
    if (std::abs(velocity.linear.x) > node_config_.max_linear_velocity) {
        velocity.linear.x = std::copysign(node_config_.max_linear_velocity, velocity.linear.x);
        modified = true;
    }
    
    // Limit angular velocity
    if (std::abs(velocity.angular.z) > node_config_.max_angular_velocity) {
        velocity.angular.z = std::copysign(node_config_.max_angular_velocity, velocity.angular.z);
        modified = true;
    }
    
    // Zero out other components for safety
    if (velocity.linear.y != 0.0 || velocity.linear.z != 0.0 ||
        velocity.angular.x != 0.0 || velocity.angular.y != 0.0) {
        velocity.linear.y = 0.0;
        velocity.linear.z = 0.0;
        velocity.angular.x = 0.0;
        velocity.angular.y = 0.0;
        modified = true;
    }
    
    return !modified;
}

/**
 * @brief Create debug visualization image
 */
bool VisualSonarNode::createDebugVisualization(const cv::Mat& input_image,
                                              const cv::Mat& edge_image,
                                              const SonarResult& sonar_result,
                                              const NavigationResult& nav_result,
                                              cv::Mat& output_image) {
    
    try {
        // Create composite debug image
        cv::Mat input_display, edge_display;
        cv::resize(input_image, input_display, cv::Size(320, 240));
        
        if (edge_image.channels() == 1) {
            cv::cvtColor(edge_image, edge_display, cv::COLOR_GRAY2BGR);
        } else {
            edge_display = edge_image.clone();
        }
        cv::resize(edge_display, edge_display, cv::Size(320, 240));
        
        // Create combined visualization
        output_image = cv::Mat::zeros(480, 640, CV_8UC3);
        
        // Place input image (top-left)
        cv::Rect input_roi(0, 0, 320, 240);
        input_display.copyTo(output_image(input_roi));
        
        // Place edge image (top-right)
        cv::Rect edge_roi(320, 0, 320, 240);
        edge_display.copyTo(output_image(edge_roi));
        
        // Add text information (bottom section)
        cv::Rect info_roi(0, 240, 640, 240);
        cv::rectangle(output_image(info_roi), cv::Point(0, 0), cv::Point(640, 240), cv::Scalar(50, 50, 50), -1);
        
        // Add navigation information
        std::string info_text = "Linear Vel: " + std::to_string(nav_result.linear_velocity) + " m/s";
        cv::putText(output_image, info_text, cv::Point(10, 270), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        
        info_text = "Angular Vel: " + std::to_string(nav_result.angular_velocity) + " rad/s";
        cv::putText(output_image, info_text, cv::Point(10, 300), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        
        info_text = "Direction: " + std::to_string(nav_result.direction_angle) + " deg";
        cv::putText(output_image, info_text, cv::Point(10, 330), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        
        info_text = "Vector Sum: (" + std::to_string(sonar_result.x_vector_sum) + ", " + std::to_string(sonar_result.y_vector_sum) + ")";
        cv::putText(output_image, info_text, cv::Point(10, 360), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        
        return true;
        
    } catch (const cv::Exception& e) {
        ROS_ERROR("Failed to create debug visualization: %s", e.what());
        return false;
    }
}

/**
 * @brief Publish debug visualization image
 */
void VisualSonarNode::publishDebugImage(const cv::Mat& debug_image) {
    if (debug_image_pub_.getNumSubscribers() > 0) {
        try {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_image).toImageMsg();
            msg->header.stamp = ros::Time::now();
            debug_image_pub_.publish(msg);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Failed to publish debug image: %s", e.what());
        }
    }
}

/**
 * @brief Safety timer callback
 */
void VisualSonarNode::safetyTimerCallback(const ros::TimerEvent& event) {
    auto current_time = std::chrono::steady_clock::now();
    auto time_since_last_image = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_image_time_).count();
    
    if (time_since_last_image > (node_config_.safety_timeout * 1000)) {
        ROS_WARN_THROTTLE(1.0, "No images received for %.2f seconds - stopping robot", 
                         time_since_last_image / 1000.0);
        emergencyStop();
    }
}

/**
 * @brief Update processing statistics
 */
void VisualSonarNode::updateStatistics(double processing_time) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    
    stats_.frames_processed++;
    stats_.max_processing_time = std::max(stats_.max_processing_time, processing_time);
    
    // Update running average
    double alpha = 0.1; // Smoothing factor
    if (stats_.frames_processed == 1) {
        stats_.avg_processing_time = processing_time;
    } else {
        stats_.avg_processing_time = alpha * processing_time + (1.0 - alpha) * stats_.avg_processing_time;
    }
}

/**
 * @brief Handle processing errors gracefully
 */
void VisualSonarNode::handleProcessingError(const std::string& error_message, bool stop_robot) {
    ROS_ERROR_THROTTLE(1.0, "Processing error: %s", error_message.c_str());
    
    if (stop_robot) {
        geometry_msgs::Twist stop_cmd;
        velocity_pub_.publish(stop_cmd);
    }
}

/**
 * @brief Convert string to EdgeDetectionMethod enum
 */
EdgeDetectionMethod stringToEdgeDetectionMethod(const std::string& method_name) {
    std::string lower_method = method_name;
    std::transform(lower_method.begin(), lower_method.end(), lower_method.begin(), ::tolower);
    
    if (lower_method == "sobel") return EdgeDetectionMethod::SOBEL;
    if (lower_method == "canny") return EdgeDetectionMethod::CANNY;
    if (lower_method == "laplacian") return EdgeDetectionMethod::LAPLACIAN;
    if (lower_method == "scharr") return EdgeDetectionMethod::SCHARR;
    if (lower_method == "roberts") return EdgeDetectionMethod::ROBERTS;
    if (lower_method == "prewitt") return EdgeDetectionMethod::PREWITT;
    
    ROS_WARN("Unknown edge detection method '%s', using Sobel", method_name.c_str());
    return EdgeDetectionMethod::SOBEL;
}

/**
 * @brief Validate node configuration
 */
bool validateNodeConfig(const NodeConfig& config) {
    if (config.processing_rate <= 0) {
        ROS_ERROR("Processing rate must be positive");
        return false;
    }
    
    if (config.max_linear_velocity <= 0.0 || config.max_angular_velocity <= 0.0) {
        ROS_ERROR("Maximum velocities must be positive");
        return false;
    }
    
    if (config.safety_timeout <= 0.0) {
        ROS_ERROR("Safety timeout must be positive");
        return false;
    }
    
    return true;
}

} // namespace visual_sonar_ros

/**
 * @brief Main function - ROS node entry point
 */
int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "visual_sonar_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    try {
        // Create node instance
        visual_sonar_ros::VisualSonarNode node(nh, pnh);
        
        // Load configuration
        visual_sonar_ros::NodeConfig config;
        // Use default configuration or load from parameters
        
        // Initialize and start the node
        if (!node.initialize(config)) {
            ROS_FATAL("Failed to initialize Visual Sonar Node");
            return -1;
        }
        
        if (!node.start()) {
            ROS_FATAL("Failed to start Visual Sonar Node");
            return -1;
        }
        
        ROS_INFO("Visual Sonar ROS Node v2.0 - Running");
        
        // Run the node
        ros::spin();
        
    } catch (const std::exception& e) {
        ROS_FATAL("Fatal exception in Visual Sonar Node: %s", e.what());
        return -1;
    }
    
    ROS_INFO("Visual Sonar Node - Shutting down");
    return 0;
}

