/**
 * @file Navigation.cpp
 * @brief Implementation of navigation control system for mobile robot
 * 
 * This file implements the navigation control system that converts Visual Sonar
 * output vectors into appropriate linear and angular velocities for mobile robot
 * movement with graduated speed control and safety mechanisms.
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
#include <cmath>
#include <algorithm>
#include <limits>

// Local includes
#include "Navigation.h"

namespace visual_sonar_ros {

// Mathematical constants
constexpr float PI = 3.14159265359f;
constexpr float RAD_TO_DEG = 180.0f / PI;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float EPSILON = 1e-6f;

/**
 * @brief Validates navigation configuration parameters
 */
bool validateNavigationConfig(const NavigationConfig& config) {
    if (config.max_linear_velocity <= 0.0f) {
        std::cerr << "Error: Max linear velocity must be positive" << std::endl;
        return false;
    }
    
    if (config.max_angular_velocity <= 0.0f) {
        std::cerr << "Error: Max angular velocity must be positive" << std::endl;
        return false;
    }
    
    if (config.min_linear_velocity < 0.0f) {
        std::cerr << "Error: Min linear velocity cannot be negative" << std::endl;
        return false;
    }
    
    if (config.min_linear_velocity >= config.max_linear_velocity) {
        std::cerr << "Error: Min linear velocity must be less than max linear velocity" << std::endl;
        return false;
    }
    
    // Validate angle thresholds are in ascending order
    if (config.straight_threshold < 0.0f ||
        config.slight_turn_threshold <= config.straight_threshold ||
        config.medium_turn_threshold <= config.slight_turn_threshold ||
        config.sharp_turn_threshold <= config.medium_turn_threshold) {
        std::cerr << "Error: Angle thresholds must be positive and in ascending order" << std::endl;
        return false;
    }
    
    return true;
}

/**
 * @brief Computes vector magnitude and direction angle
 */
std::pair<float, float> computeVectorProperties(int x_vector, int y_vector) {
    // Calculate magnitude using proper mathematical operations (FIXED BUG)
    // Original bug: sqrt((X_Absolute^2)+(Y_Absolute^2)) used XOR (^) instead of power
    const float magnitude = std::sqrt(static_cast<float>(x_vector * x_vector + y_vector * y_vector));
    
    // Calculate angle using atan2 for proper quadrant handling
    const float angle_rad = std::atan2(static_cast<float>(x_vector), static_cast<float>(y_vector));
    const float angle_deg = angle_rad * RAD_TO_DEG;
    
    // Normalize angle to be relative to forward direction (0 degrees = straight ahead)
    float normalized_angle = 90.0f + angle_deg;
    
    // Normalize to [-180, 180] range
    while (normalized_angle > 180.0f) normalized_angle -= 360.0f;
    while (normalized_angle < -180.0f) normalized_angle += 360.0f;
    
    return std::make_pair(magnitude, normalized_angle);
}

/**
 * @brief Applies safety limits to computed velocities
 */
void applySafetyLimits(float& linear_vel, float& angular_vel, const NavigationConfig& config) {
    // Limit linear velocity
    linear_vel = std::max(0.0f, std::min(linear_vel, config.max_linear_velocity));
    
    // Apply minimum threshold for linear velocity (to avoid very slow movements)
    if (linear_vel > 0.0f && linear_vel < config.min_linear_velocity) {
        linear_vel = config.min_linear_velocity;
    }
    
    // Limit angular velocity
    angular_vel = std::max(-config.max_angular_velocity, std::min(angular_vel, config.max_angular_velocity));
}

/**
 * @brief Modern navigation control function
 */
bool computeNavigationCommands(int x_vector_sum, int y_vector_sum, 
                             const NavigationConfig& config, NavigationResult& result) {
    // Reset result
    result.reset();
    
    // Validate configuration
    if (!validateNavigationConfig(config)) {
        return false;
    }
    
    // Handle zero vector case
    if (x_vector_sum == 0 && y_vector_sum == 0) {
        std::cout << "Warning: Zero input vectors, stopping robot" << std::endl;
        // result is already reset to zero values
        result.is_valid = true;
        return true;
    }
    
    // Compute vector properties
    auto [magnitude, direction_angle] = computeVectorProperties(x_vector_sum, y_vector_sum);
    
    result.vector_magnitude = magnitude;
    result.direction_angle = direction_angle;
    
    // Get absolute angle for threshold comparison
    const float abs_angle = std::abs(direction_angle);
    
    // Determine linear velocity based on turning angle (graduated speed control)
    float linear_velocity = 0.0f;
    
    if (abs_angle <= config.straight_threshold) {
        // Straight movement - highest speed
        linear_velocity = config.straight_speed_factor;
    } else if (abs_angle <= config.slight_turn_threshold) {
        // Slight turn - reduced speed
        linear_velocity = config.slight_turn_factor;
    } else if (abs_angle <= config.medium_turn_threshold) {
        // Medium turn - further reduced speed
        linear_velocity = config.medium_turn_factor;
    } else if (abs_angle <= config.sharp_turn_threshold) {
        // Sharp turn - very low speed
        linear_velocity = config.sharp_turn_factor;
    } else {
        // Very sharp turn - stop and turn in place
        linear_velocity = 0.0f;
    }
    
    // Calculate angular velocity based on direction angle
    float angular_velocity = direction_angle / config.angular_scale_factor;
    
    // Apply safety limits
    applySafetyLimits(linear_velocity, angular_velocity, config);
    
    // Store results
    result.linear_velocity = linear_velocity;
    result.angular_velocity = angular_velocity;
    result.is_valid = true;
    
    return true;
}

/**
 * @brief Legacy navigation function for backward compatibility
 * @deprecated Use computeNavigationCommands() with NavigationConfig and NavigationResult instead
 */
void navigation(int X_Vector_Sum, int Y_Vector_Sum, float& Angular_Velocity, float& Linear_Velocity) {
    // Create default configuration
    NavigationConfig config;
    
    // Create result structure
    NavigationResult result;
    
    // Compute using modern function
    if (computeNavigationCommands(X_Vector_Sum, Y_Vector_Sum, config, result)) {
        Linear_Velocity = result.linear_velocity;
        Angular_Velocity = result.angular_velocity;
    } else {
        std::cerr << "Warning: Navigation computation failed, stopping robot" << std::endl;
        Linear_Velocity = 0.0f;
        Angular_Velocity = 0.0f;
    }
}

} // namespace visual_sonar_ros
