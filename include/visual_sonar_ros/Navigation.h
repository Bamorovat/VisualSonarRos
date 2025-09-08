/**
 * @file Navigation.h
 * @brief Navigation control system for mobile robot based on Visual Sonar vectors
 * 
 * This header defines the navigation control system that converts Visual Sonar
 * output vectors into appropriate linear and angular velocities for mobile robot
 * movement. The system implements graduated speed control based on obstacle
 * proximity and desired direction changes.
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

#ifndef VISUAL_SONAR_ROS_NAVIGATION_H
#define VISUAL_SONAR_ROS_NAVIGATION_H

// Standard C++ includes
#include <cmath>
#include <utility>
#include <limits>

namespace visual_sonar_ros {

/**
 * @struct NavigationConfig
 * @brief Configuration parameters for navigation control
 */
struct NavigationConfig {
    // Speed control parameters
    float max_linear_velocity;      ///< Maximum allowed linear velocity (m/s)
    float max_angular_velocity;     ///< Maximum allowed angular velocity (rad/s)
    float min_linear_velocity;      ///< Minimum linear velocity for movement (m/s)
    
    // Angle thresholds for graduated speed control (degrees)
    float straight_threshold;       ///< Threshold for straight movement (±degrees)
    float slight_turn_threshold;    ///< Threshold for slight turns (±degrees)
    float medium_turn_threshold;    ///< Threshold for medium turns (±degrees)
    float sharp_turn_threshold;     ///< Threshold for sharp turns (±degrees)
    
    // Speed scaling factors
    float straight_speed_factor;    ///< Speed factor for straight movement
    float slight_turn_factor;       ///< Speed factor for slight turns
    float medium_turn_factor;       ///< Speed factor for medium turns
    float sharp_turn_factor;        ///< Speed factor for sharp turns
    
    // Angular velocity scaling
    float angular_scale_factor;     ///< Scaling factor for angular velocity calculation
    
    /**
     * @brief Default constructor with standard navigation parameters
     */
    NavigationConfig()
        : max_linear_velocity(0.3f)
        , max_angular_velocity(1.0f)
        , min_linear_velocity(0.05f)
        , straight_threshold(2.0f)
        , slight_turn_threshold(10.0f)
        , medium_turn_threshold(25.0f)
        , sharp_turn_threshold(40.0f)
        , straight_speed_factor(0.2f)
        , slight_turn_factor(0.15f)
        , medium_turn_factor(0.1f)
        , sharp_turn_factor(0.06f)
        , angular_scale_factor(50.0f) {}
};

/**
 * @struct NavigationResult
 * @brief Result structure containing computed velocities and navigation info
 */
struct NavigationResult {
    float linear_velocity;      ///< Computed linear velocity (m/s)
    float angular_velocity;     ///< Computed angular velocity (rad/s)
    float direction_angle;      ///< Direction angle in degrees
    float vector_magnitude;     ///< Magnitude of the input vector
    bool is_valid;             ///< Flag indicating if the result is valid
    
    /**
     * @brief Default constructor
     */
    NavigationResult() 
        : linear_velocity(0.0f)
        , angular_velocity(0.0f)
        , direction_angle(0.0f)
        , vector_magnitude(0.0f)
        , is_valid(false) {}
        
    /**
     * @brief Reset the result to default values
     */
    void reset() {
        linear_velocity = 0.0f;
        angular_velocity = 0.0f;
        direction_angle = 0.0f;
        vector_magnitude = 0.0f;
        is_valid = false;
    }
};

/**
 * @brief Modern navigation control function
 * 
 * Converts Visual Sonar output vectors into appropriate linear and angular
 * velocities using graduated speed control based on required turning angle.
 * 
 * @param x_vector_sum Sum of X components from Visual Sonar processing
 * @param y_vector_sum Sum of Y components from Visual Sonar processing
 * @param config Navigation configuration parameters
 * @param result Output structure containing computed velocities
 * @return true if navigation computation was successful, false otherwise
 * 
 * @note The function implements safety checks and bounds limiting
 * @warning Input vectors should be from valid Visual Sonar processing
 */
bool computeNavigationCommands(int x_vector_sum, 
                             int y_vector_sum, 
                             const NavigationConfig& config, 
                             NavigationResult& result);

/**
 * @brief Legacy navigation function for backward compatibility
 * 
 * @deprecated Use computeNavigationCommands() with NavigationConfig and NavigationResult instead
 * 
 * @param X_Vector_Sum Sum of X components from sonar processing
 * @param Y_Vector_Sum Sum of Y components from sonar processing
 * @param Angular_Velocity Reference to output angular velocity
 * @param Linear_Velocity Reference to output linear velocity
 */
void navigation(int X_Vector_Sum, int Y_Vector_Sum, float& Angular_Velocity, float& Linear_Velocity);

/**
 * @brief Validate navigation configuration parameters
 * 
 * @param config Configuration to validate
 * @return true if configuration is valid, false otherwise
 */
bool validateNavigationConfig(const NavigationConfig& config);

/**
 * @brief Compute vector magnitude and direction angle
 * 
 * @param x_vector X component of the vector
 * @param y_vector Y component of the vector
 * @return std::pair<magnitude, angle_degrees> where angle is in degrees
 */
std::pair<float, float> computeVectorProperties(int x_vector, int y_vector);

/**
 * @brief Apply safety limits to computed velocities
 * 
 * @param linear_vel Reference to linear velocity to be limited
 * @param angular_vel Reference to angular velocity to be limited
 * @param config Configuration containing velocity limits
 */
void applySafetyLimits(float& linear_vel, float& angular_vel, const NavigationConfig& config);

/**
 * @brief Convert degrees to radians
 * 
 * @param degrees Angle in degrees
 * @return Angle in radians
 */
inline float degreesToRadians(float degrees) {
    return degrees * M_PI / 180.0f;
}

/**
 * @brief Convert radians to degrees
 * 
 * @param radians Angle in radians
 * @return Angle in degrees
 */
inline float radiansToDegrees(float radians) {
    return radians * 180.0f / M_PI;
}

} // namespace visual_sonar_ros

#endif // VISUAL_SONAR_ROS_NAVIGATION_H
