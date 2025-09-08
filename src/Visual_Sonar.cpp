/**
 * @file Visual_Sonar.cpp
 * @brief Implementation of Visual Sonar algorithm for mobile robot navigation
 * 
 * This file implements the Visual Sonar algorithm that processes omnidirectional
 * camera images to simulate sonar-like obstacle detection for autonomous navigation.
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
#include <vector>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

// Local includes
#include "Visual_Sonar.h"
#include "light_reflection_elimination.h"

namespace visual_sonar_ros {

// Constants
constexpr double PI_DEGREES = 180.0;         ///< Pi in degrees
constexpr double PI_RADIANS = 3.14159265359; ///< Pi in radians
constexpr int LIGHT_REFLECTION_THRESHOLD = 170; ///< Threshold for light reflection elimination
constexpr int GRADIENT_THRESHOLD = 255;      ///< Gradient threshold for obstacle detection

// Static variables for window management
static bool windows_created = false;

/**
 * @brief Creates OpenCV windows for visualization if not already created
 */
static void createVisualizationWindows() {
    if (!windows_created) {
        cv::namedWindow("Main Image", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Sonar_Vision", cv::WINDOW_AUTOSIZE);
        windows_created = true;
    }
}

/**
 * @brief Validates input parameters for sonar processing
 */
bool validateSonarConfig(const SonarConfig& config) {
    if (config.sonar_number <= 0) {
        std::cerr << "Error: Sonar number must be positive" << std::endl;
        return false;
    }
    
    if (config.first_sonar_degree < 0 || config.first_sonar_degree >= config.sonar_number) {
        std::cerr << "Error: First sonar degree out of range [0, " << config.sonar_number << ")" << std::endl;
        return false;
    }
    
    if (config.last_sonar_degree <= config.first_sonar_degree || config.last_sonar_degree > config.sonar_number) {
        std::cerr << "Error: Last sonar degree must be > first_sonar_degree and <= sonar_number" << std::endl;
        return false;
    }
    
    if (config.first_pixel <= 0) {
        std::cerr << "Error: First pixel must be positive" << std::endl;
        return false;
    }
    
    return true;
}

/**
 * @brief Validates input images for processing
 */
bool validateInputImages(const cv::Mat& main_image, const cv::Mat& gradient_image) {
    if (main_image.empty()) {
        std::cerr << "Error: Main image is empty" << std::endl;
        return false;
    }
    
    if (gradient_image.empty()) {
        std::cerr << "Error: Gradient image is empty" << std::endl;
        return false;
    }
    
    if (main_image.size() != gradient_image.size()) {
        std::cerr << "Error: Main and gradient images must have the same size" << std::endl;
        return false;
    }
    
    if (gradient_image.type() != CV_8UC1 && gradient_image.type() != CV_8UC3) {
        std::cerr << "Error: Gradient image must be 8-bit single or 3-channel" << std::endl;
        return false;
    }
    
    return true;
}

/**
 * @brief Processes a single sonar beam to detect obstacles
 */
static std::pair<int, int> processSonarBeam(const cv::Mat& main_image,
                                          cv::Mat& gradient_image,
                                          int beam_index,
                                          const SonarConfig& config,
                                          std::vector<std::vector<int>>& image_matrix) {
    
    const int center_x = main_image.rows / 2;
    const int center_y = main_image.cols / 2;
    
    // Calculate beam angle
    const double degree = (2.0 * beam_index * PI_DEGREES) / config.sonar_number;
    const double alpha = degree * PI_RADIANS / PI_DEGREES;
    
    int x_vector_start = 0, y_vector_start = 0;
    int x_vector_end = 0, y_vector_end = 0;
    
    // Process pixels along the sonar beam
    for (int pixel_counter = config.first_pixel; pixel_counter <= center_x; ++pixel_counter) {
        // Calculate current pixel position
        const int x_vector = static_cast<int>(pixel_counter * std::sin(alpha)) + center_x;
        const int y_vector = static_cast<int>(pixel_counter * std::cos(alpha)) + center_y;
        
        // Boundary check
        if (x_vector < 0 || x_vector >= gradient_image.rows || 
            y_vector < 0 || y_vector >= gradient_image.cols) {
            break;
        }
        
        // Store start position
        if (pixel_counter == config.first_pixel) {
            x_vector_start = x_vector;
            y_vector_start = y_vector;
        }
        
        // Store end position (will be updated until obstacle is found)
        x_vector_end = x_vector;
        y_vector_end = y_vector;
        
        // Get pixel intensity
        uchar* data = gradient_image.ptr<uchar>(x_vector, y_vector);
        const int current_intensity = *data;
        
        // Store in image matrix for analysis
        if (beam_index < static_cast<int>(image_matrix.size()) && 
            pixel_counter < static_cast<int>(image_matrix[beam_index].size())) {
            image_matrix[beam_index][pixel_counter] = current_intensity;
        }
        
        // Obstacle detection logic
        if (pixel_counter > config.first_pixel) {
            const int prev_intensity = (beam_index < static_cast<int>(image_matrix.size()) && 
                                      (pixel_counter - 1) < static_cast<int>(image_matrix[beam_index].size())) ?
                                      image_matrix[beam_index][pixel_counter - 1] : 0;
            
            if (current_intensity + prev_intensity < GRADIENT_THRESHOLD) {
                *data = 255; // Mark as free space
            } else if (current_intensity + prev_intensity >= GRADIENT_THRESHOLD) {
                // Check for light reflection to avoid false positives
                const int light_intensity = light_reflection_elimination(main_image, x_vector, y_vector);
                
                if (light_intensity >= LIGHT_REFLECTION_THRESHOLD) {
                    *data = 255; // Mark as free space (light reflection)
                } else {
                    // Obstacle detected - stop beam processing
                    break;
                }
            }
        }
    }
    
    // Return the vector components
    const int x_length = x_vector_end - x_vector_start;
    const int y_length = y_vector_end - y_vector_start;
    
    return std::make_pair(x_length, y_length);
}

/**
 * @brief Main Visual Sonar processing function
 */
bool processVisualSonar(const cv::Mat& main_image, 
                       cv::Mat& gradient_image, 
                       const SonarConfig& config, 
                       SonarResult& result) {
    
    // Reset result
    result.reset();
    
    // Validate inputs
    if (!validateSonarConfig(config)) {
        return false;
    }
    
    if (!validateInputImages(main_image, gradient_image)) {
        return false;
    }
    
    // Create visualization windows
    createVisualizationWindows();
    
    // Initialize image matrix for storing pixel intensities
    std::vector<std::vector<int>> image_matrix(60, std::vector<int>(500, 0));
    
    const int center_x = main_image.rows / 2;
    const int center_y = main_image.cols / 2;
    
    // Process each sonar beam
    for (int k = config.first_sonar_degree; k < config.last_sonar_degree; ++k) {
        std::pair<int, int> beam_result = processSonarBeam(main_image, gradient_image, k, config, image_matrix);
        
        result.x_vector_sum += beam_result.first;
        result.y_vector_sum += beam_result.second;
    }
    
    // Draw the computed navigation vector
    const cv::Point center(center_y, center_x);
    const cv::Point vector_end(result.y_vector_sum + center_y, result.x_vector_sum + center_x);
    
    // Draw on gradient image (thicker line)
    cv::line(gradient_image, center, vector_end, cv::Scalar(255), 2);
    
    // Draw on main image (thinner line) - create a copy to avoid modifying original
    cv::Mat main_image_copy = main_image.clone();
    cv::line(main_image_copy, center, vector_end, cv::Scalar(0, 255, 0), 1);
    
    // Display results
    cv::imshow("Main Image", main_image_copy);
    cv::imshow("Sonar_Vision", gradient_image);
    
    result.is_valid = true;
    return true;
}

/**
 * @brief Legacy wrapper function for backward compatibility
 * @deprecated Use processVisualSonar() with SonarConfig and SonarResult instead
 */
void visual_sonar(cv::Mat Main_Image, cv::Mat Gradian_Image, int Sonar_Number, int First_Sonar_Degree, 
                 int Last_Sonar_Degree, int First_pixel, int& X_Vector_Sum, int& Y_Vector_Sum) {
    
    // Create config from legacy parameters
    SonarConfig config(Sonar_Number, First_Sonar_Degree, Last_Sonar_Degree, First_pixel);
    
    // Create result structure
    SonarResult result;
    
    // Process using modern function
    if (processVisualSonar(Main_Image, Gradian_Image, config, result)) {
        X_Vector_Sum = result.x_vector_sum;
        Y_Vector_Sum = result.y_vector_sum;
    } else {
        std::cerr << "Warning: Visual sonar processing failed, returning zero vectors" << std::endl;
        X_Vector_Sum = 0;
        Y_Vector_Sum = 0;
    }
}

} // namespace visual_sonar_ros

