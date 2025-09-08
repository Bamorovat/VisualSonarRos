/**
 * @file Light_Reflection_Elimination.cpp
 * @brief Implementation of light reflection elimination algorithms
 * 
 * This file implements algorithms for detecting and eliminating light reflections
 * from camera images to improve obstacle detection accuracy in Visual Sonar.
 * The implementation uses HSV color space analysis to distinguish between actual
 * obstacles and bright reflections.
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
#include <algorithm>
#include <vector>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Local includes
#include "light_reflection_elimination.h"

namespace visual_sonar_ros {

/**
 * @brief Validates image and coordinates for reflection analysis
 */
bool validateReflectionInputs(const cv::Mat& image, int x_coord, int y_coord) {
    if (image.empty()) {
        std::cerr << "Error: Input image is empty" << std::endl;
        return false;
    }
    
    if (image.channels() != 3) {
        std::cerr << "Error: Input image must be 3-channel BGR" << std::endl;
        return false;
    }
    
    if (x_coord < 0 || x_coord >= image.rows) {
        std::cerr << "Error: X coordinate " << x_coord << " out of bounds [0, " << image.rows << ")" << std::endl;
        return false;
    }
    
    if (y_coord < 0 || y_coord >= image.cols) {
        std::cerr << "Error: Y coordinate " << y_coord << " out of bounds [0, " << image.cols << ")" << std::endl;
        return false;
    }
    
    return true;
}

/**
 * @brief Converts BGR image to HSV color space
 */
bool convertToHSV(const cv::Mat& bgr_image, cv::Mat& hsv_image) {
    if (bgr_image.empty()) {
        std::cerr << "Error: Cannot convert empty image to HSV" << std::endl;
        return false;
    }
    
    try {
        cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);
        return true;
    } catch (const cv::Exception& e) {
        std::cerr << "Error converting image to HSV: " << e.what() << std::endl;
        return false;
    }
}

/**
 * @brief Modern light reflection analysis function
 */
bool analyzePixelReflection(const cv::Mat& main_image,
                          int x_coordinate,
                          int y_coordinate,
                          const HSVThresholds& thresholds,
                          ReflectionAnalysis& analysis) {
    
    // Reset analysis result
    analysis = ReflectionAnalysis();
    
    // Validate inputs
    if (!validateReflectionInputs(main_image, x_coordinate, y_coordinate)) {
        return false;
    }
    
    // Convert to HSV color space
    cv::Mat hsv_image;
    if (!convertToHSV(main_image, hsv_image)) {
        return false;
    }
    
    // Extract HSV values at the specified pixel
    const cv::Vec3b& hsv_pixel = hsv_image.at<cv::Vec3b>(x_coordinate, y_coordinate);
    
    analysis.hue = hsv_pixel[0];        // Hue: 0-179
    analysis.saturation = hsv_pixel[1]; // Saturation: 0-255
    analysis.value = hsv_pixel[2];      // Value (brightness): 0-255
    
    // Compute reflection score using the established algorithm
    // High brightness with low saturation typically indicates reflection
    analysis.reflection_score = computeReflectionScore(analysis.hue, analysis.saturation, analysis.value);
    
    // Classify as reflection based on threshold
    analysis.is_reflection = isReflection(analysis.reflection_score, thresholds);
    
    analysis.is_valid = true;
    return true;
}

/**
 * @brief Analyzes reflection in a local region around a point
 */
int analyzeRegionReflection(const cv::Mat& main_image,
                          int x_coordinate,
                          int y_coordinate,
                          int region_size,
                          const HSVThresholds& thresholds) {
    
    // Validate basic inputs
    if (!validateReflectionInputs(main_image, x_coordinate, y_coordinate)) {
        return 0;
    }
    
    if (region_size <= 0 || region_size % 2 == 0) {
        std::cerr << "Warning: Region size should be positive and odd, using default size 3" << std::endl;
        region_size = 3;
    }
    
    // Convert to HSV once for efficiency
    cv::Mat hsv_image;
    if (!convertToHSV(main_image, hsv_image)) {
        return 0;
    }
    
    const int half_size = region_size / 2;
    int total_score = 0;
    int valid_pixels = 0;
    
    // Analyze pixels in the region
    for (int dx = -half_size; dx <= half_size; ++dx) {
        for (int dy = -half_size; dy <= half_size; ++dy) {
            const int px = x_coordinate + dx;
            const int py = y_coordinate + dy;
            
            // Check bounds
            if (px >= 0 && px < hsv_image.rows && py >= 0 && py < hsv_image.cols) {
                const cv::Vec3b& hsv_pixel = hsv_image.at<cv::Vec3b>(px, py);
                const int hue = hsv_pixel[0];
                const int saturation = hsv_pixel[1];
                const int value = hsv_pixel[2];
                
                total_score += computeReflectionScore(hue, saturation, value);
                valid_pixels++;
            }
        }
    }
    
    // Return average score
    return (valid_pixels > 0) ? (total_score / valid_pixels) : 0;
}

/**
 * @brief Creates a binary mask highlighting reflection regions
 */
bool createReflectionMask(const cv::Mat& main_image,
                        const HSVThresholds& thresholds,
                        cv::Mat& reflection_mask) {
    
    if (main_image.empty()) {
        std::cerr << "Error: Cannot create reflection mask from empty image" << std::endl;
        return false;
    }
    
    // Convert to HSV
    cv::Mat hsv_image;
    if (!convertToHSV(main_image, hsv_image)) {
        return false;
    }
    
    // Initialize mask
    reflection_mask = cv::Mat::zeros(main_image.size(), CV_8UC1);
    
    // Process each pixel
    for (int row = 0; row < hsv_image.rows; ++row) {
        for (int col = 0; col < hsv_image.cols; ++col) {
            const cv::Vec3b& hsv_pixel = hsv_image.at<cv::Vec3b>(row, col);
            const int hue = hsv_pixel[0];
            const int saturation = hsv_pixel[1];
            const int value = hsv_pixel[2];
            
            const int reflection_score = computeReflectionScore(hue, saturation, value);
            
            if (isReflection(reflection_score, thresholds)) {
                reflection_mask.at<uchar>(row, col) = 255;
            }
        }
    }
    
    return true;
}

/**
 * @brief Legacy light reflection elimination function
 * @deprecated Use analyzePixelReflection() with HSVThresholds and ReflectionAnalysis instead
 */
int light_reflection_elimination(cv::Mat Main_Image, int x_vector_point, int y_vector_point) {
    // Create default thresholds
    HSVThresholds thresholds;
    
    // Create analysis structure
    ReflectionAnalysis analysis;
    
    // Use modern function
    if (analyzePixelReflection(Main_Image, x_vector_point, y_vector_point, thresholds, analysis)) {
        return analysis.reflection_score;
    } else {
        std::cerr << "Warning: Reflection analysis failed, returning zero score" << std::endl;
        return 0;
    }
}

} // namespace visual_sonar_ros

