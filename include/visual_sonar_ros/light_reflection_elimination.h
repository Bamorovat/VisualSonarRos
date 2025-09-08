
/**
 * @file light_reflection_elimination.h
 * @brief Light reflection elimination algorithms for Visual Sonar processing
 * 
 * This header defines algorithms for detecting and eliminating light reflections
 * from camera images to improve obstacle detection accuracy in Visual Sonar.
 * The algorithms use HSV color space analysis to distinguish between actual
 * obstacles and bright light reflections that could cause false positives.
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

#ifndef VISUAL_SONAR_ROS_LIGHT_REFLECTION_ELIMINATION_H
#define VISUAL_SONAR_ROS_LIGHT_REFLECTION_ELIMINATION_H

// Standard C++ includes
#include <vector>
#include <utility>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace visual_sonar_ros {

/**
 * @struct HSVThresholds
 * @brief Thresholds for HSV-based light reflection detection
 */
struct HSVThresholds {
    int min_value;              ///< Minimum value (brightness) for reflection detection
    int max_saturation;         ///< Maximum saturation for reflection detection
    int reflection_threshold;   ///< Threshold for reflection classification
    
    /**
     * @brief Default constructor with standard thresholds
     */
    HSVThresholds()
        : min_value(100)
        , max_saturation(50)
        , reflection_threshold(170) {}
        
    /**
     * @brief Parameterized constructor
     * @param min_val Minimum brightness value
     * @param max_sat Maximum saturation value
     * @param refl_thresh Reflection classification threshold
     */
    HSVThresholds(int min_val, int max_sat, int refl_thresh)
        : min_value(min_val)
        , max_saturation(max_sat)
        , reflection_threshold(refl_thresh) {}
};

/**
 * @struct ReflectionAnalysis
 * @brief Result structure for reflection analysis
 */
struct ReflectionAnalysis {
    int hue;                    ///< Hue component (0-179)
    int saturation;             ///< Saturation component (0-255)
    int value;                  ///< Value/brightness component (0-255)
    int reflection_score;       ///< Computed reflection score
    bool is_reflection;         ///< True if pixel is classified as reflection
    bool is_valid;              ///< True if analysis was successful
    
    /**
     * @brief Default constructor
     */
    ReflectionAnalysis()
        : hue(0), saturation(0), value(0)
        , reflection_score(0), is_reflection(false), is_valid(false) {}
};

/**
 * @brief Modern light reflection elimination function
 * 
 * Analyzes a pixel in the image to determine if it represents a light reflection
 * rather than a genuine obstacle. Uses HSV color space analysis where reflections
 * typically have high brightness (Value) but low color saturation.
 * 
 * @param main_image Input BGR image from camera
 * @param x_coordinate X coordinate of pixel to analyze (row)
 * @param y_coordinate Y coordinate of pixel to analyze (column)
 * @param thresholds HSV thresholds for reflection detection
 * @param analysis Output structure containing detailed analysis results
 * @return true if analysis was successful, false on error
 * 
 * @note Coordinates are in OpenCV format (row, column)
 * @warning Input coordinates must be within image bounds
 */
bool analyzePixelReflection(const cv::Mat& main_image,
                          int x_coordinate,
                          int y_coordinate,
                          const HSVThresholds& thresholds,
                          ReflectionAnalysis& analysis);

/**
 * @brief Analyze reflection in a local region around a point
 * 
 * Performs reflection analysis on a small region around the specified point
 * to provide more robust detection by considering neighboring pixels.
 * 
 * @param main_image Input BGR image from camera
 * @param x_coordinate X coordinate of center pixel (row)
 * @param y_coordinate Y coordinate of center pixel (column)
 * @param region_size Size of the square region to analyze (default: 3x3)
 * @param thresholds HSV thresholds for reflection detection
 * @return Average reflection score for the region
 */
int analyzeRegionReflection(const cv::Mat& main_image,
                          int x_coordinate,
                          int y_coordinate,
                          int region_size = 3,
                          const HSVThresholds& thresholds = HSVThresholds());

/**
 * @brief Legacy light reflection elimination function
 * 
 * @deprecated Use analyzePixelReflection() with HSVThresholds and ReflectionAnalysis instead
 * 
 * @param Main_Image Input BGR image from camera
 * @param x_vector_point X coordinate of pixel to analyze
 * @param y_vector_point Y coordinate of pixel to analyze
 * @return Reflection score (higher values indicate more likely reflection)
 */
int light_reflection_elimination(cv::Mat Main_Image, int x_vector_point, int y_vector_point);

/**
 * @brief Validate image and coordinates for reflection analysis
 * 
 * @param image Input image to validate
 * @param x_coord X coordinate to check
 * @param y_coord Y coordinate to check
 * @return true if inputs are valid, false otherwise
 */
bool validateReflectionInputs(const cv::Mat& image, int x_coord, int y_coord);

/**
 * @brief Convert BGR image to HSV for analysis
 * 
 * @param bgr_image Input BGR image
 * @param hsv_image Output HSV image
 * @return true if conversion was successful, false otherwise
 */
bool convertToHSV(const cv::Mat& bgr_image, cv::Mat& hsv_image);

/**
 * @brief Compute reflection score from HSV values
 * 
 * The reflection score is computed as (Value - Saturation), where:
 * - High Value (brightness) with low Saturation typically indicates reflection
 * - Higher scores suggest more likely reflection
 * 
 * @param hue Hue component (0-179)
 * @param saturation Saturation component (0-255)
 * @param value Value/brightness component (0-255)
 * @return Reflection score
 */
inline int computeReflectionScore(int hue, int saturation, int value) {
    (void)hue; // Hue not used in current algorithm but kept for future enhancements
    return value - saturation;
}

/**
 * @brief Classify pixel as reflection based on score and thresholds
 * 
 * @param reflection_score Computed reflection score
 * @param thresholds HSV thresholds for classification
 * @return true if pixel is classified as reflection, false otherwise
 */
inline bool isReflection(int reflection_score, const HSVThresholds& thresholds) {
    return reflection_score >= thresholds.reflection_threshold;
}

/**
 * @brief Create a mask highlighting likely reflection regions
 * 
 * @param main_image Input BGR image
 * @param thresholds HSV thresholds for reflection detection
 * @param reflection_mask Output binary mask (255 = reflection, 0 = not reflection)
 * @return true if mask creation was successful, false otherwise
 */
bool createReflectionMask(const cv::Mat& main_image,
                        const HSVThresholds& thresholds,
                        cv::Mat& reflection_mask);

} // namespace visual_sonar_ros

#endif // VISUAL_SONAR_ROS_LIGHT_REFLECTION_ELIMINATION_H
