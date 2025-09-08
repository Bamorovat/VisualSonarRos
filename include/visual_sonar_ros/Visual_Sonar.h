
/**
 * @file Visual_Sonar.h
 * @brief Visual Sonar algorithm for mobile robot navigation using omnidirectional vision
 * 
 * This header defines the Visual Sonar algorithm that simulates sonar-like obstacle detection
 * using computer vision techniques. The algorithm processes omnidirectional camera images
 * to detect obstacles and compute free path vectors for autonomous navigation.
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

#ifndef VISUAL_SONAR_ROS_VISUAL_SONAR_H
#define VISUAL_SONAR_ROS_VISUAL_SONAR_H

// Standard C++ includes
#include <memory>
#include <vector>
#include <utility>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

// Local includes
#include "light_reflection_elimination.h"

namespace visual_sonar_ros {

/**
 * @struct SonarConfig
 * @brief Configuration parameters for the Visual Sonar algorithm
 */
struct SonarConfig {
    int sonar_number;           ///< Total number of virtual sonar beams (default: 24)
    int first_sonar_degree;     ///< Starting sonar beam index (default: 10)
    int last_sonar_degree;      ///< Ending sonar beam index (default: 27)
    int first_pixel;            ///< Starting pixel distance from center (default: 65)
    
    /**
     * @brief Default constructor with standard parameters
     */
    SonarConfig() 
        : sonar_number(24)
        , first_sonar_degree(10)
        , last_sonar_degree(27)
        , first_pixel(65) {}
        
    /**
     * @brief Parameterized constructor
     * @param sonar_num Total number of sonar beams
     * @param first_deg First sonar degree
     * @param last_deg Last sonar degree  
     * @param first_pix First pixel distance
     */
    SonarConfig(int sonar_num, int first_deg, int last_deg, int first_pix)
        : sonar_number(sonar_num)
        , first_sonar_degree(first_deg)
        , last_sonar_degree(last_deg)
        , first_pixel(first_pix) {}
};

/**
 * @struct SonarResult
 * @brief Result structure containing computed navigation vectors
 */
struct SonarResult {
    int x_vector_sum;    ///< Sum of X components from all sonar beams
    int y_vector_sum;    ///< Sum of Y components from all sonar beams
    bool is_valid;       ///< Flag indicating if the result is valid
    
    /**
     * @brief Default constructor
     */
    SonarResult() : x_vector_sum(0), y_vector_sum(0), is_valid(false) {}
    
    /**
     * @brief Reset the result to default values
     */
    void reset() {
        x_vector_sum = 0;
        y_vector_sum = 0;
        is_valid = false;
    }
};

/**
 * @brief Main Visual Sonar processing function
 * 
 * Processes input images using virtual sonar beams to detect obstacles and compute
 * navigation vectors. This function simulates multiple sonar beams radiating from
 * the robot center to find free paths and obstacles.
 * 
 * @param main_image Original input image from camera
 * @param gradient_image Edge-detected image (processed with Sobel filter)
 * @param config Configuration parameters for sonar processing
 * @param result Output structure containing computed vectors
 * @return true if processing was successful, false otherwise
 * 
 * @note The function modifies the gradient_image by drawing the computed vector
 * @warning Input images must be valid and non-empty
 */
bool processVisualSonar(const cv::Mat& main_image, 
                       cv::Mat& gradient_image, 
                       const SonarConfig& config, 
                       SonarResult& result);

/**
 * @brief Legacy wrapper function for backward compatibility
 * 
 * @deprecated Use processVisualSonar() with SonarConfig and SonarResult instead
 * 
 * @param Main_Image Original input image
 * @param Gradian_Image Edge-detected image  
 * @param Sonar_Number Total number of sonar beams
 * @param First_Sonar_Degree Starting sonar beam index
 * @param Last_Sonar_Degree Ending sonar beam index
 * @param First_pixel Starting pixel distance from center
 * @param X_Vector_Sum Reference to X vector sum output
 * @param Y_Vector_Sum Reference to Y vector sum output
 */
void visual_sonar(cv::Mat Main_Image, 
                 cv::Mat Gradian_Image, 
                 int Sonar_Number, 
                 int First_Sonar_Degree, 
                 int Last_Sonar_Degree, 
                 int First_pixel, 
                 int& X_Vector_Sum, 
                 int& Y_Vector_Sum);

/**
 * @brief Validate input parameters for sonar processing
 * 
 * @param config Configuration to validate
 * @return true if configuration is valid, false otherwise
 */
bool validateSonarConfig(const SonarConfig& config);

/**
 * @brief Validate input images for processing
 * 
 * @param main_image Main input image
 * @param gradient_image Gradient/edge image
 * @return true if images are valid for processing, false otherwise
 */
bool validateInputImages(const cv::Mat& main_image, const cv::Mat& gradient_image);

} // namespace visual_sonar_ros

#endif // VISUAL_SONAR_ROS_VISUAL_SONAR_H
