/**
 * @file edge_detection.h
 * @brief Multiple edge detection algorithms for Visual Sonar image processing
 * 
 * This header defines various edge detection algorithms optimized for Visual Sonar
 * obstacle detection. The algorithms include Sobel, Canny, Laplacian, and Scharr
 * methods with configurable parameters for different lighting and environmental
 * conditions in mobile robot navigation scenarios.
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

#ifndef VISUAL_SONAR_ROS_EDGE_DETECTION_H
#define VISUAL_SONAR_ROS_EDGE_DETECTION_H

// Standard C++ includes
#include <string>
#include <vector>
#include <memory>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace visual_sonar_ros {

/**
 * @enum EdgeDetectionMethod
 * @brief Available edge detection algorithms
 */
enum class EdgeDetectionMethod {
    SOBEL,              ///< Sobel edge detection (default, good for general use)
    CANNY,              ///< Canny edge detection (precise edges, good noise handling)
    LAPLACIAN,          ///< Laplacian edge detection (sensitive to noise but fast)
    SCHARR,             ///< Scharr edge detection (improved Sobel, better rotation invariance)
    ROBERTS,            ///< Roberts cross-gradient (simple, fast for basic edges)
    PREWITT             ///< Prewitt operator (similar to Sobel, different kernel)
};

/**
 * @struct SobelConfig
 * @brief Configuration parameters for Sobel edge detection
 */
struct SobelConfig {
    int kernel_size;        ///< Sobel kernel size (1, 3, 5, or 7)
    double scale;           ///< Scale factor for derivatives
    double delta;           ///< Optional delta value added to results
    double x_weight;        ///< Weight for X gradient (default: 0.7)
    double y_weight;        ///< Weight for Y gradient (default: 0.7)
    int blur_kernel_size;   ///< Gaussian blur kernel size for noise reduction
    
    /**
     * @brief Default constructor with standard Sobel parameters
     */
    SobelConfig()
        : kernel_size(3)
        , scale(1.0)
        , delta(0.0)
        , x_weight(0.7)
        , y_weight(0.7)
        , blur_kernel_size(5) {}
};

/**
 * @struct CannyConfig
 * @brief Configuration parameters for Canny edge detection
 */
struct CannyConfig {
    double low_threshold;   ///< Lower threshold for edge linking
    double high_threshold;  ///< Upper threshold for edge linking
    int aperture_size;      ///< Aperture size for Sobel operator
    bool l2_gradient;       ///< Use L2 gradient instead of L1
    int blur_kernel_size;   ///< Gaussian blur kernel size
    
    /**
     * @brief Default constructor with standard Canny parameters
     */
    CannyConfig()
        : low_threshold(50.0)
        , high_threshold(150.0)
        , aperture_size(3)
        , l2_gradient(false)
        , blur_kernel_size(5) {}
};

/**
 * @struct LaplacianConfig
 * @brief Configuration parameters for Laplacian edge detection
 */
struct LaplacianConfig {
    int kernel_size;        ///< Laplacian kernel size (1, 3, 5, or 7)
    double scale;           ///< Scale factor for the derivative
    double delta;           ///< Optional delta value
    int blur_kernel_size;   ///< Gaussian blur kernel size
    
    /**
     * @brief Default constructor with standard Laplacian parameters
     */
    LaplacianConfig()
        : kernel_size(3)
        , scale(1.0)
        , delta(0.0)
        , blur_kernel_size(5) {}
};

/**
 * @struct ScharrConfig
 * @brief Configuration parameters for Scharr edge detection
 */
struct ScharrConfig {
    double scale;           ///< Scale factor for derivatives
    double delta;           ///< Optional delta value
    double x_weight;        ///< Weight for X gradient
    double y_weight;        ///< Weight for Y gradient
    int blur_kernel_size;   ///< Gaussian blur kernel size
    
    /**
     * @brief Default constructor with standard Scharr parameters
     */
    ScharrConfig()
        : scale(1.0)
        , delta(0.0)
        , x_weight(0.7)
        , y_weight(0.7)
        , blur_kernel_size(5) {}
};

/**
 * @struct EdgeDetectionConfig
 * @brief Unified configuration for all edge detection methods
 */
struct EdgeDetectionConfig {
    EdgeDetectionMethod method;     ///< Selected edge detection method
    int threshold;                  ///< Final binary threshold (0-255)
    bool enable_preprocessing;      ///< Enable Gaussian blur preprocessing
    bool enable_postprocessing;     ///< Enable morphological operations
    
    // Method-specific configurations
    SobelConfig sobel_config;
    CannyConfig canny_config;
    LaplacianConfig laplacian_config;
    ScharrConfig scharr_config;
    
    /**
     * @brief Default constructor with Sobel method
     */
    EdgeDetectionConfig()
        : method(EdgeDetectionMethod::SOBEL)
        , threshold(27)
        , enable_preprocessing(true)
        , enable_postprocessing(false) {}
        
    /**
     * @brief Constructor with specified method
     * @param detection_method Edge detection method to use
     * @param binary_threshold Threshold for binary output
     */
    EdgeDetectionConfig(EdgeDetectionMethod detection_method, int binary_threshold = 27)
        : method(detection_method)
        , threshold(binary_threshold)
        , enable_preprocessing(true)
        , enable_postprocessing(false) {}
};

/**
 * @struct EdgeDetectionResult
 * @brief Result structure containing edge detection information
 */
struct EdgeDetectionResult {
    cv::Mat edge_image;             ///< Output edge image (binary)
    cv::Mat gradient_magnitude;     ///< Gradient magnitude (if applicable)
    cv::Mat gradient_direction;     ///< Gradient direction (if applicable)
    EdgeDetectionMethod method_used; ///< Method that was used
    bool is_valid;                  ///< Success flag
    std::string error_message;      ///< Error description if failed
    
    /**
     * @brief Default constructor
     */
    EdgeDetectionResult()
        : method_used(EdgeDetectionMethod::SOBEL), is_valid(false) {}
};

/**
 * @brief Main edge detection function with multiple algorithm support
 * 
 * Applies the specified edge detection algorithm to the input image and returns
 * a binary edge image suitable for Visual Sonar processing.
 * 
 * @param input_image Input BGR or grayscale image
 * @param config Configuration specifying method and parameters
 * @param result Output structure containing edge image and metadata
 * @return true if edge detection was successful, false otherwise
 * 
 * @note Output image is binary (0 or 255 values)
 * @warning Input image must be valid and non-empty
 */
bool detectEdges(const cv::Mat& input_image,
                const EdgeDetectionConfig& config,
                EdgeDetectionResult& result);

/**
 * @brief Sobel edge detection implementation
 * 
 * @param input_image Input image (BGR or grayscale)
 * @param config Sobel-specific configuration
 * @param result Output edge detection result
 * @return true if successful, false otherwise
 */
bool applySobelEdgeDetection(const cv::Mat& input_image,
                           const SobelConfig& config,
                           EdgeDetectionResult& result);

/**
 * @brief Canny edge detection implementation
 * 
 * @param input_image Input image (BGR or grayscale)
 * @param config Canny-specific configuration
 * @param result Output edge detection result
 * @return true if successful, false otherwise
 */
bool applyCannyEdgeDetection(const cv::Mat& input_image,
                           const CannyConfig& config,
                           EdgeDetectionResult& result);

/**
 * @brief Laplacian edge detection implementation
 * 
 * @param input_image Input image (BGR or grayscale)
 * @param config Laplacian-specific configuration
 * @param result Output edge detection result
 * @return true if successful, false otherwise
 */
bool applyLaplacianEdgeDetection(const cv::Mat& input_image,
                               const LaplacianConfig& config,
                               EdgeDetectionResult& result);

/**
 * @brief Scharr edge detection implementation
 * 
 * @param input_image Input image (BGR or grayscale)
 * @param config Scharr-specific configuration
 * @param result Output edge detection result
 * @return true if successful, false otherwise
 */
bool applyScharrEdgeDetection(const cv::Mat& input_image,
                            const ScharrConfig& config,
                            EdgeDetectionResult& result);

/**
 * @brief Roberts cross-gradient edge detection implementation
 * 
 * @param input_image Input image (BGR or grayscale)
 * @param result Output edge detection result
 * @return true if successful, false otherwise
 */
bool applyRobertsEdgeDetection(const cv::Mat& input_image,
                             EdgeDetectionResult& result);

/**
 * @brief Prewitt edge detection implementation
 * 
 * @param input_image Input image (BGR or grayscale)
 * @param result Output edge detection result
 * @return true if successful, false otherwise
 */
bool applyPrewittEdgeDetection(const cv::Mat& input_image,
                             EdgeDetectionResult& result);

/**
 * @brief Legacy Sobel function for backward compatibility
 * 
 * @deprecated Use detectEdges() with EdgeDetectionConfig instead
 * 
 * @param _main_Image Input image
 * @param grad2 Output gradient image
 * @param Threshold Binary threshold value
 */
void sobel(cv::Mat _main_Image, cv::Mat& grad2, int Threshold);

/**
 * @brief Validate edge detection configuration
 * 
 * @param config Configuration to validate
 * @return true if configuration is valid, false otherwise
 */
bool validateEdgeDetectionConfig(const EdgeDetectionConfig& config);

/**
 * @brief Convert edge detection method enum to string
 * 
 * @param method Edge detection method
 * @return String representation of the method
 */
std::string edgeDetectionMethodToString(EdgeDetectionMethod method);

/**
 * @brief Get recommended configuration for specific scenarios
 * 
 * @param scenario Scenario name ("indoor", "outdoor", "low_light", "high_contrast")
 * @return Optimized configuration for the scenario
 */
EdgeDetectionConfig getRecommendedConfig(const std::string& scenario);

/**
 * @brief Apply image preprocessing (Gaussian blur, etc.)
 * 
 * @param input_image Input image
 * @param output_image Output preprocessed image
 * @param kernel_size Blur kernel size
 * @return true if successful, false otherwise
 */
bool applyPreprocessing(const cv::Mat& input_image, cv::Mat& output_image, int kernel_size = 5);

/**
 * @brief Apply post-processing operations (morphological operations, etc.)
 * 
 * @param input_image Input edge image
 * @param output_image Output processed image
 * @return true if successful, false otherwise
 */
bool applyPostprocessing(const cv::Mat& input_image, cv::Mat& output_image);

} // namespace visual_sonar_ros

#endif // VISUAL_SONAR_ROS_EDGE_DETECTION_H
