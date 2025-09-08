/**
 * @file edge_detection.cpp
 * @brief Implementation of multiple edge detection algorithms for Visual Sonar
 * 
 * This file implements various edge detection algorithms optimized for Visual Sonar
 * obstacle detection, including Sobel, Canny, Laplacian, Scharr, Roberts, and Prewitt
 * methods with comprehensive parameter tuning and preprocessing capabilities.
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
#include <map>
#include <cmath>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Local includes
#include "edge_detection.h"

namespace visual_sonar_ros {

/**
 * @brief Validates edge detection configuration parameters
 */
bool validateEdgeDetectionConfig(const EdgeDetectionConfig& config) {
    if (config.threshold < 0 || config.threshold > 255) {
        std::cerr << "Error: Threshold must be in range [0, 255]" << std::endl;
        return false;
    }
    
    // Method-specific validation
    switch (config.method) {
        case EdgeDetectionMethod::SOBEL:
            if (config.sobel_config.kernel_size != 1 && config.sobel_config.kernel_size != 3 &&
                config.sobel_config.kernel_size != 5 && config.sobel_config.kernel_size != 7) {
                std::cerr << "Error: Sobel kernel size must be 1, 3, 5, or 7" << std::endl;
                return false;
            }
            break;
            
        case EdgeDetectionMethod::CANNY:
            if (config.canny_config.low_threshold >= config.canny_config.high_threshold) {
                std::cerr << "Error: Canny low threshold must be less than high threshold" << std::endl;
                return false;
            }
            break;
            
        case EdgeDetectionMethod::LAPLACIAN:
            if (config.laplacian_config.kernel_size != 1 && config.laplacian_config.kernel_size != 3 &&
                config.laplacian_config.kernel_size != 5 && config.laplacian_config.kernel_size != 7) {
                std::cerr << "Error: Laplacian kernel size must be 1, 3, 5, or 7" << std::endl;
                return false;
            }
            break;
            
        default:
            // Other methods have fewer constraints
            break;
    }
    
    return true;
}

/**
 * @brief Applies image preprocessing
 */
bool applyPreprocessing(const cv::Mat& input_image, cv::Mat& output_image, int kernel_size) {
    if (input_image.empty()) {
        std::cerr << "Error: Cannot preprocess empty image" << std::endl;
        return false;
    }
    
    if (kernel_size <= 0 || kernel_size % 2 == 0) {
        std::cerr << "Warning: Kernel size should be positive and odd, using default 5" << std::endl;
        kernel_size = 5;
    }
    
    try {
        cv::GaussianBlur(input_image, output_image, cv::Size(kernel_size, kernel_size), 0, 0, cv::BORDER_DEFAULT);
        return true;
    } catch (const cv::Exception& e) {
        std::cerr << "Error applying preprocessing: " << e.what() << std::endl;
        return false;
    }
}

/**
 * @brief Applies post-processing operations
 */
bool applyPostprocessing(const cv::Mat& input_image, cv::Mat& output_image) {
    if (input_image.empty()) {
        std::cerr << "Error: Cannot postprocess empty image" << std::endl;
        return false;
    }
    
    try {
        // Apply morphological operations to clean up edges
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        
        // Close small gaps in edges
        cv::morphologyEx(input_image, output_image, cv::MORPH_CLOSE, kernel);
        
        // Remove small noise
        cv::morphologyEx(output_image, output_image, cv::MORPH_OPEN, kernel);
        
        return true;
    } catch (const cv::Exception& e) {
        std::cerr << "Error applying postprocessing: " << e.what() << std::endl;
        return false;
    }
}

/**
 * @brief Sobel edge detection implementation
 */
bool applySobelEdgeDetection(const cv::Mat& input_image,
                           const SobelConfig& config,
                           EdgeDetectionResult& result) {
    
    try {
        cv::Mat gray_image, blurred_image;
        
        // Convert to grayscale if needed
        if (input_image.channels() == 3) {
            cv::cvtColor(input_image, gray_image, cv::COLOR_BGR2GRAY);
        } else {
            gray_image = input_image.clone();
        }
        
        // Apply Gaussian blur for noise reduction
        cv::GaussianBlur(gray_image, blurred_image, 
                        cv::Size(config.blur_kernel_size, config.blur_kernel_size), 0, 0, cv::BORDER_DEFAULT);
        
        // Compute gradients
        cv::Mat grad_x, grad_y;
        cv::Mat abs_grad_x, abs_grad_y;
        
        // X gradient
        cv::Sobel(blurred_image, grad_x, CV_16S, 1, 0, config.kernel_size, config.scale, config.delta, cv::BORDER_DEFAULT);
        cv::convertScaleAbs(grad_x, abs_grad_x);
        
        // Y gradient
        cv::Sobel(blurred_image, grad_y, CV_16S, 0, 1, config.kernel_size, config.scale, config.delta, cv::BORDER_DEFAULT);
        cv::convertScaleAbs(grad_y, abs_grad_y);
        
        // Combine gradients
        cv::Mat combined_grad;
        cv::addWeighted(abs_grad_x, config.x_weight, abs_grad_y, config.y_weight, 0, combined_grad);
        
        // Store gradient magnitude and direction
        result.gradient_magnitude = combined_grad.clone();
        cv::phase(grad_x, grad_y, result.gradient_direction, true);
        
        return true;
        
    } catch (const cv::Exception& e) {
        result.error_message = "Sobel edge detection failed: " + std::string(e.what());
        return false;
    }
}

/**
 * @brief Canny edge detection implementation
 */
bool applyCannyEdgeDetection(const cv::Mat& input_image,
                           const CannyConfig& config,
                           EdgeDetectionResult& result) {
    
    try {
        cv::Mat gray_image, blurred_image;
        
        // Convert to grayscale if needed
        if (input_image.channels() == 3) {
            cv::cvtColor(input_image, gray_image, cv::COLOR_BGR2GRAY);
        } else {
            gray_image = input_image.clone();
        }
        
        // Apply Gaussian blur
        cv::GaussianBlur(gray_image, blurred_image,
                        cv::Size(config.blur_kernel_size, config.blur_kernel_size), 0, 0, cv::BORDER_DEFAULT);
        
        // Apply Canny edge detection
        cv::Mat canny_edges;
        cv::Canny(blurred_image, canny_edges, config.low_threshold, config.high_threshold,
                 config.aperture_size, config.l2_gradient);
        
        result.gradient_magnitude = canny_edges.clone();
        
        return true;
        
    } catch (const cv::Exception& e) {
        result.error_message = "Canny edge detection failed: " + std::string(e.what());
        return false;
    }
}

/**
 * @brief Laplacian edge detection implementation
 */
bool applyLaplacianEdgeDetection(const cv::Mat& input_image,
                               const LaplacianConfig& config,
                               EdgeDetectionResult& result) {
    
    try {
        cv::Mat gray_image, blurred_image;
        
        // Convert to grayscale if needed
        if (input_image.channels() == 3) {
            cv::cvtColor(input_image, gray_image, cv::COLOR_BGR2GRAY);
        } else {
            gray_image = input_image.clone();
        }
        
        // Apply Gaussian blur
        cv::GaussianBlur(gray_image, blurred_image,
                        cv::Size(config.blur_kernel_size, config.blur_kernel_size), 0, 0, cv::BORDER_DEFAULT);
        
        // Apply Laplacian
        cv::Mat laplacian;
        cv::Laplacian(blurred_image, laplacian, CV_16S, config.kernel_size, config.scale, config.delta, cv::BORDER_DEFAULT);
        
        // Convert to absolute values
        cv::Mat abs_laplacian;
        cv::convertScaleAbs(laplacian, abs_laplacian);
        
        result.gradient_magnitude = abs_laplacian.clone();
        
        return true;
        
    } catch (const cv::Exception& e) {
        result.error_message = "Laplacian edge detection failed: " + std::string(e.what());
        return false;
    }
}

/**
 * @brief Scharr edge detection implementation
 */
bool applyScharrEdgeDetection(const cv::Mat& input_image,
                            const ScharrConfig& config,
                            EdgeDetectionResult& result) {
    
    try {
        cv::Mat gray_image, blurred_image;
        
        // Convert to grayscale if needed
        if (input_image.channels() == 3) {
            cv::cvtColor(input_image, gray_image, cv::COLOR_BGR2GRAY);
        } else {
            gray_image = input_image.clone();
        }
        
        // Apply Gaussian blur
        cv::GaussianBlur(gray_image, blurred_image,
                        cv::Size(config.blur_kernel_size, config.blur_kernel_size), 0, 0, cv::BORDER_DEFAULT);
        
        // Compute Scharr gradients
        cv::Mat grad_x, grad_y;
        cv::Mat abs_grad_x, abs_grad_y;
        
        // X gradient
        cv::Scharr(blurred_image, grad_x, CV_16S, 1, 0, config.scale, config.delta, cv::BORDER_DEFAULT);
        cv::convertScaleAbs(grad_x, abs_grad_x);
        
        // Y gradient
        cv::Scharr(blurred_image, grad_y, CV_16S, 0, 1, config.scale, config.delta, cv::BORDER_DEFAULT);
        cv::convertScaleAbs(grad_y, abs_grad_y);
        
        // Combine gradients
        cv::Mat combined_grad;
        cv::addWeighted(abs_grad_x, config.x_weight, abs_grad_y, config.y_weight, 0, combined_grad);
        
        result.gradient_magnitude = combined_grad.clone();
        cv::phase(grad_x, grad_y, result.gradient_direction, true);
        
        return true;
        
    } catch (const cv::Exception& e) {
        result.error_message = "Scharr edge detection failed: " + std::string(e.what());
        return false;
    }
}

/**
 * @brief Roberts cross-gradient edge detection implementation
 */
bool applyRobertsEdgeDetection(const cv::Mat& input_image, EdgeDetectionResult& result) {
    
    try {
        cv::Mat gray_image;
        
        // Convert to grayscale if needed
        if (input_image.channels() == 3) {
            cv::cvtColor(input_image, gray_image, cv::COLOR_BGR2GRAY);
        } else {
            gray_image = input_image.clone();
        }
        
        // Roberts kernels
        cv::Mat roberts_x = (cv::Mat_<float>(2, 2) << 1, 0, 0, -1);
        cv::Mat roberts_y = (cv::Mat_<float>(2, 2) << 0, 1, -1, 0);
        
        cv::Mat grad_x, grad_y;
        cv::filter2D(gray_image, grad_x, CV_32F, roberts_x);
        cv::filter2D(gray_image, grad_y, CV_32F, roberts_y);
        
        // Compute magnitude
        cv::Mat magnitude;
        cv::magnitude(grad_x, grad_y, magnitude);
        
        // Convert to 8-bit
        cv::Mat abs_magnitude;
        cv::convertScaleAbs(magnitude, abs_magnitude);
        
        result.gradient_magnitude = abs_magnitude.clone();
        
        return true;
        
    } catch (const cv::Exception& e) {
        result.error_message = "Roberts edge detection failed: " + std::string(e.what());
        return false;
    }
}

/**
 * @brief Prewitt edge detection implementation
 */
bool applyPrewittEdgeDetection(const cv::Mat& input_image, EdgeDetectionResult& result) {
    
    try {
        cv::Mat gray_image;
        
        // Convert to grayscale if needed
        if (input_image.channels() == 3) {
            cv::cvtColor(input_image, gray_image, cv::COLOR_BGR2GRAY);
        } else {
            gray_image = input_image.clone();
        }
        
        // Prewitt kernels
        cv::Mat prewitt_x = (cv::Mat_<float>(3, 3) << -1, 0, 1, -1, 0, 1, -1, 0, 1);
        cv::Mat prewitt_y = (cv::Mat_<float>(3, 3) << -1, -1, -1, 0, 0, 0, 1, 1, 1);
        
        cv::Mat grad_x, grad_y;
        cv::filter2D(gray_image, grad_x, CV_32F, prewitt_x);
        cv::filter2D(gray_image, grad_y, CV_32F, prewitt_y);
        
        // Compute magnitude
        cv::Mat magnitude;
        cv::magnitude(grad_x, grad_y, magnitude);
        
        // Convert to 8-bit
        cv::Mat abs_magnitude;
        cv::convertScaleAbs(magnitude, abs_magnitude);
        
        result.gradient_magnitude = abs_magnitude.clone();
        
        return true;
        
    } catch (const cv::Exception& e) {
        result.error_message = "Prewitt edge detection failed: " + std::string(e.what());
        return false;
    }
}

/**
 * @brief Main edge detection function
 */
bool detectEdges(const cv::Mat& input_image,
                const EdgeDetectionConfig& config,
                EdgeDetectionResult& result) {
    
    // Reset result
    result = EdgeDetectionResult();
    result.method_used = config.method;
    
    // Validate input
    if (input_image.empty()) {
        result.error_message = "Input image is empty";
        return false;
    }
    
    // Validate configuration
    if (!validateEdgeDetectionConfig(config)) {
        result.error_message = "Invalid configuration";
        return false;
    }
    
    // Apply preprocessing if enabled
    cv::Mat processed_image;
    if (config.enable_preprocessing) {
        if (!applyPreprocessing(input_image, processed_image)) {
            result.error_message = "Preprocessing failed";
            return false;
        }
    } else {
        processed_image = input_image.clone();
    }
    
    // Apply the selected edge detection method
    bool method_success = false;
    
    switch (config.method) {
        case EdgeDetectionMethod::SOBEL:
            method_success = applySobelEdgeDetection(processed_image, config.sobel_config, result);
            break;
            
        case EdgeDetectionMethod::CANNY:
            method_success = applyCannyEdgeDetection(processed_image, config.canny_config, result);
            break;
            
        case EdgeDetectionMethod::LAPLACIAN:
            method_success = applyLaplacianEdgeDetection(processed_image, config.laplacian_config, result);
            break;
            
        case EdgeDetectionMethod::SCHARR:
            method_success = applyScharrEdgeDetection(processed_image, config.scharr_config, result);
            break;
            
        case EdgeDetectionMethod::ROBERTS:
            method_success = applyRobertsEdgeDetection(processed_image, result);
            break;
            
        case EdgeDetectionMethod::PREWITT:
            method_success = applyPrewittEdgeDetection(processed_image, result);
            break;
            
        default:
            result.error_message = "Unknown edge detection method";
            return false;
    }
    
    if (!method_success) {
        return false;
    }
    
    // Apply binary threshold (except for Canny which is already binary)
    if (config.method != EdgeDetectionMethod::CANNY) {
        cv::Mat binary_edges;
        cv::threshold(result.gradient_magnitude, binary_edges, config.threshold, 255, cv::THRESH_BINARY);
        result.edge_image = binary_edges;
    } else {
        result.edge_image = result.gradient_magnitude.clone();
    }
    
    // Apply post-processing if enabled
    if (config.enable_postprocessing) {
        cv::Mat postprocessed_image;
        if (applyPostprocessing(result.edge_image, postprocessed_image)) {
            result.edge_image = postprocessed_image;
        }
    }
    
    result.is_valid = true;
    return true;
}

/**
 * @brief Convert edge detection method enum to string
 */
std::string edgeDetectionMethodToString(EdgeDetectionMethod method) {
    static const std::map<EdgeDetectionMethod, std::string> method_names = {
        {EdgeDetectionMethod::SOBEL, "Sobel"},
        {EdgeDetectionMethod::CANNY, "Canny"},
        {EdgeDetectionMethod::LAPLACIAN, "Laplacian"},
        {EdgeDetectionMethod::SCHARR, "Scharr"},
        {EdgeDetectionMethod::ROBERTS, "Roberts"},
        {EdgeDetectionMethod::PREWITT, "Prewitt"}
    };
    
    auto it = method_names.find(method);
    return (it != method_names.end()) ? it->second : "Unknown";
}

/**
 * @brief Get recommended configuration for specific scenarios
 */
EdgeDetectionConfig getRecommendedConfig(const std::string& scenario) {
    EdgeDetectionConfig config;
    
    if (scenario == "indoor") {
        // Indoor: good lighting, less noise
        config.method = EdgeDetectionMethod::SOBEL;
        config.threshold = 30;
        config.sobel_config.blur_kernel_size = 3;
        
    } else if (scenario == "outdoor") {
        // Outdoor: variable lighting, more noise
        config.method = EdgeDetectionMethod::CANNY;
        config.canny_config.low_threshold = 50;
        config.canny_config.high_threshold = 150;
        config.canny_config.blur_kernel_size = 5;
        
    } else if (scenario == "low_light") {
        // Low light: amplify edges, reduce noise
        config.method = EdgeDetectionMethod::SCHARR;
        config.threshold = 20;
        config.scharr_config.blur_kernel_size = 7;
        config.enable_postprocessing = true;
        
    } else if (scenario == "high_contrast") {
        // High contrast: precise edge detection
        config.method = EdgeDetectionMethod::LAPLACIAN;
        config.threshold = 40;
        config.laplacian_config.blur_kernel_size = 3;
        
    } else {
        // Default: balanced performance
        config.method = EdgeDetectionMethod::SOBEL;
        config.threshold = 27;
    }
    
    return config;
}

/**
 * @brief Legacy Sobel function for backward compatibility
 * @deprecated Use detectEdges() with EdgeDetectionConfig instead
 */
void sobel(cv::Mat _main_Image, cv::Mat& grad2, int Threshold) {
    // Create configuration for legacy compatibility
    EdgeDetectionConfig config(EdgeDetectionMethod::SOBEL, Threshold);
    
    // Create result structure
    EdgeDetectionResult result;
    
    // Apply modern edge detection
    if (detectEdges(_main_Image, config, result)) {
        grad2 = result.edge_image.clone();
    } else {
        std::cerr << "Warning: Edge detection failed, creating empty result" << std::endl;
        grad2 = cv::Mat::zeros(_main_Image.size(), CV_8UC1);
    }
}

} // namespace visual_sonar_ros
