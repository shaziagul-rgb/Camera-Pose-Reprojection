#pragma once


#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>


// Required to export symbols to DLL in windows
#ifdef _WIN32
#ifdef DLLEXPORT  
#define DECLSPEC __declspec(dllexport)
#else  
#define DECLSPEC __declspec(dllimport)
#endif 
#else
#define DECLSPEC  
#endif


#define deg2rad(x) ((x) * (M_PI / 180.0))
#define rad2deg(x) ((x) * (180.0 / M_PI))


namespace Utils
{
    // Library type conversions OpenCV/Eigen3
    DECLSPEC Eigen::Matrix3d MatToMatrix3d(const cv::Mat &m);
    DECLSPEC Eigen::Vector3d MatToVector3d(const cv::Mat &m);
    DECLSPEC Eigen::Vector4d MatToVector4d(const cv::Mat &m);
    DECLSPEC cv::Mat Matrix3dToMat(const Eigen::Matrix3d &m);
    DECLSPEC cv::Mat Vector3dToMat(const Eigen::Vector3d &m);
    DECLSPEC cv::Mat Vector4dToMat(const Eigen::Vector4d &m);


	// Convert a quaternion vector to a rotation matrix (CityOfSights)
	DECLSPEC cv::Mat quaternionToRotmat(const cv::Vec4f& q);
	DECLSPEC cv::Mat quaternionToRotmatColmap(const cv::Vec4f& q);


    // Invert K matrix (for converting from pixel co-ords to normalized 2D)
    DECLSPEC cv::Mat invertKMat(const cv::Mat &K);


	// Scale a K matrix given the original resolution, and the target resolution in format ORIG_w,ORIG_h,TARG_w,TARG_h
	DECLSPEC cv::Mat scaleKMat(const cv::Mat &K, const std::vector<int> &scaleParams);


}

