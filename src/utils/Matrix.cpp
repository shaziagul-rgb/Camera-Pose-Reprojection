#include <vector>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "Matrix.hpp"


namespace Utils
{
    // Library type conversions OpenCV/Eigen3
    Eigen::Matrix3d MatToMatrix3d(const cv::Mat &m)
    {
		Eigen::Matrix3d result;
		if (m.depth() == CV_32F)
		{
			for (int i = 0; i < 3; ++i) {
				for (int j = 0; j < 3; ++j) {
					result(i, j) = (double)m.at<float>(i, j);
				}
			}
		}
		else
		{
			for (int i = 0; i < 3; ++i) {
				for (int j = 0; j < 3; ++j) {
					result(i, j) = m.at<double>(i, j);
				}
			}
		}
		return result;
    }
    Eigen::Vector3d MatToVector3d(const cv::Mat &m)
    {
        Eigen::Vector3d result;
		if (m.depth() == CV_32F)
		{
			for (int i = 0; i < 3; ++i) {
				result(i, 0) = (double)m.at<float>(i, 0);
			}
		}
		else
		{
			for (int i = 0; i < 3; ++i) {
				result(i, 0) = m.at<double>(i, 0);
			}
		}
        return result;
    }
    Eigen::Vector4d MatToVector4d(const cv::Mat &m)
    {
        Eigen::Vector4d result;
		if (m.depth() == CV_32F)
		{
			for (int i = 0; i < 4; ++i) {
				result(i, 0) = (double)m.at<float>(i, 0);
			}
		}
		else
		{
			for (int i = 0; i < 4; ++i) {
				result(i, 0) = m.at<double>(i, 0);
			}
		}
        return result;
    }
    cv::Mat Matrix3dToMat(const Eigen::Matrix3d &m)
    {
        cv::Mat result(3, 3, CV_32FC1);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result.at<float>(i, j) = m(i, j);
            }
        }
        return result;
    }
    cv::Mat Vector3dToMat(const Eigen::Vector3d &m)
    {
        cv::Mat result(3, 1, CV_32FC1);
        for (int i = 0; i < 3; ++i) {
            result.at<float>(i, 0) = m(i, 0);
        }
        return result;
    }
    cv::Mat Vector4dToMat(const Eigen::Vector4d &m)
    {
        cv::Mat result(4, 1, CV_32FC1);
        for (int i = 0; i < 4; ++i) {
            result.at<float>(i, 0) = m(i, 0);
        }
        return result;
    }


	// Convert a quaternion vector to a rotation matrix (CityOfSights)
	cv::Mat quaternionToRotmat(const cv::Vec4f &q)
	{   // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm

		cv::Mat result(cv::Size(3, 3), CV_32F);
		float qx = q[0], qy = q[1], qz = q[2], qw = q[3];

		result.at<float>(0, 0) = 1.0 - 2.0*qy*qy + 2.0*qz*qz;
		result.at<float>(0, 1) = 2.0*qx*qy - 2.0*qz*qw;
		result.at<float>(0, 2) = 2.0*qx*qz + 2.0*qy*qw;
		result.at<float>(1, 0) = 2.0*qx*qy + 2.0*qz*qw;
		result.at<float>(1, 1) = 1.0 - 2.0*qx*qx + 2.0*qz*qz;
		result.at<float>(1, 2) = 2 * qy*qz - 2 * qx*qw;
		result.at<float>(2, 0) = 2.0*qx*qz - 2.0*qy*qw;
		result.at<float>(2, 1) = 2.0*qy*qz + 2.0*qx*qw;
		result.at<float>(2, 2) = 1.0 - 2.0*qx*qx + 2.0*qy*qy;

		//result.at<float>(0, 0) = 1.0 - 2.0*qy*qy - 2.0*qz*qz;
		//result.at<float>(1, 1) = 1.0 - 2.0*qx*qx - 2.0*qz*qz;
		//result.at<float>(2, 2) = 1.0 - 2.0*qx*qx - 2.0*qy*qy;

		return result;
	}
	cv::Mat quaternionToRotmatColmap(const cv::Vec4f& q)
	{	//https://fzheng.me/2017/11/12/quaternion_conventions_en/
		cv::Mat result(cv::Size(3, 3), CV_32F);
		float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
		
		Eigen::Quaternionf quat(qw, qx, qy, qz);
		Eigen::Matrix3f rotmat = quat.toRotationMatrix();

		result.at<float>(0, 0) = rotmat(0, 0);
		result.at<float>(0, 1) = rotmat(0, 1);
		result.at<float>(0, 2) = rotmat(0, 2);
		result.at<float>(1, 0) = rotmat(1, 0);
		result.at<float>(1, 1) = rotmat(1, 1);
		result.at<float>(1, 2) = rotmat(1, 2);
		result.at<float>(2, 0) = rotmat(2, 0);
		result.at<float>(2, 1) = rotmat(2, 1);
		result.at<float>(2, 2) = rotmat(2, 2);

		return result;
	}


    // Invert K matrix (for converting from pixel co-ords to normalized 2D)
    cv::Mat invertKMat(const cv::Mat &K)
    {
        cv::Mat result = cv::Mat::eye(K.size(), K.type());
		result.at<float>(0, 0) = 1.0 / K.at<float>(0, 0);
		result.at<float>(1, 1) = 1.0 / K.at<float>(1, 1);
		result.at<float>(0, 2) = -(K.at<float>(0, 2) / K.at<float>(0, 0));
		result.at<float>(1, 2) = -(K.at<float>(1, 2) / K.at<float>(1, 1));
		return result;
    }


	// Scale a K matrix given the original resolution, and the target resolution in format ORIG_w,ORIG_h,TARG_w,TARG_h
	cv::Mat scaleKMat(const cv::Mat &K, const std::vector<int> &scaleParams)
	{
		if (scaleParams.size() != 4)
		{
			std::cout << "[Error] -- To scale a K matrix, specify four integers in format ORIG_w,ORIG_h,TARG_w,TARG_h" << std::endl;
			return K;
		}

		cv::Mat result = K.clone();
		result.at<float>(0, 0) = (K.at<float>(0, 0) / scaleParams[0]) * scaleParams[2]; // fx
		result.at<float>(0, 2) = (K.at<float>(0, 2) / scaleParams[0]) * scaleParams[2]; // cx
		result.at<float>(1, 1) = (K.at<float>(1, 1) / scaleParams[1]) * scaleParams[3]; // fy
		result.at<float>(1, 2) = (K.at<float>(1, 2) / scaleParams[1]) * scaleParams[3]; // cy
		return result;
	}


}

