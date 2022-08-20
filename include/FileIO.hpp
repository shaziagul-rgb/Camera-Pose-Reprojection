#pragma once

#include <vector>
#include <string>
#include <unordered_map>

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

namespace Utils
{
	// Read calibration files
	DECLSPEC cv::Mat getProjectionMatrix(const std::string &file);
	DECLSPEC cv::Mat getKMatrix(const std::string &file);
	DECLSPEC cv::Mat getRMatrix(const std::string &file);
	DECLSPEC cv::Mat getTMatrix(const std::string &file);

	// Read calibration from CityOfSightsData
	DECLSPEC cv::Mat getRMatrixCoS(const std::string &csvPoseFile, int frameID);
	DECLSPEC cv::Mat getTMatrixCoS(const std::string &csvPoseFile, int frameID);

	// Read point cloud from Ply file
	DECLSPEC void getPointsPlyFile(const std::string &mapFile, std::vector<cv::Mat> &outVertices, cv::Mat &outColours);

	// Reads 3dpoint, observations, and pose data from a COLMAP text model, given the model directory (containing images.txt, points3D.txt, and cameras.txt), and the image name
	DECLSPEC bool colmapGetReconstructionQuaternion(
		const std::string &colmap_model_dirname,
		std::unordered_map<int, cv::Point3f> &dst_mps,
		std::unordered_map<int, std::vector<std::pair<int, int> > > &dst_obs,
		std::unordered_map<int, std::vector<int> > &dst_corr,
		std::unordered_map<int, cv::Mat> &dst_Rs,
		std::unordered_map<int, cv::Mat> &dst_ts);

	// Reads 3dpoint, observations, and pose data from a COLMAP text model, given the model directory (containing images.txt, points3D.txt, and cameras.txt), and the image name
	DECLSPEC bool colmapGetReconstruction(
		const std::string &colmap_model_dirname,
		std::unordered_map<int, cv::Point3f> &dst_mps,
		std::unordered_map<int, std::vector<std::pair<int, int> > > &dst_obs,
		std::unordered_map<int, std::vector<int> > &dst_corr,
		std::unordered_map<int, cv::Mat> &dst_Rs,
		std::unordered_map<int, cv::Mat> &dst_ts,
		std::unordered_map<int, float> &dst_fs);

	DECLSPEC std::vector<cv::Point2f> get2DPoints(
		const std::string &points_dirname,
		const std::string &ImageName

	);

	DECLSPEC std::vector<cv::Point3f> get3DPoints(
		const std::string &points_dirname,
		const std::string &ImageName

	);

	// Reads 3dpoint, observations, and pose data from a COLMAP text model, given the model directory (containing images.txt, points3D.txt, and cameras.txt), and the image name
	DECLSPEC bool colmapGetReconstruction(
		const std::string &colmap_model_dirname,
		std::unordered_map<int, cv::Point3f> &dst_mps,
		std::unordered_map<int, cv::Vec3b> &dst_mps_colrs,
		std::unordered_map<int, std::vector<std::pair<int, int> > > &dst_obs,
		std::unordered_map<int, std::vector<int> > &dst_corr,
		std::unordered_map<int, cv::Mat> &dst_Rs,
		std::unordered_map<int, cv::Mat> &dst_ts,
		std::unordered_map<int, float> &dst_fs);

	// Reads keypoint and descriptor data from a COLMAP db model
	DECLSPEC bool colmapGetFeatures(
		const std::string &db_filename,
		std::unordered_map<int, std::vector<cv::KeyPoint> > &dst_kps,
		std::unordered_map<int, cv::Mat> &dst_descs,
		std::unordered_map<int, std::string> &dst_names);

	// Find the image_id given a filename
	DECLSPEC bool colmapSearchImage(
		const std::string &db_filename,
		const std::string &image_filename,
		int &dst_image_id);
}
