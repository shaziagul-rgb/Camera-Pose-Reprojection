#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <map>
#include <Eigen/Core>
#include <opencv2/core.hpp>
#include "FileIO.hpp"
#include "Matrix.hpp"

#include <sqlite3.h>

namespace Utils
{
	// Read calibration files
	cv::Mat getProjectionMatrix(const std::string &file)
	{
		std::ifstream inFile(file, std::ifstream::in);
		if (!inFile.is_open())
		{
			std::cout << "[Warning] -- Failed to open file: " << file << std::endl;
			return cv::Mat();
		}
		cv::Mat tmp = cv::Mat(3, 4, CV_32F);
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 4; ++j)
			{
				float v;
				inFile >> v;
				tmp.at<float>(i, j) = v;
			}
		}
		return tmp;
	}
	cv::Mat getKMatrix(const std::string &file)
	{
		std::ifstream inFile(file, std::ifstream::in);
		if (!inFile.is_open())
		{
			std::cout << "[Warning] -- Failed to open file: " << file << std::endl;
			return cv::Mat();
		}
		cv::Mat tmp = cv::Mat(3, 3, CV_32F);
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				float v;
				inFile >> v;
				tmp.at<float>(i, j) = v;
			}
		}
		return tmp;
	}
	cv::Mat getRMatrix(const std::string &file)
	{
		std::ifstream inFile(file, std::ifstream::in);
		if (!inFile.is_open())
		{
			std::cout << "[Warning] -- Failed to open file: " << file << std::endl;
			return cv::Mat();
		}
		cv::Mat tmp = cv::Mat(3, 3, CV_32F);
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				float v;
				inFile >> v;
				tmp.at<float>(i, j) = v;
			}
		}
		return tmp;
	}
	cv::Mat getTMatrix(const std::string &file)
	{
		std::ifstream inFile(file, std::ifstream::in);
		if (!inFile.is_open())
		{
			std::cout << "[Warning] -- Failed to open file: " << file << std::endl;
			return cv::Mat();
		}
		cv::Mat tmp = cv::Mat(3, 1, CV_32F);
		for (int i = 0; i < 3; ++i)
		{
			float v;
			inFile >> v;
			tmp.at<float>(i, 0) = v;
		}
		return tmp;
	}

	// Read calibration from CityOfSightsData
	cv::Mat getRMatrixCoS(const std::string &csvPoseFile, int frameID)
	{
		// open input file
		std::ifstream ifsLine;
		ifsLine.open(csvPoseFile, std::ios::in);

		cv::Mat result;
		if (!ifsLine.is_open())
		{
			std::cout << "[Warning] -- Failed to open file: " << csvPoseFile << std::endl;
			return result;
		}

		std::string line;
		bool parsingHeader = true;
		int nPoints;
		int lineNum = 0;
		while (std::getline(ifsLine, line))
		{
			if (lineNum > 0) // skip the first line
			{
				std::stringstream ss(line);
				std::vector<std::string> tokens;
				std::string token;
				while (std::getline(ss, token, ','))
				{
					tokens.push_back(token);
				}
				if (stoi(tokens[8]) == frameID)
				{
					cv::Vec4f q;
					q[0] = stof(tokens[4]); // x
					q[1] = stof(tokens[5]); // y
					q[2] = stof(tokens[6]); // z
					q[3] = stof(tokens[7]); // w
					result = quaternionToRotmat(q);
					break;
				}
			}
			lineNum++;
		}
		return result;
	}
	cv::Mat getTMatrixCoS(const std::string &csvPoseFile, int frameID)
	{
		// open input file
		std::ifstream ifsLine;
		ifsLine.open(csvPoseFile, std::ios::in);

		cv::Mat result;
		if (!ifsLine.is_open())
		{
			std::cout << "[Warning] -- Failed to open file: " << csvPoseFile << std::endl;
			return result;
		}

		std::string line;
		bool parsingHeader = true;
		int nPoints;
		int lineNum = 0;
		while (std::getline(ifsLine, line))
		{
			if (lineNum > 0) // skip the first line
			{
				std::stringstream ss(line);
				std::vector<std::string> tokens;
				std::string token;
				while (std::getline(ss, token, ','))
				{
					tokens.push_back(token);
				}
				if (stoi(tokens[8]) == frameID)
				{
					Eigen::Vector3d tVec(stof(tokens[1]), stof(tokens[2]), stof(tokens[3]));

					// convert result to cv Mat
					result = Vector3dToMat(tVec);
					break;
				}
			}
			lineNum++;
		}
		return result;
	}

	// Read point cloud from Ply file
	void getPointsPlyFile(const std::string &mapFile, std::vector<cv::Mat> &outVertices, cv::Mat &outColours)
	{
		// open input file
		std::ifstream ifsLine;
		ifsLine.open(mapFile, std::ios::in);

		if (!ifsLine.is_open())
		{
			std::cout << "[Warning] -- Failed to open file: " << mapFile << std::endl;
			return;
		}

		// read lines
		std::string line;
		bool parsingHeader = true;
		int nPoints;
		int lineNum = 0;
		while (getline(ifsLine, line))
		{
			// skip header
			if (parsingHeader && line.compare("end_header") == 0)
			{
				parsingHeader = false;
				continue;
			}

			// find num vertices
			if (parsingHeader && line.substr(0, 14).compare("element vertex") == 0)
			{
				nPoints = atoi(line.substr(15).c_str());
				outColours = cv::Mat(nPoints, 1, CV_32SC3);
				lineNum = 0;
			}

			// parse the cameras
			if (!parsingHeader)
			{
				// parse the t vector
				double X, Y, Z;
				int R, G, B;
				std::istringstream ss(line);
				ss >> X >> Y >> Z >> R >> G >> B;

				cv::Mat vertex(4, 1, CV_32F);
				vertex.at<float>(0, 0) = X;
				vertex.at<float>(1, 0) = Y;
				vertex.at<float>(2, 0) = Z;
				vertex.at<float>(3, 0) = 1.0;
				outVertices.push_back(vertex);

				outColours.at<cv::Vec3i>(lineNum, 0) = cv::Vec3i(R, G, B);
				lineNum++;
			}
		}
	}

	// Compare basename of filename strings (not in header)
	int basenameCompare(const std::string &s1, const std::string &s2)
	{
		std::string s1_copy = s1.substr(s1.find_last_of("/\\") + 1);
		std::string s2_copy = s2.substr(s2.find_last_of("/\\") + 1);
		return s1_copy.compare(s2_copy);
	}

	// Reads 3dpoint, observations, and pose data from a COLMAP text model, given the model directory (containing images.txt, points3D.txt, and cameras.txt), and the image name
	bool colmapGetReconstructionQuaternion(
		const std::string &colmap_model_dirname,
		std::unordered_map<int, cv::Point3f> &dst_mps,
		std::unordered_map<int, std::vector<std::pair<int, int> > > &dst_obs,
		std::unordered_map<int, std::vector<int> > &dst_corr,
		std::unordered_map<int, cv::Mat> &dst_Rs,
		std::unordered_map<int, cv::Mat> &dst_ts)
	{
		// filenames
		const std::string imagesTxtFilename = colmap_model_dirname + "/images.txt";
		const std::string points3DTxtFilename = colmap_model_dirname + "/points3D.txt";

		// open the files
		std::ifstream imagesTxtFile(imagesTxtFilename, std::ios::in);
		std::ifstream points3DTxtFile(points3DTxtFilename, std::ios::in);

		// read line by line
		std::string line;
		std::getline(imagesTxtFile, line);

		// skip comment lines beginning with #
		while (!line.empty() && line.at(0) == '#')
			std::getline(imagesTxtFile, line);

		// parse every second line
		while (!line.empty())
		{
			// output vars
			int image_id, camera_id;
			float qw, qx, qy, qz, tx, ty, tz;
			std::string name;

			// scan the image data line
			std::stringstream ss_image(line);
			ss_image >> image_id >> qw >> qx >> qy >> qz >> tx >> ty >> tz >> camera_id >> name;

			// output the pose
			dst_Rs[image_id] = cv::Mat(cv::Vec4f(qx, qy, qz, qw));
			dst_ts[image_id] = cv::Mat(cv::Vec3f(tx, ty, tz));

			// parse the keypoints line to get keypoint-to-mappoint correspondences
			std::getline(imagesTxtFile, line);
			std::stringstream ss_kps(line);
			std::vector<int> corr;
			while (ss_kps.good())
			{
				float x, y;
				int point3d_id;
				ss_kps >> x >> y >> point3d_id;
				corr.push_back(point3d_id);
			}
			dst_corr[image_id] = corr;

			// get the next IMAGE_ID line
			std::getline(imagesTxtFile, line);
		}

		// now parse the points3d file
		std::getline(points3DTxtFile, line);

		// skip comment lines beginning with #
		while (!line.empty() && line.at(0) == '#')
			std::getline(points3DTxtFile, line);

		// parse every second line
		while (!line.empty())
		{
			int point3d_id;
			float x, y, z;
			int r, g, b;
			float error;

			// scan the point3d data line
			std::stringstream ss_point3d(line);
			ss_point3d >> point3d_id >> x >> y >> z >> r >> g >> b >> error;

			// store the map point
			dst_mps[point3d_id] = cv::Point3f(x, y, z);

			// now parse all the observations
			std::vector<std::pair<int, int> > obs;
			while (ss_point3d.good())
			{
				int image_id, point2d_idx;
				ss_point3d >> image_id >> point2d_idx;
				obs.push_back(std::make_pair(image_id, point2d_idx));
			}
			dst_obs[point3d_id] = obs;

			// get the next point3d line
			std::getline(points3DTxtFile, line);
		}

		return true;
	}

	// Reads 3dpoint, observations, and pose data from a COLMAP text model, given the model directory (containing images.txt, points3D.txt, and cameras.txt), and the image name
	bool colmapGetReconstruction(
		const std::string &colmap_model_dirname,
		std::unordered_map<int, cv::Point3f> &dst_mps,
		std::unordered_map<int, cv::Vec3b> &dst_mps_colrs,
		std::unordered_map<int, std::vector<std::pair<int, int> > > &dst_obs,
		std::unordered_map<int, std::vector<int> > &dst_corr,
		std::unordered_map<int, cv::Mat> &dst_Rs,
		std::unordered_map<int, cv::Mat> &dst_ts,
		std::unordered_map<int, float> &dst_fs)
	{
		// filenames
		const std::string imagesTxtFilename = colmap_model_dirname + "/images.txt";
		const std::string points3DTxtFilename = colmap_model_dirname + "/points3D.txt";
		const std::string camerasTxtFilename = colmap_model_dirname + "/cameras.txt";

		// open the files
		std::ifstream imagesTxtFile(imagesTxtFilename, std::ios::in);
		std::ifstream points3DTxtFile(points3DTxtFilename, std::ios::in);
		std::ifstream camerasTxtFile(camerasTxtFilename, std::ios::in);

		// read line by line
		std::string line;

		//
		// PARSE CAMERAS FILE TO GET FOCAL LENGTHS FIRST
		//
		std::getline(camerasTxtFile, line);

		// skip comment lines beginning with #
		while (!line.empty() && line.at(0) == '#')
			std::getline(camerasTxtFile, line);

		// parse every line
		std::unordered_map<int, float> fs;
		while (!line.empty())
		{
			// output vars
			int camera_id, width, height;
			std::string model;
			float f;

			// scan the camera data line
			std::stringstream ss_camera(line);
			ss_camera >> camera_id >> model >> width >> height >> f;

			// output the focal length into temp vector
			fs[camera_id] = f;

			// get the next camera line
			std::getline(camerasTxtFile, line);
		}

		//
		// NEXT PARSE THE IMAGES FILE TO GET POSES
		//
		std::getline(imagesTxtFile, line);

		// skip comment lines beginning with #
		while (!line.empty() && line.at(0) == '#')
			std::getline(imagesTxtFile, line);

		// parse every second line
		while (!line.empty())
		{
			// output vars
			int image_id, camera_id;
			float qw, qx, qy, qz, tx, ty, tz;
			std::string name;

			// scan the image data line
			std::stringstream ss_image(line);
			ss_image >> image_id >> qw >> qx >> qy >> qz >> tx >> ty >> tz >> camera_id >> name;

			// output the data
			dst_Rs[image_id] = quaternionToRotmatColmap(cv::Vec4f(qw, qx, qy, qz));
			dst_ts[image_id] = cv::Mat(cv::Vec3f(tx, ty, tz));
			dst_fs[image_id] = fs[camera_id];

			// parse the keypoints line to get keypoint-to-mappoint correspondences
			std::getline(imagesTxtFile, line);
			std::stringstream ss_kps(line);
			std::vector<int> corr;
			while (ss_kps.good())
			{
				float x, y;
				int point3d_id;
				ss_kps >> x >> y >> point3d_id;
				corr.push_back(point3d_id);
			}
			dst_corr[image_id] = corr;

			// get the next IMAGE_ID line
			std::getline(imagesTxtFile, line);
		}

		//
		// THEN PARSE THE POINTS3D FILE TO GET POINTS
		//
		std::getline(points3DTxtFile, line);

		// skip comment lines beginning with #
		while (!line.empty() && line.at(0) == '#')
			std::getline(points3DTxtFile, line);

		// parse every line
		while (!line.empty())
		{
			int point3d_id;
			float x, y, z;
			int r, g, b;
			float error;

			// scan the point3d data line
			std::stringstream ss_point3d(line);
			ss_point3d >> point3d_id >> x >> y >> z >> r >> g >> b >> error;

			// store the map point
			dst_mps[point3d_id] = cv::Point3f(x, y, z);
			dst_mps_colrs[point3d_id] = cv::Vec3b(b, g, r);

			// now parse all the observations
			std::vector<std::pair<int, int> > obs;
			while (ss_point3d.good())
			{
				int image_id, point2d_idx;
				ss_point3d >> image_id >> point2d_idx;
				obs.push_back(std::make_pair(image_id, point2d_idx));
			}
			dst_obs[point3d_id] = obs;

			// get the next point3d line
			std::getline(points3DTxtFile, line);
		}

		return true;
	}

	// Reads 3dpoint, observations, and pose data from a COLMAP text model, given the model directory (containing images.txt, points3D.txt, and cameras.txt), and the image name
	bool colmapGetReconstruction(
		const std::string &colmap_model_dirname,
		std::unordered_map<int, cv::Point3f> &dst_mps,
		std::unordered_map<int, std::vector<std::pair<int, int> > > &dst_obs,
		std::unordered_map<int, std::vector<int> > &dst_corr,
		std::unordered_map<int, cv::Mat> &dst_Rs,
		std::unordered_map<int, cv::Mat> &dst_ts,
		std::unordered_map<int, float> &dst_fs)
	{
		// filenames
		const std::string imagesTxtFilename = colmap_model_dirname + "/images.txt";
		const std::string points3DTxtFilename = colmap_model_dirname + "/points3D.txt";
		const std::string camerasTxtFilename = colmap_model_dirname + "/cameras.txt";

		// open the files
		std::ifstream imagesTxtFile(imagesTxtFilename, std::ios::in);
		std::ifstream points3DTxtFile(points3DTxtFilename, std::ios::in);
		std::ifstream camerasTxtFile(camerasTxtFilename, std::ios::in);

		// read line by line
		std::string line;

		//
		// PARSE CAMERAS FILE TO GET FOCAL LENGTHS FIRST
		//
		std::getline(camerasTxtFile, line);

		// skip comment lines beginning with #
		while (!line.empty() && line.at(0) == '#')
			std::getline(camerasTxtFile, line);

		// parse every line
		std::unordered_map<int, float> fs;
		std::unordered_map<int, float> cx;
		std::unordered_map<int, cv::Point2f> cxcy;
		while (!line.empty())
		{
			// output vars
			int camera_id, width, height;
			std::string model;
			float f;
			float cx;
			float cy;

			// scan the camera data line
			std::stringstream ss_camera(line);
			ss_camera >> camera_id >> model >> width >> height >> f >> cx >> cy;

			// output the focal length into temp vector
			fs[camera_id] = f;

			cxcy[camera_id] = cv::Point2f(cx, cy);

			// get the next camera line
			std::getline(camerasTxtFile, line);
		}

		//
		// NEXT PARSE THE IMAGES FILE TO GET POSES
		//
		std::getline(imagesTxtFile, line);

		// skip comment lines beginning with #
		while (!line.empty() && line.at(0) == '#')
			std::getline(imagesTxtFile, line);

		// parse every second line
		while (!line.empty())
		{
			// output vars
			int image_id, camera_id;
			float qw, qx, qy, qz, tx, ty, tz;
			std::string name;

			// scan the image data line
			std::stringstream ss_image(line);
			ss_image >> image_id >> qw >> qx >> qy >> qz >> tx >> ty >> tz >> camera_id >> name;

			// output the data
			dst_Rs[image_id] = quaternionToRotmatColmap(cv::Vec4f(qw, qx, qy, qz));
			dst_ts[image_id] = cv::Mat(cv::Vec3f(tx, ty, tz));
			dst_fs[image_id] = fs[camera_id];

			// parse the keypoints line to get keypoint-to-mappoint correspondences
			std::getline(imagesTxtFile, line);
			std::stringstream ss_kps(line);
			std::vector<int> corr;
			while (ss_kps.good())
			{
				float x, y;
				int point3d_id;
				ss_kps >> x >> y >> point3d_id;
				corr.push_back(point3d_id);
			}
			dst_corr[image_id] = corr;

			// get the next IMAGE_ID line
			std::getline(imagesTxtFile, line);
		}

		//
		// THEN PARSE THE POINTS3D FILE TO GET POINTS
		//
		std::getline(points3DTxtFile, line);

		// skip comment lines beginning with #
		while (!line.empty() && line.at(0) == '#')
			std::getline(points3DTxtFile, line);

		// parse every line
		while (!line.empty())
		{
			int point3d_id;
			float x, y, z;
			int r, g, b;
			float error;

			// scan the point3d data line
			std::stringstream ss_point3d(line);
			ss_point3d >> point3d_id >> x >> y >> z >> r >> g >> b >> error;

			// store the map point
			dst_mps[point3d_id] = cv::Point3f(x, y, z);

			// now parse all the observations
			std::vector<std::pair<int, int> > obs;
			while (ss_point3d.good())
			{
				int image_id, point2d_idx;
				ss_point3d >> image_id >> point2d_idx;
				obs.push_back(std::make_pair(image_id, point2d_idx));
			}
			dst_obs[point3d_id] = obs;

			// get the next point3d line
			std::getline(points3DTxtFile, line);
		}

		return true;
	}

	sqlite3 *db = nullptr;

	// Reads keypoint and descriptor data from a COLMAP db model
	bool colmapGetFeatures(
		const std::string &db_filename,
		std::unordered_map<int, std::vector<cv::KeyPoint> > &dst_kps,
		std::unordered_map<int, cv::Mat> &dst_descs,
		std::unordered_map<int, std::string> &dst_names)
	{
		// open database
		int rc = 1;
		if (db == nullptr)
		{
			rc = sqlite3_open(db_filename.c_str(), &db);
			if (rc)
			{
				std::cout << "[Error] -- Can't open database: " << db_filename << std::endl;
				return false;
			}
		}

		// read keypoints
		{
			int i = 1;
			sqlite3_stmt *stmt;
			sqlite3_prepare_v2(db, "SELECT * FROM keypoints", -1, &stmt, 0);
			while (true)
			{
				// get the next row from the statement
				rc = sqlite3_step(stmt);
				if (rc != SQLITE_ROW)
					break;

				// get the data from the row
				int image_id = sqlite3_column_int(stmt, 0);
				int rows = sqlite3_column_int(stmt, 1);
				int cols = sqlite3_column_int(stmt, 2);
				const float *data = (const float *)sqlite3_column_blob(stmt, 3);

				// parse the kp data from the blob row by row, 6 columns at a time
				std::vector<cv::KeyPoint> kps;
				for (int r = 0; r < rows; ++r)
				{
					// 6 columns according to COLMAP: src/feature/types.h
					float x, y, a11, a12, a21, a22;
					x = data[(r * cols) + 0];
					y = data[(r * cols) + 1];
					a11 = data[(r * cols) + 2];
					a12 = data[(r * cols) + 3];
					a21 = data[(r * cols) + 4];
					a22 = data[(r * cols) + 5];

					float orientation = rad2deg(std::atan2(a21, a11)) + 180.0f;
					float scale_x = std::sqrt(a11 * a11 + a21 * a21);
					float scale_y = std::sqrt(a12 * a12 + a22 * a22);
					float scale = (scale_x + scale_y) / 2.0f;

					// create an OpenCV keypoint
					cv::KeyPoint kp;
					kp.pt = cv::Point(x, y);
					kp.angle = orientation;
					kp.size = scale;
					kps.push_back(kp);
				}

				// send it to the result map
				dst_kps[image_id] = kps;

				++i;
			};
		}

		// read descriptors
		{
			int i = 1;
			sqlite3_stmt *stmt;
			sqlite3_prepare_v2(db, "SELECT * FROM descriptors", -1, &stmt, 0);
			while (true)
			{
				// get the next row from the statement
				rc = sqlite3_step(stmt);
				if (rc != SQLITE_ROW)
					break;

				// get the data from the row
				int image_id = sqlite3_column_int(stmt, 0);
				int rows = sqlite3_column_int(stmt, 1);
				int cols = sqlite3_column_int(stmt, 2);
				const char *data = (const char *)sqlite3_column_blob(stmt, 3);

				// parse the descriptor data from the blob row by row, 6 columns at a time
				cv::Mat descs(rows, cols, CV_8U);
				for (int r = 0; r < rows; ++r)
				{
					for (int c = 0; c < cols; ++c)
					{
						int idx = (r * cols) + c;
						descs.at<unsigned char>(r, c) = data[idx];
					}
				}

				// send it to the result map
				dst_descs[image_id] = descs;

				++i;
			}
		}

		// read names
		{
			int i = 1;
			sqlite3_stmt *stmt;
			sqlite3_prepare_v2(db, "SELECT * FROM images", -1, &stmt, 0);
			while (true)
			{
				// get the next row from the statement
				rc = sqlite3_step(stmt);
				if (rc != SQLITE_ROW)
					break;

				// get the data from the row
				int image_id = sqlite3_column_int(stmt, 0);
				const unsigned char *name = sqlite3_column_text(stmt, 1);

				// send it to the result map
				dst_names[image_id] = std::string((const char *)name);
				++i;
			}
		}

		return true;
	}

	// Find the image_id given a filename
	bool colmapSearchImage(
		const std::string &db_filename,
		const std::string &image_filename,
		int &dst_image_id)
	{
		// open database
		int rc = 1;
		if (db == nullptr)
		{
			rc = sqlite3_open(db_filename.c_str(), &db);
			if (rc)
			{
				std::cout << "[Error] -- Can't open database: " << db_filename << std::endl;
				return false;
			}
		}

		// read the images table
		sqlite3_stmt *stmt;
		std::string image_filename_basename = image_filename.substr(image_filename.find_last_of("/\\") + 1);
		const std::string stmt_str = std::string("SELECT image_id from images WHERE name LIKE \"%") + image_filename_basename + "\"";
		sqlite3_prepare_v2(db, stmt_str.c_str(), -1, &stmt, 0);

		// get the next row from the statement
		rc = sqlite3_step(stmt);
		if (rc != SQLITE_ROW)
		{
			std::cout << "[Warning] -- Image: " << image_filename_basename << " not found in database: " << db_filename << std::endl;
			return false;
		}

		// get the data from the row
		dst_image_id = sqlite3_column_int(stmt, 0);
		return true;
	}

	std::vector<cv::Point3f> points_3d;
	std::vector<cv::Point3f> get3DPoints(
		const std::string &points_dirname,
		const std::string &ImageName)
	{

        points_3d.clear();
		const std::string points3DTxtFilename = points_dirname + "/3dPoints" + "/" + ImageName + ".3dpoints.txt";
		//const std::string points3DTxtFilename = points_dirname + "/3dpoints.txt";
		std::cout << points3DTxtFilename << std::endl;

		std::ifstream points3DTxtFile(points3DTxtFilename); //, std::ios::in);
		if (!points3DTxtFile)
		{
			return points_3d;
		}
		// read line by line
		std::string line1;

		while (!points3DTxtFile.eof())
		{
			std::getline(points3DTxtFile, line1);
			if (line1.empty())
			{
				//std::cout<<"Line empty "<< std::endl;
				continue;
			}
			else if (line1.at(0) == '#')
			{
				//std::cout<<"Line Comments"<< std::endl;
				continue;
			}
			//std::cout<<"Line " << line1<< std::endl;
			float x, y, z;
			// scan the point3d data line
			std::stringstream ss_point3d(line1);
			ss_point3d >> x >> y >> z;
			points_3d.push_back(cv::Point3f(x, y, z));
		}


		return points_3d;
	}

	std::vector<cv::Point2f> get2DPoints(
		const std::string &points_dirname,
		const std::string &ImageName)
	{
		std::vector<cv::Point2f> points_2d;
		const std::string points2DTxtFilename = points_dirname + "/2dPoints" + "/" + ImageName + ".2dpoints.txt";
		//std::cout << "points2DTxtFilename " << points2DTxtFilename << std::endl;
		std::ifstream points2DTxtFile(points2DTxtFilename); //, std::ios::in);
		if (!points2DTxtFile)
		{
			return points_2d;
		}
		// read line by line
		std::string line1;

		std::cout << "Start Reading " << points2DTxtFilename << std::endl;
		while (!points2DTxtFile.eof())
		{
			std::getline(points2DTxtFile, line1);
			if (line1.empty())
			{
				//std::cout<<"Line empty "<< std::endl;
				continue;
			}
			else if (line1.at(0) == '#')
			{
				//std::cout<<"Line Comments"<< std::endl;
				continue;
			}

			// line1.replace(line1.indexof(','),1,' ');

			//std::cout<<"Line " << line1<< std::endl;
			float x, y;
			char coma;
			// scan the point3d data line
			std::stringstream ss_point2d(line1);
			ss_point2d >> x >> coma >> y;
			points_2d.push_back(cv::Point2f(x, y));
		}

		return points_2d;
	}

}
