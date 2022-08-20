#pragma once

#include <vector>
#include <string>
#include <fstream>

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


namespace Utils
{
    // Structure and methods for a 2D line
    class DECLSPEC Line2d
    {
        cv::Mat p1CV;
        cv::Mat p2CV;
        Eigen::Vector3d p1Eigen;
        Eigen::Vector3d p2Eigen;
        cv::Scalar colr;

    public:
        Line2d();
        Line2d(const cv::Vec3f &p1, const cv::Vec3f &p2);
        Line2d(const cv::Vec4f &p);
        Line2d(const cv::Vec4i &p);
        Line2d(const cv::Mat &p1, const cv::Mat &p2);
        Eigen::Vector3d getP1Eigen() const;
        Eigen::Vector3d getP2Eigen() const;
        cv::Mat getP1CV() const;
        cv::Mat getP2CV() const;
        cv::Scalar getColor() const;
        void setColor(const cv::Scalar &newCol);
        bool isVisible(const cv::Mat &K) const;
		bool isNotMasked(const cv::Mat &mask) const;
        double length() const;
        void draw(cv::Mat &outIm) const;
        bool operator < (const Line2d& lin) const;
        bool operator > (const Line2d& lin) const;
    };


    // Structure and methods for a 3D line
    class DECLSPEC Line3d
    {
        cv::Mat P1CV;
        cv::Mat P2CV;
        Eigen::Vector4d P1Eigen;
        Eigen::Vector4d P2Eigen;
        cv::Scalar colr;

    public:
        Line3d();
        Line3d(const cv::Vec4f &P1, const cv::Vec4f &P2);
        Line3d(const cv::Mat &P1, const cv::Mat &P2);
        Eigen::Vector4d getP1Eigen() const;
        Eigen::Vector4d getP2Eigen() const;
        cv::Mat getP1CV() const;
        cv::Mat getP2CV() const;
        cv::Scalar getColor() const;
        void setColor(const cv::Scalar &newCol);
        double distance(const Line3d &l) const;
        void draw(cv::Mat &outIm, const cv::Mat &K, const cv::Mat &R, const cv::Mat &t, bool bundlerFormat) const;
        Line2d project(const cv::Mat &K, const cv::Mat &R, const cv::Mat &t, bool bundlerFormat) const;
        void projectAndSave(std::ofstream &oFile, std::ofstream &oFileN, const cv::Mat &K, const cv::Mat &R, const cv::Mat &t, bool bundlerFormat) const;
        double length() const;
        Line3d operator + (const Line3d& a) const;
        Line3d operator / (const float& a) const;
        bool operator < (const Line3d& lin) const;
        bool operator > (const Line3d& lin) const;
    };


    // Convert vector of Line3d/Line2d to a vector of homogeneous Eigen3 vector types
    DECLSPEC std::vector<Eigen::Vector4d> line3dToHVector4d(const std::vector<Line3d> &l3d);
    DECLSPEC std::vector<Eigen::Vector3d> line2dToHVector3d(const std::vector<Line2d> &l2d);


    // Convert vector of Line2d from pixel co-ords to normalzied 2Ds (for softposit)
    DECLSPEC std::vector<Line2d> normalizeLines(const std::vector<Line2d> &input, const cv::Mat &K);
    DECLSPEC Line2d unnormalize(const Line2d &line, const cv::Mat &K, bool bundlerFormat);


    // Get 3D lines from TXT (Line3Dpp), STL, or Lin file, and 2D lines from STL, or Lin file
    DECLSPEC std::vector<Line3d> get3dLinesTxtFile(const std::string &file);
    DECLSPEC std::vector<Line3d> get3dLinesStlFile(const std::string &file);
    DECLSPEC std::vector<Line3d> get3dLinesLinFile(const std::string &file);
    DECLSPEC std::vector<Line3d> get3dLines(const std::string &file);
    DECLSPEC std::vector<Line2d> get2dLinesStlFile(const std::string &file);
    DECLSPEC std::vector<Line2d> get2dLinesLinFile(const std::string &file);
    DECLSPEC std::vector<Line2d> get2dLines(const std::string &file);


    // Write 3D or 2D lines to STL, or Lin file
    DECLSPEC void write3dLinesStlFile(const std::string &file, const std::vector<Line3d> &lines);
    DECLSPEC void write3dLinesLinFile(const std::string &file, const std::vector<Line3d> &lines);
    DECLSPEC void write3dLines(const std::string &file, const std::vector<Line3d> &lines);
    DECLSPEC void write2dLinesStlFile(const std::string &file, const std::vector<Line2d> &lines);
    DECLSPEC void write2dLinesLinFile(const std::string &file, const std::vector<Line2d> &lines);
    DECLSPEC void write2dLines(const std::string &file, const std::vector<Line2d> &lines);


    // Measure reprojection error of measured 2d lines and projected 3d lines using camera calibration K, R, and t
	DECLSPEC double lineReprojectionError(const std::vector<Line2d> &lines2d, const std::vector<Line3d> &lines3d, const Eigen::Matrix3d &K, const Eigen::Matrix3d &R, const Eigen::Vector3d &t, bool bundlerFormat);
	DECLSPEC double lineReprojectionError(const std::vector<Line3d> &lines3d, const Eigen::Matrix3d &gtK, const Eigen::Matrix3d &gtR, const Eigen::Vector3d &gtt, const Eigen::Matrix3d &estK, const Eigen::Matrix3d &estR, const Eigen::Vector3d &estt, bool bundlerFormat);


    // 2D and 3D perturbation / mutation
    DECLSPEC cv::Vec4f randomPoint(float minX, float maxX, float minY, float maxY, float minZ, float maxZ);
    DECLSPEC void add3dClutter(std::vector<Line3d> &inOutLines, int n);
    DECLSPEC void add2dClutter(std::vector<Line2d> &inOutLines, int n);
    DECLSPEC void hide3dLines(std::vector<Line3d> &inOutLines, int n);
    DECLSPEC void hide2dLines(std::vector<Line2d> &inOutLines, int n);
    DECLSPEC void modLength2dLines(std::vector<Line2d> &inOutLines, int n);
    DECLSPEC void duplicate3dLines(std::vector<Line3d> &inOutLines, int n);
    DECLSPEC void modEndpoints2dLines(std::vector<Line2d> &inOutLines, int n);
    DECLSPEC Eigen::Vector3d minimumOffset(const std::vector<Line3d> &inLines, std::vector<Line3d> &outLines);
	DECLSPEC void modPoseTargetError(const std::vector<Line3d> &modelLines, const cv::Mat &K, const cv::Mat &R, const cv::Mat &T, cv::Mat &outR, cv::Mat &outT, bool bundlerFormat);
	DECLSPEC void modPose(const cv::Mat &R, const cv::Mat &T, double rotErrorRange, double transErrorRange, cv::Mat &outR, cv::Mat &outT);
	DECLSPEC void offsetPose(const Eigen::Vector3d &offset, const cv::Mat &R, const cv::Mat &t, cv::Mat &outR, cv::Mat &outt);


}

