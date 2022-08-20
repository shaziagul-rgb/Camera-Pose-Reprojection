#include <vector>
#include <string>
#include <fstream>
#include <iostream>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "Lines.hpp"
#include "Matrix.hpp"


namespace Utils
{
    // Structure and methods for a 2D line
    Line2d::Line2d()
        : p1CV(cv::Vec3f(0, 0, 0)), p2CV(cv::Vec3f(0, 0, 0)), colr(cv::Scalar(255, 255, 255)), p1Eigen(0, 0, 0), p2Eigen(0, 0, 0)
    { }
    Line2d::Line2d(const cv::Vec3f &p1, const cv::Vec3f &p2)
        : p1CV(p1), p2CV(p2), colr(cv::Scalar(255, 255, 255)), p1Eigen(p1[0], p1[1], p1[2]), p2Eigen(p2[0], p2[1], p2[2])
    { }
    Line2d::Line2d(const cv::Vec4f &p)
        : p1CV(cv::Vec3f(p[0], p[1], 1.0)), p2CV(cv::Vec3f(p[2], p[3], 1.0)), colr(cv::Scalar(255, 255, 255)), p1Eigen(p[0], p[1], 1), p2Eigen(p[2], p[3], 1)
    { }
    Line2d::Line2d(const cv::Vec4i &p)
        : p1CV(cv::Vec3f(p[0], p[1], 1.0)), p2CV(cv::Vec3f(p[2], p[3], 1.0)), colr(cv::Scalar(255, 255, 255)), p1Eigen(p[0], p[1], 1), p2Eigen(p[2], p[3], 1)
    { }
    Line2d::Line2d(const cv::Mat &p1, const cv::Mat &p2)
        : p1CV(p1), p2CV(p2), colr(cv::Scalar(255, 255, 255)), p1Eigen(cv::Vec3f(p1)[0], cv::Vec3f(p1)[1], cv::Vec3f(p1)[2]), p2Eigen(cv::Vec3f(p2)[0], cv::Vec3f(p2)[1], cv::Vec3f(p2)[2])
    { }
    Eigen::Vector3d Line2d::getP1Eigen() const
    {
        return Eigen::Vector3d(p1Eigen);
    }
    Eigen::Vector3d Line2d::getP2Eigen() const
    {
        return Eigen::Vector3d(p2Eigen);
    }
    cv::Mat Line2d::getP1CV() const
    {
        return p1CV.clone();
    }
    cv::Mat Line2d::getP2CV() const
    {
        return p2CV.clone();
    }
    cv::Scalar Line2d::getColor() const
    {
        return cv::Scalar(colr);
    }
    void Line2d::setColor(const cv::Scalar &newCol)
    {
        colr = newCol;
    }
    bool Line2d::isVisible(const cv::Mat &K) const
    {
        cv::Vec3f p1vec = cv::Vec3f(p1CV);
        cv::Vec3f p2vec = cv::Vec3f(p2CV);
        float width = K.at<float>(0, 2) * 2.0;
        float height = K.at<float>(1, 2) * 2.0;
        if (p1vec[2] < 0 || p2vec[2] < 0)  // one endpoint is behind the camera
        {
            return false;
        }
        if ((p1vec[0] > 0 && p1vec[0] < width && p1vec[1] > 0 && p1vec[1] < height) &&
            (p2vec[0] > 0 && p2vec[0] < width && p2vec[1] > 0 && p2vec[1] < height)) // at least one endpoint is in FOV
        {
            return true;
        }
        return false;
    }
	bool Line2d::isNotMasked(const cv::Mat &mask) const
	{
		if (mask.empty())
		{
			return true;
		}

		if (mask.at<uchar>(getP1Eigen().y(), getP1Eigen().x()) > 0 &&
			mask.at<uchar>(getP2Eigen().y(), getP2Eigen().x()) > 0) {
			return true;
		}
		return false;
	}
    double Line2d::length() const
    {
        return cv::norm(p2CV - p1CV);
    }
    void Line2d::draw(cv::Mat &outIm) const
    {
        cv::Vec3f p1Vec(p1CV);
        cv::Vec3f p2Vec(p2CV);
        cv::line(outIm, cv::Point(p1Vec[0], p1Vec[1]), cv::Point(p2Vec[0], p2Vec[1]), getColor());
    }
    bool Line2d::operator < (const Line2d& lin) const
    {
        return (length() < lin.length());
    }
    bool Line2d::operator > (const Line2d& lin) const
    {
        return (length() > lin.length());
    }


    // Structure and methods for a 3D line
    Line3d::Line3d()
        : P1CV(cv::Vec4f(0, 0, 0, 0)), P2CV(cv::Vec4f(0, 0, 0, 0)), colr(cv::Scalar(255, 255, 255)), P1Eigen(0, 0, 0, 0), P2Eigen(0, 0, 0, 0)
    { }
    Line3d::Line3d(const cv::Vec4f &P1, const cv::Vec4f &P2)
        : P1CV(P1), P2CV(P2), colr(cv::Scalar(255, 255, 255)), P1Eigen(P1[0], P1[1], P1[2], P1[3]), P2Eigen(P2[0], P2[1], P2[2], P2[3])
    { }
    Line3d::Line3d(const cv::Mat &P1, const cv::Mat &P2)
        : P1CV(P1), P2CV(P2), colr(cv::Scalar(255, 255, 255)), P1Eigen(cv::Vec4f(P1)[0], cv::Vec4f(P1)[1], cv::Vec4f(P1)[2], cv::Vec4f(P1)[3]), P2Eigen(cv::Vec4f(P2)[0], cv::Vec4f(P2)[1], cv::Vec4f(P2)[2], cv::Vec4f(P2)[3])
    { }
    Eigen::Vector4d Line3d::getP1Eigen() const
    {
        return Eigen::Vector4d(P1Eigen);
    }
    Eigen::Vector4d Line3d::getP2Eigen() const
    {
        return Eigen::Vector4d(P2Eigen);
    }
    cv::Mat Line3d::getP1CV() const
    {
        return P1CV.clone();
    }
    cv::Mat Line3d::getP2CV() const
    {
        return P2CV.clone();
    }
    cv::Scalar Line3d::getColor() const
    {
        return cv::Scalar(colr);
    }
    void Line3d::setColor(const cv::Scalar &newCol)
    {
        colr = newCol;
    }
    double Line3d::distance(const Line3d &l) const
    {
        // is P1 closest to l.getP1CV() or l.P2 ?
        double d1 = cv::norm(P1CV - l.P1CV)*cv::norm(P1CV - l.P1CV);
        double d2 = cv::norm(P1CV - l.P2CV)*cv::norm(P1CV - l.P2CV);

        // l.P1 is closer to P1, so the endpoints 'match' in a sense
        if (d1 < d2)
        {
            return cv::norm(P1CV - l.P1CV)*cv::norm(P1CV - l.P1CV) + cv::norm(P2CV - l.P2CV)*cv::norm(P2CV - l.P2CV);
        }
        // l.P2 is closer to P1, so the endpoints 'match' inversely
        else
        {
            return cv::norm(P1CV - l.P2CV)*cv::norm(P1CV - l.P2CV) + cv::norm(P2CV - l.P1CV)*cv::norm(P2CV - l.P1CV);
        }
    }
    void Line3d::draw(cv::Mat &outIm, const cv::Mat &K, const cv::Mat &R, const cv::Mat &t, bool bundlerFormat) const
    {
        cv::Mat rt = cv::Mat::eye(3, 4, CV_32F); // 3x4
        R.copyTo(rt(cv::Rect(cv::Point(0, 0), cv::Size(3, 3)))); // copy Rmat in
        t.copyTo(rt(cv::Rect(cv::Point(3, 0), cv::Size(1, 3)))); // copy tvec in

        // http://www.land-of-kain.de/docs/coords/ BUNDLER INFO
        cv::Mat Z = cv::Mat::eye(3, 3, CV_32F); // 3x3
        Z.at<float>(1, 1) = bundlerFormat ? -1 : 1; // bundler requires -1 ...
        Z.at<float>(2, 2) = bundlerFormat ? -1 : 1; // bundler requires -1 ...

        cv::Mat p1Mat = K * Z * rt * P1CV;
        cv::Mat p2Mat = K * Z * rt * P2CV;
        cv::Vec3f p1 = cv::Vec3f(p1Mat);
        cv::Vec3f p2 = cv::Vec3f(p2Mat);

        double z1 = p1[2];
        double z2 = p2[2];
        p1 /= z1;
        p2 /= z2;

        //std::cout << p1 << " " << p2 << std::endl;

        if (z2 > 0 && z1 > 0) // bundler
            cv::line(outIm, cv::Point(p1[0], p1[1]), cv::Point(p2[0], p2[1]), getColor());
    }
    Line2d Line3d::project(const cv::Mat &K, const cv::Mat &R, const cv::Mat &t, bool bundlerFormat) const
    {
        cv::Mat rt(3, 4, CV_32F);
        R.copyTo(rt(cv::Rect(cv::Point(0, 0), cv::Size(3, 3)))); // copy Rmat in
        t.copyTo(rt(cv::Rect(cv::Point(3, 0), cv::Size(1, 3)))); // copy tvec in

        if (!K.empty()) // K specified, so return result as pixel coordinates
        {
            // http://www.land-of-kain.de/docs/coords/ BUNDLER INFO
            cv::Mat Z = cv::Mat::eye(3, 3, CV_32F); // 3x3
            Z.at<float>(1, 1) = bundlerFormat ? -1 : 1; // bundler requires -1 ...
            Z.at<float>(2, 2) = bundlerFormat ? -1 : 1; // bundler requires -1 ...

            cv::Mat p1Mat = K * Z * rt * P1CV;
            cv::Mat p2Mat = K * Z * rt * P2CV;
            cv::Vec3f p1 = cv::Vec3f(p1Mat);
            cv::Vec3f p2 = cv::Vec3f(p2Mat);

            double z1 = p1[2];
            double z2 = p2[2];
            p1 /= z1;
            p2 /= z2;

            return Line2d(p1, p2);
        }
        else // K not specified, so return result as normalized image coordinates
        {
            // http://www.land-of-kain.de/docs/coords/ BUNDLER INFO
            cv::Mat Z = cv::Mat::eye(3, 3, CV_32F); // 3x3
            Z.at<float>(1, 1) = bundlerFormat ? -1 : 1; // bundler requires -1 ...
            Z.at<float>(2, 2) = bundlerFormat ? -1 : 1; // bundler requires -1 ...

            cv::Mat p1Mat = Z * rt * P1CV;
            cv::Mat p2Mat = Z * rt * P2CV;
            cv::Vec3f p1n = cv::Vec3f(p1Mat);
            cv::Vec3f p2n = cv::Vec3f(p2Mat);

            double z1 = p1n[2];
            double z2 = p2n[2];
            p1n /= z1;
            p2n /= z2;

            return Line2d(p1n, p2n);
        }
    }
    void Line3d::projectAndSave(std::ofstream &oFile, std::ofstream &oFileN, const cv::Mat &K, const cv::Mat &R, const cv::Mat &t, bool bundlerFormat) const
    {
        cv::Mat rt(3, 4, CV_32F);
        R.copyTo(rt(cv::Rect(cv::Point(0, 0), cv::Size(3, 3)))); // copy Rmat in
        t.copyTo(rt(cv::Rect(cv::Point(3, 0), cv::Size(1, 3)))); // copy tvec in

        if (oFile.is_open())
        {
            // http://www.land-of-kain.de/docs/coords/ BUNDLER INFO
            cv::Mat Z = cv::Mat::eye(3, 3, CV_32F); // 3x3
            Z.at<float>(1, 1) = bundlerFormat ? -1 : 1; // bundler requires -1 ...
            Z.at<float>(2, 2) = bundlerFormat ? -1 : 1; // bundler requires -1 ...

            cv::Mat p1Mat = K * Z * rt * P1CV;
            cv::Mat p2Mat = K * Z * rt * P2CV;
            cv::Vec3f p1 = cv::Vec3f(p1Mat);
            cv::Vec3f p2 = cv::Vec3f(p2Mat);

            double z1 = p1[2];
            double z2 = p2[2];
            p1 /= z1;
            p2 /= z2;
            oFile << p1[0] << " " << p1[1] << " " << p2[0] << " " << p2[1] << std::endl;
        }

        if (oFileN.is_open())
        {
            // http://www.land-of-kain.de/docs/coords/ BUNDLER INFO
            cv::Mat Z = cv::Mat::eye(3, 3, CV_32F); // 3x3
            Z.at<float>(1, 1) = bundlerFormat ? -1 : 1; // bundler requires -1 ...
            Z.at<float>(2, 2) = bundlerFormat ? -1 : 1; // bundler requires -1 ...

            cv::Mat p1Mat = Z * rt * P1CV;
            cv::Mat p2Mat = Z * rt * P2CV;
            cv::Vec3f p1n = cv::Vec3f(p1Mat);
            cv::Vec3f p2n = cv::Vec3f(p2Mat);

            double z1 = p1n[2];
            double z2 = p2n[2];
            p1n /= z1;
            p2n /= z2;
            oFileN << p1n[0] << " " << p1n[1] << " " << p2n[0] << " " << p2n[1] << std::endl;
        }
    }
    double Line3d::length() const
    {
        return cv::norm(P2CV - P1CV);
    }
    Line3d Line3d::operator + (const Line3d& a) const
    {
        return Line3d(P1CV + a.P1CV, P2CV + a.P2CV);
    }
    Line3d Line3d::operator / (const float& a) const
    {
        return Line3d(P1CV / a, P2CV / a);
    }
    bool Line3d::operator < (const Line3d& lin) const
    {
        return (length() < lin.length());
    }
    bool Line3d::operator > (const Line3d& lin) const
    {
        return (length() > lin.length());
    }


    // Convert vector of Line3d/Line2d to a vector of homogeneous Eigen3 vector types
    std::vector<Eigen::Vector4d> line3dToHVector4d(const std::vector<Line3d> &l3d)
    {
        std::vector<Eigen::Vector4d> result;
        for (Line3d l : l3d)
        {
            result.push_back(l.getP1Eigen());
            result.push_back(l.getP2Eigen());
        }
        return result;
    }
    std::vector<Eigen::Vector3d> line2dToHVector3d(const std::vector<Line2d> &l2d)
    {
        std::vector<Eigen::Vector3d> result;
        for (Line2d l : l2d)
        {
            result.push_back(l.getP1Eigen());
            result.push_back(l.getP2Eigen());
        }
        return result;
    }


    // Convert vector of Line2d from pixel co-ords to normalized 2D (for softposit)
	std::vector<Line2d> normalizeLines(const std::vector<Line2d> &input, const cv::Mat &K)
    {
        cv::Mat invK = invertKMat(K);
		std::vector<Line2d> result;
		result.resize(input.size());
        for (int i = 0; i < input.size(); ++i)
        {
            cv::Mat p1n = invK * input[i].getP1CV();
            cv::Mat p2n = invK * input[i].getP2CV();
			result[i] = Line2d(p1n, p2n);
        }
		return result;
    }
    Line2d unnormalize(const Line2d &line, const cv::Mat &K, bool bundlerFormat)
    {
        // http://www.land-of-kain.de/docs/coords/ BUNDLER INFO
        cv::Mat Z = cv::Mat::eye(3, 3, CV_32F); // 3x3
        Z.at<float>(0, 0) = bundlerFormat ? -1 : 1; // bundler requires -1 ...
        return Line2d(cv::Mat((K * Z) * line.getP1CV()), cv::Mat((K * Z) * line.getP2CV()));
    }


    // Get 3D lines from TXT (Line3Dpp), STL, or Lin file, and 2D lines from STL, or Lin file
	std::vector<Line3d> get3dLinesTxtFile(const std::string &file)
    {
		std::vector<Line3d> outLines;
        std::ifstream inFile(file, std::ifstream::in);

        if (!inFile.is_open())
        {
            std::cout << "[Warning] -- Failed to open file: " << file << std::endl;
            return outLines;
        }

        // read each line of the .txt file
        std::vector<std::string> l3dLines;
        std::string inLine;
        while (std::getline(inFile, inLine))
        {
            l3dLines.push_back(inLine);
        }
        inFile.close();
        for (std::string currLine : l3dLines)
        {
            int n;
            double x1, y1, z1, x2, y2, z2;
            std::istringstream ss(currLine);
            ss >> n;
            ss >> x1 >> y1 >> z1 >> x2 >> y2 >> z2;
			outLines.push_back(Line3d(cv::Vec4f(x1, y1, z1, 1.0), cv::Vec4f(x2, y2, z2, 1.0)));
        }
		return outLines;
    }
	std::vector<Line3d> get3dLinesStlFile(const std::string &file)
    {
		std::vector<Line3d> outLines;
        std::ifstream inFile(file, std::ifstream::in);

        if (!inFile.is_open())
        {
            std::cout << "[Warning] -- Failed to open file: " << file << std::endl;
            return outLines;
        }

        std::string line;
        int vertexNo = 0;
        cv::Vec4f tmp[3];

        // stl line models are lists of 'facets' or triangles
        // each triangle is a line of the line model
        // where the 3rd triangle vertex is a duplicate of the first
        // so the third vertex for each facet can be ignored
        // so this method just reads 3 'vertex' lines and takes the
        // points from the first two lines
        while (getline(inFile, line))
        {
            std::istringstream ssLine(line);
            std::string firstToken;
            ssLine >> firstToken;
            if (firstToken.compare("vertex") == 0) {
                ssLine >> tmp[vertexNo][0];
                ssLine >> tmp[vertexNo][1];
                ssLine >> tmp[vertexNo][2];
                tmp[vertexNo++][3] = 1.0;
            }
            if (vertexNo == 3)
            {
				outLines.push_back(Line3d(tmp[0], tmp[1]));
                vertexNo = 0;
            }
        }
		return outLines;
    }
	std::vector<Line3d> get3dLinesLinFile(const std::string &file)
    {
		std::vector<Line3d> outLines;
        std::ifstream inFile(file, std::ifstream::in);
        if (!inFile.is_open())
        {
            std::cout << "[Warning] -- Failed to open file: " << file << std::endl;
            return outLines;
        }

        int nLines;
        inFile >> nLines;

        outLines.resize(nLines);
        for (int i = 0; i < nLines; ++i) {
            cv::Vec4f P1, P2;
            inFile >> P1[0];
            inFile >> P1[1];
            inFile >> P1[2];
            P1[3] = 1.0;

            inFile >> P2[0];
            inFile >> P2[1];
            inFile >> P2[2];
            P2[3] = 1.0;

			outLines[i] = Line3d(P1, P2);
        }
		return outLines;
    }
	std::vector<Line3d> get3dLines(const std::string &file)
    {
        if (file.substr(file.find_last_of('.')).compare(".lin") == 0)
        {
            return get3dLinesLinFile(file);
        }
        else if (file.substr(file.find_last_of('.')).compare(".stl") == 0)
        {
            return get3dLinesStlFile(file);
        }

        else if (file.substr(file.find_last_of('.')).compare(".txt") == 0)
        {
            return get3dLinesTxtFile(file);
        }
		else
		{
			std::cout << "[Warning] -- Failed to get 3D lines from file: " << file << ". Unknown file extension." << std::endl;
			return std::vector<Line3d>();
		}
    }
	std::vector<Line2d> get2dLinesStlFile(const std::string &file)
    {
		std::vector<Line2d> outLines;
        std::ifstream inFile(file, std::ifstream::in);

        if (!inFile.is_open())
        {
            std::cout << "[Warning] -- Failed to open file: " << file << std::endl;
            return outLines;
        }

        std::string line;
        int vertexNo = 0;
        cv::Vec3f tmp[3];

        // stl line models are lists of 'facets' or triangles
        // each triangle is a line of the line model
        // where the 3rd triangle vertex is a duplicate of the first
        // so the third vertex for each facet can be ignored
        // so this method just reads 3 'vertex' lines and takes the
        // points from the first two lines
        while (getline(inFile, line))
        {
            std::istringstream ssLine(line);
            std::string firstToken;
            ssLine >> firstToken;
            if (firstToken.compare("vertex") == 0) {
                ssLine >> tmp[vertexNo][0];
                ssLine >> tmp[vertexNo][1];
                tmp[vertexNo++][2] = 1.0;
            }
            if (vertexNo == 3)
            {
                Line2d newLine = Line2d(tmp[0], tmp[1]);
                newLine.setColor(cv::Scalar(80, 80, 80));
				outLines.push_back(newLine);
                vertexNo = 0;
            }
        }
		return outLines;
    }
	std::vector<Line2d> get2dLinesLinFile(const std::string &file)
    {
		std::vector<Line2d> outLines;
        std::ifstream inFile(file, std::ifstream::in);

        if (!inFile.is_open())
        {
            std::cout << "[Warning] -- Failed to open file: " << file << std::endl;
            return outLines;
        }

        int nLines;
        inFile >> nLines;

		outLines.resize(nLines);
        for (int i = 0; i < nLines; ++i) {
            cv::Vec3f p1, p2;
            inFile >> p1[0];
            inFile >> p1[1];
            p1[2] = 1.0;

            inFile >> p2[0];
            inFile >> p2[1];
            p2[2] = 1.0;
			outLines[i] = Line2d(p1, p2);
			outLines[i].setColor(cv::Scalar(80, 80, 80));
        }
		return outLines;
    }
	std::vector<Line2d> get2dLines(const std::string &file)
    {
        if (file.substr(file.find_last_of('.')).compare(".lin") == 0)
        {
            return get2dLinesLinFile(file);
        }
        else if (file.substr(file.find_last_of('.')).compare(".stl") == 0)
        {
            return get2dLinesStlFile(file);
        }
        else
        {
            std::cout << "[Warning] -- Failed to get 2D lines from file: " << file << ". Unknown file extension." << std::endl;
			return std::vector<Line2d>();
		}
    }


    // Write 3D or 2D lines to STL, or Lin file
    void write3dLinesStlFile(const std::string &file, const std::vector<Line3d> &lines)
    {
        std::ofstream ofs(file, std::ofstream::out);
        ofs << "solid lineModel" << std::endl;
        for (Line3d line : lines)
        {
            ofs << " facet normal 1 0 0" << std::endl;
            ofs << "  outer loop" << std::endl;
            ofs << "   vertex " << cv::Vec4f(line.getP1CV())[0] << " " << cv::Vec4f(line.getP1CV())[1] << " " << cv::Vec4f(line.getP1CV())[2] << std::endl;
            ofs << "   vertex " << cv::Vec4f(line.getP2CV())[0] << " " << cv::Vec4f(line.getP2CV())[1] << " " << cv::Vec4f(line.getP2CV())[2] << std::endl;
            ofs << "   vertex " << cv::Vec4f(line.getP1CV())[0] << " " << cv::Vec4f(line.getP1CV())[1] << " " << cv::Vec4f(line.getP1CV())[2] << std::endl;
            ofs << "  endloop" << std::endl;
            ofs << " endfacet" << std::endl;
        }
        ofs << "endsolid lineModel" << std::endl;
    }
    void write3dLinesLinFile(const std::string &file, const std::vector<Line3d> &lines)
    {
        std::ofstream ofs(file, std::ofstream::out);
        ofs << lines.size() << std::endl;
        for (Line3d line : lines)
        {
            ofs << cv::Vec4f(line.getP1CV())[0] << " " << cv::Vec4f(line.getP1CV())[1] << " " << cv::Vec4f(line.getP1CV())[2] << " " << cv::Vec4f(line.getP2CV())[0] << " " << cv::Vec4f(line.getP2CV())[1] << " " << cv::Vec4f(line.getP2CV())[2] << std::endl;
        }
    }
    void write3dLines(const std::string &file, const std::vector<Line3d> &lines)
    {
        if (file.substr(file.find_last_of('.')).compare(".lin") == 0)
        {
            write3dLinesLinFile(file, lines);
        }
        else if (file.substr(file.find_last_of('.')).compare(".stl") == 0)
        {
            write3dLinesStlFile(file, lines);
        }
        else
        {
            std::cout << "[Warning] -- Failed to write 3D lines to file: " << file << ". Unknown file extension." << std::endl;
        }
    }
    void write2dLinesStlFile(const std::string &file, const std::vector<Line2d> &lines)
    {
        std::ofstream ofs(file, std::ofstream::out);
        ofs << "solid lineModel" << std::endl;
        for (Line2d line : lines)
        {
            ofs << " facet normal 1.0e+000 0.0e+000 0.0e+000" << std::endl;
            ofs << "  outer loop" << std::endl;
            ofs << "   vertex " << cv::Vec3f(line.getP1CV())[0] << " " << cv::Vec3f(line.getP1CV())[1] << " " << 1.0 << std::endl;
            ofs << "   vertex " << cv::Vec3f(line.getP2CV())[0] << " " << cv::Vec3f(line.getP2CV())[1] << " " << 1.0 << std::endl;
            ofs << "   vertex " << cv::Vec3f(line.getP1CV())[0] << " " << cv::Vec3f(line.getP1CV())[1] << " " << 1.0 << std::endl;
            ofs << "  endloop" << std::endl;
            ofs << " endfacet" << std::endl;
        }
        ofs << "endsolid lineModel" << std::endl;
    }
    void write2dLinesLinFile(const std::string &file, const std::vector<Line2d> &lines)
    {
        std::ofstream ofs(file, std::ofstream::out);
        ofs << lines.size() << std::endl;
        for (Line2d line : lines)
        {
            ofs << cv::Vec3f(line.getP1CV())[0] << " " << cv::Vec3f(line.getP1CV())[1] << " " << cv::Vec3f(line.getP2CV())[0] << " " << cv::Vec3f(line.getP2CV())[1] << std::endl;
        }
    }
    void write2dLines(const std::string &file, const std::vector<Line2d> &lines)
    {
        if (file.substr(file.find_last_of('.')).compare(".lin") == 0)
        {
            write2dLinesLinFile(file, lines);
        }
        else if (file.substr(file.find_last_of('.')).compare(".stl") == 0)
        {
            write2dLinesStlFile(file, lines);
        }
        else
        {
            std::cout << "[Warning] -- Failed to write 2D lines to file: " << file << ". Unknown file extension." << std::endl;
        }
    }


    // Measure reprojection error of measured 2d lines and projected 3d lines using camera calibration K, R, and t
    double lineReprojectionError(const std::vector<Line2d> &lines2d, const std::vector<Line3d> &lines3d, const Eigen::Matrix3d &K, const Eigen::Matrix3d &R, const Eigen::Vector3d &t, bool bundlerFormat)
    {
        double totalErr = 0.0;
        int n = 0;
        for (int i = 0; i < lines2d.size() && i < lines3d.size(); ++i)
        {
            Line2d currLine = unnormalize(lines2d[i], Matrix3dToMat(K), bundlerFormat);
            Line2d projectedLine = lines3d[i].project(Matrix3dToMat(K), Matrix3dToMat(R), Vector3dToMat(t), bundlerFormat);

            if (!currLine.isVisible(Matrix3dToMat(K)) && !projectedLine.isVisible(Matrix3dToMat(K)))
            {
                continue;
            }

            double errP1 = cv::norm(currLine.getP1CV() - projectedLine.getP1CV());
            double errP2 = cv::norm(currLine.getP1CV() - projectedLine.getP2CV());

            if (errP1 < errP2)
            {
                // p1 matches projected p1, and p2 matches projected p2
                totalErr += cv::norm(currLine.getP1CV() - projectedLine.getP1CV());
                totalErr += cv::norm(currLine.getP2CV() - projectedLine.getP2CV());
            }
            else
            {
                // p1 matches projected p2, and p2 matches projected p1
                totalErr += cv::norm(currLine.getP1CV() - projectedLine.getP2CV());
                totalErr += cv::norm(currLine.getP2CV() - projectedLine.getP1CV());
            }
            ++n;
        }
        return totalErr / (2 * n);
    }
    double lineReprojectionError(const std::vector<Line3d> &lines3d, const Eigen::Matrix3d &gtK, const Eigen::Matrix3d &gtR, const Eigen::Vector3d &gtt, const Eigen::Matrix3d &estK, const Eigen::Matrix3d &estR, const Eigen::Vector3d &estt, bool bundlerFormat)
    {
        double totalErr = 0.0;
        int n = 0;
        for (int i = 0; i < lines3d.size(); ++i)
        {
            Line2d gtLineProj = lines3d[i].project(Matrix3dToMat(gtK), Matrix3dToMat(gtR), Vector3dToMat(gtt), bundlerFormat);
            Line2d estLineProj = lines3d[i].project(Matrix3dToMat(estK), Matrix3dToMat(estR), Vector3dToMat(estt), bundlerFormat);

            //estLineProj.setColor(cv::Scalar(80, 0, 0));
            //cv::Mat im = cv::Mat::zeros(cv::Size(gtK(0, 2)*2, gtK(1, 2)*2), CV_8UC1);
            //gtLineProj.draw(im);
            //estLineProj.draw(im);
            //cv::imshow("im", im);
            //cv::waitKey(0);

            if (!gtLineProj.isVisible(Matrix3dToMat(gtK)) && !estLineProj.isVisible(Matrix3dToMat(estK)))
            {
                continue;
            }

            double errP1 = cv::norm(gtLineProj.getP1CV() - estLineProj.getP1CV());
            double errP2 = cv::norm(gtLineProj.getP1CV() - estLineProj.getP2CV());

            if (errP1 < errP2)
            {
                // p1 matches projected p1, and p2 matches projected p2
                totalErr += cv::norm(gtLineProj.getP1CV() - estLineProj.getP1CV());
                totalErr += cv::norm(gtLineProj.getP2CV() - estLineProj.getP2CV());
            }
            else
            {
                // p1 matches projected p2, and p2 matches projected p1
                totalErr += cv::norm(gtLineProj.getP1CV() - estLineProj.getP2CV());
                totalErr += cv::norm(gtLineProj.getP2CV() - estLineProj.getP1CV());
            }
            ++n;
        }
        return totalErr / (2 * n);
    }


    // 2D and 3D line perturbation / mutation
    cv::Vec4f randomPoint(float minX, float maxX, float minY, float maxY, float minZ, float maxZ)
    {
        float rx = minX + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (maxX - minX)));
        float ry = minY + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (maxY - minY)));
        float rz = minZ + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (maxZ - minZ)));
        return cv::Vec4f(rx, ry, rz, 1.0);
    }
    void add3dClutter(std::vector<Line3d> &inOutLines, int n)
    {
        // number of extra random lines to add
        int extraLines = n;

        // find the 3D range of the lines, and their average length
        float
            minX = FLT_MAX,
            maxX = -FLT_MAX,
            minY = FLT_MAX,
            maxY = -FLT_MAX,
            minZ = FLT_MAX,
            maxZ = -FLT_MAX;

        float averageLength = 0.0;
        for (Line3d l : inOutLines)
        {
            if (l.getP1CV().at<float>(0, 0) < minX) minX = l.getP1CV().at<float>(0, 0);
            if (l.getP1CV().at<float>(0, 0) > maxX) maxX = l.getP1CV().at<float>(0, 0);
            if (l.getP1CV().at<float>(1, 0) < minY) minY = l.getP1CV().at<float>(1, 0);
            if (l.getP1CV().at<float>(1, 0) > maxY) maxY = l.getP1CV().at<float>(1, 0);
            if (l.getP1CV().at<float>(2, 0) < minZ) minZ = l.getP1CV().at<float>(2, 0);
            if (l.getP1CV().at<float>(2, 0) > maxZ) maxZ = l.getP1CV().at<float>(2, 0);
            if (l.getP2CV().at<float>(0, 0) < minX) minX = l.getP2CV().at<float>(0, 0);
            if (l.getP2CV().at<float>(0, 0) > maxX) maxX = l.getP2CV().at<float>(0, 0);
            if (l.getP2CV().at<float>(1, 0) < minY) minY = l.getP2CV().at<float>(1, 0);
            if (l.getP2CV().at<float>(1, 0) > maxY) maxY = l.getP2CV().at<float>(1, 0);
            if (l.getP2CV().at<float>(2, 0) < minZ) minZ = l.getP2CV().at<float>(2, 0);
            if (l.getP2CV().at<float>(2, 0) > maxZ) maxZ = l.getP2CV().at<float>(2, 0);
            averageLength += l.length();
        }
        averageLength /= inOutLines.size();

        /*cv::Vec4f tlb(minX, minY, minZ, 1.0);
        cv::Vec4f trb(maxX, minY, minZ, 1.0);
        cv::Vec4f blb(minX, maxY, minZ, 1.0);
        cv::Vec4f brb(maxX, maxY, minZ, 1.0);
        cv::Vec4f tlf(minX, minY, maxZ, 1.0);
        cv::Vec4f trf(maxX, minY, maxZ, 1.0);
        cv::Vec4f blf(minX, maxY, maxZ, 1.0);
        cv::Vec4f brf(maxX, maxY, maxZ, 1.0);
        lines.push_back(Line3d(tlb, trb));
        lines.push_back(Line3d(trb, brb));
        lines.push_back(Line3d(brb, blb));
        lines.push_back(Line3d(blb, tlb));
        lines.push_back(Line3d(tlb, tlf));
        lines.push_back(Line3d(trb, trf));
        lines.push_back(Line3d(brb, brf));
        lines.push_back(Line3d(blb, blf));
        lines.push_back(Line3d(tlf, trf));
        lines.push_back(Line3d(trf, brf));
        lines.push_back(Line3d(brf, blf));
        lines.push_back(Line3d(blf, tlf));*/

        // generate the extra lines
        for (int i = 0; i < extraLines; ++i)
        {
            cv::Vec4f p1 = randomPoint(minX, maxX, minY, maxY, minZ, maxZ);
            cv::Vec4f p2dir = randomPoint(-1, 1, -1, 1, -1, 1);
            p2dir *= (2 * averageLength);
            cv::Vec4f p2 = p1 + p2dir;

            Line3d newLine(p1, p2);
            newLine.setColor(cv::Scalar(0, 0, 255));
			inOutLines.push_back(newLine);
        }
    }
    void add2dClutter(std::vector<Line2d> &inOutLines, int n)
    {
        // number of extra random lines to add
        int extraLines = n;

        // find the 3D range of the lines, and their average length
        float
            minX = FLT_MAX,
            maxX = -FLT_MAX,
            minY = FLT_MAX,
            maxY = -FLT_MAX;

        float averageLength = 0.0;
        for (Line2d l : inOutLines)
        {
            if (l.getP1CV().at<float>(0, 0) < minX) minX = l.getP1CV().at<float>(0, 0);
            if (l.getP1CV().at<float>(0, 0) > maxX) maxX = l.getP1CV().at<float>(0, 0);
            if (l.getP1CV().at<float>(1, 0) < minY) minY = l.getP1CV().at<float>(1, 0);
            if (l.getP1CV().at<float>(1, 0) > maxY) maxY = l.getP1CV().at<float>(1, 0);
            if (l.getP2CV().at<float>(0, 0) < minX) minX = l.getP2CV().at<float>(0, 0);
            if (l.getP2CV().at<float>(0, 0) > maxX) maxX = l.getP2CV().at<float>(0, 0);
            if (l.getP2CV().at<float>(1, 0) < minY) minY = l.getP2CV().at<float>(1, 0);
            if (l.getP2CV().at<float>(1, 0) > maxY) maxY = l.getP2CV().at<float>(1, 0);
            averageLength += l.length();
        }
        averageLength /= inOutLines.size();

        /*cv::Vec4f tlb(minX, minY, minZ, 1.0);
        cv::Vec4f trb(maxX, minY, minZ, 1.0);
        cv::Vec4f blb(minX, maxY, minZ, 1.0);
        cv::Vec4f brb(maxX, maxY, minZ, 1.0);
        cv::Vec4f tlf(minX, minY, maxZ, 1.0);
        cv::Vec4f trf(maxX, minY, maxZ, 1.0);
        cv::Vec4f blf(minX, maxY, maxZ, 1.0);
        cv::Vec4f brf(maxX, maxY, maxZ, 1.0);
        lines.push_back(Line3d(tlb, trb));
        lines.push_back(Line3d(trb, brb));
        lines.push_back(Line3d(brb, blb));
        lines.push_back(Line3d(blb, tlb));
        lines.push_back(Line3d(tlb, tlf));
        lines.push_back(Line3d(trb, trf));
        lines.push_back(Line3d(brb, brf));
        lines.push_back(Line3d(blb, blf));
        lines.push_back(Line3d(tlf, trf));
        lines.push_back(Line3d(trf, brf));
        lines.push_back(Line3d(brf, blf));
        lines.push_back(Line3d(blf, tlf));*/

        // generate the extra lines
        for (int i = 0; i < extraLines; ++i)
        {
            cv::Vec4f p1 = randomPoint(minX, maxX, minY, maxY, 0, 1);
            cv::Vec4f p2dir = randomPoint(-1, 1, -1, 1, -1, 1);
            p2dir *= (2 * averageLength);
            cv::Vec4f p2 = p1 + p2dir;

            Line2d newLine(cv::Vec4f(p1[0], p1[1], p2[0], p2[1]));
            newLine.setColor(cv::Scalar(0, 0, 80));
			inOutLines.push_back(newLine);
        }
    }
    void hide3dLines(std::vector<Line3d> &inOutLines, int n)
    {
        while (n > 0)
        {
            // pick a random 3d line
            int idx = rand() % inOutLines.size();

            // delete it from the vector of 3d lines
			inOutLines.erase(inOutLines.begin() + idx);
            n--;
        }
    }
    void hide2dLines(std::vector<Line2d> &inOutLines, int n)
    {
        while (n > 0)
        {
            // pick a random 2d line
            int idx = rand() % inOutLines.size();

            // delete it from the vector of 2d lines
			inOutLines.erase(inOutLines.begin() + idx);
            n--;
        }
    }
    void modLength2dLines(std::vector<Line2d> &inOutLines, int n)
    {
        while (n > 0)
        {
            // pick a random 2d line
            int idx = rand() % inOutLines.size();
            Line2d line = inOutLines[idx];
            Eigen::Vector2d p1 = line.getP1Eigen().head<2>();
            Eigen::Vector2d p2 = line.getP2Eigen().head<2>();

            // determine its direction
            Eigen::Vector2d dir = (p2 - p1).normalized();
            double length = line.length();

            // generate a random offset for scaling the direction vector somewhere within the perturbation radius
            double perturbRadius = 0.5;
            float r1 = -perturbRadius + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (perturbRadius - -perturbRadius)));
            float r2 = -perturbRadius + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (perturbRadius - -perturbRadius)));

            // mutate the endpoints by adding a scaled version of the direction vector to them
            p1 = p1 + (dir * r1*length);
            p2 = p2 + (dir * r2*length);

            // replace the line with the new perturbed line
			inOutLines[idx] = Line2d(cv::Vec4f(p1(0), p1(1), p2(0), p2(1)));
			inOutLines[idx].setColor(cv::Scalar(80, 80, 0));
            n--;
        }
    }
    void duplicate3dLines(std::vector<Line3d> &inOutLines, int n)
    {
        // store the initial number of lines
        int numInitialLines = inOutLines.size();
        while (n > 0)
        {
            // pick a random 3d line
            int idx = rand() % numInitialLines;

            // shift the duplicated line a little bit (let's not bother)
            //double radius = 0.00001;
            //cv::Vec4f rp1 = randomPoint(-radius, radius, -radius, radius, -radius, radius); //unused
            //cv::Vec4f rp2 = randomPoint(-radius, radius, -radius, radius, -radius, radius); //unused

            // just copy the line and add it to the vector of 3d lines
            Line3d line(cv::Vec4f(inOutLines[idx].getP1CV()), cv::Vec4f(inOutLines[idx].getP2CV()));
            line.setColor(cv::Scalar(0, 80, 80));
			inOutLines.push_back(line);
            n--;
        }
    }
    void modEndpoints2dLines(std::vector<Line2d> &inOutLines, int n)
    {
        while (n > 0)
        {
            // pick a random 2d line
            int idx = rand() % inOutLines.size();
            Line2d line = inOutLines[idx];
            Eigen::Vector2d p1 = line.getP1Eigen().head<2>();
            Eigen::Vector2d p2 = line.getP2Eigen().head<2>();

            // generate a random offset for x and y for both points (4 random vals) somewhere within the perturbation radius
            double perturbRadius = 0.01;
            float r1 = -perturbRadius + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (perturbRadius - -perturbRadius)));
            float r2 = -perturbRadius + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (perturbRadius - -perturbRadius)));
            float r3 = -perturbRadius + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (perturbRadius - -perturbRadius)));
            float r4 = -perturbRadius + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (perturbRadius - -perturbRadius)));

            // apply the perturbation
            p1(0) += r1; p1(1) += r2;
            p2(0) += r3; p2(1) += r4;

            // replace the line with the new perturbed line
			inOutLines[idx] = Line2d(cv::Vec4f(p1(0), p1(1), p2(0), p2(1)));
			inOutLines[idx].setColor(cv::Scalar(40, 0, 120));
            n--;
        }
    }
    Eigen::Vector3d minimumOffset(const std::vector<Line3d> &inLines, std::vector<Line3d> &outLines)
	{
		double minDist = DBL_MAX;
		Eigen::Vector4d offset;

		// figure out which line endpoint is closest to the origin
		for (Utils::Line3d l : inLines)
		{
			if (l.getP1Eigen().norm() < minDist)
			{
				minDist = l.getP1Eigen().head<3>().norm();
				offset = l.getP1Eigen();
			}
			if (l.getP2Eigen().norm() < minDist)
			{
				minDist = l.getP2Eigen().head<3>().norm();
				offset = l.getP2Eigen();
			}
		}
		offset(3) = 0.0;
		//offset = Eigen::Vector4d(0, 0, 0, 0); // uncomment this to disable the offset

		// offset all lines by this closest endpoint (so the closest endpoint becomes origin)
		outLines.resize(inLines.size());
		for (int i = 0; i < inLines.size(); ++i)
		{
			Utils::Line3d newLine(Utils::Vector4dToMat(inLines[i].getP1Eigen() - offset), Utils::Vector4dToMat(inLines[i].getP2Eigen() - offset));
			outLines[i] = newLine;
		}

		// return the offset so it can be applied and unapplied to the pose
		return offset.head<3>();
	}
	void modPoseTargetError(const std::vector<Line3d> &modelLines, const cv::Mat &K, const cv::Mat &R, const cv::Mat &T, cv::Mat &outR, cv::Mat &outT, bool bundlerFormat)
	{
		// generate a random direction
		double tRadius = 1.0f;
		double dx = -tRadius + static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / (2.0 * tRadius)));
		double dy = -tRadius + static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / (2.0 * tRadius)));
		double dz = -tRadius + static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / (2.0 * tRadius)));
		Eigen::Vector3d randTrans = Eigen::Vector3d(dx, dy, dz).normalized();

		// find out roooouuughly what the model scale is by looking at the length of the lines
		double avLength = 0.0;
		for (Line3d l : modelLines)
		{
			avLength += l.length();
		}
		avLength /= modelLines.size();

		// make the random direction increment "small" with respect to the average line length
		randTrans *= (avLength / 50.0);

		// convert the mats
		Eigen::Matrix3d gtK = MatToMatrix3d(K);
		Eigen::Matrix3d gtR = MatToMatrix3d(R);
		Eigen::Vector3d gtt = MatToVector3d(T);
		Eigen::Vector3d gtPos = (-gtR.transpose()) * gtt;

		Eigen::Vector3d currentT = gtt;
		Eigen::Matrix3d currentR = gtR;
		Eigen::Vector3d currentPos = gtPos;

		int i = 1;
		double currentError = -DBL_MAX;
		double targetErrorMin = 0.01 * (K.at<float>(0, 2) * 2); //  1% of image width
		double targetErrorMax = 0.10 * (K.at<float>(0, 2) * 2); // 10% of image width
		double targetError = targetErrorMin + static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / ((targetErrorMax - targetErrorMin))));
		std::cout << "[Info] -- Iteratively computing a camera postion that results in an error of: " << targetError << " ... " << std::flush;
		do
		{
			currentPos = gtPos + (i*randTrans); // modify the camera position
			currentT = (-gtR.transpose()).inverse() * currentPos; // recompute the t vector
			currentError = lineReprojectionError(modelLines, gtK, gtR, gtt, gtK, gtR, currentT, bundlerFormat); // calc error based on original rotation
			++i;
		} while (currentError < targetError);
		std::cout << "Done in " << i << " iterations" << std::endl;

		//// modify the camera orientation
		//double rotErrorRange = 5.0;
		//double rRadius = deg2rad(rotErrorRange); // tunable param (degrees)
		//float r1 = -rRadius + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (2.0 * rRadius)));
		//float r2 = -rRadius + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (2.0 * rRadius)));
		//float r3 = -rRadius + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (2.0 * rRadius)));
		//Eigen::Matrix3d rMod;
		//double c1 = cos(r1), c2 = cos(r2), c3 = cos(r3); // FROM: https://en.wikipedia.org/wiki/Euler_angles
		//double s1 = sin(r1), s2 = sin(r2), s3 = sin(r3);
		//rMod(0, 0) = c2; rMod(0, 1) = -c3 * s2; rMod(0, 2) = s2 * s3;
		//rMod(1, 0) = c1 * s2; rMod(1, 1) = c1 * c2*c3 - s1 * s3; rMod(1, 2) = -c3 * s1 - c1 * c2*s3;
		//rMod(2, 0) = s1 * s2; rMod(2, 1) = c1 * s3 + c2 * c3*s1; rMod(2, 2) = c1 * c3 - c2 * s1*s3;
		//currentR = gtR * rMod;
		currentT = (-currentR.transpose()).inverse() * currentPos; // recompute the t vector

		// copy out the results
		Utils::Vector3dToMat(currentT).copyTo(outT);
		Utils::Matrix3dToMat(currentR).copyTo(outR);
	}
	void modPose(const cv::Mat &R, const cv::Mat &T, double rotErrorRange, double transErrorRange, cv::Mat &outR, cv::Mat &outT)
	{
		Eigen::Matrix3d Re = MatToMatrix3d(R);
		Eigen::Vector3d Te = MatToVector3d(T);
		Eigen::Vector3d Cpos = (-Re.transpose()) * Te;

		// modify the camera orientation
		double rRadius = deg2rad(rotErrorRange); // tunable param (degrees)
		float r1 = -rRadius + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (2.0 * rRadius)));
		float r2 = -rRadius + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (2.0 * rRadius)));
		float r3 = -rRadius + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (2.0 * rRadius)));
		Eigen::Matrix3d rMod;
		double c1 = cos(r1), c2 = cos(r2), c3 = cos(r3); // FROM: https://en.wikipedia.org/wiki/Euler_angles
		double s1 = sin(r1), s2 = sin(r2), s3 = sin(r3);
		rMod(0, 0) = c2; rMod(0, 1) = -c3*s2; rMod(0, 2) = s2*s3;
		rMod(1, 0) = c1*s2; rMod(1, 1) = c1*c2*c3 - s1*s3; rMod(1, 2) = -c3*s1 - c1*c2*s3;
		rMod(2, 0) = s1*s2; rMod(2, 1) = c1*s3 + c2*c3*s1; rMod(2, 2) = c1*c3 - c2*s1*s3;

		// apply the modification
		Re = Re * rMod;

		// modify the camera position
		double tRadius = transErrorRange; // tunable param
		float dx = -tRadius + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (2.0 * tRadius)));
		float dy = -tRadius + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (2.0 * tRadius)));
		float dz = -tRadius + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (2.0 * tRadius)));

		// apply the modification
		// NOTES:
		// stadium real life width == APPROX 120 metres
		// stadium model width == APPROX 5.5 units
		// GPS accuracy is at best within a few metres (let's say 3m)
		// (3/120) * 5.5 = 0.1375, so lets use this as the translation radius for stadium

		Cpos += Eigen::Vector3d(dx, dy, dz);
		Te = (-Re.transpose()).inverse() * Cpos; // recompute the t vector

		// return the results
		outR = Matrix3dToMat(Re);
		outT = Vector3dToMat(Te);
	}
	void offsetPose(const Eigen::Vector3d &offset, const cv::Mat &R, const cv::Mat &t, cv::Mat &outR, cv::Mat &outt)
	{
		// ' == transpose, ^ == inverse
		// Cpos = -R' * t
		// t = (-R')^ * Cpos
		Eigen::Vector3d cameraPos = (-Utils::MatToMatrix3d(R).transpose()) * Utils::MatToVector3d(t);
		cameraPos -= offset;
		outt = Utils::Vector3dToMat((-Utils::MatToMatrix3d(R).transpose()).inverse() * cameraPos);
		outR = R.clone(); // R is not affected
	}


}

