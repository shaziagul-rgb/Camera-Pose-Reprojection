//
// Projects 3D scene points into 2D images using estimated camera poses
// and calculates reprojection error against manually selected 2D points.
//
// The program:
//   1. Reads camera pose data from a pose file.
//   2. Loads manually selected 2D image points.
//   3. Loads corresponding 3D points.
//   4. Builds the camera intrinsic matrix.
//   5. Projects the 3D points into the image using cv::projectPoints.
//   6. Calculates the Euclidean reprojection error for each point.
//   7. Calculates the mean reprojection error for each image.
//   8. Writes per-image and per-point results to output files.
//   9. Displays the projected and manually selected points.
//
// Note:
// This version calculates reprojection errors but does NOT classify
// points as outliers. An outlier threshold should be added separately
// if required.
//

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "CommandLine.hpp"
#include "FileIO.hpp"
#include "Matrix.hpp"

//
// Calculate Euclidean distance between two 2D points.
//
double calculateEuclideanDistance(
    const cv::Point2f& point1,
    const cv::Point2f& point2)
{
    const double dx = static_cast<double>(point1.x) - point2.x;
    const double dy = static_cast<double>(point1.y) - point2.y;

    return std::sqrt(dx * dx + dy * dy);
}

//
// Render the COLMAP 3D model into the input image using the supplied
// camera intrinsic matrix and camera pose.
//
cv::Mat renderModel(
    const cv::Mat& queryImage,
    const Eigen::Matrix3d& K,
    const Eigen::Matrix3d& R,
    const Eigen::Vector3d& T,
    const std::unordered_map<int, cv::Point3f>& modelPoints)
{
    Eigen::Matrix<double, 3, 4> pose;

    pose.block<3, 3>(0, 0) = R;
    pose.block<3, 1>(0, 3) = T;

    const Eigen::Matrix<double, 3, 4> projectionMatrix = K * pose;

    cv::Mat outputImage = queryImage.clone();

    for (const auto& entry : modelPoints)
    {
        const cv::Point3f& point3D = entry.second;

        Eigen::Vector3d projected =
            projectionMatrix *
            Eigen::Vector4d(
                point3D.x,
                point3D.y,
                point3D.z,
                1.0);

        //
        // Only render points in front of the camera.
        //
        if (projected.z() <= 0.0)
        {
            continue;
        }

        projected /= projected.z();

        cv::circle(
            outputImage,
            cv::Point(
                static_cast<int>(projected.x()),
                static_cast<int>(projected.y())),
            1,
            cv::Scalar(255, 255, 255),
            -1);
    }

    return outputImage;
}

int main(int argc, const char* argv[])
{
    //
    // ---------------------------------------------------------------------
    // Parse command-line arguments
    // ---------------------------------------------------------------------
    //

    Utils::CommandLine cmd(argc, argv);

    std::string datasetDirectory = "../TestingPaperIVCNZ";

    //
    // Focal length used for the camera intrinsic matrix.
    // This can be overridden using -f.
    //
    double focalLength = 1452.6675;

    if (cmd.ContainsKey("m"))
    {
        cmd.GetStringValue("m", datasetDirectory);
    }

    std::cout << "Dataset directory: "
              << datasetDirectory
              << std::endl;

    //
    // COLMAP database.
    //
    std::string colmapDatabase =
        datasetDirectory + "/database.db";

    if (cmd.ContainsKey("d"))
    {
        cmd.GetStringValue("d", colmapDatabase);
    }

    //
    // Estimated camera pose file.
    //
    std::string poseFilename =
        datasetDirectory + "/poses_esac_.txt";

    if (cmd.ContainsKey("p"))
    {
        cmd.GetStringValue("p", poseFilename);
    }

    if (cmd.ContainsKey("f"))
    {
        cmd.GetDoubleValue("f", focalLength);
    }

    //
    // Image index is retained for compatibility with the original
    // implementation. It is currently not used to filter images.
    //
    int imageId = 0;

    if (cmd.ContainsKey("i"))
    {
        cmd.GetIntValue("i", imageId);
    }

    //
    // Validate required input paths.
    //
    if (datasetDirectory.empty())
    {
        std::cerr
            << "Error: dataset directory was not provided."
            << std::endl;

        return EXIT_FAILURE;
    }

    if (poseFilename.empty())
    {
        std::cerr
            << "Error: pose file was not provided."
            << std::endl;

        return EXIT_FAILURE;
    }

    if (colmapDatabase.empty())
    {
        std::cerr
            << "Error: COLMAP database path was not provided."
            << std::endl;

        return EXIT_FAILURE;
    }

    //
    // ---------------------------------------------------------------------
    // Load COLMAP reconstruction
    // ---------------------------------------------------------------------
    //

    std::unordered_map<int, cv::Point3f> sfmPoints;
    std::unordered_map<int, cv::Vec3b> sfmPointColours;

    std::unordered_map<
        int,
        std::vector<std::pair<int, int>>> sfmObservations;

    std::unordered_map<int, std::vector<int>> sfmCorrespondences;

    std::unordered_map<int, cv::Mat> sfmRotations;
    std::unordered_map<int, cv::Mat> sfmTranslations;

    std::unordered_map<int, float> sfmFocalLengths;

    const std::string colmapModelDirectory =
        datasetDirectory + "/sparse";

    const bool reconstructionLoaded =
        Utils::colmapGetReconstruction(
            colmapModelDirectory,
            sfmPoints,
            sfmPointColours,
            sfmObservations,
            sfmCorrespondences,
            sfmRotations,
            sfmTranslations,
            sfmFocalLengths);

    if (!reconstructionLoaded)
    {
        std::cerr
            << "Warning: COLMAP reconstruction could not be loaded."
            << std::endl;
    }

    std::cout
        << "Loaded "
        << sfmPoints.size()
        << " 3D points from COLMAP."
        << std::endl;

    //
    // ---------------------------------------------------------------------
    // Open pose file
    // ---------------------------------------------------------------------
    //

    std::ifstream poseFile(
        poseFilename,
        std::ios::in);

    if (!poseFile.is_open())
    {
        std::cerr
            << "Error: could not open pose file: "
            << poseFilename
            << std::endl;

        return EXIT_FAILURE;
    }

    //
    // Consolidated results for all images.
    //
    std::ofstream consolidatedResults(
        "consolidatedResults.csv",
        std::ios::out);

    if (!consolidatedResults.is_open())
    {
        std::cerr
            << "Error: could not create consolidatedResults.csv"
            << std::endl;

        return EXIT_FAILURE;
    }

    consolidatedResults
        << "imgName"
        << ",Mean"
        << ",Sum"
        << ",NumPoints"
        << std::endl;

    //
    // ---------------------------------------------------------------------
    // Process each image pose
    // ---------------------------------------------------------------------
    //

    std::string line;

    while (std::getline(poseFile, line))
    {
        if (line.empty())
        {
            continue;
        }

        //
        // Each pose line is expected to contain:
        //
        // imageName qw qx qy qz tx ty tz
        //
        std::stringstream stream(line);

        std::string imageFilename;

        float qw;
        float qx;
        float qy;
        float qz;

        float tx;
        float ty;
        float tz;

        stream
            >> imageFilename
            >> qw
            >> qx
            >> qy
            >> qz
            >> tx
            >> ty
            >> tz;

        if (stream.fail())
        {
            std::cerr
                << "Warning: invalid pose line: "
                << line
                << std::endl;

            continue;
        }

        std::cout
            << "\nProcessing image: "
            << imageFilename
            << std::endl;

        //
        // -----------------------------------------------------------------
        // Load manually selected 2D points
        // -----------------------------------------------------------------
        //

        const std::vector<cv::Point2f> imagePoints =
            Utils::get2DPoints(
                datasetDirectory,
                imageFilename);

        //
        // Load corresponding 3D points.
        //
        const std::vector<cv::Point3f> objectPoints =
            Utils::get3DPoints(
                datasetDirectory,
                imageFilename);

        //
        // Both lists must contain the same number of points because
        // point i in the 2D list corresponds to point i in the 3D list.
        //
        if (imagePoints.size() != objectPoints.size())
        {
            std::cerr
                << "Warning: number of 2D and 3D points does not match "
                << "for image "
                << imageFilename
                << ". 2D points: "
                << imagePoints.size()
                << ", 3D points: "
                << objectPoints.size()
                << std::endl;

            continue;
        }

        if (imagePoints.empty())
        {
            std::cerr
                << "Warning: no points found for "
                << imageFilename
                << std::endl;

            continue;
        }

        //
        // -----------------------------------------------------------------
        // Load image
        // -----------------------------------------------------------------
        //

        const std::string imagePath =
            datasetDirectory + "/rgb/" + imageFilename;

        cv::Mat image =
            cv::imread(
                imagePath,
                cv::IMREAD_COLOR);

        if (image.empty())
        {
            std::cerr
                << "Warning: could not load image: "
                << imagePath
                << std::endl;

            continue;
        }

        //
        // -----------------------------------------------------------------
        // Build camera intrinsic matrix
        // -----------------------------------------------------------------
        //
        // Current implementation assumes:
        //
        //   fx = fy = supplied focal length
        //   cx = image width / 2
        //   cy = image height / 2
        //
        // This is an approximation. For accurate camera calibration,
        // these values should come from the actual camera calibration.
        //

        cv::Mat cameraMatrix =
            cv::Mat::eye(
                3,
                3,
                CV_64F);

        cameraMatrix.at<double>(0, 0) = focalLength;
        cameraMatrix.at<double>(1, 1) = focalLength;

        cameraMatrix.at<double>(0, 2) =
            image.cols / 2.0;

        cameraMatrix.at<double>(1, 2) =
            image.rows / 2.0;

        //
        // No lens distortion is currently applied.
        //
        cv::Mat distortionCoefficients =
            cv::Mat::zeros(
                4,
                1,
                CV_64F);

        //
        // -----------------------------------------------------------------
        // Convert camera pose
        // -----------------------------------------------------------------
        //

        //
        // COLMAP quaternion format:
        //
        // qw qx qy qz
        //
        Eigen::Quaternionf quaternion(
            qw,
            qx,
            qy,
            qz);

        const Eigen::Matrix3f rotationFloat =
            quaternion.toRotationMatrix();

        cv::Mat rotationMatrix(
            3,
            3,
            CV_64F);

        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                rotationMatrix.at<double>(row, col) =
                    static_cast<double>(
                        rotationFloat(row, col));
            }
        }

        cv::Mat translationVector =
            (cv::Mat_<double>(3, 1)
                << tx,
                   ty,
                   tz);

        //
        // -----------------------------------------------------------------
        // Project 3D points into the image
        // -----------------------------------------------------------------
        //

        std::vector<cv::Point2f> projectedPoints;

        cv::projectPoints(
            objectPoints,
            rotationMatrix,
            translationVector,
            cameraMatrix,
            distortionCoefficients,
            projectedPoints);

        if (projectedPoints.size() != imagePoints.size())
        {
            std::cerr
                << "Error: number of projected points does not match "
                << "number of manually selected points."
                << std::endl;

            continue;
        }

        //
        // -----------------------------------------------------------------
        // Calculate reprojection error
        // -----------------------------------------------------------------
        //

        double sumError = 0.0;

        //
        // Output file containing the individual point errors.
        //
        std::ofstream pointResults(
            imageFilename + ".txt",
            std::ios::out);

        if (!pointResults.is_open())
        {
            std::cerr
                << "Warning: could not create output file for "
                << imageFilename
                << std::endl;
        }

        for (std::size_t i = 0;
             i < imagePoints.size();
             ++i)
        {
            //
            // Euclidean distance between:
            //
            //   manually selected 2D point
            //             and
            //   projected 3D point
            //
            const double error =
                calculateEuclideanDistance(
                    imagePoints[i],
                    projectedPoints[i]);

            sumError += error;

            if (pointResults.is_open())
            {
                pointResults
                    << "Point " << i
                    << "\n  Manual 2D point: "
                    << imagePoints[i]
                    << "\n  Projected point: "
                    << projectedPoints[i]
                    << "\n  Reprojection error: "
                    << error
                    << "\n\n";
            }

            //
            // Draw projected point.
            //
            cv::circle(
                image,
                projectedPoints[i],
                9,
                cv::Scalar(255, 0, 0),
                -1);

            //
            // Draw manually selected point.
            //
            cv::circle(
                image,
                imagePoints[i],
                10,
                cv::Scalar(0, 255, 255),
                -1);
        }

        //
        // -----------------------------------------------------------------
        // Calculate mean reprojection error
        // -----------------------------------------------------------------
        //

        const double meanError =
            sumError /
            static_cast<double>(imagePoints.size());

        std::cout
            << "Mean reprojection error: "
            << meanError
            << std::endl;

        if (pointResults.is_open())
        {
            pointResults
                << "Mean reprojection error = "
                << meanError
                << std::endl;
        }

        //
        // -----------------------------------------------------------------
        // Write consolidated result
        // -----------------------------------------------------------------
        //

        consolidatedResults
            << imageFilename
            << ","
            << meanError
            << ","
            << sumError
            << ","
            << imagePoints.size()
            << std::endl;

        //
        // -----------------------------------------------------------------
        // Render the complete COLMAP model for visualisation
        // -----------------------------------------------------------------
        //

        Eigen::Matrix3d eigenK =
            Eigen::Matrix3d::Identity();

        eigenK(0, 0) = focalLength;
        eigenK(1, 1) = focalLength;

        eigenK(0, 2) =
            image.cols / 2.0;

        eigenK(1, 2) =
            image.rows / 2.0;

        Eigen::Matrix3d eigenR =
            rotationFloat.cast<double>();

        Eigen::Vector3d eigenT(
            tx,
            ty,
            tz);

        cv::Mat renderedImage =
            renderModel(
                image,
                eigenK,
                eigenR,
                eigenT,
                sfmPoints);

        //
        // -----------------------------------------------------------------
        // Display results
        // -----------------------------------------------------------------
        //

        cv::imshow(
            "Reprojection",
            image);

        cv::imshow(
            "COLMAP Model",
            renderedImage);

        cv::waitKey(1);
    }

    poseFile.close();
    consolidatedResults.close();

    return EXIT_SUCCESS;
}