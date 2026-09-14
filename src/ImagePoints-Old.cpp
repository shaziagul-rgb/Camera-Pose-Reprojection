//
// CREATES POSE, AND CALIBRATION FILES FOR ESAC
//

#include <string>
#include <fstream>
#include <ctime>
#include <unordered_map>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "CommandLine.hpp"
#include "FileIO.hpp"
#include "Matrix.hpp"

#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#endif

std::vector<cv::Point3f> Generate3DPoints;
std::vector<cv::Point2f> Generate2Dpoints;
std::vector<cv::Point2f> projectedPoints;

cv::Mat renderModel(const cv::Mat &query, const Eigen::Matrix3d &K, const Eigen::Matrix3d &R, const Eigen::Vector3d &T, const std::unordered_map<int, cv::Point3f> mps)
{
  Eigen::Matrix<double, 3, 4> pose;
  pose.block<3, 3>(0, 0) = R;
  pose.block<3, 1>(0, 3) = T;

  Eigen::Matrix<double, 3, 4> P = K * pose;

  cv::Mat out = query.clone();
  // cv::Mat out = cv::Mat::zeros(query.size(), query.type());

  for (const auto &mp_ent : mps)
  {
    const cv::Point3f &pt = mp_ent.second;
    int idx = mp_ent.first;
    Eigen::Vector3d proj = P * Eigen::Vector4d(pt.x, pt.y, pt.z, 1.0);
    if (proj.z() > 0)
    {
      proj /= proj.z();
      // const cv::Vec3b& colr = mps_colrs.at(idx);
      const cv::Vec3b colr(255, 255, 255);
      cv::circle(out, cv::Point(proj.x(), proj.y()), 1, cv::Scalar(colr[0], colr[1], colr[2]), -1);
    }
  }

  // cv::namedWindow("render", cv::WINDOW_FREERATIO);
  cv::imshow("render", out);
  // int k;
  // while (true)
  // {
  //     k = cv::waitKey(33);
  //     if (k == 27) break;
  // }

  return out;
}

double distanceCalculate(double x1, double y1, double x2, double y2)
{
  double x = x1 - x2; // calculating number to square in next step
  double y = y1 - y2;
  double dist;

  dist = pow(x, 2) + pow(y, 2); // calculating Euclidean distance
  dist = sqrt(dist);

  return dist;
}

int main(int argc, const char *argv[])
{
  // parse args
  Utils::CommandLine cmd(argc, argv);
  std::string points_dirname;
  points_dirname = "../TestingPaperIVCNZ";
  double focalLength = 1452.6675;
  if (cmd.ContainsKey("m"))
  {
    cmd.GetStringValue("m", points_dirname);
  }
  std::cout << "directory: " << points_dirname << std::endl;
  std::string colmap_db_filename;
  colmap_db_filename = points_dirname + "/database.db";

  if (cmd.ContainsKey("d"))
  {
    cmd.GetStringValue("d", colmap_db_filename);
  }
  std::string pose_filename;
  pose_filename = points_dirname + "/poses_esac_.txt";

  if (cmd.ContainsKey("p"))
  {
    cmd.GetStringValue("p", pose_filename);
  }
  if (cmd.ContainsKey("f"))
  {
    cmd.GetDoubleValue("f", focalLength);
  }

  int image_id = 0;

  if (cmd.ContainsKey("i"))
  {
    cmd.GetIntValue("i", image_id);
  }

  if (pose_filename.empty())
  {
    std::cout << "Pose File Not available" << std::endl;
    return EXIT_FAILURE;
  }

  else if (points_dirname.empty())
  {
    std::cout << "Please Input Directory containing 3D and 2D points" << std::endl;
    return EXIT_FAILURE;
  }

  else if (colmap_db_filename.empty())
  {
    std::cout << "Please Enter Colmap Database Directory" << std::endl;
    return EXIT_FAILURE;
  }

  // read colmap model
  std::unordered_map<int, cv::Point3f> mps_sfm;
  std::unordered_map<int, std::vector<std::pair<int, int> > > obs_sfm;
  std::unordered_map<int, std::vector<int> > corr_sfm;
  std::unordered_map<int, cv::Mat> Rs_sfm;
  std::unordered_map<int, cv::Mat> ts_sfm;
  std::unordered_map<int, float> fs_sfm; // double check that these are the right ones as they come from the sfm model
  std::unordered_map<int, cv::Point2f> cxcy_sfm;

  std::unordered_map<int, cv::Vec3b> mps_colrs;
  std::vector<cv::Vec3b> colorsRGB;
  std::string colmap_model_dirname = points_dirname + "/sparse";
  bool res = Utils::colmapGetReconstruction(colmap_model_dirname, mps_sfm, mps_colrs, obs_sfm, corr_sfm, Rs_sfm, ts_sfm, fs_sfm);

  std::string line;
  // std::cout << "pose_filename" << pose_filename << std::endl;
  std::ofstream out_allResults_file("consolidatedResults.csv", std::ios::out);
  std::ifstream pose_file(pose_filename, std::ios::in); // input file stream used for reading a text file

  out_allResults_file << "imgName"
                      << ","
                      << "Mean"
                      << ", Sum"
                      << ", NumPoints" << std::endl;
  while (std::getline(pose_file, line))
  { // read a string or a line from an input stream.
    // std::cout <<"-----------------------------"<<std::endl;
    double sum = 0;

    std::cout << "pose_file content" << line << std::endl;
    std::stringstream ss_line(line);
    std::string img_filename;
    float qw, qx, qy, qz;
    float tx, ty, tz;
    ss_line >> img_filename >> qw >> qx >> qy >> qz >> tx >> ty >> tz;

    std::cout << "img_filename" << img_filename << std::endl;

    Generate2Dpoints = Utils::get2DPoints(points_dirname, img_filename);

    cv::Mat outputImg = cv::imread(points_dirname + "/rgb/" + img_filename);
    if (!outputImg.empty())
    {
      std::cout << "2D Points List: " << std::endl;
      for (unsigned int i = 0; i < Generate2Dpoints.size(); ++i)
      {
        std::cout << Generate2Dpoints[i] << std::endl;
      }

      Generate3DPoints = Utils::get3DPoints(points_dirname, img_filename);
      std::cout << "3D Points List" << std::endl;
      for (unsigned int i = 0; i < Generate3DPoints.size(); ++i)
      {
        std::cout << Generate3DPoints[i] << Generate3DPoints.size() << std::endl;
      }

      cv::Mat tVec(3, 1, cv::DataType<double>::type); // get Translation vector
      tVec = cv::Vec3d(tx, ty, tz);
      tVec.at<double>(0) = tx;
      tVec.at<double>(1) = ty;
      tVec.at<double>(2) = tz;

      cv::Mat R_mat = Utils::quaternionToRotmatColmap(cv::Vec4f(qw, qx, qy, qz));
      cv::Mat t_mat = cv::Mat(cv::Vec3f(tx, ty, tz));

      // make camera matrix
      Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
      K(0, 0) = fs_sfm[0];
      K(1, 1) = fs_sfm[0];
      K(0, 0) = focalLength; // hard coded for testing -needs to come from current camera model
      K(1, 1) = focalLength;
      K(2, 2) = 1;
      K(0, 2) = outputImg.cols / 2; // just approximation ... needs to come from camera calibration
      K(1, 2) = outputImg.rows / 2;

      // K(0, 0) = 1.7667549822707035e+03;
      // K(1, 0) = 0;
      // K(2, 0) = 0;

      // K(0, 1) = 0;
      // K(1, 1) = 1.7667549822707035e+03;
      // K(2, 1) = 0;

      // K(0, 2) = 9.4594660674506576e+02;
      // K(1, 2) = 5.3438877731363050e+02;
      // K(2, 2) = 1;

      // make pose matrices
      Eigen::Matrix3d R, gtR;
      Eigen::Vector3d t, gtt;
      R = Utils::MatToMatrix3d(R_mat);
      t = Utils::MatToVector3d(t_mat);

      for (int r = 0; r < 3; ++r)
      {
        for (int c = 0; c < 3; ++c)
        {
          // std::cout<< R(r, c) << " ";
        }
        //  std::cout << t(r, 0) << std::endl;
      }
      std::cout << "0 0 0 1" << std::endl;

      cv::Mat renderImg = renderModel(outputImg, K, R, t, mps_sfm);

      Eigen::Quaternionf quat(qw, qx, qy, qz);
      Eigen::Matrix3f rotmat = quat.toRotationMatrix(); // convert Quaternion to Rotation matrix
      cv::Mat rMatx(3, 3, cv::DataType<double>::type);

      rMatx.at<double>(0, 0) = (double)rotmat(0, 0);
      rMatx.at<double>(0, 1) = (double)rotmat(0, 1);
      rMatx.at<double>(0, 2) = (double)rotmat(0, 2);
      rMatx.at<double>(1, 0) = (double)rotmat(1, 0);
      rMatx.at<double>(1, 1) = (double)rotmat(1, 1);
      rMatx.at<double>(1, 2) = (double)rotmat(1, 2);
      rMatx.at<double>(2, 0) = (double)rotmat(2, 0);
      rMatx.at<double>(2, 1) = (double)rotmat(2, 1);
      rMatx.at<double>(2, 2) = (double)rotmat(2, 2);

      cv::Mat distCoeffs(4, 1, cv::DataType<double>::type); // Assign Ditortion values to 0
      distCoeffs.at<double>(0) = 0.0;
      distCoeffs.at<double>(1) = 0.0;
      distCoeffs.at<double>(2) = 0.0;
      distCoeffs.at<double>(3) = 0.0;

      ////// Problem !!!! -> this should be loaded from the intrisicn parameters from the camera images
      // this should use the K matrix and convert from eigen format to opencv
      cv::Mat camMatrix(3, 3, cv::DataType<double>::type); // Create Camera Matrix to Identity
      cv::setIdentity(camMatrix);

      camMatrix.at<double>(0, 0) = focalLength;
      camMatrix.at<double>(1, 0) = 0;
      camMatrix.at<double>(2, 0) = 0;

      camMatrix.at<double>(0, 1) = 0;
      camMatrix.at<double>(1, 1) = focalLength;
      camMatrix.at<double>(2, 1) = 0;

      camMatrix.at<double>(0, 2) = outputImg.cols / 2;
      camMatrix.at<double>(1, 2) = outputImg.rows / 2;
      camMatrix.at<double>(2, 2) = 1;

      std::cout << "rMatx" << rMatx << std::endl;
      std::cout << "tVec" << tVec << std::endl;
      //   std::cout << "camMatrix"<< camMatrix<<std::endl;

      for (const auto &mp_ent : mps_colrs)
      {

        const cv::Vec3b &pt = mp_ent.second;
        colorsRGB.push_back(pt);
      }

      if (mps_sfm.size() > 0)
      { //
        std::vector<cv::Point3f> scene3DPoints;
        std::vector<cv::Point2f> scene2Dpoints;
        for (const auto &mp_ent : mps_sfm)
        {

          const cv::Point3f &pt = mp_ent.second;
          //std::cout << "p3d: " << pt << std::endl;
          scene3DPoints.push_back(pt);
        }

        cv::projectPoints(scene3DPoints, rMatx, tVec, camMatrix, distCoeffs, scene2Dpoints);

        for (unsigned int i = 0; i < scene2Dpoints.size(); ++i)
        {
          // std::cout << "2D Points List" << Generate2Dpoints[i] << std::endl;

          // cv::circle(outputImg, scene2Dpoints[i],1, colorsRGB[i], -1);
        }
      }
      if (Generate3DPoints.size() > 0)
        cv::projectPoints(Generate3DPoints, rMatx, tVec, camMatrix, distCoeffs, projectedPoints);

      std::ofstream out_pose_file(img_filename + ".txt", std::ios::out);
      for (unsigned int r = 0; r < projectedPoints.size(); ++r)
      {

        // std::cout << "Object Points " << Generate3DPoints[r] << "Projected To " << projectedPoints[r] << std::endl;
        //  out_pose_file << "Object Points " << Generate3DPoints[r] << "Projected To " << projectedPoints[r] << std::endl;
        cv::circle(outputImg, projectedPoints[r], 9, cv::Scalar(255, 0, 0), -1); // red
                                                                                 //  std::cout << "Projected Points " <<projectedPoints[r] << std::endl;
      }

      for (unsigned int i = 0; i < Generate2Dpoints.size(); ++i)
      {
        // std::cout << "2D Points List" << Generate2Dpoints[i] << std::endl;

        double dist = distanceCalculate(Generate2Dpoints[i].x, Generate2Dpoints[i].y, projectedPoints[i].x, projectedPoints[i].y);
        sum += dist;
        out_pose_file << "Euclidean distance Btw Generated Points " << Generate2Dpoints[i] << "& Projected Points are-- " << projectedPoints[i] << dist << std::endl; // Resulting Euclidean distance

        // out_pose_file << "Distance Between Vectors " << dist << std::endl;
        cv::circle(outputImg, Generate2Dpoints[i], 10, cv::Scalar(0, 255, 255), -1);
        cv::imwrite(img_filename, outputImg);

        //  std::cout << "Distance Between Vectors " << Generate2Dpoints[i]<<projectedPoints[i] << std::endl;
      }

      // Resulting Euclidean distance

      cv::imshow("Window", outputImg);
      cv::imshow("RenderPoints", renderImg);

      cv::waitKey(1);

      double mean = -1.0;
      if (Generate2Dpoints.size() > 0)
      {
        mean = sum / Generate2Dpoints.size();
        std::cout << "Average Sum" << mean << std::endl;
        out_pose_file << "Calculated Mean =  " << mean << std::endl;
      }
      Generate3DPoints.clear();
      out_allResults_file << img_filename << "," << mean << ", " << sum << ", " << Generate2Dpoints.size() << std::endl;
    }
  }

  return EXIT_SUCCESS;
}
