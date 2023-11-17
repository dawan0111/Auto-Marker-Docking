// ArucoMarkerDetector.hpp

#ifndef ARUCO_MARKER_DETECTOR_HPP
#define ARUCO_MARKER_DETECTOR_HPP

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <vector>

class ArucoMarkerDetector {
public:
  ArucoMarkerDetector(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                      int dictionaryId);
  ~ArucoMarkerDetector();

  bool detectMarkers(const cv::Mat &image);
  std::vector<cv::Vec3d> getTranslationVectors() const;
  std::vector<cv::Vec3d> getRotationVectors() const;

private:
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners;
  std::vector<cv::Vec3d> translationVectors;
  std::vector<cv::Vec3d> rotationVectors;

  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;
  cv::Ptr<cv::aruco::Dictionary> dictionary;
};

#endif // ARUCO_MARKER_DETECTOR_HPP