// ArucoMarkerDetector.cpp

#include "auto_marker_docking/aruco_marker_detector.hpp"

ArucoMarkerDetector::ArucoMarkerDetector(const cv::Mat &cameraMatrix,
                                         const cv::Mat &distCoeffs,
                                         int dictionaryId)
    : cameraMatrix(cameraMatrix.clone()), distCoeffs(distCoeffs.clone()) {
  dictionary = cv::aruco::getPredefinedDictionary(
      cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
}

ArucoMarkerDetector::~ArucoMarkerDetector() {
  // 필요한 경우 여기에 정리 코드를 추가합니다.
}

bool ArucoMarkerDetector::detectMarkers(const cv::Mat &image) {
  // 마커 감지
  cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds);

  // 포즈 추정
  if (!markerCorners.empty()) {
    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.1, cameraMatrix,
                                         distCoeffs, rotationVectors,
                                         translationVectors);
    return true;
  }

  return false;
}

std::vector<cv::Vec3d> ArucoMarkerDetector::getTranslationVectors() const {
  return translationVectors;
}

std::vector<cv::Vec3d> ArucoMarkerDetector::getRotationVectors() const {
  return rotationVectors;
}
