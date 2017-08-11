/**
 * NOTICE: This software  source code and any of  its derivatives are the
 * confidential  and  proprietary   information  of  Vecna  Technologies,
 * Inc. (such source  and its derivatives are hereinafter  referred to as
 * "Confidential Information"). The  Confidential Information is intended
 * to be  used exclusively by  individuals or entities that  have entered
 * into either  a non-disclosure agreement or license  agreement (or both
 * of  these agreements,  if  applicable) with  Vecna Technologies,  Inc.
 * ("Vecna")   regarding  the  use   of  the   Confidential  Information.
 * Furthermore,  the  Confidential  Information  shall be  used  only  in
 * accordance  with   the  terms   of  such  license   or  non-disclosure
 * agreements.   All  parties using  the  Confidential Information  shall
 * verify that their  intended use of the Confidential  Information is in
 * compliance  with and  not in  violation of  any applicable  license or
 * non-disclosure  agreements.  Unless expressly  authorized by  Vecna in
 * writing, the Confidential Information  shall not be printed, retained,
 * copied, or  otherwise disseminated,  in part or  whole.  Additionally,
 * any party using the Confidential  Information shall be held liable for
 * any and  all damages incurred  by Vecna due  to any disclosure  of the
 * Confidential  Information (including  accidental disclosure).   In the
 * event that  the applicable  non-disclosure or license  agreements with
 * Vecna  have  expired, or  if  none  currently  exists, all  copies  of
 * Confidential Information in your  possession, whether in electronic or
 * printed  form, shall be  destroyed or  returned to  Vecna immediately.
 * Vecna  makes no  representations  or warranties  hereby regarding  the
 * suitability  of  the   Confidential  Information,  either  express  or
 * implied,  including  but not  limited  to  the  implied warranties  of
 * merchantability,    fitness    for    a   particular    purpose,    or
 * non-infringement. Vecna  shall not be liable for  any damages suffered
 * by  licensee as  a result  of  using, modifying  or distributing  this
 * Confidential Information.  Please email [info@vecnatech.com]  with any
 * questions regarding the use of the Confidential Information.
 */

/**
 * FiducialDetectionNode.cpp
 *
 *   Created on:   08/09/2017
 *       Author:   Ashwin Thangali
 */

// C++ standard include files
#include <iostream>
#include <cstring>
#include <cmath>
#include <vector>
#include <list>
#include <ctime>

#include <boost/thread/mutex.hpp>

// Flycapture include files
#include <flycapture/FlyCapture2.h>
// OpenCV include files
#include <opencv2/opencv.hpp>
// Eigen include files
#include <Eigen/Core>

// AprilTags include files
#include <AprilTags/TagDetector.h>
#include <AprilTags/TagDetection.h>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>

// ROS msg headers for fiducial_detector
#include <fiducial_detector/AprilTag.h>
#include <fiducial_detector/AprilTagList.h>

// ROS include files
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#define SOURCE_PGR 0

class PGRCapture {
 public:
  PGRCapture() {
  }
  ~PGRCapture() {
  }

  bool initialize(const int camera_serial_number, const float shutter_speed_millis, const float framerate_fps, const float gain_DB) {

    for (int attempt = 0; attempt < 3; attempt++) {
      m_error = m_busMgr.RescanBus();
      if (m_error != FlyCapture2::PGRERROR_OK) {
        ROS_ERROR("PointGrey Camera: m_busMgr.RescanBus() error, %s.", m_error.GetDescription());
      }
      ros::Time::sleepUntil(ros::Time::now() + ros::Duration(0.2));
    }

    unsigned int numDetectedPtGreyCameras;
    m_error = m_busMgr.GetNumOfCameras(&numDetectedPtGreyCameras);
    if (m_error != FlyCapture2::PGRERROR_OK) {
      ROS_ERROR("PointGrey Camera: Failed to GetNumOfCameras(), %s.", m_error.GetDescription());
      return false;
    }
    if (numDetectedPtGreyCameras == 0) {
      ROS_WARN("PointGrey Camera: numDetectedPtGreyCameras == 0!");
      numDetectedPtGreyCameras=1;
    }

    bool camera_connected = false;
    for (int ptgrey_camera_id = 0; ptgrey_camera_id < numDetectedPtGreyCameras; ptgrey_camera_id++) {
      FlyCapture2::PGRGuid ptgrey_guid;
      FlyCapture2::PGRGuid* ptgrey_guid_ptr = NULL;
      m_error = m_busMgr.GetCameraFromIndex(ptgrey_camera_id, &ptgrey_guid);
      if (m_error != FlyCapture2::PGRERROR_OK) {
        ROS_ERROR("PointGrey Camera: m_busMgr.GetCameraFromIndex failed for camera %d: %s!", ptgrey_camera_id, m_error.GetDescription());
        ptgrey_guid_ptr = NULL;
      } else {
        ptgrey_guid_ptr = &ptgrey_guid;
      }

      // Connect the camera
      m_error = m_camera.Connect(ptgrey_guid_ptr);
      if (m_error != FlyCapture2::PGRERROR_OK) {
        ROS_ERROR("PointGrey Camera: Failed to connect to camera %d: %s!", ptgrey_camera_id, m_error.GetDescription());
        continue;
      }

      // Get the camera info and print it out
      m_error = m_camera.GetCameraInfo(&m_camInfo);
      if (m_error == FlyCapture2::PGRERROR_OK) {
        if (camera_serial_number == 0 || camera_serial_number == m_camInfo.serialNumber) {
          camera_connected = true;
          break;
        } else {
          ROS_INFO("PointGrey Camera: input camera_serial_number != m_camInfo.serialNumber (%u != %u), skipping camera %d.",
                     camera_serial_number, m_camInfo.serialNumber);
        }
      } else {
        ROS_ERROR("PointGrey Camera: Failed to get camera info from camera %d: %s!", ptgrey_camera_id, m_error.GetDescription());
      }

      m_error = m_camera.Disconnect();
      if (m_error != FlyCapture2::PGRERROR_OK) {
        ROS_ERROR("PointGrey Camera: Failed to disconnect from camera %d: %s.", ptgrey_camera_id, m_error.GetDescription());
      }
    }
    if (!camera_connected) {
      ROS_ERROR("PointGrey Camera: No camera was successfully connected among %d detected cameras!", numDetectedPtGreyCameras);
      return false;
    }

    //Declare a Property struct.
    FlyCapture2::Property prop;

    prop.onOff = false;
    prop.autoManualMode = false;
    prop.absControl = true;

    // turn off AUTO_EXPOSURE
    prop.type = FlyCapture2::AUTO_EXPOSURE;
    m_error = m_camera.SetProperty(&prop);
    if (m_error == FlyCapture2::PGRERROR_PROPERTY_FAILED) {
      ROS_ERROR("PointGrey Camera: Failed to set AUTO_EXPOSURE false!");
      return false;
    }

    // turn off BRIGHTNESS
    prop.type = FlyCapture2::BRIGHTNESS;
    m_error = m_camera.SetProperty(&prop);
    if (m_error == FlyCapture2::PGRERROR_PROPERTY_FAILED) {
      ROS_ERROR("PointGrey Camera: Failed to set BRIGHTNESS false!");
      return false;
    }

    // turn off SHARPNESS
    prop.type = FlyCapture2::SHARPNESS;
    m_error = m_camera.SetProperty(&prop);
    if (m_error == FlyCapture2::PGRERROR_PROPERTY_FAILED) {
      ROS_ERROR("PointGrey Camera: Failed to set SHARPNESS false!");
      return false;
    }

    // turn off GAMMA
    prop.type = FlyCapture2::GAMMA;
    prop.onOff = false;
    m_error = m_camera.SetProperty(&prop);
    if (m_error == FlyCapture2::PGRERROR_PROPERTY_FAILED) {
      ROS_ERROR("PointGrey Camera: Failed to set GAMMA false!");
      return false;
    }

    prop.onOff = true;

    //Set the absolute value of shutter in ms.
    prop.type = FlyCapture2::SHUTTER;
    prop.absValue = shutter_speed_millis;
    m_error = m_camera.SetProperty(&prop);
    if (m_error == FlyCapture2::PGRERROR_PROPERTY_FAILED) {
      ROS_ERROR("PointGrey Camera: Failed to set shutter speed!");
      return false;
    } else {
      ROS_INFO("PointGrey Camera id %d: shutter speed set to %.3f", m_camInfo.serialNumber, prop.absValue);
    }

    //Set the absolute value of frame rate in fps.
    prop.type = FlyCapture2::FRAME_RATE;
    prop.absValue = framerate_fps;
    m_error = m_camera.SetProperty(&prop);
    if (m_error == FlyCapture2::PGRERROR_PROPERTY_FAILED) {
      ROS_ERROR("PointGrey Camera: Failed to set shutter speed!");
      return false;
    } else {
      ROS_INFO("PointGrey Camera id %d: frame rate set to %.3f", m_camInfo.serialNumber, prop.absValue);
    }

    //Set the absolute value of gain in DB.
    prop.type = FlyCapture2::GAIN;
    prop.absValue = gain_DB;
    m_error = m_camera.SetProperty(&prop);
    if (m_error == FlyCapture2::PGRERROR_PROPERTY_FAILED) {
      ROS_ERROR("PointGrey Camera: Failed to set shutter speed!");
      return false;
    } else {
      ROS_INFO("PointGrey Camera id %d: gain set to %.3f", m_camInfo.serialNumber, prop.absValue);
    }

    ROS_INFO("Done setting capture properties for PointGrey Camera id: %u", m_camInfo.serialNumber);
    return true;
  }

  bool start() {
    m_error = m_camera.StartCapture();
    if (m_error == FlyCapture2::PGRERROR_ISOCH_BANDWIDTH_EXCEEDED) {
      ROS_ERROR("PointGrey Camera: Bandwidth exceeded!");
      return false;
    } else if (m_error != FlyCapture2::PGRERROR_OK) {
      ROS_ERROR("PointGrey Camera: Failed to start image capture!");
      return false;
    }
    return true;
  }

  bool retrieve(FlyCapture2::Image& rawImage) {
    m_error = m_camera.RetrieveBuffer(&rawImage);
    if (m_error != FlyCapture2::PGRERROR_OK) {
      ROS_DEBUG("PointGrey Camera: Failed to retrieve frame, %s!", m_error.GetDescription());
      return false;
    }
    return true;
  }

  bool stop() {
    m_error = m_camera.StopCapture();
    if (m_error != FlyCapture2::PGRERROR_OK) {
      // This may fail when the camera was removed, so don't show an error message
    }
    m_camera.Disconnect();
    return true;
  }

 private:
  FlyCapture2::Error m_error;
  FlyCapture2::BusManager m_busMgr;
  FlyCapture2::Camera m_camera;
  FlyCapture2::CameraInfo m_camInfo;
};

class AprilTagDetector {
 public:

  AprilTagDetector(ros::NodeHandle (&n))
 : m_nodeHandle(n),
   m_it(m_nodeHandle) {

    m_nodeHandle.param("tag_family", m_tagCode, std::string("36h11"));
    m_nodeHandle.param("tag_size", m_tagSize, double(0.156));
    m_nodeHandle.param("debug", m_debug, false);
    m_nodeHandle.param("undistort", m_undistort, false);
    m_nodeHandle.param("capture_mode", m_captureMode, false);

    setTagFamily(m_tagCode);

    m_nodeHandle.param("camera_name", m_cameraName, std::string("camera"));

    m_nodeHandle.param("caminfo_topic", m_camInfoTopic, m_nodeHandle.getNamespace() + "/cam_info");
    m_camInfoSub = m_nodeHandle.subscribe(m_camInfoTopic, 1, &AprilTagDetector::camInfo_callback, this);

    m_tagSizeSub = m_nodeHandle.subscribe(m_nodeHandle.getNamespace() + "/april_tag_size", 1, &AprilTagDetector::tagSizeCallback,
                                          this);

    m_tagPub = m_nodeHandle.advertise<fiducial_detector::AprilTagList>(m_nodeHandle.getNamespace() + "/april_tags", 100);
    m_imagePub = m_it.advertise(m_nodeHandle.getNamespace() + "/image_raw", 1);
    m_debugImagePub = m_it.advertise(m_nodeHandle.getNamespace() + "/image_with_detections", 1);

    if (m_captureMode) {
      ROS_INFO("Starting the node in capture mode with Pointgrey drivers!");
    } else {
      m_nodeHandle.param("image_topic", m_imageTopic, std::string("/image_raw"));
      m_imageSub = m_it.subscribe(m_imageTopic, 1, &AprilTagDetector::image_callback, this);
      ROS_INFO("Starting the node in image mode!");
    }
  }

  ~AprilTagDetector() {
  }

  bool captureModeSet() {
    return m_captureMode;
  }

  void camInfo_callback(const sensor_msgs::CameraInfoConstPtr& cameraParams) {
    boost::mutex::scoped_lock cameraParamsLock(m_cameraParamsMutex);

    m_cameraMatrix = cv::Mat::eye(3, 3, CV_32FC1);
    for (int row = 0, elem = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++, elem++) {
        m_cameraMatrix.at<float>(row, col) = cameraParams->K.at(elem);
      }
    }
    m_cameraMatrix.at<float>(0, 1) = 0;
    m_cameraMatrix.at<float>(1, 0) = 0;

    m_distCoeff = cv::Mat::zeros(1, cameraParams->D.size(), CV_32FC1);
    for (int elem = 0; elem < (int) cameraParams->D.size(); elem++) {
      m_distCoeff.at<float>(0, elem) = cameraParams->D[elem];
    }

    if (m_undistort) {
      cv::Size undistortedImageSize(1.1f * cameraParams->width, 1.1f * cameraParams->height);
      m_undistortedCameraMatrix = cv::getOptimalNewCameraMatrix(m_cameraMatrix, m_distCoeff,
                                                                cv::Size(cameraParams->width, cameraParams->height), 1,
                                                                undistortedImageSize);
      cv::initUndistortRectifyMap(m_cameraMatrix, m_distCoeff, cv::Mat(), m_undistortedCameraMatrix,
                                  undistortedImageSize, CV_32FC1, m_map1, m_map2);
    } else {
      m_cameraMatrix.copyTo(m_undistortedCameraMatrix);
    }

    char cameraMatrixStr[1000];
    char* cameraMatrixStrPtr = cameraMatrixStr;
    char undistortedCameraMatrixStr[1000];
    char* undistortedCameraMatrixStrPtr = undistortedCameraMatrixStr;
    for (int row = 0, elem = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++, elem++) {
        cameraMatrixStrPtr += sprintf(cameraMatrixStrPtr, "%.4f ", m_cameraMatrix.at<float>(row, col));
        undistortedCameraMatrixStrPtr += sprintf(undistortedCameraMatrixStrPtr, "%.4f ", m_undistortedCameraMatrix.at<float>(row, col));
      }
    }

    std::stringstream ss;
    ss << "w: " << cameraParams->width << ", h: " << cameraParams->height << ", D: [" << m_distCoeff << "], ";
    ROS_INFO("Got new camera params: %s", ss.str().c_str());
    ROS_INFO("Got new camera params: K: [%s]", cameraMatrixStr);
    ROS_INFO("Got new camera params: undistortK: [%s]", undistortedCameraMatrixStr);

    m_cameraParams = boost::make_shared<sensor_msgs::CameraInfo>(*cameraParams);
  }

  void tagSizeCallback(const std_msgs::Float32ConstPtr tagSize) {
    m_tagSize = tagSize->data;
    ROS_INFO("Got new tag size: %.5f", m_tagSize);
  }

  double standardRad(double t) {
    if (t >= 0.) {
      t = std::fmod(t + M_PI, 2 * M_PI) - M_PI;
    } else {
      t = std::fmod(t - M_PI, -2 * M_PI) + M_PI;
    }
    return t;
  }

  void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
    yaw = standardRad(std::atan2((double) wRo(1, 0), (double) wRo(0, 0)));
    double c = std::cos(yaw);
    double s = std::sin(yaw);
    pitch = standardRad(std::atan2((double) -wRo(2, 0), (double) wRo(0, 0) * c + (double) wRo(1, 0) * s));
    roll = standardRad(
        std::atan2((double) wRo(0, 2) * s - (double) wRo(1, 2) * c, (double) -wRo(0, 1) * s + (double) wRo(1, 1) * c));
  }

  void setImage(cv::Mat& image) {
    m_image = image.clone();
  }

  void setTagFamily(std::string code) {
    AprilTags::TagCodes tagFamily(AprilTags::tagCodes36h11);
    if (code == "16h5") {
      tagFamily = AprilTags::tagCodes16h5;
    } else if (code == "25h7") {
      tagFamily = AprilTags::tagCodes25h7;
    } else if (code == "25h9") {
      tagFamily = AprilTags::tagCodes25h9;
    } else if (code == "36h9") {
      tagFamily = AprilTags::tagCodes36h9;
    } else if (code == "36h11") {
      tagFamily = AprilTags::tagCodes36h11;
    } else {
      ROS_ERROR("Invalid tag family specified!");
      exit(EXIT_FAILURE);
    }
    m_detector = boost::make_shared<AprilTags::TagDetector>(tagFamily);
  }

  void processImage() {
    if (m_undistort) {
      boost::mutex::scoped_lock cameraParamsLock(m_cameraParamsMutex);
      if (m_cameraParams.get() != NULL) {
        if (m_cameraParams->width == m_image.cols && m_cameraParams->height == m_image.rows) {
          cv::Mat rectImage;
          cv::remap(m_image, rectImage, m_map1, m_map2, CV_INTER_LINEAR);
          cv::cvtColor(rectImage, m_undistortedImage, CV_BGR2GRAY);
        } else {
          ROS_ERROR("CameraInfo width/height does not match captured image size (%d != %d || %d != %d)", m_cameraParams->width,
                      m_image.cols, m_cameraParams->height, m_image.rows);
          cv::cvtColor(m_image, m_undistortedImage, CV_BGR2GRAY);
        }
      } else {
        ROS_WARN("Not undistorting images as camera parameters are not yet initialized");
        cv::cvtColor(m_image, m_undistortedImage, CV_BGR2GRAY);
      }
    } else {
      cv::cvtColor(m_image, m_undistortedImage, CV_BGR2GRAY);
    }
    m_detections = m_detector->extractTags(m_undistortedImage);
  }

  void publishDetections(ros::Time time) {
    boost::mutex::scoped_lock cameraParamsLock(m_cameraParamsMutex);

    if (m_cameraParams.get() == NULL) {
      ROS_WARN("Not publishing tag detections as camera parameters are not yet initialized, "
          "waiting for cameraParams to be published on '%s'.",
          m_camInfoTopic.c_str());
      return;
    }

    cv::cvtColor(m_undistortedImage, m_imageWithDetections, CV_GRAY2BGR);

    fiducial_detector::AprilTagList tagList;
    tagList.header.frame_id = m_cameraName;
    tagList.header.stamp = time;
    for (std::vector<AprilTags::TagDetection>::iterator it = m_detections.begin(); it != m_detections.end(); ++it) {

      Eigen::Vector3d translation;
      Eigen::Matrix3d rotation;
      (*it).getRelativeTranslationRotation(m_tagSize,  //
                                           m_undistortedCameraMatrix.at<float>(0, 0), m_undistortedCameraMatrix.at<float>(1, 1),  //
                                           m_undistortedCameraMatrix.at<float>(0, 2), m_undistortedCameraMatrix.at<float>(1, 2),  //
                                           translation, rotation);
      Eigen::Matrix3d F;
      F << 1, 0, 0, 0, -1, 0, 0, 0, 1;
      Eigen::Matrix3d fixed_rot = F * rotation;
      double yaw, pitch, roll;
      wRo_to_euler(fixed_rot, yaw, pitch, roll);

      fiducial_detector::AprilTag tag;
      tag.header.frame_id = m_cameraName;
      tag.header.stamp = time;
      tag.id = (*it).id;
      tag.hamming_distance = (*it).hammingDistance;
      tag.distance = translation.norm();
      tag.z = translation(0);  // depth from camera
      tag.x = translation(1);  // horizontal displacement (camera pov right = +ve)
      tag.y = translation(2);  // vertical displacement
      tag.yaw = yaw;
      tag.pitch = pitch;
      tag.roll = roll;
      tagList.april_tags.push_back(tag);

      if (m_debug) {
        // Print the detections to stdout
        if (false) {
          fprintf(stdout, " Id=%d, distance=%8.4fm, x=%8.4f,y=%8.4f,z=%8.4f,yaw=%8.4f,pitch=%8.4f,roll=%8.4f\n", (*it).id,
                  translation.norm(), translation(0), translation(1), translation(2), yaw, pitch, roll);
        }
        // Draw the detections on debug image
        (*it).draw(m_imageWithDetections);
      }
    }
    m_tagPub.publish(tagList);
  }

  void publishImage(ros::Time time) {
    sensor_msgs::ImagePtr msg;
    std_msgs::Header header = std_msgs::Header();
    header.stamp = time;
    header.frame_id = m_cameraName;

    msg = cv_bridge::CvImage(header, "bgr8", m_image).toImageMsg();
    m_imagePub.publish(msg);
  }

  void publishDebugImage(ros::Time time) {
    if (m_debug) {
      sensor_msgs::ImagePtr msg;
      std_msgs::Header header = std_msgs::Header();
      header.stamp = time;
      header.frame_id = m_cameraName;

      msg = cv_bridge::CvImage(header, "bgr8", m_imageWithDetections).toImageMsg();
      m_debugImagePub.publish(msg);
    }
  }

  void image_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr bufPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    m_image = bufPtr->image.clone();

    ros::Time timeNow = ros::Time::now();
    publishImage(timeNow);

    processImage();
    publishDetections(timeNow);
    publishDebugImage(timeNow);
  }

 private:
  boost::shared_ptr<AprilTags::TagDetector> m_detector;

  ros::NodeHandle m_nodeHandle;

  std::string m_camInfoTopic;
  ros::Subscriber m_camInfoSub;

  ros::Subscriber m_tagSizeSub;

  image_transport::ImageTransport m_it;

  std::string m_imageTopic;
  image_transport::Subscriber m_imageSub;

  image_transport::Publisher m_imagePub;
  image_transport::Publisher m_debugImagePub;

  ros::Publisher m_tagPub;

  std::vector<AprilTags::TagDetection> m_detections;

  std::string m_tagCode;
  bool m_debug;
  bool m_undistort;
  double m_tagSize;
  bool m_captureMode;
  std::string m_cameraName;

  boost::mutex m_cameraParamsMutex;
  sensor_msgs::CameraInfoPtr m_cameraParams;
  cv::Mat m_cameraMatrix, m_undistortedCameraMatrix;
  cv::Mat m_distCoeff;
  cv::Mat m_map1, m_map2;

  cv::Mat m_image;
  cv::Mat m_undistortedImage;
  cv::Mat m_imageWithDetections;
};

int main(int argc, char** argv) {

  ros::init(argc, argv, "apriltag_detector");
  ros::NodeHandle n("~");

  AprilTagDetector detector(n);

  std::string camera_serial_number_str = "0";
  float shutter_speed_millis = 4;
  float framerate_fps = 15;
  float gain_DB = 10;

  n.param("camera_serial_number", camera_serial_number_str, camera_serial_number_str);
  n.param("shutter_speed_millis", shutter_speed_millis, shutter_speed_millis);
  n.param("framerate_fps", framerate_fps, framerate_fps);
  n.param("gain_DB", gain_DB, gain_DB);

  int camera_serial_number = 0;
  if (sscanf(camera_serial_number_str.c_str(), "%u", &camera_serial_number) != 1) {
    ROS_ERROR("Cannot parse camera_serial_number from input parameter \"%s\", setting to 0", camera_serial_number_str.c_str());
    camera_serial_number = 0;
  }

  if (detector.captureModeSet()) {
    PGRCapture capture;
    if (capture.initialize(camera_serial_number, shutter_speed_millis, framerate_fps, gain_DB) && capture.start()) {
      // capture loop
      while (ros::ok()) {
        FlyCapture2::Image rawImage;
        if (capture.retrieve(rawImage)) {
          ros::Time imageGrabbedRosTime = ros::Time::now();

          // convert to rgb
          FlyCapture2::Image rgbImage;
          rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage);

          // convert to OpenCV Mat
          unsigned int rowBytes = (double) rgbImage.GetReceivedDataSize() / (double) rgbImage.GetRows();
          cv::Mat image = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(), rowBytes);

          detector.setImage(image);
          detector.publishImage(imageGrabbedRosTime);

          detector.processImage();
          detector.publishDetections(imageGrabbedRosTime);
          detector.publishDebugImage(imageGrabbedRosTime);
        }
        ros::spinOnce();
      }
      capture.stop();
    } else {
      ROS_ERROR("PGRCapture failed to either initialize or start.");
    }
  } else {
    ros::spin();
  }
  return EXIT_SUCCESS;
}
