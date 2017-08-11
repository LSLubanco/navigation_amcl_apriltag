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
 * LandmarkPreprocessor.cpp
 *
 *   Created on:   08/09/2017
 *       Author:   Ashwin Thangali
 */

#include "LandmarkPreprocessor.h"

#include <visualization_msgs/MarkerArray.h>

LandmarkPreprocessor::LandmarkPreprocessor()
    : m_private_nh("~") {

  // Get params from nodehandle
  ROS_INFO("Getting params.");
  m_private_nh.param("landmark_map_file", m_map_filename, m_map_filename);
  m_private_nh.param("first_fiducial_detector_node", m_first_fiducial_detector_node, m_first_fiducial_detector_node);
  m_private_nh.param("second_fiducial_detector_node", m_second_fiducial_detector_node, m_second_fiducial_detector_node);
  m_private_nh.param("landmark_output_topic", m_output_topic, m_output_topic);
  ROS_INFO("Got params.");

  ROS_INFO("Loading map file from (%s)", m_map_filename.c_str());
  loadMapFile(m_map_filename);
  ROS_INFO("Loaded map file.");

  // Set up publishers and subscribers
  ROS_INFO("Setting up pub-sub.");
  if (!m_first_fiducial_detector_node.empty()) {
    m_firstRawLandmarkDetectionsSub = m_nh.subscribe(m_first_fiducial_detector_node + "/april_tags", 1,
                                                     &LandmarkPreprocessor::rawLandmarkReceived, this);
  }
  if (!m_second_fiducial_detector_node.empty()) {
    m_secondRawLandmarkDetectionsSub = m_nh.subscribe(m_second_fiducial_detector_node + "/april_tags", 1,
                                                      &LandmarkPreprocessor::rawLandmarkReceived, this);
  }

  m_transformListener = std::make_shared<tf::TransformListener>();
  m_processedLandmarkDetectionsPub = m_private_nh.advertise<fiducial_detector::LandmarkDetectionArray>(m_output_topic, 1);
  m_processedLandmarksVisualizationPub = m_nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1, true);
  ROS_INFO("Done setting up pub-sub.");
}

LandmarkPreprocessor::~LandmarkPreprocessor() {
  //TODO: Are there any resources here to free?
}

void LandmarkPreprocessor::loadMapFile(std::string filename) {
  try {
    YAML::Node doc = YAML::LoadFile(filename);
    ROS_INFO("Loading landmarks from '%s'", filename.c_str());
    m_world_frame = doc["frame_id"].as<std::string>();
    const YAML::Node& landmarks = doc["landmarks"];
    // Iterate over all landmarks in yaml file
    for (unsigned int i = 0; i < landmarks.size(); i++) {
      // Populate pose object for map
      unsigned int landmark_id = landmarks[i]["id"].as<unsigned int>();

      geometry_msgs::PoseStamped landmarkPose = geometry_msgs::PoseStamped();
      landmarkPose.header.frame_id = m_world_frame;

      const YAML::Node landmarkPoseTranslation = landmarks[i]["Translation"];
      landmarkPose.pose.position.x = landmarkPoseTranslation[0].as<double>();
      landmarkPose.pose.position.y = landmarkPoseTranslation[1].as<double>();
      landmarkPose.pose.position.z = landmarkPoseTranslation[2].as<double>();

      const YAML::Node landmarkPoseRPY = landmarks[i]["RPY"];
      double roll = landmarkPoseRPY[0].as<double>();
      double pitch = landmarkPoseRPY[1].as<double>();
      double yaw = landmarkPoseRPY[2].as<double>();
      landmarkPose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

      ROS_INFO("%5d   %s XYZ: [%14.6f %14.6f %14.6f] RPY: [%14.6f %14.6f %14.6f]", landmark_id, m_world_frame.c_str(),
                 landmarkPose.pose.position.x, landmarkPose.pose.position.y, landmarkPose.pose.position.z, roll, pitch, yaw);

      // Add to member variable map
      m_expectedLandmarkPoses[landmark_id] = landmarkPose;
    }
  } catch (YAML::Exception& e) {
    ROS_ERROR("Yaml loadMapFile file parsing exception: %s", e.what());
  }
}

bool getLatestTransform(std::shared_ptr<tf::TransformListener> transformListener,
                        std::string referenceFrameId, std::string targetFrameId,
                         unsigned int maxFailures, std::shared_ptr<tf::StampedTransform> outputTf) {
  int lookupFailures = 0;
  bool foundTransforms = false;

  while (!ros::isShuttingDown() && lookupFailures < maxFailures) {

    // Get a transform to convert the trajectory to the correct frame.
    try {
      transformListener->lookupTransform(referenceFrameId, targetFrameId, ros::Time(0), *outputTf);
      foundTransforms = true;
      break;
    } catch (tf::ConnectivityException& e) {
      ROS_WARN("Connectivity exception when looking up transform: %s", e.what());
    } catch (tf::ExtrapolationException& e) {
      ROS_WARN("Extrapolation exception when looking up transform: %s", e.what());
    } catch (tf::InvalidArgument& e) {
      ROS_WARN("Invalid argument exception when looking up transform: %s", e.what());
    } catch (tf::LookupException& e) {
      ROS_WARN("Lookup exception when transforming looking up transform: %s", e.what());
    }
    lookupFailures += 1;
  }
  // Prepare output.
  return foundTransforms;
}

tf::Transform poseToTF(geometry_msgs::Pose pose) {
  tf::Transform transform;
  tf::poseMsgToTF(pose, transform);

  return transform;
}

geometry_msgs::Pose tfToPose(tf::Transform transform) {
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(transform, pose);

  return pose;
}

void LandmarkPreprocessor::rawLandmarkReceived(const fiducial_detector::AprilTagList& landmarkMsg) {
  // Instantiate message
  fiducial_detector::LandmarkDetectionArray landmarkArrayMsg;
  visualization_msgs::MarkerArray landmarkMarkerArrayMsg;

  if (!landmarkMsg.april_tags.empty()) {

    landmarkArrayMsg.header.frame_id = "all_landmarks";
    landmarkArrayMsg.header.stamp = landmarkMsg.header.stamp;

    // Iterate over list and populate PoseWithCovariance and Pose for AMCL
    for (std::vector<fiducial_detector::AprilTag>::const_iterator detectedLandmarkIt = landmarkMsg.april_tags.begin();
        detectedLandmarkIt != landmarkMsg.april_tags.end(); detectedLandmarkIt++) {

      const fiducial_detector::AprilTag& detectedLandmark = *detectedLandmarkIt;
      geometry_msgs::PoseWithCovarianceStamped detectedLandmarkPoseCov = LandmarkPreprocessor::processRawLandmark(detectedLandmark);

      char tagIdStr[100];
      snprintf(tagIdStr, 100, "%d", detectedLandmark.id);
      detectedLandmarkPoseCov.header.frame_id = tagIdStr;

      geometry_msgs::Pose detectedLandmarkPose = detectedLandmarkPoseCov.pose.pose;
      tf::Transform toRotate = poseToTF(detectedLandmarkPose);
      toRotate.setRotation(toRotate.getRotation() * tf::createQuaternionFromRPY(0, M_PI / 2, 0));
      detectedLandmarkPose = tfToPose(toRotate);

      char detectedLandmarkFrame[500];
      snprintf(detectedLandmarkFrame, 500, "detected_landmark_%05d", detectedLandmark.id);
      tf::Transform detectedLandmarkTf = poseToTF(detectedLandmarkPose);
      m_tfBroadcaster.sendTransform(
          tf::StampedTransform(detectedLandmarkTf, detectedLandmark.header.stamp, detectedLandmark.header.frame_id,
                               detectedLandmarkFrame));

      const double arrowMarkerDiameter = 0.09;
      const double arrowMarkerLength = 0.4;

      visualization_msgs::Marker landmarkDetectionMarker;
      landmarkDetectionMarker.header.frame_id = detectedLandmarkFrame;
      landmarkDetectionMarker.ns = "detected_landmark";
      landmarkDetectionMarker.id = detectedLandmark.id;
      landmarkDetectionMarker.type = visualization_msgs::Marker::ARROW;
      landmarkDetectionMarker.pose = geometry_msgs::Pose();
      landmarkDetectionMarker.scale.x = arrowMarkerLength;
      landmarkDetectionMarker.scale.y = arrowMarkerDiameter;
      landmarkDetectionMarker.scale.z = arrowMarkerDiameter;
      landmarkDetectionMarker.color.g = 28;
      landmarkDetectionMarker.color.r = 28;
      landmarkDetectionMarker.color.b = 0;
      landmarkDetectionMarker.color.a = 1;
      landmarkDetectionMarker.lifetime = ros::Duration(0.25);
      landmarkMarkerArrayMsg.markers.push_back(landmarkDetectionMarker);

      if (m_expectedLandmarkPoses.find(detectedLandmark.id) != m_expectedLandmarkPoses.end()) {

        geometry_msgs::PoseStamped expectedPose = m_expectedLandmarkPoses[detectedLandmark.id];

        visualization_msgs::Marker landmarkExpectedMarker;
        landmarkExpectedMarker.header.frame_id = m_world_frame;
        landmarkExpectedMarker.ns = "expected_landmark";
        landmarkExpectedMarker.id = detectedLandmark.id;
        landmarkExpectedMarker.type = visualization_msgs::Marker::ARROW;
        landmarkExpectedMarker.pose = expectedPose.pose;
        landmarkExpectedMarker.scale.x = arrowMarkerLength;
        landmarkExpectedMarker.scale.y = arrowMarkerDiameter;
        landmarkExpectedMarker.scale.z = arrowMarkerDiameter;
        landmarkExpectedMarker.color.g = 0;
        landmarkExpectedMarker.color.r = 28;
        landmarkExpectedMarker.color.b = 28;
        landmarkExpectedMarker.color.a = 1;
        landmarkExpectedMarker.lifetime = ros::Duration(0.25);
        landmarkMarkerArrayMsg.markers.push_back(landmarkExpectedMarker);

        geometry_msgs::Pose expectedLandmarkPose = expectedPose.pose;
        char expectedLandmarkFrame[500];
        snprintf(expectedLandmarkFrame, 500, "expected_landmark_%05d", detectedLandmark.id);
        tf::Transform expectedLandmarkTf = poseToTF(expectedLandmarkPose);
        m_tfBroadcaster.sendTransform(
            tf::StampedTransform(expectedLandmarkTf, detectedLandmark.header.stamp, m_world_frame, expectedLandmarkFrame));

        bool robotToFiducialFrameTfFound = false;
        std::shared_ptr<tf::StampedTransform> robotToFiducialFrameTf = std::make_shared<tf::StampedTransform>();
        if (m_robotToFiducialFrameTfs.find(detectedLandmark.header.frame_id) == m_robotToFiducialFrameTfs.end()) {
          robotToFiducialFrameTfFound = getLatestTransform(m_transformListener, "robot", detectedLandmark.header.frame_id,
                                                           10, robotToFiducialFrameTf);
          if (robotToFiducialFrameTfFound) {
            m_robotToFiducialFrameTfs[detectedLandmark.header.frame_id] = robotToFiducialFrameTf;
          }
        } else {
          robotToFiducialFrameTfFound = true;
          robotToFiducialFrameTf = m_robotToFiducialFrameTfs[detectedLandmark.header.frame_id];
        }

        if (robotToFiducialFrameTfFound) {
          tf::StampedTransform robotToDetectedLandmarkTf;
          robotToDetectedLandmarkTf.mult(*robotToFiducialFrameTf, detectedLandmarkTf);

          char detectedRobotFrame[500];
          snprintf(detectedRobotFrame, 500, "robot_landmark_%05d", detectedLandmark.id);
          tf::Transform worldToDetectedRobotFrameTf;
          worldToDetectedRobotFrameTf.mult(expectedLandmarkTf, robotToDetectedLandmarkTf.inverse());
          m_tfBroadcaster.sendTransform(
              tf::StampedTransform(worldToDetectedRobotFrameTf, detectedLandmark.header.stamp, m_world_frame, detectedRobotFrame));
        }

        fiducial_detector::LandmarkDetection landmarkDetectionMsg;
        landmarkDetectionMsg.detected_pose = detectedLandmarkPoseCov;
        landmarkDetectionMsg.expected_pose = expectedPose;
        landmarkArrayMsg.landmarks.push_back(landmarkDetectionMsg);
      }
    }
  }

  if (!landmarkArrayMsg.landmarks.empty()) {
    m_processedLandmarkDetectionsPub.publish(landmarkArrayMsg);
  }
  if (!landmarkMarkerArrayMsg.markers.empty()) {
    m_processedLandmarksVisualizationPub.publish(landmarkMarkerArrayMsg);
  }
}

geometry_msgs::PoseWithCovarianceStamped LandmarkPreprocessor::processRawLandmark(const fiducial_detector::AprilTag& landmarkMsg) {

  geometry_msgs::PoseWithCovarianceStamped landmarkPoseWithUncertainty = geometry_msgs::PoseWithCovarianceStamped();
  landmarkPoseWithUncertainty.header.stamp = landmarkMsg.header.stamp;
  landmarkPoseWithUncertainty.pose.pose.position.x = landmarkMsg.x;
  landmarkPoseWithUncertainty.pose.pose.position.y = landmarkMsg.y;
  landmarkPoseWithUncertainty.pose.pose.position.z = landmarkMsg.z;
  landmarkPoseWithUncertainty.pose.pose.orientation = //
      tf::createQuaternionMsgFromRollPitchYaw(-landmarkMsg.roll, -landmarkMsg.pitch, -landmarkMsg.yaw);

  // TODO: [RLS-5960] Create functions for Covariance
  // landmarkPoseWithUnceratinty.pose.covariance[0] = ????  // X covar
  // landmarkPoseWithUnceratinty.pose.covariance[7] = ????  // Y covar
  // landmarkPoseWithUnceratinty.pose.covariance[35] = ???? // Theta Covar

  return landmarkPoseWithUncertainty;
}
