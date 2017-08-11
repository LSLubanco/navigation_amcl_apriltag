#include <fstream>
#include <sstream>
#include <iostream>
#include <stdlib.h>

#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <fiducial_detector/AprilTagList.h>
#include <fiducial_detector/AprilTag.h>

#include <fiducial_detector/LandmarkDetection.h>
#include <fiducial_detector/LandmarkDetectionArray.h>

class LandmarkPreprocessor {

 public:
  /**
   * Constructor.
   */
  LandmarkPreprocessor();

  /**
   * Destructor
   */
  ~LandmarkPreprocessor();

 private:
  /**
   * Loads the map file containing TFs from datum_origin to each fiducial ID
   * @param filename String containing the map file.
   */
  void loadMapFile(std::string filename);

  /**
   * Processes fiducial detection message
   * @param msg The fiducial detection message
   */
  void rawLandmarkReceived(const fiducial_detector::AprilTagList& msg);

  /**
   * Convenience method to extract pose from fiducial detection message
   * @param msg Fiducial detection message
   * @return Detected fiducial pose
   */
  geometry_msgs::PoseWithCovarianceStamped processRawLandmark(const fiducial_detector::AprilTag& msg);

  /**
   * Stores a map of fiducial ID to TF from datum_origin to fiducial ID
   */
  std::map<int, geometry_msgs::PoseStamped> m_expectedLandmarkPoses;

  /**
   * Map filename string
   */
  std::string m_map_filename;

  /**
   * First fiducial detector node name
   */
  std::string m_first_fiducial_detector_node;

  /**
   * Second fiducial detector node name
   */
  std::string m_second_fiducial_detector_node;

  /**
   * World frame name retrieved from map file
   */
  std::string m_world_frame;

  /**
   * Topic on which to output the processed landmark messages
   */
  std::string m_output_topic = "landmarks";

  /**
   * Map to cache TFs between sensor frame and robot frame
   */
  std::map<std::string, std::shared_ptr<tf::StampedTransform>> m_robotToFiducialFrameTfs;

  /**
   * Node handle
   */
  ros::NodeHandle m_nh;

  /**
   * Node handle
   */
  ros::NodeHandle m_private_nh;

  /**
   * Subscriber for messages produced by first fiducial detection node
   */
  ros::Subscriber m_firstRawLandmarkDetectionsSub;

  /**
   * Subscriber for messages produced by second fiducial detection node
   */
  ros::Subscriber m_secondRawLandmarkDetectionsSub;

  /**
   * PUblisher for the processed landmark messages
   */
  ros::Publisher m_processedLandmarkDetectionsPub;

  /**
   * PUblisher for visualizing the processed landmarks
   */
  ros::Publisher m_processedLandmarksVisualizationPub;

  /**
   * TF tree listener
   */
  std::shared_ptr<tf::TransformListener> m_transformListener;

  /**
   * TF tree broadcaster
   */
  tf::TransformBroadcaster m_tfBroadcaster;

};
