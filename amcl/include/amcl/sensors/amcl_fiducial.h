#include "amcl_sensor.h"
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <tf/tf.h>

namespace amcl {

struct FiducialPoint {

  geometry_msgs::Pose detected_robot_pose;     // pose of robot in world frame as detected by camera
  geometry_msgs::Pose detected_landmark_pose;  // pose of marker in camera frame as known by measurement

  int id;
  double distance_uncertainty;
  double bearing_uncertainty;
};

class AMCLFiducialData : public AMCLSensorData {

 public:
  AMCLFiducialData() {
  }
  virtual ~AMCLFiducialData() {
  }

  // Array of poses/structs that contain individual fiducial information
  std::vector<FiducialPoint> fiducials;
};

class AMCLFiducial : public AMCLSensor {

 public:
  AMCLFiducial(double varianceXY, double varianceYaw, double fiducialParticleSupportXYVarianceThreshold,
               double fiducialParticleSupportYawVarianceThreshold, double numFiducialParticlesSupportFraction);

  bool UpdatePfWithFiducialDetections(pf_t* pf_, AMCLFiducialData* fiducialData);

 private:
  // Variance of Gaussian Model for distance detection from landmark
  double m_varianceXY;
  // Variance of Gaussian Model for bearing detection from landmark
  double m_varianceYaw;
  //
  double m_fiducialParticleSupportXYVarianceThreshold;
  //
  double m_fiducialParticleSupportYawVarianceThreshold;
  //
  double m_numFiducialParticlesSupportFraction;
};

}
