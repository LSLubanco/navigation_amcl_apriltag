/////////////////////////////////////////////////////////////////////////////////
//
// Desc: AMCL fiducial routine
// Author : Alex Sher,
// Date : 25 Mar 2015
// Author : Ashwin Thangali
// Date : 26 July 2017
//
/////////////////////////////////////////////////////////////////////////////////

extern "C" {
#include "amcl/pf/pf.h"
#include "amcl/pf/pf_pdf.h"
#include "amcl/pf/pf_kdtree.h"
}

#include "amcl/sensors/amcl_fiducial.h"

static const bool g_print_debug = true;

namespace amcl {

AMCLFiducial::AMCLFiducial(double varianceXY, double varianceYaw, double fiducialParticleSupportXYVarianceThreshold,
                           double fiducialParticleSupportYawVarianceThreshold, double numFiducialParticlesSupportFraction)
    : AMCLSensor() {

  this->m_varianceXY = varianceXY;
  this->m_varianceYaw = varianceYaw;
  this->m_numFiducialParticlesSupportFraction = numFiducialParticlesSupportFraction;
  this->m_fiducialParticleSupportXYVarianceThreshold = fiducialParticleSupportXYVarianceThreshold;
  this->m_fiducialParticleSupportYawVarianceThreshold = fiducialParticleSupportYawVarianceThreshold;
}

static double angle_diff(const double a, const double b) {
  const double pi = M_PI;
  const double two_pi = 2 * M_PI;
  double dif = fmod(b - a + pi, 2*pi);
  if (dif < 0) {
    dif += 2*pi;
  }
  return dif - pi;
}

bool AMCLFiducial::UpdatePfWithFiducialDetections(pf_t* pf, AMCLFiducialData* fiducialData) {

  pf_update_resample(pf);

  pf_sample_set_t* pfSampleSet = pf->sets + pf->current_set;
  pf_sample_t* pfSamples = pfSampleSet->samples;
  int numPfSamples = pfSampleSet->sample_count;

  const unsigned int numFiducialDetections = fiducialData->fiducials.size();

  int numFiducialParticlesSupportThreshold = (int) (m_numFiducialParticlesSupportFraction * numPfSamples);

  std::vector<bool> fiducialsWithParticleSupport(numFiducialDetections, false);
  std::vector<int> fiducialsParticleSupportXY(numFiducialDetections, 0);
  std::vector<int> fiducialsParticleSupportYaw(numFiducialDetections, 0);
  std::vector<int> fiducialsParticleSupport(numFiducialDetections, 0);
  int numFiducialsWithParticleSupport = 0;
  for (unsigned int fiducialsi = 0; fiducialsi < numFiducialDetections; fiducialsi++) {

    const geometry_msgs::Pose& detectedRobotPose = fiducialData->fiducials.at(fiducialsi).detected_robot_pose;

    const double fiducial_pose_x = detectedRobotPose.position.x;
    const double fiducial_pose_y = detectedRobotPose.position.y;
    const double fiducial_pose_yaw = tf::getYaw(detectedRobotPose.orientation);

    for (int pfSamplei = 0; pfSamplei < numPfSamples; ++pfSamplei) {

      const double xdiff = fiducial_pose_x - pfSamples[pfSamplei].pose.v[0];
      const double ydiff = fiducial_pose_y - pfSamples[pfSamplei].pose.v[1];
      const double yawdiff = angle_diff(fiducial_pose_yaw, (double)pfSamples[pfSamplei].pose.v[2]);

      double rangeDiff = std::pow(xdiff, 2) + std::pow(ydiff, 2);
      double bearingDiff = std::pow(yawdiff, 2);

      const bool particleXYSupport = rangeDiff < m_fiducialParticleSupportXYVarianceThreshold;
      const bool particleYawSupport = bearingDiff < m_fiducialParticleSupportYawVarianceThreshold;

      if (particleXYSupport && particleYawSupport) {
        fiducialsParticleSupport.at(fiducialsi)++;
      }
      if (particleXYSupport) {
        fiducialsParticleSupportXY.at(fiducialsi)++;
      }
      if (particleYawSupport) {
        fiducialsParticleSupportYaw.at(fiducialsi)++;
      }
    }

    if (fiducialsParticleSupport.at(fiducialsi) > numFiducialParticlesSupportThreshold) {
      fiducialsWithParticleSupport.at(fiducialsi) = true;
      numFiducialsWithParticleSupport++;
    } else {
      fiducialsWithParticleSupport.at(fiducialsi) = false;
    }
  }
  int numFiducialsWithoutParticleSupport = numFiducialDetections - numFiducialsWithParticleSupport;

  char debugStr[1000];
  char *debugStrPtr = debugStr;
  if (g_print_debug) {
    debugStrPtr += sprintf(debugStrPtr, "%s:%d numFiducialDetections %d, numFiducialsWithParticleSupport %d, fiducialsParticleSupport: ",
                           __FILE__, __LINE__, numFiducialDetections, numFiducialsWithParticleSupport);
    for (unsigned int fiducialsi = 0; fiducialsi < numFiducialDetections; fiducialsi++) {
      debugStrPtr += sprintf(debugStrPtr, "(%d, %s, %d %d), ", fiducialsParticleSupport.at(fiducialsi),
                             fiducialsWithParticleSupport.at(fiducialsi) ? "true" : "false",
                                 fiducialsParticleSupportXY.at(fiducialsi), fiducialsParticleSupportYaw.at(fiducialsi));
    }
    fprintf(stdout, "%s\n", debugStr);
  }

  for (unsigned int fiducialsi = 0; fiducialsi < numFiducialDetections; fiducialsi++) {

    if (fiducialsWithParticleSupport.at(fiducialsi)) {

      const geometry_msgs::Pose& detectedRobotPose = fiducialData->fiducials.at(fiducialsi).detected_robot_pose;

      const double fiducial_pose_x = detectedRobotPose.position.x;
      const double fiducial_pose_y = detectedRobotPose.position.y;
      const double fiducial_pose_yaw = tf::getYaw(detectedRobotPose.orientation);

      // iterate over all particles
      for (int pfSamplei = 0; pfSamplei < numPfSamples; ++pfSamplei) {

        // sample poses are in global_frame_id_, i.e., datum_origin
        const double xdiff = fiducial_pose_x - pfSamples[pfSamplei].pose.v[0];
        const double ydiff = fiducial_pose_y - pfSamples[pfSamplei].pose.v[1];
        const double yawdiff = angle_diff(fiducial_pose_yaw, (double)pfSamples[pfSamplei].pose.v[2]);

        double rangeDiff = std::pow(xdiff, 2) + std::pow(ydiff, 2);
        double bearingDiff = std::pow(yawdiff, 2);

        // Compute probability of observing landmark at observed bearing
        // which we assume is Gaussian for now
        double distProb = std::exp(-rangeDiff / (2 * m_varianceXY));
        double bearingProb = std::exp(-bearingDiff / (2 * m_varianceYaw));

        pfSampleSet->samples[pfSamplei].weight *= (distProb * bearingProb);
      }
    }
  }

  double total_weight = 0;
  for (int samplei = 0; samplei < numPfSamples; samplei++) {
    total_weight += pfSampleSet->samples[samplei].weight;
  }
  double normalize_total_weight = 1.f / total_weight;
  for (int samplei = 0; samplei < numPfSamples; samplei++) {
    pfSampleSet->samples[samplei].weight *= normalize_total_weight;
  }

  pf_update_resample(pf);
  //set converged to 0
  pf_init_converged(pf);

  if (numFiducialsWithoutParticleSupport == 0) {

    pf_cluster_stats(pf, pfSampleSet);

  } else {

    pfSampleSet = pf->sets + pf->current_set;
    pfSamples = pfSampleSet->samples;
    numPfSamples = pfSampleSet->sample_count;

    numFiducialParticlesSupportThreshold = (int) (m_numFiducialParticlesSupportFraction * pf->max_samples);

    std::vector<bool> fiducialsWithParticleSupport(numFiducialDetections, false);
    std::vector<int> fiducialsParticleSupport(numFiducialDetections, 0);
    int numFiducialsWithParticleSupport = 0;
    for (unsigned int fiducialsi = 0; fiducialsi < numFiducialDetections; fiducialsi++) {

      const geometry_msgs::Pose& detectedRobotPose = fiducialData->fiducials.at(fiducialsi).detected_robot_pose;

      const double fiducial_pose_x = detectedRobotPose.position.x;
      const double fiducial_pose_y = detectedRobotPose.position.y;
      const double fiducial_pose_yaw = tf::getYaw(detectedRobotPose.orientation);

      for (int pfSamplei = 0; pfSamplei < numPfSamples; ++pfSamplei) {

        const double xdiff = fiducial_pose_x - pfSamples[pfSamplei].pose.v[0];
        const double ydiff = fiducial_pose_y - pfSamples[pfSamplei].pose.v[1];
        const double yawdiff = angle_diff(fiducial_pose_yaw, (double)pfSamples[pfSamplei].pose.v[2]);

        double rangeDiff = std::pow(xdiff, 2) + std::pow(ydiff, 2);
        double bearingDiff = std::pow(yawdiff, 2);

        const bool particleXYSupport = rangeDiff < m_fiducialParticleSupportXYVarianceThreshold;
        const bool particleYawSupport = bearingDiff < m_fiducialParticleSupportYawVarianceThreshold;

        if (particleXYSupport && particleYawSupport) {
          fiducialsParticleSupport.at(fiducialsi)++;
        }
      }

      if (fiducialsParticleSupport.at(fiducialsi) > numFiducialParticlesSupportThreshold) {
        fiducialsWithParticleSupport.at(fiducialsi) = true;
        numFiducialsWithParticleSupport++;
      } else {
        fiducialsWithParticleSupport.at(fiducialsi) = false;
      }
    }

    pf_sample_set_t* pfOutputSampleSet = pf->sets + (pf->current_set + 1) % 2;
    pf_sample_t* pfOutputSamples = pfOutputSampleSet->samples;

    int numSamplesAllFiducials = 0;
    std::vector<int> numNewSamplesPerFiducial(numFiducialDetections, 0);
    for (unsigned int fiducialsi = 0; fiducialsi < numFiducialDetections; fiducialsi++) {
      if (!fiducialsWithParticleSupport.at(fiducialsi)) {
        numNewSamplesPerFiducial.at(fiducialsi) = numFiducialParticlesSupportThreshold - fiducialsParticleSupport.at(fiducialsi);
        numSamplesAllFiducials += numNewSamplesPerFiducial.at(fiducialsi);
      }
    }

    if (g_print_debug) {
      debugStrPtr = debugStr;
      debugStrPtr += sprintf(debugStrPtr, "%s:%d numFiducialsWithParticleSupport %d, numSamplesAllFiducials %d, fiducialsParticleSupport:  ",
                             __FILE__, __LINE__, numFiducialsWithParticleSupport, numSamplesAllFiducials);
      for (unsigned int fiducialsi = 0; fiducialsi < numFiducialDetections; fiducialsi++) {
        debugStrPtr += sprintf(debugStrPtr, "(%d, %s, %d), ", fiducialsParticleSupport.at(fiducialsi),
                               fiducialsWithParticleSupport.at(fiducialsi) ? "true" : "false", numNewSamplesPerFiducial.at(fiducialsi));
      }
      fprintf(stdout, "%s\n", debugStr);
    }

    pfOutputSampleSet->sample_count = std::min(numPfSamples + numSamplesAllFiducials, pf->max_samples);
    int numKeepPfSamples = pfOutputSampleSet->sample_count - numSamplesAllFiducials;
    double keeppfSampleFraction = (double) numKeepPfSamples / numPfSamples;
    int outputSamplei = 0;
    for (int pfSamplei = 0; pfSamplei < numPfSamples && outputSamplei < pf->max_samples; ++pfSamplei) {
      if (numKeepPfSamples == numPfSamples || drand48() <= keeppfSampleFraction) {
        pfOutputSamples[outputSamplei++] = pfSamples[pfSamplei];
      }
    }

    for (unsigned int fiducialsi = 0; fiducialsi < numFiducialsWithoutParticleSupport; fiducialsi++) {
      if (!fiducialsWithParticleSupport.at(fiducialsi)) {

        const geometry_msgs::Pose& detectedRobotPose = fiducialData->fiducials.at(fiducialsi).detected_robot_pose;

        pf_vector_t fiducial_pose_mean = pf_vector_zero();
        fiducial_pose_mean.v[0] = detectedRobotPose.position.x;
        fiducial_pose_mean.v[1] = detectedRobotPose.position.y;
        fiducial_pose_mean.v[2] = tf::getYaw(detectedRobotPose.orientation);

        pf_matrix_t fiducial_pose_cov = pf_matrix_zero();
        fiducial_pose_cov.m[0][0] = m_varianceXY;
        fiducial_pose_cov.m[1][1] = m_varianceXY;
        fiducial_pose_cov.m[2][2] = m_varianceYaw;

        pf_pdf_gaussian_t* fiducialSamplePdf = pf_pdf_gaussian_alloc(fiducial_pose_mean, fiducial_pose_cov);
        for (int fiducialSamplei = 0; fiducialSamplei < numNewSamplesPerFiducial.at(fiducialsi) && outputSamplei < pf->max_samples;
            ++fiducialSamplei) {
          pfOutputSamples[outputSamplei++].pose = pf_pdf_gaussian_sample(fiducialSamplePdf);
        }
        pf_pdf_gaussian_free(fiducialSamplePdf);
      }
    }

    assert(outputSamplei <= pf->max_samples);
    pfOutputSampleSet->sample_count = outputSamplei;

    pf_kdtree_clear(pfOutputSampleSet->kdtree);
    double outputSampleWeight = 1.f / pfOutputSampleSet->sample_count;
    for (outputSamplei = 0; outputSamplei < pfOutputSampleSet->sample_count; ++outputSamplei) {
      pfOutputSamples[outputSamplei].weight = outputSampleWeight;

      pf_kdtree_insert(pfOutputSampleSet->kdtree, pfOutputSamples[outputSamplei].pose, pfOutputSamples[outputSamplei].weight);
    }

    pf->w_slow = pf->w_fast = 0.0;

    // Re-compute cluster statistics
    pf_cluster_stats(pf, pfOutputSampleSet);

    pf->current_set = (pf->current_set + 1) % 2;
  }

  //set converged to 0
  pf_init_converged(pf);

  return true;
}

} // end namespace amcl
