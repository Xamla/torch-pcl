#include <pcl/registration/correspondence_estimation.h>

/*
determineCorrespondences (pcl::Correspondences &correspondences,
  243                                   double max_distance = std::numeric_limits<double>::max ()) = 0;

pcl::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
est.setInputSource (source);
est.setInputTarget (target);
pcl::Correspondences all_correspondences;
// Determine all reciprocal correspondences
est.determineReciprocalCorrespondences (all_correspondences);
*/