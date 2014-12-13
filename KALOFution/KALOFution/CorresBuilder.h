#ifndef _CORRESBUILDER_H_
#define _CORRESBUILDER_H_

#include <vector>
#include <map>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <pcl/point_cloud.h>

template <typename PointType>
class CorresBuilder
{
    typedef typename PointType::ScalarType ScalarType;
    typedef pcl::PointCloud<PointType> CloudType;
    typedef std::pair<int, int> CloudPair;
    typedef std::pair<int, int> PointPair;
    typedef Eigen::Transform<ScalarType, 3, Eigen::Affine> CloudTransform;

public:
    CorresBuilder();
    ~CorresBuilder();

private:
    std::vector<CloudType> m_pointClouds;
    std::vector<CloudTransform, Eigen::aligned_allocator<CloudTransform> > m_initCloudTransform;
};

#endif  // _CORRESBUILDER_H_