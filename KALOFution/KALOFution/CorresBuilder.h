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
    typedef pcl::PointCloud<PointType> CloudType;
    typedef std::pair<int, int> CloudPair;
    typedef std::pair<int, int> PointPair;

public:
    CorresBuilder();
    ~CorresBuilder();

private:
    std::vector<CloudType> m_pointClouds;
    
};

#endif  // _CORRESBUILDER_H_