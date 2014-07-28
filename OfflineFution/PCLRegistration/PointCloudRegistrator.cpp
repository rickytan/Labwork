
#include "PointCloudRegistrator.h"

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

PointCloudRegistrator::PointCloudRegistrator()
{
    
}


PointCloudRegistrator::~PointCloudRegistrator()
{
}

void PointCloudRegistrator::addPointCloud(const PointCloudPtr cloud, const Eigen::Matrix4f &transform)
{
    if (m_pointCloud == NULL) {
        m_pointCloud = cloud;
        return;
    }

    IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    icp.setInputCloud(m_pointCloud);
    icp.setInputTarget(cloud);
    PointCloud out;
    icp.align(out, transform);
    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    *m_pointCloud = out;
}

void PointCloudRegistrator::addPointCloudFromPLY(const string &filename)
{
    PointCloud out;
    pcl::io::loadPLYFile(filename, out);
    addPointCloud(PointCloudPtr(&out));
}

void PointCloudRegistrator::getResultCloud(PointCloud &cloud)
{
    cloud = *m_pointCloud;
}
