
#include "PointCloudRegistrator.h"

#include <random>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/bilateral.hpp>
#include <pcl/range_image/range_image.h>

void PointCloudRegistrator::addPointCloud(const PointCloudPtr cloud, const Eigen::Matrix4f &transform)
{
    addPointCloud(*cloud, transform);
}

void PointCloudRegistrator::addPointCloud( const PointCloud &cloud, const Eigen::Matrix4f &transform /*= Eigen::Matrix4f::Identity()*/ )
{
    PointCloudPtr downSampledCloud = downSamplePointCloud(cloud);
    /*
    PointCloud filteredCloud;
    pcl::search::Search<pcl::PointXYZI>::Ptr tree (new pcl::KdTreeFLANN<PointXYZI>);
    pcl::BilateralFilter<PointXYZ> filter;
    filter.setSearchMethod(tree);
    filter.setInputCloud(downSampledCloud);
    filter.setHalfSize(10.);
    filter.setStdDev(10.);
    filter.filter(filteredCloud);
    */

    if (m_pointCloud == NULL) {
        m_pointCloud = downSampledCloud;
        return;
    }
    IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    icp.setInputCloud(m_pointCloud);
    icp.setInputTarget(downSampledCloud);
    icp.setMaximumIterations(20);
    PointCloud out;
    icp.align(out, transform);
    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    PointCloud transformedCloud;
    pcl::transformPointCloud(*m_pointCloud, transformedCloud, icp.getFinalTransformation());
    *m_pointCloud = out + *downSampledCloud;
}

void PointCloudRegistrator::addPointCloudFromPLY(const string &filename)
{
    PointCloud out;
    pcl::io::loadPLYFile(filename, out);
    //PointCloudPtr cloudPtr(out);
    addPointCloud(out);
}

void PointCloudRegistrator::getResultCloud(PointCloud &cloud)
{
    cloud = *m_pointCloud;
}

PointCloudRegistrator::PointCloudPtr PointCloudRegistrator::downSamplePointCloud( const PointCloud &cloud, size_t size /*= 5000*/ )
{
    if (cloud.points.size() < size)
        return PointCloudPtr(new PointCloud(cloud));

    PointCloudPtr out(new PointCloud);
    out->width = size;
    out->height = 1;
    out->is_dense = false;
    out->points.resize(size);
    
    std::mt19937 eng((std::random_device())());
    std::uniform_int_distribution<> dist(0, cloud.points.size());
    for (size_t i=0; i<size; ++i)
    {
        out->points[i] = cloud.points[dist(eng)];
    }
    return out;
}

