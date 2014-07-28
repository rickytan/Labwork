#pragma once

#include <iostream>
#include <string>
#include <limits>

#ifdef max
#undef max
#endif // max

#ifdef min
#undef min
#endif

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>


using namespace std;
using namespace pcl;

class PointCloudRegistrator
{
public:
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

    PointCloudRegistrator();
    ~PointCloudRegistrator();

    void addPointCloud(const PointCloudPtr cloud, const Eigen::Matrix4f &transform = Eigen::Matrix4f::Identity());
    void addPointCloudFromPLY(const string &filename);
    void getResultCloud(PointCloud &cloud);

private:
    PointCloudPtr m_pointCloud;
};

