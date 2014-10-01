#pragma once
#include <Windows.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <NuiApi.h>
#include <NuiKinectFusionApi.h>

#include <limits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


typedef pcl::PointCloud<pcl::PointXYZINormal> PCloud;
typedef PCloud::Ptr PCloudPtr;
typedef PCloud::ConstPtr PCloudConstPtr;

namespace Helper
{
    Eigen::Matrix4f convertToEigenMatrix(const Matrix4 &mat);
    Matrix4 convertFromEigenMatrix(const Eigen::Matrix4f &M);


    void depthFloatToPointCloud(NUI_FUSION_IMAGE_FRAME * pDepthFloatImage, PCloud &cloud);

    void savePointCloudTo(const std::string &path, PCloud &cloud, const Eigen::Matrix4f &transform = Eigen::Matrix4f::Identity());
    void saveSequenceTo(const std::string &dir, PCloud &cloud, const Eigen::Matrix4f &transform = Eigen::Matrix4f::Identity());
};

