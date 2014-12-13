#ifndef OFFLINEFUTION_HELPER_H_
#define OFFLINEFUTION_HELPER_H_

#include "stdafx.h"
#include "Helper.h"

#include <iostream>
#include <limits>
#include <cmath>

#include <ppl.h>

#include <boost/math/special_functions.hpp>

#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>

using namespace boost::math;

namespace Helper {
    void computePointNormal(PCloud& cloud)
    {
        /*
        pcl::NormalEstimation<pcl::PointXYZINormal, pcl::PointXYZINormal> ne;
        ne.setInputCloud(cloud.makeShared());
        pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZINormal>());
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(0.03);
        ne.compute(cloud);
        */
    }
};

Eigen::Matrix4f Helper::convertToEigenMatrix(const Matrix4 &mat)
{
    Eigen::Matrix4f M;
    M << mat.M11, mat.M12, mat.M13, mat.M14,
         mat.M21, mat.M22, mat.M23, mat.M24,
         mat.M31, mat.M32, mat.M33, mat.M34,
         mat.M41, mat.M42, mat.M43, mat.M44;
    return M.transpose();
}

Matrix4 Helper::convertFromEigenMatrix(const Eigen::Matrix4f &M)
{
    Matrix4 mat = *(Matrix4 *)M.data();
    return mat;
}

void Helper::depthFloatToPointCloud(::NUI_FUSION_IMAGE_FRAME * pDepthFloatImage, PCloud &outCloud)
{
    
    INuiFrameTexture *imageFrameTexture = pDepthFloatImage->pFrameTexture;
    NUI_LOCKED_RECT LockedRect;

    imageFrameTexture->LockRect(0, &LockedRect, nullptr, 0);

    if (LockedRect.Pitch) {

        assert(NUI_FUSION_IMAGE_TYPE_FLOAT == pDepthFloatImage->imageType);

        outCloud.width = pDepthFloatImage->width * pDepthFloatImage->height;
        outCloud.height = 1;
        outCloud.is_dense = true;
        outCloud.points.reserve(outCloud.width * outCloud.height);

        const float *pFloatBuffer = reinterpret_cast<float *>(LockedRect.pBits);
        const float cx = 319.5f;// pDepthFloatImage->pCameraParameters->principalPointX;
        const float cy = 239.5f;// pDepthFloatImage->pCameraParameters->principalPointY;
        const float fx = 1.0 / 571.4;// pDepthFloatImage->pCameraParameters->focalLengthX;
        const float fy = 1.0 / 571.4;// pDepthFloatImage->pCameraParameters->focalLengthY;

        //Concurrency::parallel_for(0u, pDepthFloatImage->height, [&](unsigned int y)
        for (unsigned int y = 0; y < pDepthFloatImage->height; ++y)
        {
            const float* pFloatRow = reinterpret_cast<const float*>(reinterpret_cast<const unsigned char*>(pFloatBuffer)+(y * LockedRect.Pitch));

            for (unsigned int x = 0; x < pDepthFloatImage->width; ++x)
            {
                float depth = pFloatRow[x];
                
                if (depth < 1e-6 || depth > 10.) {
                    continue;
                }
                float vx = (1.f * x - cx) * fx * depth;
                float vy = (1.f * y - cy) * fy * depth;
                float vz = depth;

                outCloud.push_back(PCloud::PointType(vx, vy, vz));
            }
        }
        //);
    }
    imageFrameTexture->UnlockRect(0);

    outCloud.width = outCloud.points.size();
}

void Helper::savePointCloudTo(const std::string &path, PCloud &cloud, const Eigen::Matrix4f &transform)
{
    Eigen::Affine3f affine(transform);
    //affine.translation() = -affine.translation();
    //pcl::transformPointCloud(cloud, cloud, affine);
    affine = affine.inverse();
    Concurrency::parallel_for(0u, cloud.points.size(), [&](unsigned int i) {
    //    for (size_t i = 0; i < cloud.points.size(); ++i)
            cloud.points[i].getVector3fMap() = affine * cloud.points[i].getVector3fMap();
    });

    pcl::io::savePLYFile(path, cloud, true);
}

void Helper::saveSequenceTo(const std::string &dir, PCloud &cloud,const Eigen::Matrix4f &transform)
{
    if (cloud.points.size() == 0)
        return;

    static int counter = 0;
    char filename[128] = { 0 };
    sprintf(filename, "/cloud_%d.ply", counter++);
    savePointCloudTo(dir + filename, cloud, transform);
}

#endif