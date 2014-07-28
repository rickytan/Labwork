#include <windows.h>

#include "KinectDepth2PointCloudBuilder.h"


KinectDepth2PointCloudBuilder::KinectDepth2PointCloudBuilder(const NUI_DEPTH_IMAGE_PIXEL *depthPixels, UINT width, UINT height)
: m_pointCloud(new PointCloudRegistrator::PointCloud)
{
    int imageWidth = width;
    int imageHeight = height;
    int imageHalfWidth = imageWidth / 2;
    int imageHalfHeight = imageHeight / 2;
    float pixelToMeterScale = tanf(NUI_CAMERA_DEPTH_NOMINAL_HORIZONTAL_FOV * M_PI / 180.f * 0.5f) / (width * .5f);

    m_pointCloud->width = width * height;
    m_pointCloud->height = 1;
    m_pointCloud->is_dense = true;
    m_pointCloud->points.reserve(width * height);
    for (size_t i = 0; i < m_pointCloud->points.size(); ++i)
    {
        int w = i % width;
        int h = i / height;
        float depth = 0.001f * (depthPixels + i)->depth;
        if (depth == 0.f)
            continue;
        float x = 1.0f * (w - imageHalfWidth) * pixelToMeterScale * depth;
        float y = -1.0f * (h - imageHalfHeight) * pixelToMeterScale * depth;
        m_pointCloud->points.push_back(PointXYZ(x, y, depth));
    }
}

KinectDepth2PointCloudBuilder::KinectDepth2PointCloudBuilder(const USHORT *depthBuffer, UINT width, UINT height)
{
    int imageWidth = width;
    int imageHeight = height;
    int imageHalfWidth = imageWidth / 2;
    int imageHalfHeight = imageHeight / 2;
    float pixelToMeterScale = tanf(NUI_CAMERA_DEPTH_NOMINAL_HORIZONTAL_FOV * M_PI / 180.f * 0.5f) / (width * .5f);

    m_pointCloud->width = width * height;
    m_pointCloud->height = 1;
    m_pointCloud->is_dense = true;
    m_pointCloud->points.reserve(width * height);
    for (size_t i = 0; i < m_pointCloud->points.size(); ++i)
    {
        int w = i % width;
        int h = i / height;
        float depth = 0.001f * NuiDepthPixelToDepth(*(depthBuffer + i));
        if (depth == 0.f)
            continue;
        float x = 1.0f * (w - imageHalfWidth) * pixelToMeterScale * depth;
        float y = -1.0f * (h - imageHalfHeight) * pixelToMeterScale * depth;
        m_pointCloud->points.push_back(PointXYZ(x, y, depth));
    }
}

