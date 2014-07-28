#pragma once

// PCL
#include "PointCloudRegistrator.h"

// Kinect
#include <NuiApi.h>


class KinectDepth2PointCloudBuilder
{
public:
    KinectDepth2PointCloudBuilder(const NUI_DEPTH_IMAGE_PIXEL *depthPixels, UINT width, UINT height);
    KinectDepth2PointCloudBuilder(const USHORT *depthBuffer, UINT width, UINT height);

    operator PointCloudRegistrator::PointCloudPtr() { return m_pointCloud; }
private:
    PointCloudRegistrator::PointCloudPtr m_pointCloud;
};

