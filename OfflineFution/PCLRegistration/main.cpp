
#include "PointCloudRegistrator.h"

int main()
{
    PointCloudRegistrator registrator;
    registrator.addPointCloudFromPLY("point_cloud0.ply");
    registrator.addPointCloudFromPLY("point_cloud1.ply");
    registrator.addPointCloudFromPLY("point_cloud2.ply");
    PointCloudRegistrator::PointCloud pc;
    registrator.getResultCloud(pc);
    pcl::io::savePLYFile("registed_point.ply", pc, true);
    return 0;
}