
#include "PointCloudRegistrator.h"

int main()
{
    PointCloudRegistrator registrator;
    registrator.addPointCloudFromPLY("cloud_0.ply");
    registrator.addPointCloudFromPLY("cloud_1.ply");
    registrator.addPointCloudFromPLY("cloud_2.ply");
    PointCloudRegistrator::PointCloud pc;
    registrator.getResultCloud(pc);
    pcl::io::savePLYFile("registed_point.ply", pc, true);
    return 0;
}