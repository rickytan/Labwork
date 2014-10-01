#include <string>
#include <iostream>
#include <random>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/registration_visualizer.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

struct VisulizationCallback
{
    void operator () (const PointCloud &cloud_src,
    const std::vector<int> &indices_src,
    const PointCloud &cloud_tgt,
    const std::vector<int> &indices_tgt) {

    }

    void operator () (pcl::visualization::PCLVisualizer &viewer) {

    }
};

void visualizationThread(pcl::visualization::PCLVisualizer &viewer)
{

}

void visualizationCallback(
    const PointCloud &cloud_src,
    const std::vector<int> &indices_src,
    const PointCloud &cloud_tgt,
    const std::vector<int> &indices_tgt) {

}

void generatePointCloud(PointCloud &cloud_out, size_t size)
{
    cloud_out.width = size;
    cloud_out.height = 1;
    cloud_out.is_dense = true;
    cloud_out.points.resize(size);

    srand(time(NULL));
    for (size_t i = 0; i < size; i++)
    {
        cloud_out.points[i] = Point(
            0.001f * (rand() % 2000 - 1000),
            0.001f * (rand() % 2000 - 1000),
            0.001f * (rand() % 2000 - 1000)
            );
    }
}

PointCloudPtr transformPointCloud(const PointCloud &cloud, const Eigen::Matrix4f &transform)
{
    PointCloud out;
    pcl::transformPointCloud(cloud, out, transform);
    return out.makeShared();
}

PointCloud::Ptr downSamplePointCloud(const PointCloud &cloud, size_t size = 5000)
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
    for (size_t i = 0; i < size; ++i)
    {
        out->points[i] = cloud.points[dist(eng)];
    }
    return out;
}

int main()
{
    PointCloud gen;
    generatePointCloud(gen, 4000);
    //Eigen::AngleAxisf
    Eigen::Quaternionf rot(Eigen::AngleAxisf(5.f * M_PI / 180.f, Eigen::Vector3f::UnitY()));
    Eigen::Matrix4f tranform;
    tranform = rot;// (rot, Eigen::Vector3f::Zero());
    PointCloudPtr tran = transformPointCloud(gen, rot.matrix());

    return 0;
    PointCloud cloud_source;
    PointCloud cloud_target;

    pcl::io::loadPLYFile("point_cloud0.ply", cloud_source);
    pcl::io::loadPLYFile("point_cloud1.ply", cloud_target);

    
    //pcl::visualization::CloudViewer viewer("Viewer");
    //viewer.showCloud(cloud_source);
    //viewer.showCloud(cloud_target);
    //viewer.runOnVisualizationThread(visualizationThread);

    pcl::PointCloud<pcl::PointXYZ> inputCloudFiltered;
    pcl::PointCloud<pcl::PointXYZ> targetCloudFiltered;

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    //  sor.setLeafSize (0.01, 0.01, 0.01);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    //  sor.setLeafSize (0.05, 0.05, 0.05);
    //  sor.setLeafSize (0.1, 0.1, 0.1);
    //  sor.setLeafSize (0.4, 0.4, 0.4);
    //  sor.setLeafSize (0.5, 0.5, 0.5);

    sor.setInputCloud(cloud_source.makeShared());
    std::cout << "\n inputCloud.size()=" << cloud_source.size() << std::endl;
    sor.filter(inputCloudFiltered);
    std::cout << "\n inputCloudFiltered.size()=" << inputCloudFiltered.size() << std::endl;

    sor.setInputCloud(cloud_target.makeShared());
    std::cout << "\n targetCloud.size()=" << cloud_target.size() << std::endl;
    sor.filter(targetCloudFiltered);
    std::cout << "\n targetCloudFiltered.size()=" << targetCloudFiltered.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::ConstPtr source = inputCloudFiltered.makeShared();
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr target = targetCloudFiltered.makeShared();

    pcl::PointCloud<pcl::PointXYZ> source_aligned;

    //pcl::RegistrationVisualizer<pcl::PointXYZ, pcl::PointXYZ> registrationVisualizer;

    
    //registrationVisualizer.setMaximumDisplayedCorrespondences(100);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    icp.setMaximumIterations(10000);
    icp.setMaxCorrespondenceDistance(0.8);
    icp.setRANSACOutlierRejectionThreshold(0.6);
    icp.setInputCloud(source);
    icp.setInputTarget(target);

    // Register the registration algorithm to the RegistrationVisualizer
    //registrationVisualizer.setRegistration(icp);
    //registrationVisualizer.startDisplay();
    // Start registration process
    icp.align(source_aligned);

    std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    return 0;
}