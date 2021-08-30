#include "plane_detection.h"

#include <pcl/ModelCoefficients.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <random>


PlaneDetection::PlaneDetection()
{
}

PlaneDetection::~PlaneDetection()
{
}

void PlaneDetection::init()
{

}

PointCloud::Ptr PlaneDetection::getPlane(const PointCloud::Ptr &full_cloud)
{

    PointCloud::Ptr filtered_cloud(new PointCloud);

    pcl::PassThrough<PointT> passthrough_filter;
    passthrough_filter.setInputCloud(full_cloud);
    passthrough_filter.setFilterFieldName("x");
    double passthrough_x_min = 0.1;
    double passthrough_x_max = 0.5;
    passthrough_filter.setFilterLimits(passthrough_x_min, passthrough_x_max);
    passthrough_filter.filter(*filtered_cloud);

    // write some code here to detect the ground plane
    // See PCL tutorials here: https://pcl.readthedocs.io/projects/tutorials/en/latest/
    // on the following topics:
    //
    // passthrough filter
    // voxel filter
    // Normal estimation
    // Plane model segmentation
    // projecting points using a parametric model
    // constructing a convex hull
    return filtered_cloud;
}


bool PlaneDetection::detectAndViewPlane(const std::string &filename)
{

    PointCloud::Ptr full_cloud(new PointCloud);
    if(pcl::io::loadPCDFile<PointT>(filename, *full_cloud) == -1)
    {
        std::cout << "failed to load" << std::endl;
        return false;
    }


    PointCloud::Ptr plane = getPlane(full_cloud);

    // Write some code here to segment objects on the detected plane
    //
    // Some PCL classes/tutorials you might need:
    // ExtractPolygonPrismData (not in the tutorials)
    // Euclidean cluster extraction

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);

    pcl::visualization::PointCloudColorHandlerRGBField<PointT> colour_handler(full_cloud);
    viewer->addPointCloud<PointT>(full_cloud, colour_handler, "full_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> green(plane, 0, 255, 0);
    viewer->addPointCloud<PointT> (plane, green, "plane");


    viewer->addCoordinateSystem(0.1, "coordinate_system", 0);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return true;
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: ./detect_plane <path to .pcd file>" << std::endl;
        exit(0);
    }
    PlaneDetection pd;
    pd.init();
    pd.detectAndViewPlane(std::string(argv[1]));
    return 0;
}

