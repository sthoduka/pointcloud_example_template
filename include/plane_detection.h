#ifndef PLANE_DETECTION_H
#define PLANE_DETECTION_H

#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/geometry/planar_polygon.h>
#include <pcl/filters/extract_indices.h>
#include <string>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::Normal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudN;

class PlaneDetection
{
private:

    PointCloud::Ptr getPlane(const PointCloud::Ptr &full_cloud);


public:
    PlaneDetection();
    virtual ~PlaneDetection();
    void init();
    bool detectAndViewPlane(const std::string &filename);
};

#endif /* PLANE_DETECTION_H */

