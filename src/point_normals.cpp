#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/extract_indices.h>

int main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/joker/Desktop/point_cloud/3.pcd", *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  pcl::PassThrough<pcl::PointXYZRGB> pass; //取出y轴在0到-10的体素//
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-10, 0.0);
  pass.filter(*cloud);

  pcl::VoxelGrid<pcl::PointXYZRGB> v; //向下取样//
  v.setInputCloud(cloud);
  v.setLeafSize(0.006f, 0.006f, 0.006f);
  v.filter(*cloud);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor; //去掉离群点//
  sor.setInputCloud(cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(0.05);
  sor.filter(*cloud);

  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setSearchSurface(cloud);

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  ne.setSearchMethod(tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

  ne.setRadiusSearch(0.1);
  ne.compute(*cloud_normals);

  pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg; //根据颜色聚类//
  reg.setInputCloud(cloud);
  reg.setSearchMethod(tree);
  reg.setDistanceThreshold(0.005);
  reg.setPointColorThreshold(20);
  reg.setRegionColorThreshold(10);
  reg.setMinClusterSize(150);
  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();

  // pcl::PointIndices::Ptr inliers(new pcl::PointIndices()); //取出索引中的点群//
  // *inliers = clusters[0];
  // pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  // extract.setInputCloud(cloud);
  // extract.setIndices(inliers);
  // extract.setNegative(false);
  // extract.filter(*cloud);

  // std::vector<int > indexs = { 80, 81, 82, 83, 84 };
  // pcl::copyPointCloud(*cloud, indexs, *cloud);

  // for (int i = 0; i < cloud->points.size(); i++)
  // {
  //   cout << "(" << cloud->points[i].x << ", " << cloud->points[i].y << ", " << cloud->points[i].z << ")" << endl;
  // }

  pcl::visualization::PCLVisualizer viewer;
  viewer.addPointCloud(colored_cloud, "cloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);
  // viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, cloud_normals, 10, 0.005, "normals");
  viewer.addCoordinateSystem(0.2);
  viewer.initCameraParameters();
  std::cout << clusters[0];
  while (!viewer.wasStopped())
  {
    viewer.spinOnce(100);
  }
  return (0);
}