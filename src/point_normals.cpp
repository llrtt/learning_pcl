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
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/console/time.h>

void mam(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, float *); //找出每个点群上下边界点坐标//

int main(int argc, char **argv)
{
  pcl::console::TicToc tt;
  tt.tic();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/joker/Desktop/point_cloud/2.pcd", *cloud) == -1)
  {
    std::cout << "Cloud reading failed." << std::endl;
    return (-1);
  }

  pcl::PassThrough<pcl::PointXYZRGB> pass; //取出y轴在0到-10的体素//
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-10, 0.0);
  pass.filter(*cloud);

  pcl::VoxelGrid<pcl::PointXYZRGB> v; //向下取样,减少点数//
  v.setInputCloud(cloud);
  v.setLeafSize(0.006f, 0.006f, 0.007f);
  v.filter(*cloud);

  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor; //去掉离群点//
  sor.setInputCloud(cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(0.05);
  sor.filter(*cloud);

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(0.007); // 7mm
  ec.setMinClusterSize(80);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*cloud, *cloud0);

  pcl::PointIndices::Ptr lower(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud0);

  float min_x = 20;
  float min_y = 20;
  float min_z = 20;
  int object; //得出目标点群的索引号//
  float last_x_max, last_x_min;
  float coordinate[3];
  float a[6];
  for (int i = 0; i < cluster_indices.size(); ++i)
  {
    float most[6] = {20, -10, 20, -10, 20, -10};
    pcl::copyPointCloud(*cloud, cluster_indices[i], *cloud1);
    mam(cloud1, most);
    if (most[0] < -0.12) //将边界点y轴最小值大于-0.12的点群去掉//
    {
      if (fabs(most[2] - min_z) <= 0.001) //z轴最小值近似相等的话比较中心点x坐标绝对值//
      {
        float average_last = (last_x_max + last_x_min) / 2;
        float average_current = (most[4] + most[5]) / 2;
        if (fabs(average_current) < fabs(average_last))
        {
          object = i;
          for (int i = 0, j = 0; i < 3; i++, j += 2)
          {
            coordinate[i] = (most[j] + most[j + 1]) / 2;
          }
        }
      }

      else if (most[2] < min_z)
      {
        object = i;
        min_z = most[2];
        for (int i = 0, j = 0; i < 3; i++, j += 2)
        {
          coordinate[i] = (most[j] + most[j + 1]) / 2;
        }
      }
    }
    last_x_max = most[5];
    last_x_min = most[4];
  }

  std::cout << endl
            << "目标点群中心坐标为:"
            << "(" << coordinate[2] << " ," << coordinate[0] << " ," << coordinate[1] << ")" << endl;
  pcl::copyPointCloud(*cloud, cluster_indices[object], *cloud0); //将目标点群输入cloud0//

  for (int i = 0; i <= 9; i++)
  {
    pcl::PointXYZRGB add_point;
    add_point.x = coordinate[2];
    add_point.y = coordinate[0];
    add_point.z = coordinate[1];
    add_point.r = 0;
    add_point.g = 255;
    add_point.b = 0;
    cloud0->points.push_back(add_point);
  }

  pcl::visualization::PCLVisualizer viewer;
  viewer.addPointCloud(cloud0, "cloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);
  viewer.addCoordinateSystem(0.2);
  viewer.initCameraParameters();
  std::cout << "运行时间为:" << tt.toc() << "ms" << endl;
  while (!viewer.wasStopped())
  {
    viewer.spinOnce(100);
  }
  return (0);
}

void mam(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float *most)
{
  float min_x = 20, max_x = -10;
  float min_y = 20, max_y = -10;
  float min_z = 20, max_z = -10;
  bool isfirst = true;
  for (int i = 0; i < cloud->points.size(); ++i)
  {
    if(isfirst)
    {
      most[0] = most[1] = cloud->points[i].y;
      most[2] = most[3] = cloud->points[i].z;
      most[4] = most[5] = cloud->points[i].x;
      isfirst = false;
      continue;
    }

    if (cloud->points[i].y < most[0])
    {
      most[0] = cloud->points[i].y;
    }
    else if(cloud->points[i].y > most[1])
    {
      most[1] = cloud->points[i].y;
    }

    if (cloud->points[i].z < most[2])
    {
      most[2] = cloud->points[i].z;
    }
    else if(cloud->points[i].z > most[3])
    {
      most[3] = cloud->points[i].z;
    }

    if (cloud->points[i].x < most[4])
    {
      most[4] = cloud->points[i].x;
    }
    else if(cloud->points[i].x > most[5])
    {
        most[5] = cloud->points[i].x;
    }
  }
}