#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

int main()
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

  // uint8_t r(255), g(15), b(15);
  // for (float z(-1.0); z <= 4.0; z += 0.01)
  // {
  //   for (float angle(0.0); angle <= 360.0; angle += 5.0)
  //   {
  //     pcl::PointXYZ basic_point;
  //     basic_point.x = 0.5 * std::cos(pcl::deg2rad(angle));
  //     basic_point.y = sinf(pcl::deg2rad(angle));
  //     basic_point.z = z;
  //     basic_cloud_ptr->points.push_back(basic_point);

  //     pcl::PointXYZRGB point;
  //     point.x = basic_point.x;
  //     point.y = basic_point.y;
  //     point.z = basic_point.z;
  //     uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
  //                     static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
  //     point.rgb = *reinterpret_cast<float *>(&rgb);
  //     point_cloud_ptr->points.push_back(point);
  //   }
  //   if (z < 0.0)
  //   {
  //     r -= 12;
  //     g += 12;
  //   }
  //   else
  //   {
  //     g -= 12;
  //     b += 12;
  //   }
  // }

  uint8_t r(255), g(15), b(15);
  float i = 0;
  for (float z = -10; z <= 10; z += 0.01)
  {
    i++;
    pcl::PointXYZ basic_point;
    basic_point.x = std::cos(0.1 * i);
    basic_point.y = sinf(0.1 * i);
    basic_point.z = z;
    basic_cloud_ptr->points.push_back(basic_point);

    pcl::PointXYZRGB point;
    point.x = basic_point.x;
    point.y = basic_point.y;
    point.z = basic_point.z;
    uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                    static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
    point.rgb = *reinterpret_cast<float *>(&rgb);
    point_cloud_ptr->points.push_back(point);
    if (z < 0.0)
    {
      r -= 2;
      g += 10;
    }
    else
    {
      g -= 12;
      b += 12;
    }
  }

  basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
  basic_cloud_ptr->height = 1;
  point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
  point_cloud_ptr->height = 1;

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
  viewer->addPointCloud<pcl::PointXYZRGB>(point_cloud_ptr, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();//aa

  float w = 0.1;
  while (!viewer->wasStopped())
  {
    point_cloud_ptr->clear();
    w+=0.01;
    if (w >= 3.141592 * 2)
    {
      w = 0.01;
    }
    uint8_t r(255), g(105), b(150);
    float i = 0;
    for (float z = -5; z <= 5; z += 0.009)
    {
      i++;
      pcl::PointXYZ basic_point;
      basic_point.x = std::cos(0.1 * i + w);
      basic_point.y = sinf(0.1 * i + w);
      basic_point.z = z;
      basic_cloud_ptr->points.push_back(basic_point);

      pcl::PointXYZRGB point;
      point.x = basic_point.x;
      point.y = basic_point.y;
      point.z = basic_point.z;
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                      static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      point.rgb = *reinterpret_cast<float *>(&rgb);
      point_cloud_ptr->points.push_back(point);
     if (z < 0.0)
    {
      r -= 1;
      g += 1;
    }
    else
    {
      g -= 1;
      b += 1;
    }
    }
      
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
    viewer->updatePointCloud<pcl::PointXYZRGB>(point_cloud_ptr, rgb, "sample cloud");
    viewer->spinOnce(100);
    
  }

  return 0;
}
