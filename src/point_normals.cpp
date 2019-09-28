#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv)

{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr Ammunition_box(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::io::loadPCDFile<pcl::PointXYZRGB> ("/home/joker/Desktop/point_cloud/1.pcd", *Ammunition_box);

  pcl::visualization::PCLVisualizer viewer("Ammunition_box");

  viewer.setCameraPosition(0,0,-3.0,0,-1,0);

  viewer.addPointCloud(Ammunition_box, "Ammunition_box");

  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Ammunition_box");

  viewer.addCoordinateSystem (0.3);

  viewer.initCameraParameters ();

  while(!viewer.wasStopped())

  viewer.spinOnce(100); 

  return 0;

}