//#include <iostream>
//
//int main() {
//    std::cout << "Hello, World!" << std::endl;
//    return 0;
//}

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

int main (int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>& cloud = *cloud_ptr;

    // Fill in the cloud data
    //cloud.width    = 5;
    //cloud.height   = 1;
    cloud.is_dense = false;
    //cloud.points.resize (cloud.width * cloud.height);

    for (float x=-0.5f; x<=0.5f; x+=0.01f)
    {
        for (float y=-0.5f; y<=0.5f; y+=0.01f)
        {
            pcl::PointXYZ point;  point.x = x;  point.y = y;  point.z = 2.0f - y;
            cloud.points.push_back (point);
        }
    }

    cloud.width = (int) cloud.points.size ();
    cloud.height = 1;


    pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
    std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

    for (size_t i = 0; i < cloud.points.size (); ++i)
        std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;


    pcl::visualization::CloudViewer viewer("pcd viewer");//直接创造一个显示窗口
    viewer.showCloud (cloud_ptr);//再这个窗口显示点云
    system("pause");
    while (!viewer.wasStopped ())
    {

    }

    return (0);
}