
/*
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
 */



//#include <boost/thread.hpp>
//#include <pcl/point_types.h>
//#include <pcl/visualization/cloud_viewer.h>
//
//
////设置键盘交互函数,按下`space`键，某事发生
//void keyboardEvent(const pcl::visualization::KeyboardEvent &event,void *nothing)
//{
//    if(event.getKeySym() == "space" && event.keyDown())
//        cout<<"space键按下"<<endl;//next_iteration = true;
//}
//
//
//
//boost::mutex updateModelMutex;
//
//boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
//{
//    // --------------------------------------------
//    // -----Open 3D viewer and add point cloud-----
//    // --------------------------------------------
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//
//    viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, -1);//设置坐标原点
//
//    viewer->setBackgroundColor(0,1,0);//设置背景色
//
//    viewer->addCoordinateSystem(100); //建立空间直角坐标系
//    // 添加直线
//    pcl::PointXYZ Origin,X, Y, Z;
//    Origin.x = Origin.y = Origin.z = 0;
//    X.x = -100,X.y = X.z = 0;
//    Y.y = -100,Y.x = Y.z = 0;
//    Z.z = -100,Z.x = Z.y = 0;
//    viewer->addLine<pcl::PointXYZ>(Origin,X,255,0,0,"lineX");//红色线段,线的名字叫做"lineX"
//    viewer->addArrow<pcl::PointXYZ> (Origin,X,255,0,0,"arrowX");  //带箭头
//
//    viewer->addLine<pcl::PointXYZ>(Origin,Y,0,255,0,"lineY");//红色线段,线的名字叫做"lineY"
//    //viewer->addArrow<pcl::PointXYZ> (Origin,Y,0,255,0,"arowY");  //带箭头
//
//    viewer->addLine<pcl::PointXYZ>(Origin,Z,0,0,255,"lineZ");//红色线段,线的名字叫做"lineZ"
//   // viewer->addArrow<pcl::PointXYZ> (Origin,Z,0,0,255,"arrowZ");  //带箭头
//
//
//
//
//    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
//
//    //添加点云后，通过点云ID来设置显示大小
//    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
//
//    viewer->initCameraParameters ();//初始化相机参数
//
//    //viewer->registerKeyboardCallback(&keyboardEvent,(void*)NULL);  //设置键盘回吊函数
//
//    return (viewer);
//}
//
//void viewerRunner(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
//{
//    int i=0;
//
//
//    while (!viewer->wasStopped ())
//    {
//        viewer->spinOnce (100);
//
//        boost::mutex::scoped_lock updateLock(updateModelMutex);
//
//        //printf("第%d次循环\r\n",i++);
//
//        viewer->updatePointCloud<pcl::PointXYZ>(cloud_ptr,"sample cloud");
//
//        updateLock.unlock();
//
//        boost::this_thread::sleep (boost::posix_time::microseconds (1000));
//
//    }
//
//}
//
//int main()
//{
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ> &pcloud1 = *cloud_ptr;
//    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//
//    pcloud1.points.push_back( pcl::PointXYZ(10, 10, 80) );
//    pcloud1.width = cloud_ptr->size();
//    pcloud1.height = 1;
//    pcloud1.is_dense = true;
//
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = simpleVis(cloud_ptr);
//    boost::thread vthread(boost::bind(&viewerRunner,viewer,cloud_ptr));
//
//
//    while(1)//循环抓取深度数据
//    {
//        boost::mutex::scoped_lock updateLock(updateModelMutex);
//
//        pcloud1.clear();
//        for ( int _row = 0; _row < 30/*disp.rows*/; _row++ )
//        {
//            for ( int _col = 0; _col <50/* disp.cols*/; _col++ )
//            {
//                //X+Y+Z+10=0
//                float x, y, z;
//                x = _row;
//                y = _col;
//                z = 0-10-x-y;
//
//                pcl::PointXYZ ptemp(x, y, z);
//                pcloud1.points.push_back( ptemp );
//            }
//        }
//        pcloud1.width = cloud_ptr->size();
//        pcloud1.height = 1;
//        pcloud1.is_dense = true;
//
//
//        //viewer->updatePointCloud<pcl::PointXYZ>(cloud_ptr,"sample cloud");
//        updateLock.unlock();
//
//        boost::this_thread::sleep (boost::posix_time::microseconds (1000));
//    }
//    vthread.join();
//
//    return 0;
//}
//


#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //viewer->setCameraPose(2,-1,-3,0,3.14,3.14,0,0,0,0);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "Cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Cloud");
    viewer->resetCameraViewpoint("Cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
}

boost::mutex updateModelMutex;
void viewerRunner(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    while (!viewer->wasStopped ())
    {
        boost::mutex::scoped_lock updateLock(updateModelMutex);
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        updateLock.unlock();

    }
}


void createCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &point_cloud_ptr, float zmin,float zmax,float anglemin,float anglemax)
{

    // ------------------------------------
    // -----Create example point cloud-----
    // ------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

    std::cout << "Genarating example point clouds.\n\n";
    // We're going to make an ellipse extruded along the z-axis. The colour for
    // the XYZRGB cloud will gradually go from red to green to blue.
    uint8_t r(255), g(15), b(15);
    for (float z=zmin; z <= zmax; z += 0.05)
    {
        for (float angle=anglemin; angle <= anglemax; angle += 5.0)
        {
            pcl::PointXYZ basic_point;
            basic_point.x = 0.5 * cosf (pcl::deg2rad(angle));
            basic_point.y = sinf (pcl::deg2rad(angle));
            basic_point.z = z;
            basic_cloud_ptr->points.push_back(basic_point);

            pcl::PointXYZRGBA point;
            point.x = basic_point.x;
            point.y = basic_point.y;
            point.z = basic_point.z;
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                            static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            point.rgb = *reinterpret_cast<float*>(&rgb);
            point_cloud_ptr->points.push_back (point);
        }
        if (z < 0.0)
        {
            r -= 12;
            g += 12;
        }
        else
        {
            g -= 12;
            b += 12;
        }
    }
    basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
    basic_cloud_ptr->height = 1;
    point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
    point_cloud_ptr->height = 1;
}

int
main (int argc, char** argv)
{

    cout<<"hello world"<<endl;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);

    // ------------------------------------
    // -----Create example point cloud-----
    // ------------------------------------
    createCloud(point_cloud_ptr,-1.0,1.0,0.0,360.0);

    // Creating PCL Viewer window and thread
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = rgbVis(point_cloud_ptr);
    //boost::thread vthread(&viewerRunner,viewer);
    boost::thread vthread(boost::bind(&viewerRunner,viewer));

    int	fcount =0;
    while (fcount<=3)
    {
        createCloud(point_cloud_ptr,-1.0-fcount,1.0+fcount,0.0,360.0);

        boost::mutex::scoped_lock updateLock(updateModelMutex);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(point_cloud_ptr);
        viewer->updatePointCloud<pcl::PointXYZRGBA>(point_cloud_ptr,rgb,"Cloud");
        updateLock.unlock();

        boost::this_thread::sleep (boost::posix_time::microseconds (200000));
        fcount++;
    }

    vthread.join();

}
