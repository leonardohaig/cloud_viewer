
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
#include <boost/thread/thread.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>


//设置键盘交互函数,按下`space`键，某事发生
void keyboardEvent(const pcl::visualization::KeyboardEvent &event,void *nothing)
{
    if(event.getKeySym() == "space" && event.keyDown())
        cout<<"space键按下"<<endl;//next_iteration = true;
}



boost::mutex updateModelMutex;

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, -1);//设置坐标原点

    viewer->setBackgroundColor(0,1,0);//设置背景色

    viewer->addCoordinateSystem(100); //建立空间直角坐标系
    // 添加直线
    pcl::PointXYZ Origin,X, Y, Z;
    Origin.x = Origin.y = Origin.z = 0;
    X.x = -100,X.y = X.z = 0;
    Y.y = -100,Y.x = Y.z = 0;
    Z.z = -100,Z.x = Z.y = 0;
    viewer->addLine<pcl::PointXYZ>(Origin,X,255,0,0,"lineX");//红色线段,线的名字叫做"lineX"
    viewer->addArrow<pcl::PointXYZ> (Origin,X,255,0,0,"arrowX");  //带箭头

    viewer->addLine<pcl::PointXYZ>(Origin,Y,0,255,0,"lineY");//红色线段,线的名字叫做"lineY"
    //viewer->addArrow<pcl::PointXYZ> (Origin,Y,0,255,0,"arowY");  //带箭头

    viewer->addLine<pcl::PointXYZ>(Origin,Z,0,0,255,"lineZ");//红色线段,线的名字叫做"lineZ"
   // viewer->addArrow<pcl::PointXYZ> (Origin,Z,0,0,255,"arrowZ");  //带箭头




    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");

    //添加点云后，通过点云ID来设置显示大小
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");

    viewer->initCameraParameters ();//初始化相机参数

    //viewer->registerKeyboardCallback(&keyboardEvent,(void*)NULL);  //设置键盘回吊函数

    return (viewer);
}

void viewerRunner(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (500);
        boost::this_thread::sleep (boost::posix_time::microseconds (1000));
    }
}

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> &pcloud1 = *cloud_ptr;
    //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    pcloud1.points.push_back( pcl::PointXYZ(10, 10, 80) );
    pcloud1.width = cloud_ptr->size();
    pcloud1.height = 1;
    pcloud1.is_dense = true;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = simpleVis(cloud_ptr);

    boost::thread vthread(boost::bind(&viewerRunner,viewer));

    while(1)//循环抓取深度数据
    {
        pcloud1.clear();
        for ( int _row = 0; _row < 30/*disp.rows*/; _row++ )
        {
            for ( int _col = 0; _col <50/* disp.cols*/; _col++ )
            {
                //X+Y+Z+10=0
                float x, y, z;
                x = _row;
                y = _col;
                z = 0-10-x-y;

                pcl::PointXYZ ptemp(x, y, z);
                pcloud1.points.push_back( ptemp );
            }
        }
        pcloud1.width = cloud_ptr->size();
        pcloud1.height = 1;
        pcloud1.is_dense = true;
        boost::mutex::scoped_lock updateLock(updateModelMutex);
        viewer->updatePointCloud<pcl::PointXYZ>(cloud_ptr,"sample cloud");
        updateLock.unlock();
        boost::this_thread::sleep (boost::posix_time::microseconds (1000));
    }
    vthread.join();

    return 0;
}