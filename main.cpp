#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/io.h>
#include <pcl/filters/statistical_outlier_removal.h>

int user_data;




void foo1(char *file_name)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPLYFile<pcl::PointXYZ> 
            (file_name, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return;
    }
    std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from test_pcd.pcd with the following fields: "
            << std::endl;

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (1.0, 3.5);
    //pass.setNegative (true);
    pass.filter (*cloud_filtered);
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-3.0, 3.0);
    pass.filter (*cloud_filtered);
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-3.0, 3.0);
    pass.filter (*cloud_filtered);
    //pcl::io::savePLYFileASCII ("test1.ply", *cloud);
    pcl::io::savePLYFileASCII ("test2.ply", *cloud_filtered);
}

void foo2(char *file_name)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPLYFile<pcl::PointXYZ> 
            (file_name, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return;
    }
    // Fill in the cloud data
    //pcl::PLYReader reader;
    // Replace the path below with the path where you saved your file
    //reader.read<pcl::PointXYZ> (file_name, *cloud);

    //std::cerr << "Cloud before filtering: " << std::endl;
    //std::cerr << *cloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;
    

    std::cout << "t1\n";
    pcl::PLYWriter writer;
    writer.write<pcl::PointXYZ> ("test1.ply", *cloud_filtered, false);

    std::cout << "t2\n";
    sor.setNegative (true);
    sor.filter (*cloud_filtered);
    writer.write<pcl::PointXYZ> ("test2.ply", *cloud_filtered, false);
}

int main ()
{
    char *file_name = "test_ply/1_AutoAligned.ply";
    //foo1(file_name);
    foo2(file_name);
    return (0);
}
