/* \author Geoffrey Biggs */

#include <iostream>
#include <thread>

#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

using namespace std::chrono_literals;

// --------------
// -----Help-----
// --------------
void
printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-f [file]    Simple visualisation ply\n"
            << "\n\n";
}


pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  int MIN_ARGC = 3;
  bool err(false);
  if (MIN_ARGC > argc)
  {
    printUsage (argv[0]);
    err = true;
  }
  else if (pcl::console::find_argument (argc, argv, "-f") == -1)
  {
    err = true;
  }

  if(err)
  {
    std::cout << "ERROR" << std::endl;
    return EXIT_FAILURE;
  }

  if (pcl::console::find_argument (argc, argv, "-h") >= 0)
  {
    printUsage (argv[0]);
    return EXIT_SUCCESS;
  }

  bool simple(true);

  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

  int file_path = pcl::console::find_argument (argc, argv, "-f") + 1;
  std::cout << argc << std::endl;
  if(MIN_ARGC == argc)
  {
    std::cout << *(argv+file_path) << std::endl;
    pcl::io::loadPLYFile (*(argv+file_path), *basic_cloud_ptr);
  }
  else
  {
    cout << "Error" << endl;
    return EXIT_FAILURE;
  }

  pcl::visualization::PCLVisualizer::Ptr viewer;
  if (simple)
  {
    viewer = simpleVis(basic_cloud_ptr);
  }

  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (1);
    std::this_thread::sleep_for(1ns);
  }
}
