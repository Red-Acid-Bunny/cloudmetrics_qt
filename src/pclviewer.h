#pragma once
#ifndef PCLVIEWER_H
#define PCLVIEWER_H


// includes
#include <iostream>
// qt
#include <QMainWindow>
#include <QFileDialog>
// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/io.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
    class MainWindow;
}

class PCLViewer : public QMainWindow
{
    
    Q_OBJECT

    public:
        explicit PCLViewer(QWidget *parent=0);
        ~PCLViewer();
    public Q_SLOTS:
        void OpenFile();
        void test_cloud();
        void CurrentFileSlider(int);
        void ResultSlider(int);
    private:
        void initCloud(PointCloudT::Ptr&);
        Ui::MainWindow *ui;
    protected:
        void refreshView();

        pcl::visualization::PCLVisualizer::Ptr viewer;
        pcl::visualization::PCLVisualizer::Ptr viewer_2;
        PointCloudT::Ptr cloud;
        PointCloudT::Ptr cloud_2;



};


#endif // PCLVIEWER_H
