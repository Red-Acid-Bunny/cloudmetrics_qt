#include "pclviewer.h"
#include "ui_pclviewer.h"
#include <vtkGenericOpenGLRenderWindow.h>
#include <QThread>

PCLViewer::PCLViewer(QWidget *parent) : 
    QMainWindow (parent),
    ui (new Ui::MainWindow)
{
    unsigned int init_size_cloud = 1,
                 the_magic_number = 1024;

    ui->setupUi(this);
    this->setWindowTitle("PCL viewer");


    initCloud(cloud);
    initCloud(cloud_2);

    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow = 
        vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    
    renderWindow->AddRenderer(renderer);
    std::cout << renderWindow->ReportCapabilities() << std::endl;
    viewer.reset(new pcl::visualization::PCLVisualizer(
                renderer, 
                renderWindow, 
                "viewer", 
                false));

    ui->pclWidget_1->setRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(
            ui->pclWidget_1->interactor(), 
            ui->pclWidget_1->renderWindow());

    
    auto renderer_2 = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow_2 = 
        vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();

    renderWindow_2->AddRenderer(renderer_2);
    std::cout << renderWindow_2->ReportCapabilities() << std::endl;
    viewer_2.reset(new pcl::visualization::PCLVisualizer(
                renderer_2, 
                renderWindow_2, 
                "viewer_2", 
                false));

    ui->pclWidget_2->setRenderWindow(viewer_2->getRenderWindow());
    viewer_2->setupInteractor(
            ui->pclWidget_2->interactor(), 
            ui->pclWidget_2->renderWindow());



    // connections
    connect (ui->Apply, SIGNAL(clicked()), this, SLOT(test_cloud()));
    connect (ui->OpenFile, SIGNAL(clicked()), this, SLOT(OpenFile()));
    connect (ui->CurrentFileSlider, 
            SIGNAL (valueChanged (int)), 
            this,
            SLOT (CurrentFileSlider (int)));

    connect (ui->ResultSlider, 
            SIGNAL (valueChanged (int)), 
            this,
            SLOT (ResultSlider (int)));

    viewer->addPointCloud(cloud, "cloud");
    viewer_2->addPointCloud(cloud_2, "cloud_2");
    viewer->resetCamera();
    viewer_2->resetCamera();
    CurrentFileSlider(1);
    ResultSlider(1);
    refreshView();
}

void PCLViewer::initCloud(PointCloudT::Ptr& cloud)
{
    cloud.reset(new PointCloudT);
    cloud->resize(1);
    for(auto& point: *cloud)
    {
        point.x = 0;
        point.y = 0;
        point.z = 0;
    }
    
}

void PCLViewer::test_cloud()
{
    cloud_2.reset(new PointCloudT);
    copyPointCloud(*cloud, *cloud_2);

    viewer_2->updatePointCloud(cloud_2, "cloud_2");
    viewer_2->addCoordinateSystem(1.0);
    viewer_2->resetCamera();
    ResultSlider(1);
    refreshView();
}


void PCLViewer::OpenFile()
{
    cloud.reset(new PointCloudT);
    QString str = QFileDialog::getOpenFileName(0, "Открыть", "", "*.ply");
    std::cout << str.toStdString() << std::endl;
    pcl::io::loadPLYFile<PointT>(str.toStdString(), *cloud);
    viewer->updatePointCloud(cloud, "cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->resetCamera();
    CurrentFileSlider(1);
    refreshView();
}

void PCLViewer::CurrentFileSlider(int value = 1)
{
    viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
            value, 
            "cloud");

    refreshView();
}

void PCLViewer::ResultSlider(int value = 1)
{
    viewer_2->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 
            value, 
            "cloud_2");

    refreshView();
}

void PCLViewer::refreshView()
{
    ui->pclWidget_1->renderWindow()->Render();
    ui->pclWidget_2->renderWindow()->Render();
}

PCLViewer::~PCLViewer()
{
    delete ui;
}
