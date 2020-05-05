/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar(new Lidar(cars,0));
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputcloud = lidar->scan();
    //renderRays(viewer,lidar->position,inputcloud);
    //renderPointCloud(viewer,inputcloud,"LiDAR",Color(1,0,0));
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>* pointprocessor(new ProcessPointClouds<pcl::PointXYZ>());
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,pcl::PointCloud<pcl::PointXYZ>::Ptr> SegmentCloud = pointprocessor->SegmentPlane(inputcloud,100,0.2);
    //renderPointCloud(viewer,SegmentCloud.first,"Obstaclescloud",Color(1,0,0));
    renderPointCloud(viewer,SegmentCloud.second,"planeCloud",Color(1,1,1));
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointprocessor->Clustering(SegmentCloud.first,1.0,3,30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0),Color(0,1,0),Color(0,0,1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters){
        std::cout << "cluster size ";
        pointprocessor->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        
        //Box box = pointprocessor->BoundingBox(cluster);
        //renderBox(viewer,box,clusterId);
        
        BoxQ qbox = pointprocessor->QBoundingBox(cluster);
        std::cout << "x coordinate- " << qbox.bboxTransform[0] << std::endl;
        std::cout << "y coordinate- " << qbox.bboxTransform[1] << std::endl;
        std::cout << "z coordinate- " << qbox.bboxTransform[2] << std::endl;
        std::cout << "q1 coordinate- " << qbox.bboxQuaternion.w() << std::endl;
        std::cout << "q2 coordinate- " << qbox.bboxQuaternion.x() << std::endl;
        std::cout << "q3 coordinate- " << qbox.bboxQuaternion.y() << std::endl;
        std::cout << "q4 coordinate- " << qbox.bboxQuaternion.z() << std::endl;
        renderBox(viewer,qbox,clusterId);
        
        ++clusterId;
    }

}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,ProcessPointClouds<pcl::PointXYZI>* pointprocessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloudI){
    // ----------------------------------------------------
    // -----Open 3D viewer and display cityBlock----- -----
    // ----------------------------------------------------
    //ProcessPointClouds<pcl::PointXYZI>* pointprocessorI(new ProcessPointClouds<pcl::PointXYZI>);
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI = pointprocessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    //renderPointCloud(viewer,inputCloudI,"inputCloudI");
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredinputCloudI = pointprocessorI->FilterCloud(inputCloudI,0.1f,Eigen::Vector4f (-25, -7, -7, 1),Eigen::Vector4f (25, 7, 7, 1));
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult = pointprocessorI->SegmentPlaneCityBlock(filteredinputCloudI,100,0.2);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clustercloud = pointprocessorI->Clustering(segResult.first,1,30,3000);
    std::vector<Color> colors = {Color(1,0,0),Color(1,1,0),Color(0,1,1),Color(0,0,1)};
    int clusterid = 0;
    int c = 0;
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : clustercloud){
        if (c>3)
            c = 0;
        pointprocessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"ObstacleCloud"+std::to_string(clusterid),colors[c]);
        Box box = pointprocessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterid);
        clusterid++;
        c++;
    }

    renderPointCloud(viewer,segResult.second,"RoadCloud",Color(0,1,0));

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_2");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    while (!viewer->wasStopped ())
    {
         // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}