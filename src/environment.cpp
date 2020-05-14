/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include <unordered_set>
//#include "quiz/cluster/kdtree.h"
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

std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> RansacPlane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
{
	auto start = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	for (int i = 0; i < maxIterations;i++){
		pcl::PointXYZI point1 = cloud->points[rand()%cloud->points.size()];
		pcl::PointXYZI point2 = cloud->points[rand()%cloud->points.size()];
		pcl::PointXYZI point3 = cloud->points[rand()%cloud->points.size()];
		float v1_x = point2.x-point1.x;
		float v1_y = point2.y-point1.y;
		float v1_z = point2.z-point1.z;
		float v2_x = point3.x-point1.x;
		float v2_y = point3.y-point1.y;
		float v2_z = point3.z-point1.z;
		float A = (v1_y*v2_z-v1_z*v2_y);
		float B = (v1_z*v2_x-v1_x*v2_z);
		float C = (v1_x*v2_y-v1_y*v2_x);
		float D =  -(A*point1.x+B*point1.y+C*point1.z);
		std::unordered_set<int> inliersResultref;
		int j = 0;
		for (pcl::PointXYZI point : cloud->points){
			float d = fabs(A*point.x + B*point.y + C*point.z + D)/sqrt(pow(A,2)+pow(B,2)+pow(C,2));
			if (d <= distanceTol){
				inliersResultref.insert(j);
			}
			++j;
		}
		if (inliersResultref.size() > inliersResult.size()){
			inliersResult = inliersResultref;
		}
	}
	auto end = std::chrono::steady_clock::now();
	auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds> (end-start);
	std::cout << " Time taken for the RANSAC plane algorithm is " << elapsed_time.count() << " milliseconds" << std::endl;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr  obstaclesCloud(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZI>());
    for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZI point = cloud->points[index];
		if(inliersResult.count(index))
			planeCloud->points.push_back(point);
		else
			obstaclesCloud->points.push_back(point);
	}
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult(obstaclesCloud, planeCloud);
    
    /*
    std::vector<pcl::PointXYZI>* obstaclesCloud(new std::vector<pcl::PointXYZI>());
	std::vector<pcl::PointXYZI>* planeCloud(new std::vector<pcl::PointXYZI>());
	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZI point = cloud->points[index];
		if(inliersResult.count(index))
			planeCloud->push_back(point);
		else
			obstaclesCloud->push_back(point);
	}
    std::pair<std::vector<pcl::PointXYZI>*, std::vector<pcl::PointXYZI>*> segResult(obstaclesCloud, planeCloud);
    */
    return segResult;

}

void proximity(const std::vector<pcl::PointXYZI>& points, std::vector<int>& cluster, std::vector<bool>& processed, int i, float distanceTol, KdTree* tree){
	processed[i] = true;
	cluster.push_back(i);
	std::vector<int> neighbor_points;
	neighbor_points = tree->search({points[i].x,points[i].y},distanceTol);
	for (int neighbor_point : neighbor_points){
		if (!processed[neighbor_point]){
			proximity(points,cluster,processed,neighbor_point,distanceTol,tree);
		}
	}
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<pcl::PointXYZI>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(),false);
	int i = 0;
	while(i<points.size()){
		if (!processed[i]){
			std::vector<int> cluster;
			proximity(points,cluster,processed,i,distanceTol,tree);
			clusters.push_back(cluster);
		}
		i++;
	}
 
	return clusters;

}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,ProcessPointClouds<pcl::PointXYZI>* pointprocessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloudI){
    // ----------------------------------------------------
    // -----Open 3D viewer and display cityBlock----- -----
    // ----------------------------------------------------
    //ProcessPointClouds<pcl::PointXYZI>* pointprocessorI(new ProcessPointClouds<pcl::PointXYZI>);
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI = pointprocessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    //renderPointCloud(viewer,inputCloudI,"inputCloudI");
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredinputCloudI = pointprocessorI->FilterCloud(inputCloudI,0.1f,Eigen::Vector4f (-25, -7, -7, 1),Eigen::Vector4f (25, 7, 7, 1));
    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult = pointprocessorI->SegmentPlaneCityBlock(filteredinputCloudI,100,0.2);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult = RansacPlane(filteredinputCloudI,100,0.2);
    //std::pair<std::vector<pcl::PointXYZI>*,std::vector<pcl::PointXYZI>*> segResult = RansacPlane(filteredinputCloudI,100,0.2);
    //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clustercloud = pointprocessorI->Clustering(segResult.first,1,30,3000);
    KdTree* tree = new KdTree;
    for (int i=0; i<(segResult.first)->size(); i++) 
    	tree->insert({(segResult.first)->points[i].x,(segResult.first)->points[i].y},i); 
    
    auto startTime = std::chrono::steady_clock::now();
  	std::vector<std::vector<int>> clusters = euclideanCluster(std::vector<pcl::PointXYZI>(((segResult.first)->points).begin(),((segResult.first)->points).end()), tree, 3.0); 
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    int clusterId = 0;
    for(std::vector<int> cluster : clusters)
  	{
  		pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
  		for(int indice: cluster)
  			clusterCloud->points.push_back((segResult.first)->points[indice]);
  		renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
        Box box = pointprocessorI->BoundingBox(clusterCloud);
        renderBox(viewer,box,clusterId);
  		++clusterId;
  	}
    /*
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
    */

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