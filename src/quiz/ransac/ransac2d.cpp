/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::unordered_set<int> temp;
	srand(time(NULL));
	std::vector<float> coefficientVector;
	std::vector<float> colVector;
	int maxSize = 0;
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line
	for( int i =0; i < maxIterations; i++){
		pcl::PointXYZ point1 = cloud->points[rand()%cloud->width];
		pcl::PointXYZ point2 = cloud->points[rand()%cloud->width];
		coefficientVector = {point1.y-point2.y, point2.x-point1.x, point1.x*point2.y-point2.x*point1.y};
		
		
	
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
		for( int j =0; j < cloud->width; j++){
			pcl::PointXYZ point3 = cloud->points[j];
			colVector ={point3.x,point3.y,1};
			if (abs(coefficientVector[0]*colVector[0]+coefficientVector[1]*colVector[1]+coefficientVector[2])/sqrt(pow(coefficientVector[0],2)+pow(coefficientVector[1],2))< distanceTol){
				temp.insert(j);
			}
		}
		if (maxSize < temp.size()){
			maxSize = temp.size();
			inliersResult.clear();
			inliersResult = temp;
			}
		temp.clear(); 
	}
	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}
std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::unordered_set<int> temp;
	srand(time(NULL));
	std::vector<float> coeff;
	std::vector<float> colVec;
	int maxSize = 0;
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line
	for( int q =0; q < maxIterations; q++){
		pcl::PointXYZ point1 = cloud->points[rand()%cloud->width];
		pcl::PointXYZ point2 = cloud->points[rand()%cloud->width];
		pcl::PointXYZ point3 = cloud->points[rand()%cloud->width];
		float i = (point2.y-point1.y)*(point3.z-point1.z)-(point2.z-point1.z)*(point3.y-point1.y);
		float j = (point2.z-point1.z)*(point3.x-point1.x)-(point2.x-point1.x)*(point3.z-point1.z);
		float k = (point2.x-point1.x)*(point3.y-point1.y)-(point2.y-point1.y)*(point3.x-point1.x);
		coeff = {i,j,k,-(i*point1.x+j*point1.y+k*point1.z)};
		
	
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
		for( int w =0; w < cloud->width; w++){
			pcl::PointXYZ point4 = cloud->points[w];
			colVec ={point4.x,point4.y,point4.z,1};
			if (abs(coeff[0]*point4.x+coeff[1]*point4.y+coeff[2]*point4.z+coeff[3])/sqrt(pow(coeff[0],2)+pow(coeff[1],2)+pow(coeff[2],2))< distanceTol){
				temp.insert(w);
			}
		}
		if (maxSize < temp.size()){
			maxSize = temp.size();
			inliersResult.clear();
			inliersResult = temp;
			}
		temp.clear(); 
	}
	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 20, 0.5);
	std::unordered_set<int> inliers = RansacPlane(cloud, 20, 0.5);
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
