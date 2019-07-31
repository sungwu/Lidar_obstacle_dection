// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT> ());
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes,filterRes,filterRes);
    sor.filter(*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
        inliers -> indices.push_back(point);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    //typename pcl::PointCloud<PointT>::Ptr obstCloud = new typename pcl::PointCloud<PointT>::Ptr inliers;
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    //typename pcl::PointCloud<PointT>::Ptr planeCloud = new typename pcl::PointCloud<PointT>::Ptr cloud;
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

    for(int index : inliers -> indices)
        planeCloud -> points.push_back(cloud -> points[index]);
    //create filtering object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
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
        temp.clear();
		PointT point1 = cloud->points[rand()%cloud->width];
		PointT point2 = cloud->points[rand()%cloud->width];
		PointT point3 = cloud->points[rand()%cloud->width];
		float i = (point2.y-point1.y)*(point3.z-point1.z)-(point2.z-point1.z)*(point3.y-point1.y);
		float j = (point2.z-point1.z)*(point3.x-point1.x)-(point2.x-point1.x)*(point3.z-point1.z);
		float k = (point2.x-point1.x)*(point3.y-point1.y)-(point2.y-point1.y)*(point3.x-point1.x);
		coeff = {i,j,k,-(i*point1.x+j*point1.y+k*point1.z)};
		
	
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
		for( int w =0; w < cloud->width; w++){
			PointT point4 = cloud->points[w];
			colVec ={point4.x,point4.y,point4.z,1};
			if (fabs(coeff[0]*point4.x+coeff[1]*point4.y+coeff[2]*point4.z+coeff[3])/sqrt(pow(coeff[0],2)+pow(coeff[1],2)+pow(coeff[2],2))< distanceTol){
				temp.insert(w);
			}
		}
		if (maxSize < temp.size()){
			maxSize = temp.size();
			inliersResult.clear();
			inliersResult = temp;
			}
	}

    typename pcl::PointCloud<PointT>::Ptr  obstCloud(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			planeCloud->points.push_back(point);
		else
			obstCloud->points.push_back(point);
	}
	// Return indicies of inliers from fitted line with most inliers
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
	return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    // Create the segmentation object 
    pcl::SACSegmentation<PointT> seg;
    //Optional
    seg.setOptimizeCoefficients(true);
    //Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    //Segment the Largest Planar component
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); //2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for(pcl::PointIndices getIndices: cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices)
            cloudCluster->points.push_back (cloud->points[index]);
             
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height =1;
        cloudCluster->is_dense = true;    

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster_idx, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
	processed[indice] = true;
	cluster_idx.push_back(indice);

	std::vector<int> nearest = tree->search(cloud->points[indice],distanceTol);
	
	for(int id : nearest)
	{
		if(!processed[id])
			clusterHelper(id, cloud, cluster_idx, processed, tree, distanceTol);
	}
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, int minSize, int maxSize)
{//typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
	//std::vector<std::vector<int>> clusters;
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

	std::vector<bool> processed(cloud->points.size(), false);

	int i = 0;
	while(i < cloud->points.size())
	{
		if(processed[i])
		{
			i++;
			continue;
		}
        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
		std::vector<int> cluster_idx;
		clusterHelper(i, cloud, cluster_idx, processed, tree, distanceTol);
        
        if (cluster_idx.size()<maxSize && cluster_idx.size()>minSize)
        {
            for(int i : cluster_idx)
            {
                cluster->points.push_back(cloud->points[i]);
                cluster->width = cluster->points.size();
                cluster->height =1;
            }
            clusters.push_back(cluster);
        }
        i++;
	}


	return clusters;

}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}