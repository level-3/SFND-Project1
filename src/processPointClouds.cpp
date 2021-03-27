// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

#include <Eigen/Core> //for printing matrice
#include <unordered_set>
#include "kdtree.h"
#include <Eigen/Geometry> 

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

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    // Create the filtering object
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud (cloud);
    voxel_grid.setLeafSize (filterRes, filterRes, filterRes);
    voxel_grid.filter (*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cloud_crop (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setInputCloud (cloud_filtered);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.filter (*cloud_crop);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setInputCloud (cloud_crop);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1.0 , 1.0));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.4 , 1.0));
    roof.filter (indices);
    
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    for (int point : indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud_crop);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter (*cloud_crop);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_crop;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());

    for (int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud,planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentRansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{

	auto startTime = std::chrono::steady_clock::now();

	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	while (maxIterations--)
	{
		std::unordered_set<int> inliers;
		while(inliers.size() < 3)
		inliers.insert(rand()%(cloud->points.size()));
		float x1,x2,x3,y1,y2,y3,z1,z2,z3;

		auto itr=inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		float i = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		float j = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		float k = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);

		float a = i;
		float b = j;
		float c = k;
		float d = - (i * x1 + j * y1 + k * z1);

		for (int index = 0; index < cloud->points.size();index++)
		{
			if (inliers.count(index) > 0)
                {
                    continue;                
                }

			PointT pt = cloud->points[index];
			float x4 = pt.x;
			float y4 = pt.y;
			float z4 = pt.z;

			float dist = fabs(a*x4 + b*y4 + c*z4 + d) / sqrt(a*a + b*b + c*c);

			if (dist <= distanceTol)
                {
                    inliers.insert(index);                
                }

            if (inliers.size()>inliersResult.size())
                {
                    inliersResult = inliers;
                }

		}

	}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
        {
            PointT point = cloud->points[index];

            if(inliersResult.count(index))
            {
                cloudInliers->points.push_back(point);      
            }
            else
            {
                cloudOutliers->points.push_back(point);
            }

        }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers,cloudInliers);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC took " << elapsedTime.count() << " milliseconds and found " << inliersResult.size() << " inliers " << std::endl;


	return segResult;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;


    return segResult;
}

static void clusterHelper(int indice , const std::vector<std::vector<float>> points , std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol )
{
	processed[indice] = true;
	cluster.push_back(indice);

	std::vector<int> nearest = tree->search(points[indice], distanceTol);

	for (int id: nearest)
        {
            if (!processed[id])
            {
                clusterHelper(id,points,cluster,processed,tree,distanceTol);                
            }
        }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::KDClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTol, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    //std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    std::vector<std::vector<float>> points;

    for (const auto& point : *cloud)
        {
            points.push_back({point.x,point.y,point.z});
        }

	KdTree* tree = new KdTree;

    for (int i=0; i<points.size(); i++) 
		tree->insert(points[i],i); 

	std::vector<std::vector<int>> clusters;
 
	std::vector<bool> processed(points.size(), false);

	int i = 0;
	while (i <points.size())
	{
		if (processed[i])
		{
			i++;
			continue;
		}

		std::vector<int> cluster ;
		clusterHelper( i, points, cluster, processed, tree, distanceTol);
        //clusters.push_back(cluster);

        if (cluster.size() >= minSize)
            {
                if (cluster.size() <= maxSize)
                    {
                        clusters.push_back(cluster);                        
                    }
            }
        else
            {
                for (int del_indice:cluster)
                {
                    processed[del_indice]=false;                    
                }
            }


		i++;
	}

    std::vector<typename pcl::PointCloud<PointT>::Ptr> cloudClusters;

	// Render clusters
	int clusterId = 0;

	for(std::vector<int> cluster : clusters)
        {
            typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
            for(int indice: cluster)
                clusterCloud->points.push_back(cloud->points[indice]);
            clusterCloud->width = clusterCloud->points.size();
            clusterCloud->height = 1;
            clusterCloud->is_dense = true;
            cloudClusters.push_back(clusterCloud);
        }

/*    
	int clusterId = 0;

    std::vector<typename pcl::PointCloud<PointT>::Ptr> cloudClusters;

	for(std::vector<int> cluster : clusters)
	{
		typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
		for(int indice: cluster)

			clusterCloud->push_back((points[indice][0],points[indice][1],0));
            cloudClusters.push_back(clusterCloud);
        ++clusterId;
	}




    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (const auto& idx : it->indices)
        cloud_cluster->push_back ((*cloud)[idx]); //*
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }

*/

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return cloudClusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);

    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (const auto& idx : it->indices)
            cloud_cluster->push_back ((*cloud)[idx]); //*
            cloud_cluster->width = cloud_cluster->size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;
            clusters.push_back(cloud_cluster);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

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
 std::pair<BoxQ, BoxQ> ProcessPointClouds<PointT>::PCABoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    bool verbose = false;
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);

    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();

    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
                                                                                    ///    the signs are different and the box doesn't get correctly oriented in some cases.
    /* // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloudSegmented);
    pca.project(*cloudSegmented, *cloudPCAprojection);
    std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
    std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;
    // In this case, pca.getEigenVectors() gives similar eigenVectors to eigenVectorsPCA.
    */

    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());

    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
    
    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());


    Eigen::Vector3f cloud_normal(pcaCentroid[0],pcaCentroid[1],pcaCentroid[2]);
    Eigen::Vector3f plane_normal(0,0,1);
    
    Eigen::Vector3f rotation = cloud_normal.cross (plane_normal);


    if(verbose)
        {
            std::cout << "pcaCentroid: " << std::endl << pcaCentroid.format(CleanFmt) << std::endl;            
            std::cout << "covariance: " << std::endl << covariance.format(CleanFmt) << std::endl;
            std::cout << "eigenVectorsPCA: " << std::endl << eigenVectorsPCA.format(CleanFmt) << std::endl;
            std::cout << "Transform: " << std::endl << projectionTransform.format(CleanFmt) << std::endl;            
            std::cout << "cloud_normal: " << std::endl << cloud_normal.format(CleanFmt) << std::endl;  
            std::cout << "plane_normal: " << std::endl << plane_normal.format(CleanFmt) << std::endl;  
            std::cout << "rotation: " << std::endl << rotation.format(CleanFmt) << std::endl;      
        }


    float length = sqrt( rotation[0]*rotation[0] + rotation[1]*rotation[1] + rotation[2]*rotation[2] );

    Eigen::Vector3f transformVector( rotation[0]/length , rotation[1]/length , rotation[2]/length ); 

    Eigen::Affine3f transformRotationOfModel = Eigen::Affine3f::Identity();

    float theta = -acos( cloud_normal[0]*plane_normal[0] + cloud_normal[1]*plane_normal[1] + cloud_normal[2]*plane_normal[2] );
    
    transformRotationOfModel.rotate (Eigen::AngleAxisf (theta, transformVector));

    BoxQ PCABox;

    PCABox.bboxTransform  = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();
    PCABox.bboxQuaternion = eigenVectorsPCA;

	PCABox.cube_length = std::abs(maxPoint.x-minPoint.x);
    PCABox.cube_width  = std::abs(maxPoint.y-minPoint.y);
    PCABox.cube_height = std::abs(maxPoint.z-minPoint.z);


    BoxQ PCARotatedBox;

    PCARotatedBox.bboxTransform  =  transformRotationOfModel * meanDiagonal + pcaCentroid.head<3>();
    PCARotatedBox.bboxQuaternion = transformRotationOfModel.rotation();


	PCARotatedBox.cube_length = std::abs(maxPoint.x-minPoint.x);
    PCARotatedBox.cube_width  = std::abs(maxPoint.y-minPoint.y);
    PCARotatedBox.cube_height = std::abs(maxPoint.z-minPoint.z);

    std::pair<BoxQ, BoxQ> BoxPair(PCABox,PCARotatedBox);

    if(verbose) 
    {
        std::cout << "transformVector: " << std::endl << transformVector.format(CleanFmt) << std::endl;  
        std::cout << "transformRotationOfModel: " << std::endl << transformRotationOfModel.rotation().format(CleanFmt) << std::endl;
        std::cout<< "angle : " << theta << std::endl;
        std::cout << "after rotation: " << std::endl << transformVector.format(CleanFmt) << std::endl;
        std::cout << "cube: LWH" << std::endl << PCABox.cube_length << std::endl << PCABox.cube_width << std::endl<< PCABox.cube_height << std::endl;        
    }


    return BoxPair;
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