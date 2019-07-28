/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <chrono>
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

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol) // TODO: Fill in this function
{

	auto startTime = std::chrono::steady_clock::now(); //Time segmentation process

	std::unordered_set<int> inliersResult;

	srand(time(NULL));

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	while(maxIterations--){ // For max iterations (everytime this loop executes it decrements, will stop when equal to 0 == FALSE)

		//randomly pick two points
		std::unordered_set<int> inliers; //store elements in random order, rejects same numbers

		while(inliers.size()<2){ //this will run to keep feeding inliers into the unordered set
			inliers.insert(rand()%(cloud->points.size()));
		} //close while

		float x1, x2, y1, y2; //point intercepts

		auto itr = inliers.begin(); //returns iterator to first element in unorderd_set

		//coordinates of p1
		x1 = cloud->points[*itr].x; //gets the x value points passed in by itr from cloud 
		y1 = cloud->points[*itr].y;
		
		itr++; //increment itr to move to next element in unordered_set

		//coordinates of p2
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;

		float a = (y1-y2); //distance between points
		float b = (x2-x1); 
		float c = ((x1*y2)-(x2*y1)); // after reorganizing rest of standard line equation with a=(x1-x2) and b=(y1-y2);

		for(int i = 0; i < cloud->points.size(); i++) { //will loop through all the points in cloud searching for inliers

			if(inliers.count(i)>0){
				continue;
			}

			pcl::PointXYZ point = cloud->points[i];
			float x3 = point.x;
			float y3 = point.y;

			float d = fabs(a*x3+b*y3+c)/sqrt(a*a+b*b); //fabs = absolute value of float

			if(d <= distanceTol){ //checking if distance satisfies tolerance to add as inlier
				inliers.insert(i); //add element to inliers
			}

		}

		if(inliers.size() > inliersResult.size()){ 
			inliersResult = inliers;
		}

	} //close while maxIterations--

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime);
	std::cout << "Ransac took " << elapsedTime.count() << " milliseconds" << std::endl;

	return inliersResult;

} //close Ransac2D
/*
std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol) // TODO: Fill in this function //this is new ransac
{

	auto startTime = std::chrono::steady_clock::now(); //Time segmentation process

	std::unordered_set<int> inliersResult;

	srand(time(NULL));

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	while(maxIterations--){ // For max iterations (everytime this loop executes it decrements, will stop when equal to 0 == FALSE)

		//randomly pick two points
		std::unordered_set<int> inliers; //store elements in random order, rejects same numbers

		while(inliers.size()<2){ //this will run to keep feeding inliers into the unordered set
			inliers.insert(rand()%(cloud->points.size()));
		} //close while

		float x1, x2, x3, y1, y2, y3, z1, z2, z3; //point intercepts for 3d
		float v1, v2, v3; //vectors

		auto itr = inliers.begin(); //returns iterator to first element in unorderd_set

		//coordinates of p1
		x1 = cloud->points[*itr].x; //gets the x value points passed in by itr from cloud 
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		
		itr++; //increment itr to move to next element in unordered_set

		//coordinates of p2
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;

		//coordinates of p3
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		//distance between points
		v1 =;
		v2 =;
		v3 =;

		float a = (x1-x2); //distance between points
		float b = (y1-y2); 
		float c = ((x1*y2)-(x2*y1)); // after reorganizing rest of standard line equation with a=(x1-x2) and b=(y1-y2);

		for(int i = 0; i < cloud->points.size(); i++) { //will loop through all the points in cloud searching for inliers

			if(inliers.count(i)>0){
				continue;
			}

			float x3, y3, z3;

			pcl::PointXYZ point = cloud->points[i];
			x3 = point.x;
			y3 = point.y;
			z3 = point.z;

			float dist = fabs(a*x3+b*y3+c*z3+d)/sqrt(a*a+b*b+c*c); //fabs = absolute value of float

			if(dist <= distanceTol){ //checking if distance satisfies tolerance to add as inlier
				inliers.insert(i); //add element to inliers else ignore
			}

		}

		if(inliers.size() > inliersResult.size()){ 
			inliersResult = inliers;
		}

	} //close while maxIterations--

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime-startTime);
	std::cout << "Ransac took " << elapsedTime.count() << " milliseconds" << std::endl;

	return inliersResult;

} //close Ransac3D
*/
int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(); //2d
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D(); //3d

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac2D(cloud, 10, 1.0);

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
