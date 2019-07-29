/* \author Aaron Brown & Roger Fleenor */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <chrono>
#include <random>
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

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol){

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

std::tuple<int, int, int> RandomTriple(int size){

    int index1 = rand()%size, index2, index3;
    while(1)
    {
        index2 = rand()%size;
        if(index1!=index2)
            break;
    }
    while(1)
    {
        index3 = rand()%size;
        if(index3!=index2 && index3!=index1)
            break;
    }
    std::tuple<int, int, int> triple {index1, index2, index3};
    return triple;
}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol){

    auto startTime = std::chrono::steady_clock::now();
    
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    
    // TODO: Fill in this function
    int a_max, b_max, c_max, d_max;
    int max_inliners = 0;
    int size = cloud->points.size();
    
    // For max iterations
    while(maxIterations--){

        int inliners = 0;
        // Randomly sample subset and fit line
        std::tuple<int, int, int> ind = RandomTriple(size);

		//3 points, 2 vectors
		
		/*
		point1=(x1,y1,z1)
		point2=(x2,y2,z2)
		point3=(x3,y3,z3)
		*/

		//create point indices for each dimension

        int ind1 = std::get<0>(ind);
        int ind2 = std::get<1>(ind);
        int ind3 = std::get<2>(ind);

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

		x1 = cloud->points[ind1].x;
		y1 = cloud->points[ind1].y;
		z1 = cloud->points[ind1].z;

		x2 = cloud->points[ind2].x;
		y2 = cloud->points[ind2].y;
		z2 = cloud->points[ind2].z;

		x3 = cloud->points[ind3].x;
		y3 = cloud->points[ind3].y;
		z3 = cloud->points[ind3].z;

		//v1 = <x2-x1, y2-y1, z2-z1>
		//v2 = <x3-x1, y3-y1, z3-z1>

		//apply cross product: v1 X v2 = <i, j, k>

		//<(y2-y1)*(z3-z1)-(z2-z1)*(y3-y1), (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1), (x2-x1)(y3-y1)-(y2-y1)(x3-x1)>
		
		float a, b, c, x, y, z;

		a = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		b = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		c = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);

        x = cloud->points[ind1].x;
        y = cloud->points[ind1].y;
        z = cloud->points[ind1].z;
        
        float d = -(a*x + b*y + c*z); //calculate distance
        
        // Measure distance between every point and fitted line

        for(int j=0;j<size;j++){

            float dist = abs(cloud->points[j].x*a + cloud->points[j].y*b + cloud->points[j].z*c + d)/sqrt(a*a + b*b + c*c);

            if(dist <= distanceTol){ // If distance is smaller than threshold count it as inlier
                inliners++;
			}

        } //close for
		
		 if(inliners > max_inliners){

            a_max = a;
            b_max = b;
            c_max = c;
            d_max = d;
            max_inliners = inliners;

        }
    }
    
    // Return indicies of inliers from fitted line with most inliers
    for(int j=0;j<size;j++)
    {
        float dist = abs(cloud->points[j].x*a_max + cloud->points[j].y*b_max + cloud->points[j].z*c_max + d_max)/sqrt(a_max*a_max + b_max*b_max + c_max*c_max);

        if(dist <= distanceTol) { // If distance is smaller than threshold count it as inlier
            inliersResult.insert(j);
		}
    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    
    return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData(); //2d
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D(); //3d

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac2D(cloud, 10, 1.0);
	std::unordered_set<int> inliers = Ransac3D(cloud, 10, 1.0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
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
