#pragma once

#include "Header.h"

//---Filter関係---//


//法線ベクトルを求める
std::vector<cv::Point3f> getNormalVectors(std::vector<cv::Point3f> points)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	for (int n = 0; n < points.size(); n++){
		cloud->push_back(pcl::PointXYZ(points[n].x, points[n].y, points[n].z));//単位はm
	}

	  // Create the normal estimation class, and pass the input dataset to it
	  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	  ne.setInputCloud (cloud);

	  // Create an empty kdtree representation, and pass it to the normal estimation object.
	  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	  ne.setSearchMethod (tree);

	  // Output datasets
	  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	  // Use all neighbors in a sphere of radius 3cm
	  ne.setRadiusSearch (0.03);

	  // Compute the features
	  ne.compute (*cloud_normals);

	  std::vector<cv::Point3f> dst_normals;
	  for(int n = 0; n < cloud_normals->size(); n++)
	  {
		  //”1.#QNAN0”を0にする
		  if(!cvIsNaN(cloud_normals->at(n).normal_x))
			  dst_normals.emplace_back(cv::Point3f(cloud_normals->at(n).normal_x, cloud_normals->at(n).normal_y, cloud_normals->at(n).normal_z));
		  else
			  dst_normals.emplace_back(cv::Point3f(0, 0, 0));
	  }
	  return dst_normals;
}

//ダウンサンプリング
std::vector<cv::Point3f> getDownSampledPoints(std::vector<cv::Point3f> points, float size)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	for (int n = 0; n < points.size(); n++){
		cloud->push_back(pcl::PointXYZ(points[n].x, points[n].y, points[n].z));//単位はm
	}

    // Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (size, size, size);
	sor.filter (*cloud_filtered);

	std::vector<cv::Point3f> dst_points;
	for(int n = 0; n < cloud_filtered->size(); n++)
		dst_points.emplace_back(cv::Point3f(cloud_filtered->at(n).x, cloud_filtered->at(n).y, cloud_filtered->at(n).z));

	return dst_points;
}


//三角メッシュを生成、メッシュ情報を返す
std::vector<cv::Point3i> getMeshVectors(std::vector<cv::Point3f> points, std::vector<cv::Point3f> normals, pcl::PolygonMesh& triangles)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	for (int n = 0; n < points.size(); n++){
		cloud->push_back(pcl::PointXYZ(points[n].x, points[n].y, points[n].z));//単位はm
		cloud_normals->push_back(pcl::Normal(normals[n].x, normals[n].y, normals[n].z));
	}
	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud, *cloud_normals, *cloud_with_normals);

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (0.025);

	// Set typical values for the parameters
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);

	//メッシュ情報を返す
	std::vector<cv::Point3i> dst_meshes;
	for(int n = 0; n < triangles.polygons.size(); n++)
	{
		dst_meshes.emplace_back(cv::Point3i(triangles.polygons[n].vertices[0], triangles.polygons[n].vertices[1], triangles.polygons[n].vertices[2])); 
	}

	return dst_meshes;
}