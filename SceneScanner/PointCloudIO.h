#pragma once

#include "Header.h"

//---保存関係---//

// XMLファイル読み込み
std::vector<cv::Point3f> loadXMLfile(const std::string &fileName)
{
	//読み込む点群
	std::vector<cv::Point3f> reconstructPoints;
	// xmlファイルの読み込み
	cv::FileStorage cvfs(fileName, cv::FileStorage::READ);
	cvfs["points"] >> reconstructPoints;

	return reconstructPoints;
}


//PLY形式で保存(法線なし)
void savePLY(std::vector<cv::Point3f> points, const std::string &fileName)
{

	//ファイルオープン
	FILE *fp;
	fp = fopen(fileName.data(), "w");

	//ファイルに書き込む
	//ヘッダの設定
	fprintf(fp,"ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nend_header\n", points.size());

	//3次元点群
	//m単位で保存（xmlはmm）
	for (int n = 0; n < points.size(); n++){
	   fprintf(fp, "%f %f %f \n", points[n].x, points[n].y, points[n].z);
	}
	//ファイルクローズ
	fclose(fp);
}

//PLY形式で保存(法線あり)
void savePLY_with_normal(std::vector<cv::Point3f> points, std::vector<cv::Point3f> normals, const std::string &fileName)
{
	//ファイルオープン
	FILE *fp;
	fp = fopen(fileName.data(), "w");

	//ファイルに書き込む
	//ヘッダの設定
	fprintf(fp,"ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nend_header\n", points.size());

	//3次元点群
	//m単位で保存（xmlはmm）
	for (int n = 0; n < points.size(); n++){
		fprintf(fp, "%f %f %f %f %f %f \n", points[n].x, points[n].y, points[n].z, normals[n].x, normals[n].y, normals[n].z);

	}
	//ファイルクローズ
	fclose(fp);
}

//PLY形式で保存(法線あり,meshあり)
void savePLY_with_normal_mesh(std::vector<cv::Point3f> points, std::vector<cv::Point3f> normals, std::vector<cv::Point3i> meshes, const std::string &fileName)
{
	//ファイルオープン
	FILE *fp;
	fp = fopen(fileName.data(), "w");

	//ファイルに書き込む
	//ヘッダの設定
	fprintf(fp,"ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\nelement face %d\nproperty list ushort int vertex_indices\nend_header\n", points.size(), meshes.size());

	//3次元点群
	//m単位で保存（xmlはmm）
	for (int n = 0; n < points.size(); n++){
	   fprintf(fp, "%f %f %f %f %f %f \n", points[n].x, points[n].y, points[n].z, normals[n].x, normals[n].y, normals[n].z);
	}
	//面情報記述
	for(int n = 0; n < meshes.size(); n++)
	{
	   fprintf(fp, "3 %d %d %d\n", meshes[n].x, meshes[n].y, meshes[n].z);
	}
	//ファイルクローズ
	fclose(fp);
}


//OBJ形式で保存(法線あり,meshあり)
void saveOBJ_with_normal_mesh(std::vector<cv::Point3f> points, std::vector<cv::Point3f> normals, std::vector<cv::Point3i> meshes, const std::string &fileName)
{
	//ファイルオープン
	FILE *fp;
	fp = fopen(fileName.data(), "w");

	//ファイルに書き込む
	//ヘッダの設定
	fprintf(fp,"o dorascan");

	//3次元点群
	//m単位で保存（xmlはmm）
	for (int n = 0; n < points.size(); n++){
	   fprintf(fp, "v %f %f %f\n", points[n].x, points[n].y, points[n].z);
	}
	//法線情報記述
	for(int n = 0; n < normals.size(); n++){
		fprintf(fp, "vn %f %f %f\n", normals[n].x, normals[n].y, normals[n].z);
	}
	//面情報記述
	for(int n = 0; n < meshes.size(); n++)
	{
	   fprintf(fp, "3 %d %d %d\n", meshes[n].x, meshes[n].y, meshes[n].z);
	}
	//ファイルクローズ
	fclose(fp);
}


