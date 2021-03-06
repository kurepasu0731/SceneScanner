#include "Graycode.h"
#include "Header.h"
#include "Calibration.h"
#include "PointCloudIO.h"
#include "PointCloudFilter.h"
#include "FeatureCloud.h"


#define MASK_ADDRESS "./GrayCodeImage/mask.bmp"
#define IMAGE_DIRECTORY "./UseImage"
#define SAVE_DIRECTORY "./UseImage/resize"

//スキャンして3次元点群を得る
std::vector<cv::Point3f> scanScene(Calibration calib, GRAYCODE gc)
{
	// グレイコード投影
	gc.code_projection();
	gc.make_thresh();
	gc.makeCorrespondence();

	//***対応点の取得(カメラ画素→3次元点)******************************************
	std::vector<cv::Point2f> imagePoint_obj;
	std::vector<cv::Point2f> projPoint_obj;
	std::vector<int> isValid_obj; //有効な対応点かどうかのフラグ
	std::vector<cv::Point3f> reconstructPoint;
	gc.getCorrespondAllPoints_ProCam(projPoint_obj, imagePoint_obj, isValid_obj);

	// 対応点の歪み除去
	std::vector<cv::Point2f> undistort_imagePoint_obj;
	std::vector<cv::Point2f> undistort_projPoint_obj;
	cv::undistortPoints(imagePoint_obj, undistort_imagePoint_obj, calib.cam_K, calib.cam_dist);
	cv::undistortPoints(projPoint_obj, undistort_projPoint_obj, calib.proj_K, calib.proj_dist);
	for(int i=0; i<imagePoint_obj.size(); ++i)
	{
		if(isValid_obj[i] == 1)
		{
			undistort_imagePoint_obj[i].x = undistort_imagePoint_obj[i].x * calib.cam_K.at<double>(0,0) + calib.cam_K.at<double>(0,2);
			undistort_imagePoint_obj[i].y = undistort_imagePoint_obj[i].y * calib.cam_K.at<double>(1,1) + calib.cam_K.at<double>(1,2);
			undistort_projPoint_obj[i].x = undistort_projPoint_obj[i].x * calib.proj_K.at<double>(0,0) + calib.proj_K.at<double>(0,2);
			undistort_projPoint_obj[i].y = undistort_projPoint_obj[i].y * calib.proj_K.at<double>(1,1) + calib.proj_K.at<double>(1,2);
		}
		else
		{
			undistort_imagePoint_obj[i].x = -1;
			undistort_imagePoint_obj[i].y = -1;
			undistort_projPoint_obj[i].x = -1;
			undistort_projPoint_obj[i].y = -1;
		}
	}

	// 3次元復元
	std::cout << "3次元復元中…" << std::ends;
	calib.reconstruction(reconstructPoint, undistort_projPoint_obj, undistort_imagePoint_obj, isValid_obj);
	std::cout << "完了" << std::endl;
	return reconstructPoint;
}

// オイラー角を行列に変換
void eular2rot(double yaw,double pitch, double roll, cv::Mat& dest)
{
    double theta = yaw/180.0*CV_PI;
    double pusai = pitch/180.0*CV_PI;
    double phi = roll/180.0*CV_PI;
 
    double datax[3][3] = {{1.0,0.0,0.0}, 
    {0.0,cos(theta),-sin(theta)}, 
    {0.0,sin(theta),cos(theta)}};
    double datay[3][3] = {{cos(pusai),0.0,sin(pusai)}, 
    {0.0,1.0,0.0}, 
    {-sin(pusai),0.0,cos(pusai)}};
    double dataz[3][3] = {{cos(phi),-sin(phi),0.0}, 
    {sin(phi),cos(phi),0.0}, 
    {0.0,0.0,1.0}};

    cv::Mat Rx(3,3,CV_64F,datax);
    cv::Mat Ry(3,3,CV_64F,datay);
    cv::Mat Rz(3,3,CV_64F,dataz);
    cv::Mat rr=Rz*Rx*Ry;

    rr.copyTo(dest);
}

//変換行列の出力
void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

//変換行列をcsvで保存(1行目にT,2行目にR)
void save4x4MatricCSV(const Eigen::Matrix4d & matrix)
{
    ofstream ofs("ICPresult.csv"); //ファイル出力ストリーム
    ofs<< matrix (0, 3) << "," << matrix (1, 3) << "," << matrix (2, 3)<<endl; //T
    ofs<< matrix (0, 0) << "," << matrix (0, 1) << "," << matrix (0, 2) << 
			  matrix (1, 0) << "," << matrix (1, 1) << "," << matrix (1, 2) <<
			  matrix (2, 0) << "," << matrix (2, 1) << "," << matrix (2, 2)  << endl;//R(1行目→2行目→3行目)
}


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
//点群の可視化
void showViewer(PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_icp)
{
  // Visualization
  pcl::visualization::PCLVisualizer viewer ("ICP demo");
  // Create two verticaly separated viewports
  int v1 (0);
  viewer.createViewPort (0.0, 0.0, 1.0, 1.0, v1);

  // Original point cloud is white
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, 255, 255, 255);
  viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in", v1);

  // ICP aligned point cloud is red
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
  viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp", v1);

  // Set background color
  viewer.setBackgroundColor (0, 0, 0, v1);

  // Set camera position and orientation
  viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
  viewer.setSize (1280, 1024);  // Visualiser window size

  // Display the visualiser
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }

}

//初期位置合わせ
void initialEstimation(PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_icp, std::string model_filename)
{

  // Load the object templates specified in the object_templates.txt file
  std::vector<FeatureCloud> object_templates;
  object_templates.resize (0); 
  FeatureCloud template_cloud;
  template_cloud.loadInputCloud (model_filename);
  object_templates.push_back (template_cloud);

  // Preprocess the cloud by...
  // ...removing distant points
  const float depth_limit = 1.0;
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (cloud_in);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, depth_limit);
  pass.filter (*cloud_in);

  // ... and downsampling the point cloud
  const float voxel_grid_size = 0.005f;
  pcl::VoxelGrid<PointT> vox_grid;
  vox_grid.setInputCloud (cloud_in);
  vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
  //vox_grid.filter (*cloud_in); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
  pcl::PointCloud<PointT>::Ptr tempCloud (new pcl::PointCloud<PointT>); 
  vox_grid.filter (*tempCloud);
  cloud_in = tempCloud; 

  // Assign to the target FeatureCloud
  FeatureCloud target_cloud;
  target_cloud.setInputCloud (cloud_in);

  // Set the TemplateAlignment inputs
  TemplateAlignment template_align;
  for (size_t i = 0; i < object_templates.size (); ++i)
  {
    template_align.addTemplateCloud (object_templates[i]);
  }
  template_align.setTargetCloud (target_cloud);

  // Find the best template alignment
  TemplateAlignment::Result best_alignment;
  int best_index = template_align.findBestAlignment (best_alignment);
  const FeatureCloud &best_template = object_templates[best_index];

  // Print the alignment fitness score (values less than 0.00002 are good)
  printf ("Best fitness score: %f\n", best_alignment.fitness_score);

  // Print the rotation matrix and translation vector
  Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);
  Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);

  printf ("\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
  printf ("\n");
  printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

  //model点群を初期位置へ移動
  PointCloudT::Ptr cloud_icp_trans (new PointCloudT);  // Original point cloud
  pcl::transformPointCloud(*cloud_icp, *cloud_icp_trans, best_alignment.final_transformation);
  cloud_icp = cloud_icp_trans;
}

//ICPによる位置検出
int detectPosition(std::string src_file, std::string model_file)
{
  // The point clouds we will be using
  PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
  PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud

  // Defining a rotation matrix and translation vector
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

  //シーン点群とモデル点群の読み込み
  if (pcl::io::loadPLYFile (src_file, *cloud_in) < 0)
  {
    PCL_ERROR ("Error loading cloud %s.\n", src_file);
    return (-1);
  }
  std::cout << "\nLoaded file " << src_file << " (" << cloud_in->size () << " points)" << std::endl;
  if (pcl::io::loadPLYFile (model_file, *cloud_icp) < 0)
  {
    PCL_ERROR ("Error loading cloud %s.\n", model_file);
    return (-1);
  }
  std::cout << "\nLoaded file " << model_file << " (" << cloud_icp->size () << " points)" << std::endl;

  ////シーン点群ダウンサンプリング
  //float size = 0.005;//5mm
  //PointCloudT::Ptr cloud_in_sampled (new PointCloudT);  // ICP output point cloud
  //// Create the filtering object
  //pcl::VoxelGrid<PointT> sor;
  //sor.setInputCloud (cloud_in);
  //sor.setLeafSize (size, size, size);
  //sor.filter (*cloud_in_sampled);
  //std::cout << "\nDown sampled " << src_file << " (" << cloud_in_sampled->size () << " points)" << std::endl;
  ////モデル点群ダウンサンプリング
  //PointCloudT::Ptr cloud_icp_sampled (new PointCloudT);  // ICP output point cloud
  //// Create the filtering object
  //pcl::VoxelGrid<PointT> sor_;
  //sor_.setInputCloud (cloud_icp);
  //sor_.setLeafSize (size, size, size);
  //sor_.filter (*cloud_icp_sampled);
  //std::cout << "\nDown sampled " << model_file << " (" << cloud_icp_sampled->size () << " points)" << std::endl;

  //初期位置合わせ
  initialEstimation(cloud_in, cloud_icp, model_file);

  //点群セット&icp
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  //icp.setMaximumIterations(100);
  icp.setInputSource (cloud_icp);
  icp.setInputTarget (cloud_in);
  icp.align (*cloud_icp);
  if (icp.hasConverged ())
  {
    std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
    transformation_matrix = icp.getFinalTransformation ().cast<double>();
    print4x4Matrix (transformation_matrix);
	save4x4MatricCSV(transformation_matrix);
	//viewer
	showViewer(cloud_in, cloud_icp);
  }
  else
  {
    PCL_ERROR ("\nICP has not converged.\n");
    return (-1);
  }

 return (0);

}

//点群確認
void viewPoints(Calibration calib, const cv::Mat &cam, const std::vector<cv::Point3f> &points)
{
			// 描画
			cv::Mat R = cv::Mat::eye(3,3,CV_64F);
			cv::Mat t = cv::Mat::zeros(3,1,CV_64F);
			int key=0;
			cv::Point3d viewpoint(0.0,0.0,400.0);		// 視点位置
			cv::Point3d lookatpoint(0.0,0.0,0.0);	// 視線方向
			const double step = 50;

			// キーボード操作
			while(true)
			{
				//// 回転の更新
				double x=(lookatpoint.x-viewpoint.x);
				double y=(lookatpoint.y-viewpoint.y);
				double z=(lookatpoint.z-viewpoint.z);
				double pitch =asin(x/sqrt(x*x+z*z))/CV_PI*180.0;
				double yaw   =asin(-y/sqrt(y*y+z*z))/CV_PI*180.0;
				eular2rot(yaw, pitch, 0, R);
				// 移動の更新
				t.at<double>(0,0)=viewpoint.x;
				t.at<double>(1,0)=viewpoint.y;
				t.at<double>(2,0)=viewpoint.z;

				//カメラ画素→3次元点
				calib.pointCloudRender(points, cam, std::string("viewer"), R, t);

				key = cv::waitKey(0);
				if(key=='w')
				{
					viewpoint.y+=step;
				}
				if(key=='s')
				{
					viewpoint.y-=step;
				}
				if(key=='a')
				{
					viewpoint.x+=step;
				}
				if(key=='d')
				{
					viewpoint.x-=step;
				}
				if(key=='z')
				{
					viewpoint.z+=step;
				}
				if(key=='x')
				{
					viewpoint.z-=step;
				}
				if(key=='q')
				{
					break;
				}
			}
}

int main()
{
	WebCamera webcamera(CAMERA_WIDTH, CAMERA_HEIGHT, "WebCamera");
	GRAYCODE gc(webcamera);

	// カメラ画像確認用
	char windowNameCamera[] = "camera";
	cv::namedWindow(windowNameCamera, cv::WINDOW_AUTOSIZE);
	cv::moveWindow(windowNameCamera, 500, 300);

	static bool prjWhite = false;

	// キャリブレーション用
	Calibration calib(10, 7, 48.0);
	std::vector<std::vector<cv::Point3f>>	worldPoints;
	std::vector<std::vector<cv::Point2f>>	cameraPoints;
	std::vector<std::vector<cv::Point2f>>	projectorPoints;
	int calib_count = 0;

	//背景の閾値(mm)
	double thresh = 100.0; //1500

	//背景と対象物の3次元点
	std::vector<cv::Point3f> reconstructPoint_back;
	std::vector<cv::Point3f> reconstructPoint_obj;

		printf("====================\n");
		printf("開始するには何かキーを押してください....\n");
		int command;

		// 白い画像を全画面で投影（撮影環境を確認しやすくするため）
		cv::Mat cam, cam2;
		while(true){
			// trueで白を投影、falseで通常のディスプレイを表示
			if(prjWhite){
				cv::Mat white = cv::Mat(PROJECTOR_WIDTH, PROJECTOR_HEIGHT, CV_8UC3, cv::Scalar(255, 255, 255));
				cv::namedWindow("white_black", 0);
				Projection::MySetFullScrean(DISPLAY_NUMBER, "white_black");
				cv::imshow("white_black", white);
			}

			// 何かのキーが入力されたらループを抜ける
			command = cv::waitKey(33);
			if ( command > 0 ) break;

			cam = webcamera.getFrame();
			cam.copyTo(cam2);

			//見やすいように適当にリサイズ
			cv::resize(cam, cam, cv::Size(), 0.45, 0.45);
			cv::imshow(windowNameCamera, cam);
		}

	//1. キャリブレーションファイル読み込み
	std::cout << "キャリブレーション結果の読み込み中…" << std::endl;
	calib.loadCalibParam("calibration.xml");

	std::cout << "1: 保存済み背景(平滑化済)の読み込み\n2: 背景をスキャン&平滑化して保存" << std::endl;
	int key = cv::waitKey(0);

	while(true)
	{
		if(key == '1')
		{
			//2-1. 背景点群(平滑化済)読み込み
			std::cout << "背景点群の読み込み中…" << std::endl;
			reconstructPoint_back = loadXMLfile("reconstructPoints_camera.xml");

			//確認
			viewPoints(calib, cam2, reconstructPoint_back);

			break;
		}
		else if(key == '2')
		{
			//2-2. 背景スキャン、平滑化して保存//
			std::cout << "背景をスキャンします…" << std::endl;
			std::vector<cv::Point3f> reconstructPoint_back_raw;
			reconstructPoint_back_raw = scanScene(calib, gc);

			std::cout << "背景を平滑化します…" << std::ends;
			//メディアンフィルタによる平滑化
			calib.smoothReconstructPoints(reconstructPoint_back_raw, reconstructPoint_back, 11); //z<0の点はソート対象外にする？
			std::cout << "完了" << std::endl;
			//==保存==//
			cv::FileStorage fs_obj("./reconstructPoints_camera.xml", cv::FileStorage::WRITE);
			write(fs_obj, "points", reconstructPoint_back);
			std::cout << "背景を保存しました…" << std::endl;

			//確認
			viewPoints(calib, cam2, reconstructPoint_back);

			break;
		}
		else 
		{
			key = cv::waitKey(0);
			continue;
		}
	}

	//--待ち--//
	std::cout << "対象物体を置いてください\n準備ができたら何かキーを押してください…" << std::endl;
	cv::waitKey(0);

	//3. 対象物体入りのシーンにグレイコード投影し、3次元復元
	std::cout << "対象物体をスキャンします…" << std::endl;
	reconstructPoint_obj = scanScene(calib, gc);

	//4. 背景差分
	for(int i = 0; i < reconstructPoint_obj.size(); i++)
	{
		//閾値よりも深度の変化が小さかったら、(-1,-1,-1)で埋める
		if(reconstructPoint_obj[i].z == -1 || reconstructPoint_back[i].z == -1 || abs(reconstructPoint_obj[i].z - reconstructPoint_back[i].z) < thresh)
		{
		reconstructPoint_obj[i].x = -1;
		reconstructPoint_obj[i].y = -1;
		reconstructPoint_obj[i].z = -1;
		}
	}

	//5. 法線、Mesh生成
	std::cout << "モデル生成中…" << std::endl;
	//有効な点のみ取りだす(= -1は除く)
	std::vector<cv::Point3f> validPoints;
	for(int n = 0; n < reconstructPoint_obj.size(); n++)
	{
		if(reconstructPoint_obj[n].x != -1) validPoints.emplace_back(cv::Point3f(reconstructPoint_obj[n].x/1000, reconstructPoint_obj[n].y/1000, reconstructPoint_obj[n].z/1000)); //単位をmに
	}
	//法線を求める
	std::vector<cv::Point3f> normalVecs = getNormalVectors(validPoints);
	//メッシュを求める
	pcl::PolygonMesh triangles;
	std::vector<cv::Point3i> meshes = getMeshVectors(validPoints, normalVecs, triangles);


	//6. PLY形式で保存
	std::cout << "モデルを保存します…" << std::endl;
	savePLY_with_normal_mesh(validPoints, normalVecs, meshes, "reconstructPoint_obj.ply");
	//6. OBJ形式で保存
	pcl::io::saveOBJFile("reconstructPoint_obj.obj", triangles); //->Unity上にはRotate(0, 0, 180)で配置
	std::cout << "モデルを保存しました…" << std::endl;


	//7. ICPで位置検出(できない。。)
	//std::cout << "ICPにより位置を検出します…" << std::endl;
	//int result = detectPosition("reconstructPoint_obj.ply", "dora_small.ply");


	std::cout << "終了するには何かキーを押してください" << std::endl;

	cv::waitKey(0);
	return 0;
}
