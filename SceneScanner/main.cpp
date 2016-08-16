#include "Graycode.h"
#include "Header.h"
#include "Calibration.h"
#include "PointCloudIO.h"
#include "PointCloudFilter.h"

#define MASK_ADDRESS "./GrayCodeImage/mask.bmp"
#define IMAGE_DIRECTORY "./UseImage"
#define SAVE_DIRECTORY "./UseImage/resize"

//�X�L��������3�����_�Q�𓾂�
std::vector<cv::Point3f> scanScene(Calibration calib, GRAYCODE gc)
{
	// �O���C�R�[�h���e
	gc.code_projection();
	gc.make_thresh();
	gc.makeCorrespondence();

	//***�Ή��_�̎擾(�J������f��3�����_)******************************************
	std::vector<cv::Point2f> imagePoint_obj;
	std::vector<cv::Point2f> projPoint_obj;
	std::vector<int> isValid_obj; //�L���ȑΉ��_���ǂ����̃t���O
	std::vector<cv::Point3f> reconstructPoint;
	gc.getCorrespondAllPoints_ProCam(projPoint_obj, imagePoint_obj, isValid_obj);

	// �Ή��_�̘c�ݏ���
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

	// 3��������
	std::cout << "3�����������c" << std::ends;
	calib.reconstruction(reconstructPoint, undistort_projPoint_obj, undistort_imagePoint_obj, isValid_obj);
	std::cout << "����" << std::endl;
	return reconstructPoint;
}

int main()
{
	WebCamera webcamera(CAMERA_WIDTH, CAMERA_HEIGHT, "WebCamera");
	GRAYCODE gc(webcamera);

	// �J�����摜�m�F�p
	char windowNameCamera[] = "camera";
	cv::namedWindow(windowNameCamera, cv::WINDOW_AUTOSIZE);
	cv::moveWindow(windowNameCamera, 500, 300);

	static bool prjWhite = false;

	// �L�����u���[�V�����p
	Calibration calib(10, 7, 48.0);
	std::vector<std::vector<cv::Point3f>>	worldPoints;
	std::vector<std::vector<cv::Point2f>>	cameraPoints;
	std::vector<std::vector<cv::Point2f>>	projectorPoints;
	int calib_count = 0;

	//�w�i��臒l(mm)
	double thresh = 100.0; //1500

	//�w�i�ƑΏە���3�����_
	std::vector<cv::Point3f> reconstructPoint_back;
	std::vector<cv::Point3f> reconstructPoint_obj;

		printf("====================\n");
		printf("�J�n����ɂ͉����L�[�������Ă�������....\n");
		int command;

		// �����摜��S��ʂœ��e�i�B�e�����m�F���₷�����邽�߁j
		cv::Mat cam, cam2;
		while(true){
			// true�Ŕ��𓊉e�Afalse�Œʏ�̃f�B�X�v���C��\��
			if(prjWhite){
				cv::Mat white = cv::Mat(PROJECTOR_WIDTH, PROJECTOR_HEIGHT, CV_8UC3, cv::Scalar(255, 255, 255));
				cv::namedWindow("white_black", 0);
				Projection::MySetFullScrean(DISPLAY_NUMBER, "white_black");
				cv::imshow("white_black", white);
			}

			// �����̃L�[�����͂��ꂽ�烋�[�v�𔲂���
			command = cv::waitKey(33);
			if ( command > 0 ) break;

			cam = webcamera.getFrame();
			cam.copyTo(cam2);

			//���₷���悤�ɓK���Ƀ��T�C�Y
			cv::resize(cam, cam, cv::Size(), 0.45, 0.45);
			cv::imshow(windowNameCamera, cam);
		}

	//1. �L�����u���[�V�����t�@�C���ǂݍ���
	std::cout << "�L�����u���[�V�������ʂ̓ǂݍ��ݒ��c" << std::endl;
	calib.loadCalibParam("calibration.xml");

	std::cout << "1: �ۑ��ςݔw�i(��������)�̓ǂݍ���\n2: �w�i���X�L����&���������ĕۑ�" << std::endl;
	int key = cv::waitKey(0);

	while(true)
	{
		if(key == '1')
		{
			//2-1. �w�i�_�Q(��������)�ǂݍ���
			std::cout << "�w�i�_�Q�̓ǂݍ��ݒ��c" << std::endl;
			reconstructPoint_back = loadXMLfile("reconstructPoints_camera.xml");
		}
		else if(key == '2')
		{
			//2-2. �w�i�X�L�����A���������ĕۑ�//
			std::cout << "�w�i���X�L�������܂��c" << std::endl;
			std::vector<cv::Point3f> reconstructPoint_back_raw;
			reconstructPoint_back_raw = scanScene(calib, gc);

			std::cout << "�w�i�𕽊������܂��c" << std::endl;
			//���f�B�A���t�B���^�ɂ�镽����
			calib.smoothReconstructPoints(reconstructPoint_back_raw, reconstructPoint_back, 11); //z<0�̓_�̓\�[�g�ΏۊO�ɂ���H

			//==�ۑ�==//
			cv::FileStorage fs_obj("./reconstructPoints_smoothed.xml", cv::FileStorage::WRITE);
			write(fs_obj, "points", reconstructPoint_back);
			std::cout << "�w�i��ۑ����܂����c" << std::endl;
		}
		else 
		{
			key = cv::waitKey(0);
			continue;
		}
	}

	//--�҂�--//
	std::cout << "�Ώە��̂�u���Ă�������\n�������ł����牽���L�[�������Ă��������c" << std::endl;
	cv::waitKey(0);

	//3. �Ώە��̓���̃V�[���ɃO���C�R�[�h���e���A3��������
	std::cout << "�Ώە��̂��X�L�������܂��c" << std::endl;
	reconstructPoint_obj = scanScene(calib, gc);

	//4. �w�i����
	for(int i = 0; i < reconstructPoint_obj.size(); i++)
	{
		//臒l�����[�x�̕ω���������������A(-1,-1,-1)�Ŗ��߂�
		if(reconstructPoint_obj[i].z == -1 || reconstructPoint_back[i].z == -1 || abs(reconstructPoint_obj[i].z - reconstructPoint_back[i].z) < thresh)
		{
		reconstructPoint_obj[i].x = -1;
		reconstructPoint_obj[i].y = -1;
		reconstructPoint_obj[i].z = -1;
		}
	}

	//5. �@���AMesh����
	std::cout << "���f���������c" << std::endl;
	//�L���ȓ_�̂ݎ�肾��(= -1�͏���)
	std::vector<cv::Point3f> validPoints;
	for(int n = 0; n < reconstructPoint_obj.size(); n++)
	{
		if(reconstructPoint_obj[n].x != -1) validPoints.emplace_back(cv::Point3f(reconstructPoint_obj[n].x/1000, reconstructPoint_obj[n].y/1000, reconstructPoint_obj[n].z/1000)); //�P�ʂ�m��
	}
	//�@�������߂�
	std::vector<cv::Point3f> normalVecs = getNormalVectors(validPoints);
	//���b�V�������߂�
	std::vector<cv::Point3i> meshes = getMeshVectors(validPoints, normalVecs);

	//6. PLY�`���ŕۑ�
	std::cout << "���f����ۑ����܂��c" << std::endl;
	savePLY_with_normal_mesh(validPoints, normalVecs, meshes, "reconstructPoint_obj.ply");
	std::cout << "���f����ۑ����܂����c\n�I������ɂ͉����L�[�������Ă�������" << std::endl;

	cv::waitKey(0);
	return 0;
}
