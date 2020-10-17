#include "stdafx.h"
#include "ReadWrite.h"
#include "POINT3D.h"
#include <iostream> //��׼���������
//#include <pcl/io/pcd_io.h> //PCL��PCD��ʽ�ļ����������ͷ�ļ�
//#include <pcl/point_types.h> //PCL�Ը��ָ�ʽ�ĵ��֧��ͷ�ļ�
//#include <pcl/io/ply_io.h> //PCL��ply��ʽ�ļ����������ͷ�ļ�
#include <vector>
////#include <pcl/filters/statistical_outlier_removal.h>
////#include <pcl/filters/voxel_grid.h>
//#include "lasreader.h"
#include "fstream"


#include<opencv2/opencv.hpp>
#include<iostream>
#include<time.h>
#include<fstream>


using namespace cv;



SYSTEMTIME _T;
using namespace std;
#define Random(x) (rand() % x)
ReadWrite::ReadWrite()
{
}


ReadWrite::~ReadWrite()
{
}





/*дtxt��ʽ������
points_fileΪ�����·�����ļ���
icloudΪд���ļ�������
*�� �� ֵ����*/
void ReadWrite::write_txtxyz(std::string points_file, std::vector<POINT3D> icloud)
{
	FILE* fp = NULL;
	fp = fopen(points_file.c_str(), "w");
	if (!fp)
		return;
	int count = 0;
	POINT3D point3d;
	//point_3d pt3d;

	/*for (int i = 0; i <icloud.size(); i++)
	{
	if (RawData_mode[i] != 1)
	continue;

	count++;
	}*/

	//fprintf(fp,"%d\n",count);
	for (int i = 0; i < icloud.size(); i++)
	{
		/*if (RawData_mode[i] == 2)
		continue;*/

		point3d = icloud[i];
		//unsigned char psm = point3d.retnum;
		//if (psm > 1) continue;
		fprintf(fp, "%.4lf %.4lf %.4lf\n", point3d.x, point3d.y, point3d.z);
		//fprintf(fp, "%.4lf %.4lf %.4lf %d %d %d %d %d\n", point3d.x, point3d.y, point3d.z,
		//	point3d.r, point3d.g, point3d.b, point3d.intens, point3d.Classification);
	}

	fclose(fp);
}


//ʹ��
/*дtxt��ʽ������
points_fileΪ�����·�����ļ���
icloudΪд���ļ�������
*�� �� ֵ����*/
void ReadWrite::write_txt(std::string points_file, std::vector<POINT3D> icloud)
{
	FILE* fp = NULL;
	fp = fopen(points_file.c_str(), "w");
	if (!fp)
		return;
	int count = 0;
	POINT3D point3d;
	//point_3d pt3d;

	/*for (int i = 0; i <icloud.size(); i++)
	{
	if (RawData_mode[i] != 1)
	continue;

	count++;
	}*/

	//fprintf(fp,"%d\n",count);
	for (int i = 0; i < icloud.size(); i++)
	{
		/*if (RawData_mode[i] == 2)
		continue;*/

		point3d = icloud[i];
		//unsigned char psm = point3d.retnum;
		//if (psm > 1) continue;
		//fprintf(fp, "%.4lf %.4lf %.4lf\n", point3d.x, point3d.y, point3d.z);
		fprintf(fp, "%.4lf %.4lf %.4lf %d %d %d %d %d\n", point3d.x, point3d.y, point3d.z,
			point3d.r, point3d.g, point3d.b, point3d.intens, point3d.Classification);
	}

	fclose(fp);
}




void ReadWrite::write_txtxyzrgb(string points_file, vector<POINT3D>& icloud)
{
	FILE* fp = NULL;
	fp = fopen(points_file.c_str(), "w");
	if (!fp)
		return;
	int count = 0;
	POINT3D point3d;
	//point_3d pt3d;

	/*for (int i = 0; i <icloud.size(); i++)
	{
	if (RawData_mode[i] != 1)
	continue;

	count++;
	}*/

	//fprintf(fp,"%d\n",count);
	for (int i = 0; i < icloud.size(); i++)
	{
		/*if (RawData_mode[i] == 2)
		continue;*/

		point3d = icloud[i];
		//unsigned char psm = point3d.retnum;
		//if (psm > 1) continue;
		//fprintf(fp, "%.4lf %.4lf %.4lf\n", point3d.x, point3d.y, point3d.z);
		fprintf(fp, "%.4lf %.4lf %.4lf %d %d %d\n", point3d.x, point3d.y, point3d.z,
			point3d.r, point3d.g, point3d.b);
	}

	fclose(fp);
}

//�������ݵķ��ౣ��
void ReadWrite::write_txt(std::string points_file, vector<POINT3D> &building,int a,int b)
{
	std::vector<POINT3D> icloud;
	FILE* fp = NULL;
	fp = fopen(points_file.c_str(), "w");
	if (!fp)
		return;
	int count = 0;
	POINT3D point3d;
	//point_3d pt3d;

	/*for (int i = 0; i <icloud.size(); i++)
	{
	if (RawData_mode[i] != 1)
	continue;

	count++;
	}*/

	//fprintf(fp,"%d\n",count);
	for (int i = 0; i < icloud.size(); i++)
	{
		/*if (RawData_mode[i] == 2)
		continue;*/

		point3d = icloud[i];
		//unsigned char psm = point3d.retnum;
		//if (psm > 1) continue;
		fprintf(fp, "%.2lf %.2lf %.2lf %d %d %d %d %d\n", point3d.x, point3d.y, point3d.z,
			point3d.r, point3d.g, point3d.b, point3d.intens, point3d.Classification);
		if (point3d.Classification==a|| point3d.Classification == b)
		{
			point3d.x = icloud[i].x;
			point3d.y = icloud[i].y;
			point3d.z = icloud[i].z;
			building.push_back(point3d);
		}
	}

	fclose(fp);
}





//������˵��������ά����ת��Ϊ���廯��ɫ������
//�����������incloud������������
//�����������ccloud�����������
//������ֵ��
vector<POINT3D> ReadWrite::IndividualRGBpointcloud(vector<vector<POINT3D>> &incloud)
{
	POINT3D point;
	vector<POINT3D> ccloud;
	for (int i = 0; i < incloud.size();i++)
	{
		unsigned short color_R = Random(255);
		unsigned short color_G = Random(255);
		unsigned short color_B = Random(255);
		if (incloud[i].size()>0)
		{
			for (int j= 0; j < incloud[i].size(); j++)
			{
				point.x = incloud[i][j].x;
				point.y = incloud[i][j].y;
				point.z = incloud[i][j].z;
				point.r = color_R;
				point.g = color_G;
				point.b = color_B;
				point.intens = incloud[i][j].intens;
				point.Classification = incloud[i][j].Classification;
				ccloud.push_back(point);
			}
		}
	}
	return ccloud;
}

//������˵�������ļ�txt��д
//�����������points_file��������·��
//�����������incloud������������
//������ֵ��
void ReadWrite::Multiple_files_write_txt(char* points_file, vector<vector<POINT3D>> &incloud)
{
	POINT3D point;
	for (int i = 0; i < incloud.size();i++)
	{
		//���ļ���д�����ӣ�https://blog.csdn.net/qq_29406323/article/details/81282165
		//���ļ�txt��д
		char data_name[MAX_PATH];
		//������data�ļ����£���data\\����ʾ�ڵ�ǰ����Ŀ¼��data�ļ����£�argv[4]
		/*sprintf_s(data_name, "%s%d%s", "E://new/_", i, ".las");*/
		sprintf_s(data_name, "%s%d%s", points_file, i, ".txt");//�����ļ���·��
		ifstream file_name(data_name);//data_name��ŵ��Ǿ�����ļ���·��
		string file_path ;
		file_path.append(data_name);
		//String intxt ;
	
		

		std::vector<POINT3D> icloud;
		for (int j = 0; j < incloud[i].size(); j++)
		{
			point.x = incloud[i][j].x;
			point.y = incloud[i][j].y;
			point.z = incloud[i][j].z;
			point.r = incloud[i][j].r;
			point.g = incloud[i][j].g;
			point.b = incloud[i][j].b;
			point.intens = incloud[i][j].intens;
			point.Classification = incloud[i][j].Classification;
			icloud.push_back(point);
		}
		write_txtxyz(file_path, icloud);

	}
	

}









int  ReadWrite::read_tsk(const char*filename)
{
	ifstream in(filename);
	if (!in) {
		cerr << "Can't open the task file." << endl;
		return -1;
	}

	string line;
	/*vector<string> words;*/
	while (getline(in, line))
		m_whole_tsk_file_path.push_back(line);

	in.close();

	
	return 0;

}

int ReadWrite::read_test(string filename, float parameter[100])
{
	
	ifstream in(filename);
	if (!in) {
		cerr << "Can't open the test file." << endl;
		return -1;
	}
	
	for (int i = 0; i < 100; i++)
		in >> parameter[i];
	/*atoi(argv[3]) = a[0];
	atoi(argv[3]) = a[1];
	atof(argv[3]) = a[2];*/
	in.close();
	std::cout << "��ɲ�������" << endl;
	return 0;

}





//-------------------- -
//���ߣ��f�
//��Դ��CSDN
//ԭ�ģ�https ://blog.csdn.net/hanshuobest/article/details/52525531 
//��Ȩ����������Ϊ����ԭ�����£�ת���븽�ϲ������ӣ�
//
void ReadWrite::readtxt(const char* str, std::vector<POINT3D>&Data, int aa, int bb)
{
	fstream ReadDataTxt;

	float X, Y, Z;                                     //���ڶ�ȡTXT�е�����
	int r, g, b, Classification, Intensity;
	POINT3D param;                                     //����һ�����ڴ洢X,Y,X��Point3D�ṹ��
	ReadDataTxt.open(str);
	while (ReadDataTxt >> X >> Y >> Z >> r >> g >> b >> Classification)
	{
		param.x = X;
		param.y = Y;
		param.z = Z;
		param.r = r;
		param.g = g;
		param.b = b;
		param.Classification = Classification;

		Data.push_back(param);
		if ((aa==Classification)|| (bb == Classification))
		{
			param.x = X;
			param.y = Y;
			param.z = Z;
			param.r = r;
			param.g = g;
			param.b = b;
			param.Classification = Classification;

			Data.push_back(param);
		}
		
	}
	ReadDataTxt.close();
	//return 0;
}



void ReadWrite::readtxt(const char* str, std::vector<POINT3D>&Data)
{
	fstream ReadDataTxt;
	double X, Y, Z;                                     //���ڶ�ȡTXT�е�����
	//int r, g, b, Classification, Intensity;
	POINT3D param;                                     //����һ�����ڴ洢X,Y,X��Point3D�ṹ��
	ReadDataTxt.open(str);
	while (ReadDataTxt >> X >> Y >> Z)
	{
		param.x = X;
		param.y = Y;
		param.z = Z;
		Data.push_back(param);
	}
	ReadDataTxt.close();
	//return 0;
}










#pragma endregion