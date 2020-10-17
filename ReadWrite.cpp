#include "stdafx.h"
#include "ReadWrite.h"
#include "POINT3D.h"
#include <iostream> //标准输入输出流
//#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
//#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
//#include <pcl/io/ply_io.h> //PCL的ply格式文件的输入输出头文件
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





/*写txt格式的数据
points_file为传入的路径个文件名
icloud为写入文件的数组
*返 回 值：空*/
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


//使用
/*写txt格式的数据
points_file为传入的路径个文件名
icloud为写入文件的数组
*返 回 值：空*/
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

//点云数据的分类保存
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





//【函数说明】将二维数据转化为单体化颜色的数据
//【输入参数】incloud——输入数组
//【输出参数】ccloud——输出数组
//【返回值】
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

//【函数说明】多文件txt的写
//【输入参数】points_file——输入路径
//【输出参数】incloud——输入数组
//【返回值】
void ReadWrite::Multiple_files_write_txt(char* points_file, vector<vector<POINT3D>> &incloud)
{
	POINT3D point;
	for (int i = 0; i < incloud.size();i++)
	{
		//多文件读写的链接：https://blog.csdn.net/qq_29406323/article/details/81282165
		//多文件txt的写
		char data_name[MAX_PATH];
		//数据在data文件夹下（“data\\”表示在当前工作目录的data文件夹下）argv[4]
		/*sprintf_s(data_name, "%s%d%s", "E://new/_", i, ".las");*/
		sprintf_s(data_name, "%s%d%s", points_file, i, ".txt");//整个文件的路径
		ifstream file_name(data_name);//data_name存放的是具体的文件的路径
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
	std::cout << "完成参数设置" << endl;
	return 0;

}





//-------------------- -
//作者：韋頁
//来源：CSDN
//原文：https ://blog.csdn.net/hanshuobest/article/details/52525531 
//版权声明：本文为博主原创文章，转载请附上博文链接！
//
void ReadWrite::readtxt(const char* str, std::vector<POINT3D>&Data, int aa, int bb)
{
	fstream ReadDataTxt;

	float X, Y, Z;                                     //用于读取TXT中的数据
	int r, g, b, Classification, Intensity;
	POINT3D param;                                     //创建一个用于存储X,Y,X的Point3D结构体
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
	double X, Y, Z;                                     //用于读取TXT中的数据
	//int r, g, b, Classification, Intensity;
	POINT3D param;                                     //创建一个用于存储X,Y,X的Point3D结构体
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