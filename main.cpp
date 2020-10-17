#include "stdafx.h"
#include "Boudary.h"
#include "CFSFDP.h"
//#include "ClusterAnalysis.h"
#include "Eight_neighborhood.h"
#include "Grid.h"
#include "lasreader.hpp"
#include "laswriter.hpp"
#include <iostream>
//#include "isodata.h"
//#include "image.h"
#include "max_min.h"
#include "myclass.h"
#include "myfunction.h"
#include "MBR.h"
#include "PLANE.h"
#include "PCA1.h"
#include "ReadWrite.h"
//#include "Roof_extraction.h"
#include <fstream>
#include <string>
#include<stdlib.h>
#include"typedef.h"
#include "time.h"
#include <vector>

#define Random(x) (rand() % x)
using namespace std;
typedef std::string String;
#pragma pack(8)

//FormatRecordLength位数的选择
static unsigned GetFormatRecordLength(uint8_t pointFormat)
{
	switch (pointFormat)
	{
	case 0:
		return 20;              //0 - base
	case 1:
		return 20 + 8;          //1 - base + GPS
	case 2:
		return 20 + 6;          //2 - base + RGB
	case 3:
		return 20 + 8 + 6;      //3 - base + GPS + RGB
	case 4:
		return 20 + 8 + 29;     //4 - base + GPS + FWF
	case 5:
		return 20 + 8 + 6 + 29; //5 - base + GPS + FWF + RGB
	case 6:
		return 30;              //6  - base (GPS included)
	case 7:
		return 30 + 6;          //7  - base + RGB
	case 8:
		return 30 + 6 + 2;      //8  - base + RGB + NIR (not used)
	case 9:
		return 30 + 29;         //9  - base + FWF
	case 10:
		return 30 + 6 + 2 + 29; //10 - base + RGB + NIR + FWF
	default:
		assert(false);
		return 0;
	}
}

////整型索引转为rgb
//void intToRGB(const int & index, unsigned short & R, unsigned short & G, unsigned short & B)
//{
//	R = (0xff << 16 & index) >> 16;
//	G = (0xff << 8 & index) >> 8;
//	B = (0xff & index);
//}




int main(int argc, char** argv)
{
	//ClusterAnalysis cla;
	int start = clock();
	///加载点云数据
	myclass mc;
	ReadWrite rw;
	MBR mbr;
	max_min mm;
	//Roof_extraction re;
	myfunction my;
	Grid grid;
	CFSFDP cf;
	Eight_neighborhood en;
	//image ig;
	Boudary bd;
	PCA1 pca;
	CFSFDP cp;
	PLANE plane;
	PLANE NormalB;
	NormalB.A = 0;
	NormalB.B = 0;
	NormalB.C = 1;
	PLANE NormalA;
	//const char* read_txt_path = argv[1]; //"E:/tsk.tsk";
	const char* read_txt_path = argv[1]; //"E:/tsk.tsk";
	//tsk文件的读取
	int c = rw.read_tsk(read_txt_path);
	if (c == -1)//异常判断，确定该tsk是否正常打开
	{
		return 0;
	}
	vector<double >a;
	for (int i = 2; i < rw.m_whole_tsk_file_path.size(); i++)
	{
		double b = atof(rw.m_whole_tsk_file_path[i].c_str());
		a.push_back(b);
	}
	//int a2 = atof(rw.m_whole_tsk_file_path[2].c_str());
	//int a3 = atof(rw.m_whole_tsk_file_path[3].c_str());

	//将对应的路径传给对应的值
	if (rw.m_whole_tsk_file_path.size() > 0)
	{
		//////读
#pragma region	


		vector<POINT3D>ground2;//地面点
		vector<POINT3D>unclassification1;//非地面点
		vector<POINT3D>noise7;//噪声7
		string intxt = rw.m_whole_tsk_file_path[0];
		Point3D point3d;
		vector<POINT3D > icloud;
		//打开las文件
		LASreadOpener lasreadopener;

		
		LASreader* lasreader;
		//string file_name = files[i];
		string file_name = rw.m_whole_tsk_file_path[0];
		const char* file_name_txt = file_name.c_str();
		vector<POINT3D>buildingpoint;
		


		lasreadopener.set_file_name(file_name.c_str());
		//LASreader* lasreader = lasreadopener.open();
		lasreader = lasreadopener.open();
		lasreader->header;
		if (lasreader == NULL)
		{
			printf("Can't open the las file !");
			return 0;
		}
		while (lasreader->read_point())
		{
			//laswriter->write_point(&lasreader->point);
			point3d.x = lasreader->point.X * lasreader->header.x_scale_factor + lasreader->header.x_offset;
			point3d.y = lasreader->point.Y * lasreader->header.y_scale_factor + lasreader->header.y_offset;
			point3d.z = lasreader->point.Z * lasreader->header.z_scale_factor + lasreader->header.z_offset;

			if ((lasreader->point.rgb[0] & 0xFF00) || (lasreader->point.rgb[1] & 0xFF00) || (lasreader->point.rgb[2] & 0xFF00))//如果是16进制就进行转换
			{
				point3d.r = lasreader->point.rgb[0] >> 8;
				point3d.g = lasreader->point.rgb[1] >> 8;
				point3d.b = lasreader->point.rgb[2] >> 8;
			}
			else//否则就不进行转换
			{
				point3d.r = lasreader->point.rgb[0];
				point3d.g = lasreader->point.rgb[1];
				point3d.b = lasreader->point.rgb[2];

			}
			/*point3d.r = lasreader->point.rgb[0] >> 8;
			point3d.g = lasreader->point.rgb[1] >> 8;
			point3d.b = lasreader->point.rgb[2] >> 8;*/

			/*point3d.r = lasreader->point.rgb[0];
			point3d.g = lasreader->point.rgb[1];
			point3d.b = lasreader->point.rgb[2];*/
			point3d.intens = lasreader->point.intensity;
			point3d.Classification = lasreader->point.classification;
			buildingpoint.push_back(point3d);

			
		}
		
		vector<POINT3D>local_buildingpoint;
		POINT3D maxpoint, minpoint;
		if (buildingpoint.size() > 0)
		{
			mm.max_min_calculation(buildingpoint, maxpoint, minpoint);
		}

		for (int i=0;i< buildingpoint.size();i++)
		{
			point3d.x = buildingpoint[i].x - minpoint.x;
			point3d.y = buildingpoint[i].y - minpoint.y;
			point3d.z = buildingpoint[i].z - minpoint.z;
			local_buildingpoint.push_back(point3d);

		}



		

#pragma endregion
		
		
		//ClusterAnalysis myClusterAnalysis;
		if (local_buildingpoint.size() > 0)
		{
			
			vector<vector<POINT3D>>individual_buildingratio1;
			vector<vector<POINT3D>>temp_individual_building = my.Multiscale_adaptation_building_individual_segmentation
			(local_buildingpoint, a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8]);
				//(rw.m_building_points, a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9], a[10], a[11], a[12], a[13], a[14], a[15], a[16], a[17]);
				//(local_buildingpoint, a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7], a[8], a[9], a[10], a[11], a[12], a[13], a[14], a[15], a[16], a[17], a[18], a[19], a[20], a[21], a[22], a[23]);

			for (int j = 0; j < temp_individual_building.size(); j++)
			{
				individual_buildingratio1.push_back(temp_individual_building[j]);
			}
			

			
			
			rw.m_building_points.clear();
			vector<POINT3D>().swap(rw.m_building_points);

			
			//las文件保存
			vector<vector<POINT3D>> m_wirte_point_cloud;
			int number = 0;
			vector<POINT3D>pointcloud;
			for (int i = 0; i < individual_buildingratio1.size(); i++)
			{
				//m_wirte_point_cloud.push_back(individual_buildingratio1[i]);
				number = number + individual_buildingratio1[i].size();
				vector<POINT3D> temp_pointcloud;
				for (int j = 0; j < individual_buildingratio1[i].size(); j++)
				{
					point3d.x = individual_buildingratio1[i][j].x + minpoint.x;
					point3d.y = individual_buildingratio1[i][j].y + minpoint.y;
					point3d.z = individual_buildingratio1[i][j].z + minpoint.z;
					pointcloud.push_back(point3d);
					temp_pointcloud.push_back(point3d);
				}
				m_wirte_point_cloud.push_back(temp_pointcloud);
			}
			individual_buildingratio1.clear();
			vector < vector<POINT3D>>().swap(individual_buildingratio1);
			POINT3D max, min;
			mm.max_min_calculation(pointcloud, max, min);
			pointcloud.clear();
			vector<POINT3D>().swap(pointcloud);
			// 输出las    单体化的建筑物
			LASpoint point;
			string inlas = rw.m_whole_tsk_file_path[1];
			LASwriteOpener laswriteopener;
			//laswriteopener.set_file_name("2.las");
			laswriteopener.set_file_name(inlas.c_str());
			if (!laswriteopener.active()) {
				return EXIT_FAILURE;
			}
			if (strcmpi(&inlas.back(), "z") == 0) {
				laswriteopener.set_format(LAS_TOOLS_FORMAT_LAZ);
			}
			else {
				laswriteopener.set_format(LAS_TOOLS_FORMAT_LAS);
			}
			LASheader lasheader;
			lasheader.max_x = max.x;
			lasheader.max_y = max.y;
			lasheader.max_z = max.z;
			lasheader.min_x = min.x;
			lasheader.min_y = min.y;
			lasheader.min_z = min.z;
			lasheader.x_offset = lasreader->header.x_offset;
			lasheader.y_offset = lasreader->header.y_offset;
			lasheader.z_offset = lasreader->header.z_offset;
			lasheader.x_scale_factor = lasreader->header.x_scale_factor;
			lasheader.y_scale_factor = lasreader->header.y_scale_factor;
			lasheader.z_scale_factor = lasreader->header.z_scale_factor;





			lasheader.point_data_format = lasreader->header.point_data_format;
			if (lasheader.point_data_format < 2) {
				lasheader.point_data_format += 2;
			}
			else if (lasheader.point_data_format == 4) {
				lasheader.point_data_format = 5;
			}
			lasheader.point_data_record_length = GetFormatRecordLength(lasheader.point_data_format);
			LASwriter* laswriter = laswriteopener.open(&lasheader);




			if (!laswriter)
			{
				return EXIT_FAILURE;
			}
			LASpoint laspoint;
			if (!laspoint.init(&lasheader, lasheader.point_data_format, lasheader.point_data_record_length, 0)) {
				return EXIT_FAILURE;
			}
			for (int i = 0; i < m_wirte_point_cloud.size(); i++)
			{
				//这种随机选择颜色的是用来显示的
				unsigned short color_R = Random(255);
				unsigned short color_G = Random(255);
				unsigned short color_B = Random(255);
				

				if (i == m_wirte_point_cloud[i].size() - 1)
				{
					std::cout << " the max index is:" << color_R << "   " << color_G << "   " << color_B << std::endl;
				}
				for (int j = 0; j < m_wirte_point_cloud[i].size(); j++)
				{
					laspoint.set_X((m_wirte_point_cloud[i][j].x - lasreader->header.x_offset) / lasreader->header.x_scale_factor);
					laspoint.set_Y((m_wirte_point_cloud[i][j].y - lasreader->header.y_offset) / lasreader->header.y_scale_factor);
					laspoint.set_Z((m_wirte_point_cloud[i][j].z - lasreader->header.z_offset) / lasreader->header.z_scale_factor);
					laspoint.set_R(color_R);
					laspoint.set_G(color_G);
					laspoint.set_B(color_B);
					laswriter->write_point(&laspoint);
					laswriter->update_inventory(&laspoint);
				}
			}
			lasreader->close();
			delete lasreader;
			laswriter->close();
			delete laswriter;

		}
	}
	//system("PAUSE");
	return 0;

}
