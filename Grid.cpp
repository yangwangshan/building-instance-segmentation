#include "stdafx.h"
#include "Grid.h"
#include "POINT3D.h"
#include "max_min.h"
#include "myclass.h"
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<vector>
#include "ReadWrite.h"
using namespace std;
using namespace cv;

Grid::Grid():isVisisted(false), havepts(false), coi(0), coj(0), cok(0),  grid_r(0), grid_g(0), grid_b(0), grid_intens(0), gird_z(0),
grid_density(0), grid_Ave_height(0), grid_Max_height(0), grid_Max_min_z(0), grid_hsv(0), grid_rgb_SD(0), grid_intens_SD(0), grid_xy_SD(0), grid_density_SD(0)

//,feature(0), IsVisisted(false)//, f(0), z_max(0), z_min(0), g(0), GWxy(0), GWh(0), a(0), b(0), c(0) m_point_n(0), 
{

}


Grid::~Grid()
{
}






//ʹ�õ�
//������˵�����ֲ����Ƹ��������еĵ���(��ά����ͶӰ��ͨ�������ڱȽ��ٵĵ㣬���Ի�øø����ڽ϶�ĵ�)
//�����������incloud--------------------��������������
//�����������smallincloud---------------������Ǿֲ�����
//�����������Gridthreshold--------------�����Ĵ�С
//�����������max------------------------���ֵ
//�����������min------------------------��Сֵ
//�����������unprojectionpoints---------ͶӰ��Χ֮��ʣ��ĵ�
//������ֵ���ֲ����Ƹ��������еĵ���
vector < vector<POINT3D>>Grid::grid2dprojectioninteriorpoint_outpoint(vector<POINT3D> &incloud, vector < vector<POINT3D>> &smallincloud, double& Gridthreshold, vector<POINT3D> &unprojectionpoints)
{
	max_min mm;
	POINT3D point3d;
	vector<vector<POINT3D>>gridpoints;
	vector<vector<POINT3D>>gridoutpoints;
	POINT3D max, min;
    mm.max_min_calculation(incloud, max, min);
	POINT3D tempmax, tempmin;
	for (int i=0;i< smallincloud.size();i++)
	{
		mm.max_min_calculation(smallincloud[i], tempmax, tempmin);
		if (tempmin.x<min.x)
		{
			min.x = tempmin.x;
		}
		if (tempmin.y < min.y)
		{
			min.y = tempmin.y;
		}
	}

	int nx = (int)((max.x - min.x) / Gridthreshold) + 1;
	int ny = (int)((max.y - min.y) / Gridthreshold) + 1;
	
	vector<vector<Grid>>grid2d(nx, vector<Grid>(ny));
	vector<vector<Grid>>smallgrid2d(nx, vector<Grid>(ny));
	for (int i = 0; i < incloud.size(); i++)
	{
		int a = floor((incloud[i].x - min.x) / Gridthreshold);
		int b = floor((incloud[i].y - min.y) / Gridthreshold);
		//grid2d[a][b].m_point_n++;//ͳ�Ƹ����ڵ���
		grid2d[a][b].m_index.push_back(i);//ÿ�������ڵ��Ƶ�����
		//point3d.x = incloud[i].x;
		//point3d.y = incloud[i].y;
		//point3d.z = incloud[i].z;
		//grid2d[a][b].Meshpts.push_back(point3d);//����Ҳ���ԣ���һ��n�����Ǹ����ݺ���
	}
	int cc = 0;
	vector<POINT3D>tempgridpoint;
	for (int i = 0; i < smallincloud.size(); i++)
	{
		POINT3D max1, min1;
		mm.max_min_calculation(smallincloud[i], max1, min1);
		for (int j = 0; j < smallincloud[i].size(); j++)
		{
			int a = floor((smallincloud[i][j].x - min.x) / Gridthreshold);
			int b = floor((smallincloud[i][j].y - min.y) / Gridthreshold);
			smallgrid2d[a][b].m_index.push_back(i);//ÿ�������ڵ��Ƶ�����
			

		}
	
		for (int m = 0; m < nx; m++)
		{
			for (int n = 0; n < ny; n++)
			{
				if (smallgrid2d[m][n].m_index.size() > 0)
				{
					for (int k = 0; k < grid2d[m][n].m_index.size(); k++)
					{
						int a = grid2d[m][n].m_index[k];
						if ((incloud[a].z<max1.z)&&(incloud[a].IsVisisted== false))
						{
							incloud[a].IsVisisted = true;
							point3d.x = incloud[a].x;
							point3d.y = incloud[a].y;
							point3d.z = incloud[a].z;
							tempgridpoint.push_back(point3d);
							
						}
					}
					smallgrid2d[m][n].m_index.clear();
				}
			}
		}
		gridpoints.push_back(tempgridpoint);
		tempgridpoint.clear();
		vector<POINT3D>().swap(tempgridpoint);
		//smallgrid2d.clear();
		//vector<vector<Grid>>().swap(smallgrid2d);
	}
	grid2d.clear();
	vector<vector<Grid>>().swap(grid2d);
	smallgrid2d.clear();
	vector<vector<Grid>>().swap(smallgrid2d);
	for (int i=0;i< incloud.size();i++)
	{
		if (incloud[i].IsVisisted== false)
		{
			point3d.x = incloud[i].x;
			point3d.y = incloud[i].y;
			point3d.z = incloud[i].z;
			unprojectionpoints.push_back(point3d);
		}
	}
	return  gridpoints;
	
	
}


//������˵���������ܶȵļ��㣨����ƽ���ܶ�+3���ı�׼�
//�����������incloud--------------------��������������
//�����������Gridthreshold--------------�����Ĵ�С
//�����������max------------------------���ֵ
//�����������min------------------------��Сֵ
//������ֵ��ÿƽ����Ƶĸ���
double Grid::KNN_dc_noconvergence(vector<POINT3D> &incloud)
{
	myclass mc;
	POINT3D point3d;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
	mc.SwitchDataAsciiToPCL(incloud, cloud_input);//ʹ�õĸ�ʽתΪpcl��ʽ�ĵ���
	pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
	kdtree.setInputCloud(cloud_input);
	pcl::PointXYZ searchPoint;
	vector<double>distance;
	vector<double>distance00;
	for (int i = 0; i < cloud_input->size(); i++)
	{
		searchPoint.x = cloud_input->points[i].x;
		searchPoint.y = cloud_input->points[i].y;
		searchPoint.z = cloud_input->points[i].z;
		// k��������
		int K = 2;
		std::vector<int>pointIdxNKNSearch(K);
		std::vector<float>pointNKNSquaredDistance(K);
		if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			distance00.push_back(pointNKNSquaredDistance[0]);
			int a = 0;
			for (int j = 0; j < pointNKNSquaredDistance.size(); j++)
			{
				distance.push_back(pointNKNSquaredDistance[1]);
			}
		}
	}

	double sum = 0;
	for (int i = 0; i < distance.size(); i++)
	{
		sum = sum + distance[i];
	}
	double mean = sum / distance.size();

	double aef = 0;
	for (int i = 0; i < distance.size(); i++)
	{
		aef = aef + (distance[i] - mean)*(distance[i] - mean);
	}
	double aef_mean = sqrt(aef / distance.size());
	double point_distance = mean + (3 * aef_mean);
	return point_distance;

}




