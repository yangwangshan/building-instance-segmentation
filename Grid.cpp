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






//使用的
//【函数说明】局部点云格网内所有的点云(二维格网投影，通过格网内比较少的点，可以获得该格网内较多的点)
//【输入参数】incloud--------------------输入的是整体点云
//【输入参数】smallincloud---------------输入的是局部点云
//【输入参数】Gridthreshold--------------格网的大小
//【输入参数】max------------------------最大值
//【输入参数】min------------------------最小值
//【输出参数】unprojectionpoints---------投影范围之外剩余的点
//【返回值】局部点云格网内所有的点云
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
		//grid2d[a][b].m_point_n++;//统计格网内点数
		grid2d[a][b].m_index.push_back(i);//每个格网内点云的索引
		//point3d.x = incloud[i].x;
		//point3d.y = incloud[i].y;
		//point3d.z = incloud[i].z;
		//grid2d[a][b].Meshpts.push_back(point3d);//这种也可以，少一个n返回那个数据好呢
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
			smallgrid2d[a][b].m_index.push_back(i);//每个格网内点云的索引
			

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


//【函数说明】点云密度的计算（点云平均密度+3倍的标准差）
//【输入参数】incloud--------------------输入的是整体点云
//【输入参数】Gridthreshold--------------格网的大小
//【输入参数】max------------------------最大值
//【输入参数】min------------------------最小值
//【返回值】每平面点云的个数
double Grid::KNN_dc_noconvergence(vector<POINT3D> &incloud)
{
	myclass mc;
	POINT3D point3d;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>);
	mc.SwitchDataAsciiToPCL(incloud, cloud_input);//使用的格式转为pcl格式的点云
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
		// k近邻搜索
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




