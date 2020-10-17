#include "stdafx.h"
#include "myclass.h"
#include <iostream> //标准输入输出流
#include <vector>
#include <Eigen/Core>
#include "iterator" //求交并补使用到的迭代器
#include <algorithm>
#include "ReadWrite.h"
#include <stdlib.h>    //malloc  free
#include <cmath>  
#include <limits.h>  
#include <boost/format.hpp>  
#include <fstream> 
#include "MBR.h"
#include "max_min.h"
#include <stdio.h>
#include <string.h>
#include "typedef.h"
#include "time.h"
#include "Eight_neighborhood.h"
#include <iomanip>
#include "max_min.h"

using namespace std;
typedef pcl::PointXYZ PointType;
#define Random(x) (rand() % x)
typedef pcl::PointXYZRGBA PointT;
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;


myclass::myclass()
{
}


myclass::~myclass()
{
}



//【函数说明】Ascii数据转换至PCL的pcd数据
//【输入参数】icloud——输入点云
//【输出参数】cloud——Ascii格式点云
//【返回值】
void myclass::SwitchDataAsciiToPCL(vector<POINT3D> &icloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
	//头文件定义
	cloud->width = (int)icloud.size();  //设置点云数目
	cloud->height = 1;     //设置为无序点云
	cloud->is_dense = false;
	cloud->points.resize(cloud->width*cloud->height);
	//拷贝点云数据
	vector<POINT3D>::iterator pit = icloud.begin();
	for (unsigned i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = pit->x;
		cloud->points[i].y = pit->y;
		cloud->points[i].z = pit->z;
		//cloud->points[i].label = pit->label;
		pit++;
	}
}


void myclass::SwitchDataAsciiToPCLXYZL(vector<POINT3D> &icloud, pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud)
{
	//头文件定义
	cloud->width = (int)icloud.size();  //设置点云数目
	cloud->height = 1;     //设置为无序点云
	cloud->is_dense = false;
	cloud->points.resize(cloud->width*cloud->height);
	//拷贝点云数据
	vector<POINT3D>::iterator pit = icloud.begin();
	for (unsigned i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = pit->x;
		cloud->points[i].y = pit->y;
		cloud->points[i].z = pit->z;
		cloud->points[i].label = pit->label;
		pit++;
	}
}

