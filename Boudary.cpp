#include "stdafx.h"
#include "Boudary.h"
#include <vector>
#include "POINT3D.h"
#include "myclass.h"
#include "PLANE.h"
#include "max_min.h"
#include "common.h"
//#include "triangle.h"

using namespace std;

//ofstream fout;
Boudary::Boudary()
{
}


Boudary::~Boudary()
{
}


//使用的
//【函数说明】点云簇是线、面、球（散乱状判断）
//【输入参数】incloud——输入点云
//【输出参数】scale——相邻两个特征值，大的远远大于小的大的倍数
//【返回值】
void Boudary::PCA_plane_normal(vector<POINT3D>&incloud, PLANE &NormalA)
{
	myclass mc;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	mc.SwitchDataAsciiToPCL(incloud, cloud);
	int cld_sz = cloud->size();
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	normals->resize(cld_sz);
	//计算中心点坐标
	double center_x = 0, center_y = 0, center_z = 0;
	for (int i = 0; i < cld_sz; i++)
	{
		center_x += cloud->points[i].x;
		center_y += cloud->points[i].y;
		center_z += cloud->points[i].z;
	}
	center_x /= cld_sz;
	center_y /= cld_sz;
	center_z /= cld_sz;
	//计算协方差矩阵
	double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
	for (int i = 0; i < cld_sz; i++)
	{
		xx += (cloud->points[i].x - center_x) * (cloud->points[i].x - center_x);
		xy += (cloud->points[i].x - center_x) * (cloud->points[i].y - center_y);
		xz += (cloud->points[i].x - center_x) * (cloud->points[i].z - center_z);
		yy += (cloud->points[i].y - center_y) * (cloud->points[i].y - center_y);
		yz += (cloud->points[i].y - center_y) * (cloud->points[i].z - center_z);
		zz += (cloud->points[i].z - center_z) * (cloud->points[i].z - center_z);
	}
	//大小为3*3的协方差矩阵
	Eigen::Matrix3f covMat(3, 3);
	covMat(0, 0) = xx / cld_sz;
	covMat(0, 1) = covMat(1, 0) = xy / cld_sz;
	covMat(0, 2) = covMat(2, 0) = xz / cld_sz;
	covMat(1, 1) = yy / cld_sz;
	covMat(1, 2) = covMat(2, 1) = yz / cld_sz;
	covMat(2, 2) = zz / cld_sz;

	//求特征值与特征向量
	Eigen::EigenSolver<Eigen::Matrix3f> es(covMat);
	Eigen::Matrix3f val = es.pseudoEigenvalueMatrix();
	Eigen::Matrix3f vec = es.pseudoEigenvectors();

	//找到最小特征值t1
	//自己对特征值进行排序
	vector<double>t_value;
	t_value.push_back(val(0, 0));
	t_value.push_back(val(1, 1));
	t_value.push_back(val(2, 2));
	sort(t_value.begin(), t_value.end());
	double tz1 = t_value[2];
	double tz2 = t_value[1];
	double tz3 = t_value[0];
	int min_i=0; int mid_i = 0; int max_i = 0;
	for (int i=0;i<3;i++)
	{
		if (t_value[0] == val(i, i))
		{
			min_i = i;
		}
		if (t_value[1] == val(i, i))
		{
			mid_i = i;
		}
		if (t_value[2] == val(i, i))
		{
			max_i = i;
		}
	}
	
	//if ((tz1 / tz2 > scale) && (tz2 / tz3 < scale))
	//{
	//	return 0;//表示线(即tz1远远大于tz2,且tz2接近tz3)
	//}
	//else if ((tz1 / tz2 < scale) && (tz2 / tz3 > scale))
	//{
	//	return 1;//表示面(即tz1接近tz2,且tz2远远大于tz3)
	//}
	//else if ((tz1 / tz2 < scale) && (tz2 / tz3 < scale))
	//{
	//	return 2;//表示球，表示散乱的点(即tz1接近tz2,且tz2接近tz3)
	//}

	//算法链接:https://blog.csdn.net/sinat_24206709/article/details/77744591?utm_medium=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-5.channel_param&depth_1-utm_source=distribute.pc_relevant.none-task-blog-BlogCommendFromMachineLearnPai2-5.channel_param
	////下面没有用
	//double t1 = val(0, 0);
 //   int ii = 0;
	//if (t1 > val(1, 1))
	//{
	//    ii = 1;
	//    t1 = val(1, 1);
	//}

	//if (t1 > val(2, 2))
	//{
	//    ii = 2;
	//    t1 = val(2, 2);
	//}
	////最小特征值对应的特征向量v_n
	//  Eigen::Vector3f v(vec(0, ii), vec(1, ii), vec(2, ii));
	////特征向量单位化
	//   v /= v.norm();
	//   for (int i = 0; i < cld_sz; i++)
	//   {
	//	normals->points[i].normal_x = v(0);
	//	normals->points[i].normal_y = v(1);
	//	normals->points[i].normal_z = v(2);
	//	normals->points[i].curvature = t1 / (val(0, 0) + val(1, 1) + val(2, 2));
	//   }
	Eigen::Vector3f v1(vec(0, max_i), vec(1, max_i), vec(2, max_i));
	Eigen::Vector3f v2(vec(0, mid_i), vec(1, mid_i), vec(2, mid_i));
	NormalA.A = v1(1)*v2(2) - v2(1)*v1(2);
	NormalA.B = -(v1(0)*v2(2) - v2(0)*v1(2));
	NormalA.C = v1(0)*v2(1) - v2(0)*v1(1);


	//int ii = 2;
	// //最小特征值对应的特征向量v_n
	//  Eigen::Vector3f v(vec(0, ii), vec(1, ii), vec(2, ii));
 //   //特征向量单位化
	//   v /= v.norm();
	//   NormalA.A = v(0);
	//   NormalA.B = v(1);
	//   NormalA.C = v(2);
	//   NormalA.curvature = val(2, ii) / (val(0, 0) + val(1, 1) + val(2, 2));
}










