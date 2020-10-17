#include "stdafx.h"
#include "PLANE.h"
#include "POINT3D.h"
#include "max_min.h"
#include "MBR.h"
#include "myclass.h"
#include "ReadWrite.h"
//#include <pcl/point_cloud.h>        //点类型定义头文件
//#include <pcl/kdtree/kdtree_flann.h> //kdtree类定义头文件
//#include<pcl/kdtree/io.h>
#include<iostream>
#include<vector>
#include<algorithm>
//#include <ros/ros.h>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/common/common.h>
#include "ReadWrite.h"
//#include "ORSA_SV.h"
//#include "Roof_extraction.h"
#include "projection.h"
#include "Grid.h"
#include "Eight_neighborhood.h"
#include "Boudary.h"
#include "Grid.h"
#include"common.h"
#include"CFSFDP.h"
#include"PCA1.h"
//#include <pcl/octree/octree.h>
#include <set>
#define DEBUG
#define RELEASE
using namespace std;

typedef pcl::PointXYZ PointT;
typedef std::vector< pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > AlignedPointTVector;   ///分配内存对齐Eigen::MatrixXf;

//#ifdef DEBUG
//#pragma  comment(lib,"/Debug/ORSA_SV.lib")
//#else
//#pragma  comment(lib,"/Release/ORSA_SV.lib")
//#endif

//#ifdef DEBUG
////#pragma  comment(lib,"libname.lib")
//#endif
//#ifdef RELEASE
////#pragma  comment(lib,"libname.lib")
//#endif


//#include "myclass.h"
//#include <iostream>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
#define PI 3.141592653
PLANE::PLANE() :A(0), B(0), C(0), D(0),curvature(0)
{
}


PLANE::~PLANE()
{
}







//【函数说明】空间向量之间的夹角
//【输入参数】NormalA——某个平面的法向量
//【输入参数】NormalB——Z轴正方向
//【返回值】两个向量之间的夹角
double PLANE::PlaneNormalsAngle3D(PLANE NormalA, PLANE NormalB)
{
	double angle;
	double AA = NormalA.A*NormalA.A + NormalA.B*NormalA.B + NormalA.C*NormalA.C;
	double BB = NormalB.A*NormalB.A + NormalB.B*NormalB.B + NormalB.C*NormalB.C;
	double AABB = sqrt(AA*BB);
	double AB = NormalA.A*NormalB.A + NormalA.B*NormalB.B + NormalA.C*NormalB.C;
	if (AABB==0)
	{
		angle = 0;
	}
	else if ((AB/AABB)>=0)
	{
		double sum = AB / AABB;
		angle = (acos(sum)) * 180 / PI;
	}
	else
	{
		double sum = AB / AABB;
		angle = (acos(sum)) * 180 / PI;
	}
	
	/*if (angle > 90)
	{
		angle = 180 - angle;
	}
	else
	{
		angle = angle;
	}*/
	return angle;
}




//使用
//【函数说明】屋顶细节结构的合并
//【输入参数】redundantpoint-----------剩余的没有添加到单体化建筑物里面的点
//【输出参数】single_building----------已经单体化的建筑物点
//【输入参数】min_cluster_size---------欧式距离聚类最少点数
//【输出参数】max_cluster_size---------欧式距离聚类最多点数
//【输入参数】mintolerance2d-----------欧氏距离聚类聚类间距
//【输出参数】height-------------------高度阈值差（剩余的点Z值的高大于屋顶高小于该阈值，丙炔在屋顶最小外接矩形内）
//【输出参数】againredundantpoint------没有被单体化对应建筑物上的点云
//【返回值】单体化的建筑物
vector<vector<POINT3D>> PLANE::redundantpoint_single_building_MBR_new(vector<vector<POINT3D>>&redundantpoint, vector<vector<POINT3D>>&single_building,
	 double maxtolerance2d,  double heightmax, double heightmin,double mbrbuffer, vector<vector<POINT3D>>&againredundantpoint,
	double small_width_longth,double u2d)
{
	myclass mc;
	max_min mm;
	MBR mbr;
	CFSFDP cf;
	vector<POINT3D>buildingmax; vector<POINT3D>buildingmin;
	vector<vector<POINT3D>> merging_building;
	//double s_mintolerance2d = mintolerance2d / 2;
	//int a = redundantpoint.size();//如果有效果不好的，说明在聚类的时候较大的聚在一起了，之前的没有分开
	vector<vector<POINT3D>> cluster_points;
	vector<vector<int>>index;
	for (int i = 0; i < redundantpoint.size(); ++i)
	{

		vector<vector<POINT3D>> cluster = cf.Shared_nearest_neighborclustering2d(redundantpoint[i], maxtolerance2d, u2d);
		//vector<vector<POINT3D>> cluster = mc.OnEuclideanClusterExtraction3D(redundantpoint[i], min_cluster_size, max_cluster_size, maxtolerance2d, index);
		for (int j = 0; j < cluster.size(); ++j)
		{
			cluster_points.push_back(cluster[j]);
		}
		cluster.clear();
		vector<vector<POINT3D>>().swap(cluster);
	}
	redundantpoint.clear();
	vector <vector<POINT3D>>().swap(redundantpoint);
	//不应该用最小外接矩形，因为在求小结构的最小外接矩形与屋顶旋转的角度不一致，不能用最小外接矩形了
	/*vector<POINT3D>buildingmax;
	vector<POINT3D>buildingmin;*/
	POINT3D point3d;
	POINT3D max;
	POINT3D min;
	/*vector<vector<POINT3D>>single_building;
	for (int i = 0; i < m_single_building.size(); ++i)
	{
		single_building.push_back(m_single_building[i]);
	}*/
	for (int i = 0; i < single_building.size(); ++i)
	{
		if (single_building[i].size() > 0)
		{
			mm.max_min_calculation(single_building[i], max, min);
			point3d.x = max.x;
			point3d.y = max.y;
			point3d.z = max.z;
			buildingmax.push_back(point3d);
			point3d.x = min.x;
			point3d.y = min.y;
			point3d.z = min.z;
			buildingmin.push_back(point3d);
		}

	}
	vector<POINT3D>redundantmax;
	vector<POINT3D>redundantmin;
	for (int i = 0; i < cluster_points.size(); ++i)
	{
		if (cluster_points[i].size() > 0)
		{
			POINT3D max, min;
			mm.max_min_calculation(cluster_points[i], max, min);
			point3d.x = max.x;
			point3d.y = max.y;
			point3d.z = max.z;
			redundantmax.push_back(point3d);
			point3d.x = min.x;
			point3d.y = min.y;
			point3d.z = min.z;
			redundantmin.push_back(point3d);
		}

	}
	vector<vector<POINT3D>> tempmerging;
	vector<int>ii;
	vector<int>jj;
	for (int i = 0; i < cluster_points.size(); i++)
	{
		//for (int j=0;j< single_building.size();++j)
		for (int j = single_building.size() - 1; j >= 0; j--)
		{
			if (((buildingmax[j].x + mbrbuffer) > redundantmax[i].x) && ((buildingmin[j].x - mbrbuffer) < redundantmin[i].x))
			{
				if (((buildingmax[j].y + mbrbuffer) > redundantmax[i].y) && ((buildingmin[j].y - mbrbuffer) < redundantmin[i].y))
				{
					if (((redundantmax[i].z - buildingmax[j].z) > heightmin) && ((redundantmax[i].z - buildingmax[j].z) < heightmax))
					{
						ii.push_back(i);
						jj.push_back(j);
						/*double longth = redundantmax[i].x - redundantmin[i].x;
						double wide = redundantmax[i].y - redundantmin[i].y;
						if ((longth< small_width_longth)&&(wide<small_width_longth))
						{
							ii.push_back(i);
							jj.push_back(j);
						}*/

					}
				}
			}
		}
	}
	vector <vector<POINT3D>>small_roof_building;
	for (int i = 0; i < ii.size(); i++)
	{
		int a = ii[i];
		small_roof_building.push_back(cluster_points[a]);
		cluster_points[a].clear();

	}
	for (int i=0;i< cluster_points.size();i++)
	{
		if (cluster_points[i].size()>0)
		{
			againredundantpoint.push_back(cluster_points[i]);
			cluster_points[i].clear();
		}
	}
	cluster_points.clear();
	vector <vector<POINT3D>>().swap(cluster_points);
	//vector <vector<POINT3D>>m_totalmergesingle_building = kdtree_mergesingle_building(small_roof_building, single_building, maxtolerance2d, mintolerance2d);
	vector <vector<POINT3D>>m_totalmergesingle_building = kdtree_mergesingle_building_knn(small_roof_building, single_building);
	small_roof_building.clear();
	vector <vector<POINT3D>>().swap(small_roof_building);
	single_building.clear();
	vector <vector<POINT3D>>().swap(single_building);

	return m_totalmergesingle_building;

}


//使用
//将小屋顶判断为单一建筑物结构
vector<vector<POINT3D>> PLANE::small_individual_building(vector<vector<POINT3D>>incloud,int individual_number2d,double ss_Width_long,
	vector<vector<POINT3D>>&buildingvertex, vector<POINT3D > &buildingmax, vector<POINT3D > &buildingmin , vector<vector<POINT3D>>&otherpoints, double rotation_angle)
{
	MBR mbr;
	max_min mm;
	PCA1  pca;
	vector<vector<POINT3D>>individual_building;
	for (int i=0;i<incloud.size();++i)
	{
		if (incloud[i].size()> individual_number2d )
		{
			double angle = 0;
			double width = 0;
			double length = 0;
			//vector<POINT3D> tempvertex = pca.PCAalgorithm(incloud[i], length, width, angle);
			vector<POINT3D> tempvertex = mbr.ImproveMBRRectangle(incloud[i], rotation_angle, angle);
			width = sqrt((tempvertex[0].x - tempvertex[1].x)*(tempvertex[0].x - tempvertex[1].x) + (tempvertex[0].y - tempvertex[1].y)*(tempvertex[0].y - tempvertex[1].y));
			length = sqrt((tempvertex[0].x - tempvertex[3].x)*(tempvertex[0].x - tempvertex[3].x) + (tempvertex[0].y - tempvertex[3].y)*(tempvertex[0].y - tempvertex[3].y));
			
				if ((width > ss_Width_long) && (length > ss_Width_long) )
				{
					individual_building.push_back(incloud[i]);
					buildingvertex.push_back(tempvertex);
					POINT3D max, min;
					mm.max_min_calculation(incloud[i], max, min);
					POINT3D point3d;
					point3d.x = max.x;
					point3d.y = max.y;
					point3d.z = max.z;
					buildingmax.push_back(point3d);
					point3d.x = min.x;
					point3d.y = min.y;
					point3d.z = min.z;
					buildingmin.push_back(point3d);
				}
				else
				{
					otherpoints.push_back(incloud[i]);
				}
				tempvertex.clear();
				vector<POINT3D>().swap(tempvertex);
				
		}
		else
		{
			otherpoints.push_back(incloud[i]);
		}
	}

	return individual_building;
}






//【函数说明】利用kdtree，根据最近点原则，将
//【输入参数】otherpoints2——噪点（零碎的建筑物点云）
//【输入参数】individual_building——单一建筑物点云
//【输入参数】gridmax2d-----------搜索最大值
//【输入参数】gridmin2d-----------搜索最小值，如果搜索范围最大值与最小值范围内都没有点，则邻近的一个点为搜索的目标
//【返回值】合并后的建筑物点云
vector < vector<POINT3D>> PLANE::kdtree_mergesingle_building_knn(vector < vector<POINT3D>>otherpoints2, vector < vector<POINT3D>>individual_building)
{
	myclass mc;
	POINT3D point3d;
	max_min mm;
	vector<POINT3D>noise;
	for (int i = 0; i < otherpoints2.size(); i++)
	{
		for (int j = 0; j < otherpoints2[i].size(); j++)
		{
			point3d.x = otherpoints2[i][j].x;
			point3d.y = otherpoints2[i][j].y;
			point3d.z = otherpoints2[i][j].z;
			noise.push_back(point3d);
		}
	}
	vector<POINT3D>building;
	for (int i = 0; i < individual_building.size(); i++)
	{
		for (int j = 0; j < individual_building[i].size(); j++)
		{
			point3d.x = individual_building[i][j].x;
			point3d.y = individual_building[i][j].y;
			point3d.z = individual_building[i][j].z;
			point3d.label = i;
			building.push_back(point3d);
		}
	}
	pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZL>);

	mc.SwitchDataAsciiToPCLXYZL(building, cloud_input);//使用的格式转为pcl格式的点云
	pcl::KdTreeFLANN<pcl::PointXYZL>kdtree;
	kdtree.setInputCloud(cloud_input);
	vector<POINT3D>temppoint;

	for (int i = 0; i < otherpoints2.size(); i++)
	{
		for (int j = 0; j < otherpoints2[i].size(); j++)
		{
			/*POINT3D max, min;
			mm.max_min_calculation(otherpoints2[i], max, min);*/
			pcl::PointXYZL searchPoint;
			searchPoint.x = otherpoints2[i][j].x;
			searchPoint.y = otherpoints2[i][j].y;
			searchPoint.z = otherpoints2[i][j].z;
			vector<int>label1;
			int K = 1;
			std::vector<int>pointIdxNKNSearch(K);
			std::vector<float>pointNKNSquaredDistance(K);
			if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
			{
				int a = pointIdxNKNSearch[0];
				//label1.push_back(building[a].label);
				label1.push_back(cloud_input->points[a].label);
				
			}
			//for (int j = 0; j < otherpoints2[i].size(); j++)
			//{
			//	point3d.x = otherpoints2[i][j].x;// cloud_input->points[pointIdxNKNSearch[i]].x;
			//	point3d.y = otherpoints2[i][j].y; //cloud_input->points[pointIdxNKNSearch[i]].y;
			//	point3d.z = otherpoints2[i][j].z; //cloud_input->points[pointIdxNKNSearch[i]].z;
			//	point3d.label = label1[0];
			//	building.push_back(point3d);
			//}
			//label1.clear();
			//vector<int>().swap(label1);
			point3d.x = otherpoints2[i][j].x;// cloud_input->points[pointIdxNKNSearch[i]].x;
			point3d.y = otherpoints2[i][j].y; //cloud_input->points[pointIdxNKNSearch[i]].y;
			point3d.z = otherpoints2[i][j].z; //cloud_input->points[pointIdxNKNSearch[i]].z;
			point3d.label = label1[0];
			building.push_back(point3d);
		}

	}

	int cluser_number = individual_building.size();
	vector < vector<POINT3D>>totalindividual_building(cluser_number);
	for (int i = 0; i < building.size(); i++)
	{
		int a = building[i].label;
		point3d.x = building[i].x;
		point3d.y = building[i].y;
		point3d.z = building[i].z;
		totalindividual_building[a].push_back(point3d);
	}
	building.clear();
	vector<POINT3D>().swap(building);
	cloud_input->clear();
	pcl::PointCloud<pcl::PointXYZL>().swap(*cloud_input);
	return totalindividual_building;
}















//使用的
////【函数说明】Octree-based region growing for point cloud segmentation
////【函数说明】基于八进制的区域增长以进行点云分割
////【输入参数】incloud------------------输入点云数据
////【输入参数】max_number---------每个平面最多含有多少个点
////【输入参数】min_number---------每个平面最少含有多少个点
////【输入参数】Neighbours----------参考邻接点个数
////【输入参数】k-----------是邻域个数，计算点的法向量
////【输入参数】SmoothnessThreshold-----------------（二维和三维）尺度渐变间距[11]
////【输入参数】CurvatureThreshold-----------二维尺度渐变最小间距（循环的时候到0.8左右最好）[10]
////【输入参数】planepoint-----------------（二维和三维）尺度渐变间距[11]noplanepoint
////【输入参数】noplanepoint-----------------（二维和三维）尺度渐变间距[11]
void PLANE::Octree_based_regiongrowing_planesegmentation(vector < POINT3D > &incloud, int max_number,int min_number,int Neighbours,
	int k, double SmoothnessThreshold,double CurvatureThreshold, vector<vector<POINT3D>>&planepoint, vector < POINT3D > &noplanepoint)
{
	myclass mc;
	POINT3D point3d;
	max_min mm;
	POINT3D max, min;
	vector < POINT3D > icloud;
	mm.max_min_calculation(incloud, max, min);
	for (int i=0;i<incloud.size();i++)
	{
		point3d.x = incloud[i].x-min.x;
		point3d.y = incloud[i].y-min.y;
		point3d.z = incloud[i].z-min.z;
		icloud.push_back(point3d);
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	mc.SwitchDataAsciiToPCL(icloud, cloud);
	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	//法向量求解**********************************************
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(k);  //k
	normal_estimator.compute(*normals);
	//基于法向量和曲率的区域生长算法**************************
	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);


	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(min_number);   //min_number
	reg.setMaxClusterSize(max_number);		//点云集最大点数max_number
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(Neighbours);		//参考邻接点个数Neighbours
	reg.setInputCloud(cloud);

	//reg.setIndices (indices);

	reg.setInputNormals(normals);
	double Smoothness = SmoothnessThreshold / 180.0 * M_PI;
	reg.setSmoothnessThreshold(Smoothness);		//平滑阈值SmoothnessThreshold
	reg.setCurvatureThreshold(CurvatureThreshold);		//曲率阈值CurvatureThreshold

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);
	
	int b = 0; 
	for (int i = 0; i < clusters.size(); i++)
	{
		vector<POINT3D>temp_planepoint;
		for (int j=0;j< clusters[i].indices.size();j++)
		{
			b = clusters[i].indices[j];
			point3d.x = icloud[b].x + min.x;
			point3d.y = icloud[b].y + min.y;
			point3d.z = icloud[b].z + min.z;
			temp_planepoint.push_back(point3d);
		}
		planepoint.push_back(temp_planepoint);
		temp_planepoint.clear();
		vector<POINT3D>().swap(temp_planepoint);

	}

	//int k = 0;
	//for (int i = 0; i < clusters.size(); i++)
	//{
	//	k = k + clusters[i].indices.size();
	//	/*for (int j=0;j< clusters[i].indices.size();j++)
	//	{

	//	}*/
	//}

	vector<int>clusters_index;
	for (int i = 0; i < clusters.size(); i++)
	{
		vector<int>temp_clusters_index;
		for (int j = 0; j < clusters[i].indices.size(); j++)
		{
			temp_clusters_index.push_back(clusters[i].indices[j]);
		}
		for (int j = 0; j < temp_clusters_index.size(); j++)
		{
			clusters_index.push_back(temp_clusters_index[j]);
		}
		temp_clusters_index.clear();
		vector<int>().swap(temp_clusters_index);
	}
	sort(clusters_index.begin(), clusters_index.end());
	int kk = 0;
	/*vector<int> inner_index;
	vector<int> out_index;*/

	for (int i=0;i< icloud.size();i++)
	{
		if (kk< clusters_index.size())
		{
			int a = clusters_index[kk];
			if (i != a)
			{
				point3d.x = icloud[i].x + min.x;
				point3d.y = icloud[i].y + min.y;
				point3d.z = icloud[i].z + min.z;
				noplanepoint.push_back(point3d);
			}
			else
			{
				kk++;

			}
		}
		else
		{
			point3d.x = icloud[i].x + min.x;
			point3d.y = icloud[i].y + min.y;
			point3d.z = icloud[i].z + min.z;
			noplanepoint.push_back(point3d);
		}
		
	}

//代码链接：https://blog.csdn.net/weixin_42795525/article/details/99653785
}


//使用的
void PLANE::roofpoint(vector < POINT3D > &incloud, int max_number, int min_number, int Neighbours,
	int k, double SmoothnessThreshold, double CurvatureThreshold, double angle_threshold, vector < POINT3D >& roofpoint, 
	vector < POINT3D >& facadepoint, double maxtolerance3d, double u3d)
	
{
	PLANE plane;
	POINT3D point3d;
	Boudary bd;
	PLANE NormalB;
	NormalB.A = 0;
	NormalB.B = 0;
	NormalB.C = 1;
	PLANE NormalA;
	CFSFDP cf;
	vector<vector<POINT3D>>planepoint;
	vector < POINT3D > noplanepoint;
	vector<vector<POINT3D>>out_cloud;
	//vector<double >m_angle;
	plane.Octree_based_regiongrowing_planesegmentation(incloud, max_number, min_number, Neighbours, k, SmoothnessThreshold, CurvatureThreshold, planepoint, noplanepoint);
	//vector < POINT3D > tempfacadepoint;
	for (int i = 0; i < planepoint.size(); i++)
	{
		bd.PCA_plane_normal(planepoint[i], NormalA);
		double angle = plane.PlaneNormalsAngle3D(NormalA, NormalB);//明天往下继续
		//m_angle.push_back(angle);
		if (angle > 90)
		{
			if ((angle - 90) > angle_threshold)
			{
				for (int j=0;j< planepoint[i].size();j++)
				{
					point3d.x = planepoint[i][j].x;
					point3d.y = planepoint[i][j].y;
					point3d.z = planepoint[i][j].z;
					roofpoint.push_back(point3d);
				}
			}
			else
			{
				for (int j = 0; j < planepoint[i].size(); j++)
				{
					point3d.x = planepoint[i][j].x;
					point3d.y = planepoint[i][j].y;
					point3d.z = planepoint[i][j].z;
					facadepoint.push_back(point3d);
				}
			}
		}
		else
		{
			if ((90 - angle) > angle_threshold)
			{
				for (int j = 0; j < planepoint[i].size(); j++)
				{
					point3d.x = planepoint[i][j].x;
					point3d.y = planepoint[i][j].y;
					point3d.z = planepoint[i][j].z;
					roofpoint.push_back(point3d);
				}
			}
			else
			{
				for (int j = 0; j < planepoint[i].size(); j++)
				{
					point3d.x = planepoint[i][j].x;
					point3d.y = planepoint[i][j].y;
					point3d.z = planepoint[i][j].z;
					facadepoint.push_back(point3d);
				}
			}
		}
	}
	//非平面点，共享邻域聚类
	//noplanepoint
	if (noplanepoint.size()>0)
	{
		vector<vector<POINT3D>>cluster_points = cf.Shared_nearest_neighborclustering(noplanepoint, maxtolerance3d, u3d);
		/*int sum = 0;
		for (int i = 0; i < cluster_points.size(); i++)
		{
			sum = sum + cluster_points[i].size();
		}*/
		//vector<POINT3D > facadepoint1;
		for (int i = 0; i < cluster_points.size(); i++)
		{
			if (cluster_points[i].size() > min_number)
			{
				bd.PCA_plane_normal(cluster_points[i], NormalA);
				double angle = plane.PlaneNormalsAngle3D(NormalA, NormalB);//明天往下继续
				//m_angle.push_back(angle);
				if (angle > 90)
				{
					if ((angle - 90) > angle_threshold)
					{
						for (int j = 0; j < cluster_points[i].size(); j++)
						{
							point3d.x = cluster_points[i][j].x;
							point3d.y = cluster_points[i][j].y;
							point3d.z = cluster_points[i][j].z;
							roofpoint.push_back(point3d);
						}
					}
					else
					{
						for (int j = 0; j < cluster_points[i].size(); j++)
						{
							point3d.x = cluster_points[i][j].x;
							point3d.y = cluster_points[i][j].y;
							point3d.z = cluster_points[i][j].z;
							facadepoint.push_back(point3d);
						}
					}
				}
				else
				{
					if ((90 - angle) > angle_threshold)
					{
						for (int j = 0; j < cluster_points[i].size(); j++)
						{
							point3d.x = cluster_points[i][j].x;
							point3d.y = cluster_points[i][j].y;
							point3d.z = cluster_points[i][j].z;
							roofpoint.push_back(point3d);
						}
					}
					else
					{
						for (int j = 0; j < cluster_points[i].size(); j++)
						{
							point3d.x = cluster_points[i][j].x;
							point3d.y = cluster_points[i][j].y;
							point3d.z = cluster_points[i][j].z;
							facadepoint.push_back(point3d);
						}
					}
				}
			}
			else
			{
				for (int j = 0; j < cluster_points[i].size(); j++)
				{
					point3d.x = cluster_points[i][j].x;
					point3d.y = cluster_points[i][j].y;
					point3d.z = cluster_points[i][j].z;
					facadepoint.push_back(point3d);
				}
			}

		}
	}
	
	
}

