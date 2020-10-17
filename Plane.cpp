#include "stdafx.h"
#include "PLANE.h"
#include "POINT3D.h"
#include "max_min.h"
#include "MBR.h"
#include "myclass.h"
#include "ReadWrite.h"
//#include <pcl/point_cloud.h>        //�����Ͷ���ͷ�ļ�
//#include <pcl/kdtree/kdtree_flann.h> //kdtree�ඨ��ͷ�ļ�
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
typedef std::vector< pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > AlignedPointTVector;   ///�����ڴ����Eigen::MatrixXf;

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







//������˵�����ռ�����֮��ļн�
//�����������NormalA����ĳ��ƽ��ķ�����
//�����������NormalB����Z��������
//������ֵ����������֮��ļн�
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




//ʹ��
//������˵�����ݶ�ϸ�ڽṹ�ĺϲ�
//�����������redundantpoint-----------ʣ���û����ӵ����廯����������ĵ�
//�����������single_building----------�Ѿ����廯�Ľ������
//�����������min_cluster_size---------ŷʽ����������ٵ���
//�����������max_cluster_size---------ŷʽ�������������
//�����������mintolerance2d-----------ŷ�Ͼ�����������
//�����������height-------------------�߶���ֵ�ʣ��ĵ�Zֵ�ĸߴ����ݶ���С�ڸ���ֵ����Ȳ���ݶ���С��Ӿ����ڣ�
//�����������againredundantpoint------û�б����廯��Ӧ�������ϵĵ���
//������ֵ�����廯�Ľ�����
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
	//int a = redundantpoint.size();//�����Ч�����õģ�˵���ھ����ʱ��ϴ�ľ���һ���ˣ�֮ǰ��û�зֿ�
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
	//��Ӧ������С��Ӿ��Σ���Ϊ����С�ṹ����С��Ӿ������ݶ���ת�ĽǶȲ�һ�£���������С��Ӿ�����
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


//ʹ��
//��С�ݶ��ж�Ϊ��һ������ṹ
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






//������˵��������kdtree�����������ԭ�򣬽�
//�����������otherpoints2������㣨����Ľ�������ƣ�
//�����������individual_building������һ���������
//�����������gridmax2d-----------�������ֵ
//�����������gridmin2d-----------������Сֵ�����������Χ���ֵ����Сֵ��Χ�ڶ�û�е㣬���ڽ���һ����Ϊ������Ŀ��
//������ֵ���ϲ���Ľ��������
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

	mc.SwitchDataAsciiToPCLXYZL(building, cloud_input);//ʹ�õĸ�ʽתΪpcl��ʽ�ĵ���
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















//ʹ�õ�
////������˵����Octree-based region growing for point cloud segmentation
////������˵�������ڰ˽��Ƶ����������Խ��е��Ʒָ�
////�����������incloud------------------�����������
////�����������max_number---------ÿ��ƽ����ຬ�ж��ٸ���
////�����������min_number---------ÿ��ƽ�����ٺ��ж��ٸ���
////�����������Neighbours----------�ο��ڽӵ����
////�����������k-----------����������������ķ�����
////�����������SmoothnessThreshold-----------------����ά����ά���߶Ƚ�����[11]
////�����������CurvatureThreshold-----------��ά�߶Ƚ�����С��ࣨѭ����ʱ��0.8������ã�[10]
////�����������planepoint-----------------����ά����ά���߶Ƚ�����[11]noplanepoint
////�����������noplanepoint-----------------����ά����ά���߶Ƚ�����[11]
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
	//���������**********************************************
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setKSearch(k);  //k
	normal_estimator.compute(*normals);
	//���ڷ����������ʵ����������㷨**************************
	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);


	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(min_number);   //min_number
	reg.setMaxClusterSize(max_number);		//���Ƽ�������max_number
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(Neighbours);		//�ο��ڽӵ����Neighbours
	reg.setInputCloud(cloud);

	//reg.setIndices (indices);

	reg.setInputNormals(normals);
	double Smoothness = SmoothnessThreshold / 180.0 * M_PI;
	reg.setSmoothnessThreshold(Smoothness);		//ƽ����ֵSmoothnessThreshold
	reg.setCurvatureThreshold(CurvatureThreshold);		//������ֵCurvatureThreshold

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

//�������ӣ�https://blog.csdn.net/weixin_42795525/article/details/99653785
}


//ʹ�õ�
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
		double angle = plane.PlaneNormalsAngle3D(NormalA, NormalB);//�������¼���
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
	//��ƽ��㣬�����������
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
				double angle = plane.PlaneNormalsAngle3D(NormalA, NormalB);//�������¼���
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

