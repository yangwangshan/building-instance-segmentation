#include "stdafx.h"
#include "MBR.h"
#include <vector>
#include "max_min.h"
#include<iostream>
#include<algorithm>
#include "myclass.h"
#include "CFSFDP.h"
#include <math.h>
#include "PLANE.h"
#include "time.h"
#include "Eight_neighborhood.h"
#include "ReadWrite.h"
//#include "Roof_extraction.h"
#include "PCA1.h"
//#include "ClusterAnalysis.h"
#include "Boudary.h"
using namespace std;
MBR::MBR()
{
}


MBR::~MBR()
{
}

//【函数说明】每一点incloud绕center旋转theta角度，zf,0706
//【输入参数】icloud——输入点云
//【输入参数】theta——旋转角度
//【返回值】vector<POINT3D>
vector<POINT3D> MBR::rotate(vector<POINT3D>  &incloud, POINT3D center, double theta)//ok
{
	POINT3D point;
	vector<POINT3D>rotatePoint;
	for (int i = 0; i < incloud.size(); ++i)
	{
		double x1 = incloud[i].x;
		double y1 = incloud[i].y;
		double z1 = incloud[i].z;
		double x0 = center.x;
		double y0 = center.y;

		double Q = theta / 180 * 3.1415926;  //角度

		double x2, y2;
		x2 = (x1 - x0)*cos(Q) - (y1 - y0)*sin(Q) + x0;   //旋转公式
		y2 = (x1 - x0)*sin(Q) + (y1 - y0)*cos(Q) + y0;
		point.x = x2;
		point.y = y2;
		point.z = z1;
		rotatePoint.push_back(point);

	}
	return rotatePoint;
}
//【函数说明】(基于二分法的最小外接矩形改进失败)求最小包围盒的顶点坐标、面积、ID
//【输入参数】icloud——输入点云
//【输入参数】theta——每一次旋转的角度
//【输出参数】vertexmax——顶点最大值
//【输出参数】vertexmin——顶点最小值
//【返回值】返回值rectanglevertex为最小外界矩形顶点坐标
vector<POINT3D> MBR::ImproveMBRRectangle(vector<POINT3D> &icloud, double theta, double &m_angle)
{
	max_min mm;
	POINT3D point3d;
	POINT3D max;
	POINT3D min;
	POINT3D max1, min1;
	mm.max_min_calculation(icloud, max, min);
	//POINT3D Centerpoint = FindCenter(icloud);
	POINT3D Centerpoint;
	Centerpoint.x = (max.x + min.x) / 2;
	Centerpoint.y = (max.y + min.y) / 2;
	//Centerpoint.z = (max.z + min.z) / 2;

	double original_area = (max.x - min.x)*(max.y - min.y);

	vector<POINT3D >rotatePoint1 = rotate(icloud, Centerpoint, theta);
	mm.max_min_calculation(rotatePoint1, max, min);
	rotatePoint1.clear();
	vector<POINT3D >().swap(rotatePoint1);
	double temptotal = (max.x - min.x)*(max.y - min.y);

	if (original_area > temptotal)
	{
		for (double j = theta + theta; j < 90; )
		{
			vector<POINT3D >rotatePoint2 = rotate(icloud, Centerpoint, j);
			mm.max_min_calculation(rotatePoint2, max, min);
			rotatePoint2.clear();
			vector<POINT3D >().swap(rotatePoint2);
			double temp = (max.x - min.x)*(max.y - min.y);
			if (temptotal > temp)
			{
				temptotal = temp;
				j = j + theta;
			}
			else
			{
				m_angle = j - theta;
				vector<POINT3D >rotatePoint3 = rotate(icloud, Centerpoint, m_angle);
				mm.max_min_calculation(rotatePoint3, max, min);
				rotatePoint3.clear();
				vector<POINT3D >().swap(rotatePoint3);
				max1.x = max.x;
				max1.y = max.y;
				min1.x = min.x;
				min1.y = min.y;
				break;
			}
		}
	}
	else
	{
		for (double j = -theta; j > -90; )
		{
			vector<POINT3D >rotatePoint = rotate(icloud, Centerpoint, j);
			mm.max_min_calculation(rotatePoint, max, min);
			rotatePoint.clear();
			vector<POINT3D >().swap(rotatePoint);
			double temp = (max.x - min.x)*(max.y - min.y);
			if (temptotal > temp)
			{
				temptotal = temp;
				j = j - theta;
			}
			else
			{
				m_angle = j + theta;
				max1.x = max.x;
				max1.y = max.y;
				min1.x = min.x;
				min1.y = min.y;
				break;
			}

		}
	}
	vector<POINT3D> vertex;
	POINT3D vertex0; POINT3D vertex1;
	POINT3D vertex2; POINT3D vertex3;
	vertex0.x = min1.x;
	vertex0.y = min1.y;
	vertex1.x = max1.x;
	vertex1.y = min1.y;

	vertex2.x = max1.x;
	vertex2.y = max1.y;
	vertex3.x = min1.x;
	vertex3.y = max1.y;

	vertex.push_back(vertex1);
	vertex.push_back(vertex2);
	vertex.push_back(vertex3);
	vertex.push_back(vertex0);//这样调整过来才给之前的顺序一样
	double angle = -(m_angle);
	vector<POINT3D> rectanglevertex = rotate(vertex, Centerpoint, angle);//Centerpoint
	if (m_angle < 0)
	{
		m_angle = m_angle + 90;
	}
	return rectanglevertex;
}


void MBR::MBR_2DMultiscaleShared_nearest_neighborclustering_newsingle(vector < POINT3D>&incloud,  double maxtolerance3d,
	double maxtolerance2d,int individual_number2d,  double ss_width_longth, double rotation_angle, double max_number_area,
	vector < vector<POINT3D>>&individual, vector < vector<POINT3D>>&unindividual, vector < vector<POINT3D>>&noise2d, 
	vector < vector<POINT3D>>&individualvertex, vector < double>& angle2d,   double u2d)
{
	PCA1 pca;
	myclass mc;
	CFSFDP cp;
	max_min mm;
	CFSFDP cf;
	POINT3D point3d;
	Eight_neighborhood en;
	vector<vector<POINT3D>>cluster_points = cf.Shared_nearest_neighborclustering2d(incloud, maxtolerance2d, u2d);
	incloud.clear();
	vector<POINT3D>().swap(incloud);
	vector<vector<POINT3D>>cloud;
	vector<double > ratio1;
	//double i = maxtolerance2d - interval;
	//double i = (maxtolerance2d + mintolerance2d) / 2;
	vector<vector<POINT3D>>tempincloud;
	vector<double > grid3ddratio;
	vector<double > grid3ddratio1;
	for (int j = 0; j < cluster_points.size(); ++j)
	{
		if (cluster_points[j].size() > individual_number2d)
		{
			/*double angle = 0;
			double width = 0;
			double length = 0;
			vector<POINT3D> vertex = pca.PCAalgorithm(cluster_points[j], length, width, angle);*/
			double angle = 0;
			vector<POINT3D> vertex = ImproveMBRRectangle(cluster_points[j], rotation_angle, angle);
			double width = sqrt((vertex[0].x - vertex[1].x)*(vertex[0].x - vertex[1].x) + (vertex[0].y - vertex[1].y)*(vertex[0].y - vertex[1].y));
			double length = sqrt((vertex[0].x - vertex[3].x)*(vertex[0].x - vertex[3].x) + (vertex[0].y - vertex[3].y)*(vertex[0].y - vertex[3].y));
			
			if ((width > ss_width_longth) && (length > ss_width_longth))
			{
				
				POINT3D max, min;
				mm.max_min_calculation(cluster_points[j], max, min);
				double grid3dd = en.grid3d_Cluster_evaluation(cluster_points[j], maxtolerance2d, min);
				
				if (grid3dd < max_number_area)
				{
					unindividual.push_back(cluster_points[j]);
				}
				else
				{
					individual.push_back(cluster_points[j]);
					cluster_points[j].clear();
					individualvertex.push_back(vertex);
					angle2d.push_back(angle);
				}
			}
			else
			{
				noise2d.push_back(cluster_points[j]);
				cluster_points[j].clear();
			}
		}
		else
		{
			noise2d.push_back(cluster_points[j]);
			cluster_points[j].clear();
		}
	}
}













void MBR::MBR_3DMultiscaleShared_nearest_neighborclustering_newsingle(vector < vector<POINT3D>>&cloud, double maxtolerance3d,
    double maxtolerance2d,int individual_number3d,  double small_width_longth, double rotation_angle, double max_number_area,
	vector < vector<POINT3D>>&individual3d, vector < vector<POINT3D>>&unindividual3d, vector < vector<POINT3D>>&noise3d, 
	vector < vector<POINT3D>>&individualvertex, vector < double>& angle3d,  double angle_threshold,  double u3d)
{
	PCA1 pca;
	myclass mc;
	CFSFDP cp;
	max_min mm;
	POINT3D point3d;
	Eight_neighborhood en;
	//Roof_extraction re;
	PLANE plane;
	CFSFDP cf;
	PLANE NormalB;
	NormalB.A = 0;
	NormalB.B = 0;
	NormalB.C = 1;
	vector<vector<POINT3D>>	cluster_pointsenlarge;
	vector < vector<POINT3D>>individual;
	vector < vector<POINT3D>>unindividual;
	vector < vector<POINT3D>>noise;
	for (int i = 0; i < cloud.size(); ++i)
	{
		POINT3D max, min;
		mm.max_min_calculation(cloud[i], max, min);
		vector<POINT3D > facadepoint;
		vector<POINT3D > normalvectorRoof;// = re.normalvectorRoofpointloudsextractionimprove(cloud[i], mintolerance3d, angle_threshold, NormalB);
		int  max_cluster_size = 10000000;
		plane.roofpoint(cloud[i], max_cluster_size, 50, 30, 50, 5, 1, 30, normalvectorRoof, facadepoint, 2, 0.1);
		if (normalvectorRoof.size() > individual_number3d)
		{
			vector<POINT3D>tempenlarge;
			for (int j = 0; j < normalvectorRoof.size(); ++j)
			{
				point3d.x = normalvectorRoof[j].x;
				point3d.y = normalvectorRoof[j].y;
				point3d.z = normalvectorRoof[j].z;// *enlargeenshrink;
				tempenlarge.push_back(point3d);
			}
			normalvectorRoof.clear();
			vector<POINT3D>().swap(normalvectorRoof);
			vector<vector<int>>index;
			vector<vector<POINT3D>>cluster_points = cf.Shared_nearest_neighborclustering(tempenlarge, maxtolerance3d, u3d);//(cluster_pointsenlarge[i], min_cluster_size, max_cluster_size, maxtolerance3d, index);
			tempenlarge.clear();
			vector<POINT3D>().swap(tempenlarge);
			index.clear();
			vector < vector<int>>().swap(index);
			for (int j = 0; j < cluster_points.size(); ++j)
			{
				if (cluster_points[j].size() > 0)
				{
					double angle = 0;
					double width = 0;
					double length = 0;
					//vector<POINT3D> vertex = pca.PCAalgorithm(cluster_points[j], length, width, angle);
					//double angle = 0;
					vector<POINT3D> vertex = ImproveMBRRectangle(cluster_points[j], rotation_angle, angle);
					 width = sqrt((vertex[0].x - vertex[1].x)*(vertex[0].x - vertex[1].x) + (vertex[0].y - vertex[1].y)*(vertex[0].y - vertex[1].y));
					 length = sqrt((vertex[0].x - vertex[3].x)*(vertex[0].x - vertex[3].x) + (vertex[0].y - vertex[3].y)*(vertex[0].y - vertex[3].y));
					//double area = width * length;
					if ((width > small_width_longth) && (length > small_width_longth))
					{
						double grid3dd = en.grid3d_Cluster_evaluation(cluster_points[j], maxtolerance2d, min);
						if (grid3dd < max_number_area)
						{
							unindividual.push_back(cluster_points[j]);
						}
						else
						{
							individual.push_back(cluster_points[j]);
							cluster_points[j].clear();
							individualvertex.push_back(vertex);
							angle3d.push_back(angle);
						}
					}
					else
					{
						noise.push_back(cluster_points[j]);
						cluster_points[j].clear();
					}
				}
				else
				{
					noise.push_back(cluster_points[j]);
					cluster_points[j].clear();
				}
			}
		}
	}
	//double shrink = 1 / enlargeenshrink;
	for (int i = 0; i < individual.size(); i++)
	{
		vector<POINT3D > temp3d;
		for (int j = 0; j < individual[i].size(); j++)
		{
			point3d.x = individual[i][j].x;
			point3d.y = individual[i][j].y;
			point3d.z = individual[i][j].z;// *shrink;
			temp3d.push_back(point3d);
		}
		individual3d.push_back(temp3d);
		temp3d.clear();
		vector<POINT3D >().swap(temp3d);
	}
	for (int i = 0; i < unindividual.size(); i++)
	{
		vector<POINT3D > temp3d;
		for (int j = 0; j < unindividual[i].size(); j++)
		{
			point3d.x = unindividual[i][j].x;
			point3d.y = unindividual[i][j].y;
			point3d.z = unindividual[i][j].z;// *shrink;
			temp3d.push_back(point3d);
		}
		unindividual3d.push_back(temp3d);
		temp3d.clear();
		vector<POINT3D >().swap(temp3d);
	}
	for (int i = 0; i < noise.size(); i++)
	{
		vector<POINT3D > temp3d;
		for (int j = 0; j < noise[i].size(); j++)
		{
			point3d.x = noise[i][j].x;
			point3d.y = noise[i][j].y;
			point3d.z = noise[i][j].z;// *shrink;
			temp3d.push_back(point3d);
		}
		noise3d.push_back(temp3d);
		temp3d.clear();
		vector<POINT3D >().swap(temp3d);
	}
}










