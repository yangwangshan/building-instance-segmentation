#include "stdafx.h"
#include "myfunction.h"
#include "MBR.h"
#include<vector>
#include "ReadWrite.h"
//#include "Roof_extraction.h"
#include "POINT3D.h"
#include "PLANE.h"
#include "Boudary.h"
#include "CFSFDP.h"
#include "Eight_neighborhood.h"
#include "max_min.h"
#include "Grid.h"
#include "PCA1.h"
//#include "image.h"
#include "Boudary.h"

//#include "ClusterAnalysis.h"
//using namespace cv;
//using namespace std;
myfunction::myfunction()
{
}


myfunction::~myfunction()
{
}



vector<vector<POINT3D>>myfunction::Multiscale_adaptation_building_individual_segmentation(vector<POINT3D>incloud,
	double maxtolerance2d, double maxtolerance3d, double ss_width_longth, double small_width_longth, double max_number_area,
	 double heightmax,  double horizontalangle, double u2d, double u3d)
{

	MBR mbr;
	ReadWrite rw;
	//Roof_extraction re;
	PLANE plane;
	Boudary bd;
	Eight_neighborhood en;
	myclass mc;
	int start = clock();
	max_min mm;
	Grid grid;
	CFSFDP cf;

	POINT3D point3d;
	vector <vector<POINT3D>>finalindividual;
	double dist1 = grid.KNN_dc_noconvergence(incloud);
	double gridmax2d = maxtolerance2d ;
	double gridmax3d = maxtolerance3d ;
	//点密度计算
	double point_density = 1 / (dist1* dist1);
	//最小建筑物的面积乘以单位面积内点数可以的最小建筑物点数
	int individual_number2d = (int)(ss_width_longth * ss_width_longth*point_density)*0.5;
	int individual_number3d = (int)(small_width_longth * small_width_longth*point_density)*0.5;
	vector<vector<POINT3D>>cloud;
	cloud.push_back(incloud);
	vector<vector<POINT3D>>individual2dd;
	vector<vector<POINT3D>>unindividual2d;
	vector<vector<POINT3D>>noise2d;
	vector<vector<POINT3D>>individualvertex2d;
	vector<double>angle2d;
	double rotation_angle = 1;
	//double angle_threshold =1;
	mbr.MBR_2DMultiscaleShared_nearest_neighborclustering_newsingle(incloud,  gridmax3d,  gridmax2d, individual_number2d,
		ss_width_longth, rotation_angle, max_number_area,individual2dd, unindividual2d, noise2d, individualvertex2d, angle2d,  u2d);
	incloud.clear();
	vector<POINT3D>().swap(incloud);
	
	if (unindividual2d.size() > 0)
	{

		vector<vector<POINT3D>>individual3dd;
		vector<vector<POINT3D>>unindividual3d;
		vector<vector<POINT3D>>noise3d;
		vector<vector<POINT3D>>individualvertex3d;
		vector<double>angle3d;
		
		mbr.MBR_3DMultiscaleShared_nearest_neighborclustering_newsingle(unindividual2d,gridmax3d, gridmax2d, individual_number3d,
			small_width_longth, rotation_angle, max_number_area,  individual3dd, unindividual3d, noise3d, individualvertex3d, angle3d,  horizontalangle,  u3d);

		vector<vector<POINT3D>>individual3d;
		for (int i = 0; i < individual3dd.size(); i++)
		{
			individual3d.push_back(individual3dd[i]);
		}
		for (int i = 0; i < unindividual3d.size(); i++)
		{
			individual3d.push_back(unindividual3d[i]);
		}

		vector<POINT3D>outcloud;
		for (int i = 0; i < unindividual2d.size(); i++)
		{
			for (int j = 0; j < unindividual2d[i].size(); j++)
			{
				point3d.x = unindividual2d[i][j].x;
				point3d.y = unindividual2d[i][j].y;
				point3d.z = unindividual2d[i][j].z;
				outcloud.push_back(point3d);
			}
		}
		for (int i = 0; i < noise2d.size(); i++)
		{
			for (int j = 0; j < noise2d[i].size(); j++)
			{
				point3d.x = noise2d[i][j].x;
				point3d.y = noise2d[i][j].y;
				point3d.z = noise2d[i][j].z;
				outcloud.push_back(point3d);
			}
		}
		unindividual2d.clear();
		vector < vector<POINT3D>>().swap(unindividual2d);
		noise2d.clear();
		vector < vector<POINT3D>>().swap(noise2d);

		vector<POINT3D>ungridprojection_new;
		vector < vector<POINT3D>>m_roofprojectionindividual = grid.grid2dprojectioninteriorpoint_outpoint(outcloud, individual3d, gridmax2d, ungridprojection_new);
		
	
		if (ungridprojection_new.size() > 0)
		{
			outcloud.clear();
			vector < POINT3D>().swap(outcloud);
			individual3d.clear();
			vector < vector<POINT3D>>().swap(individual3d);
			vector<vector<POINT3D>>redundantpointotherpoints1;//屋顶细节结构的合并
			/*vector<POINT3D > buildingmax1;
			vector<POINT3D > buildingmin1;*/
			vector<vector<POINT3D>>projectionotherpoints;
			projectionotherpoints.push_back(ungridprojection_new);
			ungridprojection_new.clear();
			vector < POINT3D>().swap(ungridprojection_new);
			double mbrbuffer = gridmax2d * 2;
			double heightmin = -heightmax * 0.5;
			vector<vector<POINT3D>>single_buildingMBR = plane.redundantpoint_single_building_MBR_new(projectionotherpoints, m_roofprojectionindividual,
				  gridmax2d, heightmax, heightmin, mbrbuffer, redundantpointotherpoints1,  small_width_longth, u2d);

			if (redundantpointotherpoints1.size() > 0)
			{
				projectionotherpoints.clear();
				vector < vector<POINT3D>>().swap(projectionotherpoints);
				m_roofprojectionindividual.clear();
				vector < vector<POINT3D>>().swap(m_roofprojectionindividual);
				vector<vector<POINT3D>>buildingvertex2;
				vector<POINT3D > buildingmax2;
				vector<POINT3D > buildingmin2;
				vector<vector<POINT3D>>small_individualotherpoints2;//为外点中非屋顶细节结构的单体建筑物
				vector<vector<POINT3D>>m_small_individual_building = plane.small_individual_building(redundantpointotherpoints1, individual_number2d, ss_width_longth,
					 buildingvertex2, buildingmax2, buildingmin2, small_individualotherpoints2, rotation_angle);

				vector <vector<POINT3D>>individualbuilding;
				for (int j = 0; j < individual2dd.size(); j++)
				{
					individualbuilding.push_back(individual2dd[j]);
				}
				for (int j = 0; j < single_buildingMBR.size(); j++)
				{
					individualbuilding.push_back(single_buildingMBR[j]);
				}
				for (int j = 0; j < m_small_individual_building.size(); j++)
				{
					individualbuilding.push_back(m_small_individual_building[j]);
				}
				
				
				if (small_individualotherpoints2.size() > 0)
				{
					redundantpointotherpoints1.clear();
					vector < vector<POINT3D>>().swap(redundantpointotherpoints1);
					vector <vector<POINT3D>>m_totalmergesingle_building = plane.kdtree_mergesingle_building_knn(small_individualotherpoints2, individualbuilding);
					small_individualotherpoints2.clear();
					vector < vector<POINT3D>>().swap(small_individualotherpoints2);
					individualbuilding.clear();
					vector < vector<POINT3D>>().swap(individualbuilding);
					for (int i = 0; i < m_totalmergesingle_building.size(); i++)
					{
						finalindividual.push_back(m_totalmergesingle_building[i]);
					}
					m_totalmergesingle_building.clear();
					vector < vector<POINT3D>>().swap(m_totalmergesingle_building);
				}
				else
				{
					for (int i = 0; i < individualbuilding.size(); i++)
					{
						finalindividual.push_back(individualbuilding[i]);
					}
					individualbuilding.clear();
					vector < vector<POINT3D>>().swap(individualbuilding);
				}
			}
			else
			{
				for (int i = 0; i < m_roofprojectionindividual.size(); i++)
				{
					finalindividual.push_back(single_buildingMBR[i]);
				}
				single_buildingMBR.clear();
				vector < vector<POINT3D>>().swap(single_buildingMBR);
			}

		}
		else
		{
			for (int i = 0; i < individual3d.size(); i++)
			{
				finalindividual.push_back(m_roofprojectionindividual[i]);
			}
			m_roofprojectionindividual.clear();
			vector < vector<POINT3D>>().swap(m_roofprojectionindividual);
			//return m_roofprojectionindividual;
		}

	}
	else
	{
		for (int i = 0; i < individual2dd.size(); i++)
		{
			finalindividual.push_back(individual2dd[i]);
		}
		individual2dd.clear();
		vector < vector<POINT3D>>().swap(individual2dd);
	}
	return finalindividual;

}






