#ifndef _PLANE_H_
#define _PLANE_H_

#include <vector>
#include "POINT3D.h"
#include "myclass.h"
using namespace std;

typedef class PLANE //typedef--用于将Point3D定义为数据类型
{

public:
	PLANE();
	~PLANE();


public:
	//平面方程的系数A
	double A;
	//平面方程的系数B
	double B;
	//平面方程的系数C
	double C;
	//平面方程的系数D
	double D;
	//平面的曲率
	double curvature;
	/*vector<PLANE> normal;*/


public:
	
	double PlaneNormalsAngle3D(PLANE NormalA, PLANE NormalB);
	
	
	
	vector<vector<POINT3D>>redundantpoint_single_building_MBR_new(vector<vector<POINT3D>>&redundantpoint, vector<vector<POINT3D>>&single_building,
		double maxtolerance2d, double heightmax, double heightmin, double mbrbuffer, vector<vector<POINT3D>>&againredundantpoint,
		double small_width_longth, double u2d);


	vector<vector<POINT3D>> small_individual_building(vector<vector<POINT3D>>incloud, int individual_number2d, double ss_Width_long,
		vector<vector<POINT3D>>&buildingvertex, vector<POINT3D > &buildingmax, vector<POINT3D > &buildingmin, vector<vector<POINT3D>>&otherpoints, double rotation_angle);

	//vector<vector<POINT3D>>SmallBuildingEmageToBigBuilding(vector<vector<POINT3D>>&individual_building, vector<vector<POINT3D>>&small_building, double radius, double max_number_area);

	
	vector < vector<POINT3D>> kdtree_mergesingle_building_knn(vector < vector<POINT3D>>otherpoints2, vector < vector<POINT3D>>individual_building);
	
	
	void Octree_based_regiongrowing_planesegmentation(vector < POINT3D > &icloud, int max_number, int min_number, int Neighbours,
		int k, double SmoothnessThreshold, double CurvatureThreshold, vector<vector<POINT3D>>&planepoint, vector < POINT3D > &noplanepoint);
	
	void roofpoint(vector < POINT3D > &incloud, int max_number, int min_number, int Neighbours,
		int k, double SmoothnessThreshold, double CurvatureThreshold, double angle_threshold, vector < POINT3D >& roofpoint,
		vector < POINT3D >& facadepoint, double maxtolerance3d, double u3d);

	
	



}plane;//Point3D--指向空间点Point3D类型的指针
#endif 
