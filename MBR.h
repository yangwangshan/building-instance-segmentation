#ifndef MBR_H
#define MBR_H

#include "POINT3D.h"
#include <vector>
using namespace std;
class MBR
{
public:
	MBR();
	~MBR();

public:
	vector<vector<POINT3D>>perfectindividualbuildings;
	//double m_angle;

	/*vector<POINT3D>centerPoint;

	vector<vector<POINT3D>>rotatePoint;

	vector<POINT3D> rectanglemaxx;
	vector<POINT3D> rectanglemaxy;
	vector<POINT3D> rectangleminx;
	vector<POINT3D> rectangleminy;*/

	//vector<POINT3D> m_vertexmax;
	//vector<POINT3D> m_vertexmin;
	//vector<double> m_minarea;
	//vector<int> m_ID;
	vector<vector<POINT3D>>m_MBRdataClusteringestimate;
public:



	

	
	vector<POINT3D> rotate(vector<POINT3D>  &incloud, POINT3D center, double theta);

	vector<POINT3D> ImproveMBRRectangle(vector<POINT3D> &icloud, double theta, double &m_angle);

	void MBR_3DMultiscaleShared_nearest_neighborclustering_newsingle(vector < vector<POINT3D>>&cloud, double maxtolerance3d,
		double maxtolerance2d, int individual_number3d, double small_width_longth, double rotation_angle, double max_number_area,
		vector < vector<POINT3D>>&individual3d, vector < vector<POINT3D>>&unindividual3d, vector < vector<POINT3D>>&noise3d,
		vector < vector<POINT3D>>&individualvertex, vector < double>& angle3d, double angle_threshold, double u3d);



	
	
	void MBR_2DMultiscaleShared_nearest_neighborclustering_newsingle(vector < POINT3D>&incloud, double maxtolerance3d,
		double maxtolerance2d, int individual_number2d,double ss_width_longth, double rotation_angle, double max_number_area,
		vector < vector<POINT3D>>&individual, vector < vector<POINT3D>>&unindividual, vector < vector<POINT3D>>&noise2d,
		vector < vector<POINT3D>>&individualvertex, vector < double>& angle2d, double u2d);

	


	





};

#endif 