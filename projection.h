#ifndef _PROJECTION_H_
#define _PROJECTION_H_
#include <vector>
#include "POINT3D.h"
using namespace std;
typedef class projection
{
public:
	projection();
	~projection();
public:
	
	vector <int> m_index;
	vector <POINT3D>m_gridpoints;
	vector < vector<POINT3D>> pointcloudprojectiongrid(vector<POINT3D> &totalcloud, vector < vector<POINT3D>> &localinclouddouble ,double Gridthreshold);



};

#endif 