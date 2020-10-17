#ifndef MYFUNCTION_H
#define MYFUNCTION_H
#include "POINT3D.h"
#include "PLANE.h"


class myfunction
{
public:
	myfunction();
	~myfunction();

	vector<vector<POINT3D>>Multiscale_adaptation_building_individual_segmentation(vector<POINT3D>incloud,
		double maxtolerance2d, double maxtolerance3d, double ss_width_longth, double small_width_longth, double max_number_area,
		double heightmax,  double horizontalangle, double u2d, double u3d);

	

};

#endif 