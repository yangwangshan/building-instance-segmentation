#pragma once

#include "POINT3D.h"
#include <vector>
using namespace std;
class CFSFDP
{
public:
	CFSFDP();
	~CFSFDP();
	vector<vector<POINT3D>>Shared_nearest_neighborclustering(vector<POINT3D> &points, double dc, double u);
	vector<vector<POINT3D>>Shared_nearest_neighborclustering2d(vector<POINT3D> &incloud, double dc, double u);



};

