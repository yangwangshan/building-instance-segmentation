#ifndef _BOUDARY_H_
#define _BOUDARY_H_
#include <vector>
#include "POINT3D.h"
//#include "triangle.h"
#include "Plane.h"
using namespace std;

typedef vector<POINT3D>	PointArray;//������

typedef vector<int>	PointArrayIndex;//����������

class Boudary
{


	


public:
	Boudary();
	~Boudary();
	

	void PCA_plane_normal(vector<POINT3D>&incloud, PLANE &NormalA);


};
#endif 
