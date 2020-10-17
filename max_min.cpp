#include "stdafx.h"
#include "max_min.h"


max_min::max_min()
{
}


max_min::~max_min()
{
}



//使用的
void max_min::max_min_calculation(vector<POINT3D > &incloud, POINT3D &max, POINT3D &min)
{
	///*POINT3D max;*/
	//double max_x = 0;            // incloud.x[0];
	//double max_y =0;
	//double max_z = 0;
	///*POINT3D min;*/
	//double min_x = 0;
	//double min_y =0;
	//double min_z = 0;

	double max_x = incloud[0].x;            // incloud.x[0];
	double max_y = incloud[0].y;
	double max_z = incloud[0].z;
	int max_i = incloud[0].intens;
	/*POINT3D min;*/
	double min_x = incloud[0].x;
	double min_y = incloud[0].y;
	double min_z = incloud[0].z;
	int min_i = incloud[0].intens;

	for (int i = 0; i < incloud.size();i++)
	//	for (int i = 0; i < incloud.reverse_iterator; ++i)
	{
		double x = incloud[i].x;
		double y = incloud[i].y;
		double z = incloud[i].z;
		int intens = incloud[i].intens;
		//////////最大值计算
		if (x>max_x)
		{
			max_x = x;
		}
		if (y > max_y)
		{
			max_y = y;
		}
		if (z > max_z)
		{
			max_z = z;
		}
		if (intens>max_i)
		{
			max_i = intens;
		}
		//////////最小值计算
		if (x < min_x)
		{
			min_x = x;
		}
		if (y < min_y)
		{
			min_y = y;
		}
		if (z < min_z)
		{
			min_z = z;
		}
		if (intens <min_i)
		{
			min_i < intens;
		}
	}
	max.x = max_x;
	max.y = max_y;
	max.z = max_z;
	max.intens = max_i;

	min.x = min_x;
	min.y = min_y;
	min.z = min_z;
	min.intens = min_i;
	//return max;
	/*int max = b[0];
	for (int i = 0; i<n; ++i)
		if (b[i]>max)
			max = b[i];*/
	/*return max;*/

	/*return max;*/
}

