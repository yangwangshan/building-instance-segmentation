#include "stdafx.h"
#include "projection.h"
#include "POINT3D.h"
#include "max_min.h"

projection::projection()
{
}


projection::~projection()
{
}
//该算法失败，应该结合四叉树进行思考
vector < vector<POINT3D>> projection::pointcloudprojectiongrid(vector<POINT3D> &totalcloud, vector < vector<POINT3D>> &localincloud,double Gridthreshold)
{
	max_min mm;
	POINT3D point3d;
	vector < vector<POINT3D>>projectionpoints;
	vector < vector<int>>totaltempindex;
	POINT3D max, min;
	mm.max_min_calculation(totalcloud, max, min);
	int nx = (int)((max.x - min.x) / Gridthreshold) + 1;
	int ny = (int)((max.y - min.y) / Gridthreshold) + 1;
	vector<vector<projection>>totalprojectiongrid(nx, vector<projection>(ny));
	for (int i=0;i< totalcloud.size();i++)
	{
		int a1 = floor((totalcloud[i].x - min.x) / Gridthreshold);
		int b1 = floor((totalcloud[i].y - min.y) / Gridthreshold);
		//grid2d[a][b].m_point_n++;//统计格网内点数
		totalprojectiongrid[a1][b1].m_index.push_back(i);//每个格网内点云的索引
		

	}
	for (int i = 0; i < nx; i++)
	{
		for (int j = 0; j < ny; j++)
		{
			totaltempindex.push_back(totalprojectiongrid[i][j].m_index);//按照顺序加入的，有没有点都要加入
			/*for (int k = 0; k < totalprojectiongrid[i][j].m_index.size(); k++)
			{
				totaltempindex.push_back(totalprojectiongrid[i][j].m_index);
			}*/

		}
	}


	vector<int>aa;//测试用的，这个数组没有用
	vector < vector<int>>aaaaa;//测试用的，这个数组没有用
	vector<int>bb;//测试用的，这个数组没有用
	vector < vector<int>>bbbbb;//测试用的，这个数组没有用
	vector < vector<int>>tempindex;
	vector < vector<int>>index2d;
	vector<vector<projection>>projectiongrid(nx, vector<projection>(ny));
	for (int i=0;i< localincloud.size();++i)
	{
		for (int j=0;j< localincloud[i].size();j++)
		{
			int a = floor((localincloud[i][j].x - min.x) / Gridthreshold);
			int b = floor((localincloud[i][j].y - min.y) / Gridthreshold);
			int c = nx * (a-1)+b;
			aa.push_back(c);
			aa.push_back(c);
			
			//grid2d[a][b].m_point_n++;//统计格网内点数
			projectiongrid[a][b].m_index.push_back(j);//每个格网内点云的索引
		}
		for (int j=0;j<nx;j++)
		{
			for (int k=0;k<ny;k++)
			{
				if (projectiongrid[j][k].m_index.size()>0)
				{
					bbbbb.push_back(projectiongrid[j][k].m_index);
				}
				for (int m=0;m< projectiongrid[j][k].m_index.size();m++)
				{
					tempindex.push_back(projectiongrid[j][k].m_index);
					projectiongrid[j][k].m_index.clear();
				}

			}
		}
		//放入一维数组里面，在去掉重复的，然后通过循环在放入二维最终的二维索引里面
		vector<int>index1d;
		for (int j=0;j< tempindex.size();++j)
		{
			for (int k=0;k< tempindex[j].size();++k)
			{
				index1d.push_back(tempindex[j][k]);
			}
		}
		tempindex.clear();
		vector < vector<int>>().swap(tempindex);
		sort(index1d.begin(), index1d.end());
		index1d.erase(unique(index1d.begin(), index1d.end()), index1d.end());
		index2d.push_back(index1d);
		index1d.clear();
		vector<int>().swap(index1d);
		/*projectiongrid.clear();
		vector<vector<projection>>().swap(projectiongrid);*/
		sort(aa.begin(), aa.end());
		aa.erase(unique(aa.begin(), aa.end()), aa.end());
		aaaaa.push_back(aa);
		aa.clear();
		vector<int>().swap(aa);//多余的实验看结果用的
	}
	vector<POINT3D>tempgridpoint;
	for (int i = 0; i < nx; i++)
	{
		for (int j = 0; j < ny; j++)
		{
			if (projectiongrid[i][j].m_index.size() > 0)
			{
				for (int k = 0; k < projectiongrid[i][j].m_index.size(); k++)
				{
					int a = projectiongrid[i][j].m_index[k];
					point3d.x = totalcloud[a].x;
					point3d.y = totalcloud[a].y;
					point3d.z = totalcloud[a].z;
					tempgridpoint.push_back(point3d);
				}
				projectionpoints.push_back(tempgridpoint);
				tempgridpoint.clear();
				vector<POINT3D>().swap(tempgridpoint);
			}
		}
	}
	projectiongrid.clear();
	vector<vector<projection>>().swap(projectiongrid);
	vector < vector <POINT3D>>temppoints;
	POINT3D tempmax, tempmin;
	for (int i = 0; i < localincloud.size(); i++)
	{
		mm.max_min_calculation(localincloud[i], tempmax, tempmin);
		int tempnx = (max.x - tempmax.x) / Gridthreshold + 1;
		int tempny = (max.y - tempmax.y) / Gridthreshold + 1;
		vector<vector<projection>>tempprojectiongrid(tempnx, vector<projection>(tempny));
		//搞不下去了，发现越来越复杂了，还不如之前的复杂程度了
		//temppoints.push_back(tempprojectiongrid);//有错误，搞不下去了，还不如之前的简单
	}
	//不旋转了，屋顶格网内有点的，对应整体格网的点加入到数组中，晚上整
	//少了一个旋转，可以快很多


	/*vector<POINT3D> minpoint;
	vector<POINT3D> maxpoint;
	for (int i=0;i< localincloud.size();i++)
	{
		mm.max_min_calculation(localincloud[i], max, min);
		point3d.x = max.x;
		point3d.y = max.y;
		point3d.z = max.z;
		maxpoint.push_back(point3d);
		point3d.x = min.x;
		point3d.y = min.y;
		point3d.z = min.z;
		minpoint.push_back(point3d);
	}*/
	/*for (int j=0;j< totalcloud.size();j++)
	{
		int a= totalcloud[j].x-
	}
	int a = floor((incloud[i].x - min.x) / Gridthreshold);
	int b = floor((incloud[i].y - min.y) / Gridthreshold);*/



	return projectionpoints;
}

