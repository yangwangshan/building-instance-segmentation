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
//���㷨ʧ�ܣ�Ӧ�ý���Ĳ�������˼��
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
		//grid2d[a][b].m_point_n++;//ͳ�Ƹ����ڵ���
		totalprojectiongrid[a1][b1].m_index.push_back(i);//ÿ�������ڵ��Ƶ�����
		

	}
	for (int i = 0; i < nx; i++)
	{
		for (int j = 0; j < ny; j++)
		{
			totaltempindex.push_back(totalprojectiongrid[i][j].m_index);//����˳�����ģ���û�е㶼Ҫ����
			/*for (int k = 0; k < totalprojectiongrid[i][j].m_index.size(); k++)
			{
				totaltempindex.push_back(totalprojectiongrid[i][j].m_index);
			}*/

		}
	}


	vector<int>aa;//�����õģ��������û����
	vector < vector<int>>aaaaa;//�����õģ��������û����
	vector<int>bb;//�����õģ��������û����
	vector < vector<int>>bbbbb;//�����õģ��������û����
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
			
			//grid2d[a][b].m_point_n++;//ͳ�Ƹ����ڵ���
			projectiongrid[a][b].m_index.push_back(j);//ÿ�������ڵ��Ƶ�����
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
		//����һά�������棬��ȥ���ظ��ģ�Ȼ��ͨ��ѭ���ڷ����ά���յĶ�ά��������
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
		vector<int>().swap(aa);//�����ʵ�鿴����õ�
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
		//�㲻��ȥ�ˣ�����Խ��Խ�����ˣ�������֮ǰ�ĸ��ӳ̶���
		//temppoints.push_back(tempprojectiongrid);//�д��󣬸㲻��ȥ�ˣ�������֮ǰ�ļ�
	}
	//����ת�ˣ��ݶ��������е�ģ���Ӧ��������ĵ���뵽�����У�������
	//����һ����ת�����Կ�ܶ�


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

