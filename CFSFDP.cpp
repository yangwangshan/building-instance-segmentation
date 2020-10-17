#include "stdafx.h"
#include "CFSFDP.h"
#include "Eight_neighborhood.h"
#include<algorithm>
#include "max_min.h"
#include "myclass.h"
//#include <pcl/point_cloud.h>
//#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include "time.h"
#include "ReadWrite.h"
#include "MBR.h"
CFSFDP::CFSFDP()
{
}


CFSFDP::~CFSFDP()
{
}














//使用的
/*【函数说明】共享邻域聚类，根据共享邻域，其具有相似性（建立了kd-tree，去掉了决策图，可以提高运算效率）
*【输入参数】 points-------------输入点云
*		      dce----------------局部密度的邻域，半径k邻域搜索半径（CFSFDP密度峰值）
*		       u-----------------密度峰值密度截断比例
*【输出参数】ccloud——输出点（返回聚类后的结果）
*【返回值】*/
vector<vector<POINT3D>> CFSFDP::Shared_nearest_neighborclustering(vector<POINT3D> &points, double dc, double u)
{
	myclass mc;
	ReadWrite rw;
	vector<vector<POINT3D>> ccloud;
	//int start = clock();
	POINT3D point3d;
	//vector<POINT3D>incloud;
	////vector<int> rho1;
	//for (int i = 0; i < points.size(); i++)
	//{
	//	point3d.x = points[i].x;
	//	point3d.y = points[i].y;
	//	point3d.z = 0;
	//	incloud.push_back(point3d);
	//	//rho1.push_back(1);
	//}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	mc.SwitchDataAsciiToPCL(points, cloud);//下面这些部分可以与求局部密度的放在一起。
	pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
	kdtree.setInputCloud(cloud);
	//vector<int> rho = getLocalDensity(incloud, dc);
	vector<int> rho;

	pcl::PointXYZ searchPoint1;
	
	std::vector<int> pointIdxRadiusSearch1;
	std::vector<float> pointRadiusSquaredDistance1;
	vector<vector<float>> distance;
	pcl::PointXYZ searchPoint11;
	std::vector<int> pointIdxRadiusSearch11;
	std::vector<float> pointRadiusSquaredDistance11;
	vector<vector<float>> distance1;
	
	vector < vector<POINT3D> >outcloud;
	for (int k = 0; k < points.size(); k++)
	{
		if (points[k].IsVisisted == false)
		{
			searchPoint1.x = points[k].x; //centre.x;
			searchPoint1.y = points[k].y;
			searchPoint1.z = points[k].z;
			/*for (int m=0;m< points.size(); m++)
			{*/
				vector<POINT3D> tempoutcloud;
				std::vector<int> pointIdx;
				
				points[k].IsVisisted = true;
				point3d.x = searchPoint1.x;
				point3d.y = searchPoint1.y;
				point3d.z = searchPoint1.z;
				tempoutcloud.push_back(point3d);
				// 在半径r内搜索近邻
				//std::vector<int> pointIdxRadiusSearch;
				//std::vector<float> pointRadiusSquaredDistance;
				if (kdtree.radiusSearch(searchPoint1, dc, pointIdxRadiusSearch1, pointRadiusSquaredDistance1) > 0)
				{
					/*int a = pointIdxRadiusSearch1.size();
					rho.push_back(a);
					distance.push_back(pointRadiusSquaredDistance1);*/
				}
				if (pointIdxRadiusSearch1.size() > 0)
				{
					vector<POINT3D> tempcloud;
					for (int j = 0; j < pointIdxRadiusSearch1.size(); j++)
					{
						
						if (points[pointIdxRadiusSearch1[j]].IsVisisted == false)
						{
							for (int i = 0; i < pointIdxRadiusSearch1.size(); i++)
							{
								pointIdx.push_back(pointIdxRadiusSearch1[i]);
							}
							searchPoint11.x = points[pointIdxRadiusSearch1[j]].x; //centre.x;
							searchPoint11.y = points[pointIdxRadiusSearch1[j]].y;
							searchPoint11.z = points[pointIdxRadiusSearch1[j]].z;

							// 在半径r内搜索近邻
							//std::vector<int> pointIdxRadiusSearch;
							//std::vector<float> pointRadiusSquaredDistance;
							if (kdtree.radiusSearch(searchPoint11, dc, pointIdxRadiusSearch11, pointRadiusSquaredDistance11) > 0)
							{
								/*int a = pointIdxRadiusSearch1.size();
								rho.push_back(a);
								distance.push_back(pointRadiusSquaredDistance1);*/
							}
							for (int i = 0; i < pointIdxRadiusSearch11.size(); i++)
							{
								pointIdx.push_back(pointIdxRadiusSearch11[i]);
							}

							sort(pointIdx.begin(), pointIdx.end());
							int t = 0;
							for (int i = 0; i < (pointIdx.size() - 1); i++)
							{
								if (pointIdx[i] == pointIdx[i + 1])
								{
									t++;
									i++;
								}
							}
							int min_t = t;
							int min_t1 = pointIdxRadiusSearch1.size();
							int min_t11 = pointIdxRadiusSearch11.size();
							if (min_t1 > min_t11)
							{
								min_t = min_t11;
							}
							else
							{
								min_t = min_t1;
							}
							double threshold = u * min_t;
							if (t > threshold)
							{
								point3d.x = searchPoint11.x;
								point3d.y = searchPoint11.y;
								point3d.z = searchPoint11.z;
								tempoutcloud.push_back(point3d);
								tempcloud.push_back(point3d);
								points[pointIdxRadiusSearch1[j]].IsVisisted = true;
							}
							//是不是在一点少了一点问题呀


							pointIdx.clear();
							std::vector<int>().swap(pointIdx);
							pointIdxRadiusSearch11.clear();
							std::vector<int>().swap(pointIdxRadiusSearch11);
							pointRadiusSquaredDistance11.clear();
							std::vector<float>().swap(pointRadiusSquaredDistance11);
							/*searchPoint1.x = searchPoint11.x;
							searchPoint1.y = searchPoint11.y;
							searchPoint1.z = searchPoint11.z;*/
						}

					}
					pointIdxRadiusSearch1.clear();
					std::vector<int>().swap(pointIdxRadiusSearch1);
					pointRadiusSquaredDistance1.clear();
					std::vector<float>().swap(pointRadiusSquaredDistance1);
					for (int j=0;j< tempcloud.size();j++)
					{
						searchPoint1.x = tempcloud[j].x; //centre.x;
						searchPoint1.y = tempcloud[j].y;
						searchPoint1.z = tempcloud[j].z;
						if (kdtree.radiusSearch(searchPoint1, dc, pointIdxRadiusSearch1, pointRadiusSquaredDistance1) > 0)
						{
						}
						if (pointIdxRadiusSearch1.size() > 1)
						{
							//vector<POINT3D> tempcloud1;
							for (int j = 0; j < pointIdxRadiusSearch1.size(); j++)
							{
								
								if (points[pointIdxRadiusSearch1[j]].IsVisisted == false)
								{
									for (int i = 0; i < pointIdxRadiusSearch1.size(); i++)
									{
										pointIdx.push_back(pointIdxRadiusSearch1[i]);
									}
									searchPoint11.x = points[pointIdxRadiusSearch1[j]].x; //centre.x;
									searchPoint11.y = points[pointIdxRadiusSearch1[j]].y;
									searchPoint11.z = points[pointIdxRadiusSearch1[j]].z;

									// 在半径r内搜索近邻
									//std::vector<int> pointIdxRadiusSearch;
									//std::vector<float> pointRadiusSquaredDistance;
									if (kdtree.radiusSearch(searchPoint11, dc, pointIdxRadiusSearch11, pointRadiusSquaredDistance11) > 0)
									{
										/*int a = pointIdxRadiusSearch1.size();
										rho.push_back(a);
										distance.push_back(pointRadiusSquaredDistance1);*/
									}
									for (int i = 0; i < pointIdxRadiusSearch11.size(); i++)
									{
										pointIdx.push_back(pointIdxRadiusSearch11[i]);
									}

									sort(pointIdx.begin(), pointIdx.end());
									int t = 0;
									for (int i = 0; i < (pointIdx.size() - 1); i++)
									{
										if (pointIdx[i] == pointIdx[i + 1])
										{
											t++;
											i++;
										}
									}
									int min_t = t;
									int min_t1 = pointIdxRadiusSearch1.size();
									int min_t11 = pointIdxRadiusSearch11.size();
									if (min_t1 > min_t11)
									{
										min_t = min_t11;
									}
									else
									{
										min_t = min_t1;
									}
									double threshold = u * min_t;
									if (t > threshold)
									{
										point3d.x = searchPoint11.x;
										point3d.y = searchPoint11.y;
										point3d.z = searchPoint11.z;
										tempoutcloud.push_back(point3d);
										tempcloud.push_back(point3d);
										points[pointIdxRadiusSearch1[j]].IsVisisted = true;
									}

									pointIdx.clear();
									std::vector<int>().swap(pointIdx);
									pointIdxRadiusSearch11.clear();
									std::vector<int>().swap(pointIdxRadiusSearch11);
									pointRadiusSquaredDistance11.clear();
									std::vector<float>().swap(pointRadiusSquaredDistance11);
									/*searchPoint1.x = searchPoint11.x;
									searchPoint1.y = searchPoint11.y;
									searchPoint1.z = searchPoint11.z;*/
								}
								/*else
								{
									pointIdx.clear();
									std::vector<int>().swap(pointIdx);
								}*/
							}

						}


					}
					outcloud.push_back(tempoutcloud);
					tempoutcloud.clear();
					vector<	POINT3D>().swap(tempoutcloud);


				}
				else
				{
				    outcloud.push_back(tempoutcloud);
				    tempoutcloud.clear();
				    vector<	POINT3D>().swap(tempoutcloud);
					break;
				}
				
			//}
			
		}
		
		

	}
	/*vector<	POINT3D>tempcloud2;
	for (int k=0;k<points.size();k++)
	{
		if (points[k].IsVisisted==false)
		{
			point3d.x = points[k].x;
			point3d.y = points[k].y;
			point3d.z = points[k].z;
			tempcloud2.push_back(point3d);
		}
	}*/

	
	return outcloud;
}






//使用的
/*【函数说明】共享邻域聚类，根据共享邻域，其具有相似性（建立了kd-tree，去掉了决策图，可以提高运算效率）
*【输入参数】 points-------------输入点云
*		      dce----------------局部密度的邻域，半径k邻域搜索半径（CFSFDP密度峰值）
*		       u-----------------密度峰值密度截断比例
*【输出参数】ccloud——输出点（返回聚类后的结果）
*【返回值】*/
vector<vector<POINT3D>> CFSFDP::Shared_nearest_neighborclustering2d(vector<POINT3D> &incloud, double dc, double u)
{
	myclass mc;
	ReadWrite rw;
	vector<vector<POINT3D>> ccloud;
	//int start = clock();
	POINT3D point3d;
	vector<POINT3D>points;
	//vector<int> rho1;
	for (int i = 0; i < incloud.size(); i++)
	{
		point3d.x = incloud[i].x;
		point3d.y = incloud[i].y;
		point3d.z = 0;
		points.push_back(point3d);
		//rho1.push_back(1);
	}
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3d(new pcl::PointCloud<pcl::PointXYZ>);
	//mc.SwitchDataAsciiToPCL(incloud, cloud3d);//下面这些部分可以与求局部密度的放在一起。
	//pcl::KdTreeFLANN<pcl::PointXYZ>kdtree3d;
	//kdtree3d.setInputCloud(cloud3d);


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	mc.SwitchDataAsciiToPCL(points, cloud);//下面这些部分可以与求局部密度的放在一起。
	pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;
	kdtree.setInputCloud(cloud);
	//vector<int> rho = getLocalDensity(incloud, dc);
	//vector<int> rho;

	pcl::PointXYZ searchPoint1;

	std::vector<int> pointIdxRadiusSearch1;
	std::vector<float> pointRadiusSquaredDistance1;
	vector<vector<float>> distance;
	pcl::PointXYZ searchPoint11;
	std::vector<int> pointIdxRadiusSearch11;
	std::vector<float> pointRadiusSquaredDistance11;
	vector<vector<float>> distance1;

	vector < vector<POINT3D> >outcloud;
	for (int k = 0; k < points.size(); k++)
	{
		if (points[k].IsVisisted == false)
		{
			searchPoint1.x = points[k].x; //centre.x;
			searchPoint1.y = points[k].y;
			searchPoint1.z = points[k].z;
			/*for (int m=0;m< points.size(); m++)
			{*/
			vector<POINT3D> tempoutcloud;
			std::vector<int> pointIdx;

			points[k].IsVisisted = true;
			//point3d.x = searchPoint1.x;
			//point3d.y = searchPoint1.y;
			//point3d.z = searchPoint1.z;
			point3d.x = incloud[k].x;
			point3d.y = incloud[k].y;
			point3d.z = incloud[k].z;
			tempoutcloud.push_back(point3d);
			// 在半径r内搜索近邻
			//std::vector<int> pointIdxRadiusSearch;
			//std::vector<float> pointRadiusSquaredDistance;
			if (kdtree.radiusSearch(searchPoint1, dc, pointIdxRadiusSearch1, pointRadiusSquaredDistance1) > 0)
			{
				/*int a = pointIdxRadiusSearch1.size();
				rho.push_back(a);
				distance.push_back(pointRadiusSquaredDistance1);*/
			}
			if (pointIdxRadiusSearch1.size() > 0)
			{
				vector<POINT3D> tempcloud;
				for (int j = 0; j < pointIdxRadiusSearch1.size(); j++)
				{

					if (points[pointIdxRadiusSearch1[j]].IsVisisted == false)
					{
						for (int i = 0; i < pointIdxRadiusSearch1.size(); i++)
						{
							pointIdx.push_back(pointIdxRadiusSearch1[i]);
						}
						searchPoint11.x = points[pointIdxRadiusSearch1[j]].x; //centre.x;
						searchPoint11.y = points[pointIdxRadiusSearch1[j]].y;
						searchPoint11.z = points[pointIdxRadiusSearch1[j]].z;

						// 在半径r内搜索近邻
						//std::vector<int> pointIdxRadiusSearch;
						//std::vector<float> pointRadiusSquaredDistance;
						if (kdtree.radiusSearch(searchPoint11, dc, pointIdxRadiusSearch11, pointRadiusSquaredDistance11) > 0)
						{
							/*int a = pointIdxRadiusSearch1.size();
							rho.push_back(a);
							distance.push_back(pointRadiusSquaredDistance1);*/
						}
						for (int i = 0; i < pointIdxRadiusSearch11.size(); i++)
						{
							pointIdx.push_back(pointIdxRadiusSearch11[i]);
						}

						sort(pointIdx.begin(), pointIdx.end());
						int t = 0;
						for (int i = 0; i < (pointIdx.size() - 1); i++)
						{
							if (pointIdx[i] == pointIdx[i + 1])
							{
								t++;
								i++;
							}
						}
						int min_t = t;
						int min_t1 = pointIdxRadiusSearch1.size();
						int min_t11 = pointIdxRadiusSearch11.size();
						if (min_t1 > min_t11)
						{
							min_t = min_t11;
						}
						else
						{
							min_t = min_t1;
						}
						double threshold = u * min_t;
						if (t > threshold)
						{
							/*if (incloud[pointIdxRadiusSearch1[j]].z == 0.00)
							{
							}
							else
							{
								point3d.x = searchPoint11.x;
								point3d.y = searchPoint11.y;
								point3d.z = searchPoint11.z;

								tempcloud.push_back(point3d);
								points[pointIdxRadiusSearch1[j]].IsVisisted = true;
								point3d.x = incloud[pointIdxRadiusSearch1[j]].x;
								point3d.y = incloud[pointIdxRadiusSearch1[j]].y;
								point3d.z = incloud[pointIdxRadiusSearch1[j]].z;
								tempoutcloud.push_back(point3d);
							}*/
							point3d.x = searchPoint11.x;
							point3d.y = searchPoint11.y;
							point3d.z = searchPoint11.z;

							tempcloud.push_back(point3d);
							points[pointIdxRadiusSearch1[j]].IsVisisted = true;
							point3d.x = incloud[pointIdxRadiusSearch1[j]].x;
							point3d.y = incloud[pointIdxRadiusSearch1[j]].y;
							point3d.z = incloud[pointIdxRadiusSearch1[j]].z;
							tempoutcloud.push_back(point3d);
							/*if (0.00==point3d.z)
							{
							}
							else
							{
								
							}*/
							



						}
						//是不是在一点少了一点问题呀


						pointIdx.clear();
						std::vector<int>().swap(pointIdx);
						pointIdxRadiusSearch11.clear();
						std::vector<int>().swap(pointIdxRadiusSearch11);
						pointRadiusSquaredDistance11.clear();
						std::vector<float>().swap(pointRadiusSquaredDistance11);
						/*searchPoint1.x = searchPoint11.x;
						searchPoint1.y = searchPoint11.y;
						searchPoint1.z = searchPoint11.z;*/
					}

				}
				pointIdxRadiusSearch1.clear();
				std::vector<int>().swap(pointIdxRadiusSearch1);
				pointRadiusSquaredDistance1.clear();
				std::vector<float>().swap(pointRadiusSquaredDistance1);
				for (int j = 0; j < tempcloud.size(); j++)
				{
					searchPoint1.x = tempcloud[j].x; //centre.x;
					searchPoint1.y = tempcloud[j].y;
					searchPoint1.z = tempcloud[j].z;
					if (kdtree.radiusSearch(searchPoint1, dc, pointIdxRadiusSearch1, pointRadiusSquaredDistance1) > 0)
					{
					}
					if (pointIdxRadiusSearch1.size() > 1)
					{
						//vector<POINT3D> tempcloud1;
						for (int j = 0; j < pointIdxRadiusSearch1.size(); j++)
						{

							if (points[pointIdxRadiusSearch1[j]].IsVisisted == false)
							{
								for (int i = 0; i < pointIdxRadiusSearch1.size(); i++)
								{
									pointIdx.push_back(pointIdxRadiusSearch1[i]);
								}
								searchPoint11.x = points[pointIdxRadiusSearch1[j]].x; //centre.x;
								searchPoint11.y = points[pointIdxRadiusSearch1[j]].y;
								searchPoint11.z = points[pointIdxRadiusSearch1[j]].z;

								// 在半径r内搜索近邻
								//std::vector<int> pointIdxRadiusSearch;
								//std::vector<float> pointRadiusSquaredDistance;
								if (kdtree.radiusSearch(searchPoint11, dc, pointIdxRadiusSearch11, pointRadiusSquaredDistance11) > 0)
								{
									/*int a = pointIdxRadiusSearch1.size();
									rho.push_back(a);
									distance.push_back(pointRadiusSquaredDistance1);*/
								}
								for (int i = 0; i < pointIdxRadiusSearch11.size(); i++)
								{
									pointIdx.push_back(pointIdxRadiusSearch11[i]);
								}

								sort(pointIdx.begin(), pointIdx.end());
								int t = 0;
								for (int i = 0; i < (pointIdx.size() - 1); i++)
								{
									if (pointIdx[i] == pointIdx[i + 1])
									{
										t++;
										i++;
									}
								}
								int min_t = t;
								int min_t1 = pointIdxRadiusSearch1.size();
								int min_t11 = pointIdxRadiusSearch11.size();
								if (min_t1 > min_t11)
								{
									min_t = min_t11;
								}
								else
								{
									min_t = min_t1;
								}
								double threshold = u * min_t;
								if (t > threshold)
								{
									/*if (incloud[pointIdxRadiusSearch1[j]].z ==0.00)
									{
									}
									else
									{
										point3d.x = searchPoint11.x;
										point3d.y = searchPoint11.y;
										point3d.z = searchPoint11.z;

										tempcloud.push_back(point3d);
										points[pointIdxRadiusSearch1[j]].IsVisisted = true;
										point3d.x = incloud[pointIdxRadiusSearch1[j]].x;
										point3d.y = incloud[pointIdxRadiusSearch1[j]].y;
										point3d.z = incloud[pointIdxRadiusSearch1[j]].z;
										tempoutcloud.push_back(point3d);
									}*/
									
									point3d.x = searchPoint11.x;
									point3d.y = searchPoint11.y;
									point3d.z = searchPoint11.z;

									tempcloud.push_back(point3d);
									points[pointIdxRadiusSearch1[j]].IsVisisted = true;
									point3d.x = incloud[pointIdxRadiusSearch1[j]].x;
									point3d.y = incloud[pointIdxRadiusSearch1[j]].y;
									point3d.z = incloud[pointIdxRadiusSearch1[j]].z;
									tempoutcloud.push_back(point3d);
									/*if (0.00== point3d.z)
									{
										
									}*/
									

									
								}

								pointIdx.clear();
								std::vector<int>().swap(pointIdx);
								pointIdxRadiusSearch11.clear();
								std::vector<int>().swap(pointIdxRadiusSearch11);
								pointRadiusSquaredDistance11.clear();
								std::vector<float>().swap(pointRadiusSquaredDistance11);
								/*searchPoint1.x = searchPoint11.x;
								searchPoint1.y = searchPoint11.y;
								searchPoint1.z = searchPoint11.z;*/
							}
							/*else
							{
								pointIdx.clear();
								std::vector<int>().swap(pointIdx);
							}*/
						}

					}


				}
				outcloud.push_back(tempoutcloud);
				tempoutcloud.clear();
				vector<	POINT3D>().swap(tempoutcloud);


			}
			else
			{
				outcloud.push_back(tempoutcloud);
				tempoutcloud.clear();
				vector<	POINT3D>().swap(tempoutcloud);
				break;
			}

		}
	}

	return outcloud;
}































