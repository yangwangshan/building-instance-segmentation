#include "stdafx.h"
//#include "common.h"
#include "PCA1.h"
#include "POINT3D.h"
#include "myclass.h"
//#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <string>
#include <vector>
#include "max_min.h"


#include <iostream>
#include <string>
#include <stdio.h>
#include <Eigen/Core>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/correspondence.h>                 //��Ӧ��ʾ����ʵ��֮���ƥ�䣨���磬�㣬�������ȣ���
#include <pcl/features/normal_3d.h>             //����

#include <pcl/filters/uniform_sampling.h>    //���Ȳ���
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>   //���ӻ�
#include <pcl/common/transforms.h>              //ת������
#include <pcl/console/parse.h>
#include <pcl/point_types.h>

#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "POINT3D.h"
#include "ReadWrite.h"
#include <vector>
using namespace std;

typedef pcl::PointXYZRGB PointT;             //Point with color
typedef pcl::PointCloud<PointT> PointCloud; //PointCloud with color







#define PI 3.14159265
using namespace Eigen;
//using namespace std;
typedef pcl::PointXYZ PointType;


typedef pcl::PointXYZRGB PointT;             //Point with color
typedef pcl::PointCloud<PointT> PointCloud; //PointCloud with color
PCA1::PCA1()
{
}


PCA1::~PCA1()
{
}



