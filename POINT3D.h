#ifndef _POINT3D_H_
#define _POINT3D_H_
typedef class POINT3D //typedef--���ڽ�Point3D����Ϊ��������
{
//public:
//	POINT3D();
//	~POINT3D();
public:
	POINT3D();
	POINT3D(const float& X, const float& Y, const float& Z);
	~POINT3D();
	POINT3D operator + (const POINT3D &point3d);//C++��չ���������
	POINT3D operator - (const POINT3D &point3d);
	POINT3D operator / (const int &sum);
	POINT3D operator * (const int &sum);
	POINT3D operator += (const POINT3D &point3d);
	POINT3D operator -= (const POINT3D &point3d);
	POINT3D operator /= (const int &sum);
	POINT3D operator *= (const int &sum);
public:
	double x;
	double y;
	double z;
	unsigned short r;
	unsigned short g;
	unsigned short b;
	unsigned char Classification;//������Ϣ
	unsigned short intens;//������
	unsigned short HSV;//HSV��ɫģ��
	double density;//�õ���ܶ�
	//unsigned short i;//�ظ��ˣ�һ�㲻��
	//bool isVisisted;
	//int f;
	int label;//���label���������
	int grid2d_index;
	int grid3d_index;
	bool IsVisisted;//�Ƿ����
	//�������ֱ��ͼ
	
	



	
	

}Point3D;//Point3D--ָ��ռ��Point3D���͵�ָ��



typedef class POINT2D //typedef--���ڽ�Point3D����Ϊ��������
{
	//public:
	//	POINT3D();
	//	~POINT3D();
public:
	POINT2D();

	~POINT2D();

public:
	double x;
	double y;
	


}Point2D;//Point3D--ָ��ռ��Point3D���͵�ָ��


struct fake_grid
{
	
	std::vector<int> index;
};



typedef class extent_3d //typedef--���ڽ�Point3D����Ϊ��������
{
public:
	//X����Сֵ
	double m_xmin;
	//X�����ֵ
	double m_xmax;
	//Y����Сֵ
	double m_ymin;
	//Y�����ֵ
	double m_ymax;
	//Z����Сֵ
	double m_zmin;
	//Z�����ֵ
	double m_zmax;

public:
	extent_3d();
	~extent_3d();

}extent;


//�Ժ��ٶ���ƽ��
typedef class  planar_3d
{
public:
	//ƽ�淽�̵�ϵ��A
	double A;
	//ƽ�淽�̵�ϵ��B
	double B;
	//ƽ�淽�̵�ϵ��C
	double C;
	//ƽ�淽�̵�ϵ��D
	double D;
public:
	planar_3d();
	~planar_3d();

} planar;     //12.3�ռ�




#endif 