#include "stdafx.h"
////#include "opencv2/opencv.hpp"
////
////using namespace cv;
////using namespace std;
////
////int _tmain(int argc, _TCHAR* argv[])
////{
////	Mat src;
////	//加载图片
////	src = imread("F://Opencv_picture//44.jpg", CV_LOAD_IMAGE_COLOR);
////	//检测是否加载成功
////	if (!src.data)  //or == if(src.empty())
////	{
////		cout << "Could not open or find the image" << endl;
////		return -1;
////	}
////	// 创建窗口
////	namedWindow("Display", CV_WINDOW_AUTOSIZE);
////	//显示图像
////	imshow("Display", src);
////
////	//暂停，等待按键结束
////	waitKey(0);
////
////	return 0;
////}
//
//
////using namespace cv;
////
////int main()
////{
////	// 读入一张图片（游戏原画）    
////	Mat img = imread("test.png");
////	// 创建一个名为 "游戏原画"窗口    
////	namedWindow("游戏原画");
////	// 在窗口中显示游戏原画    
////	imshow("游戏原画", img);
////	// 等待6000 ms后窗口自动关闭    
////	waitKey(600000000000);
////	return 0;
////}
//
//
//
////
////#include "stdafx.h"
////#include <opencv2/opencv.hpp>
////#include <iostream>
////
////using namespace cv;
////int main(int argc, char** argv) {
////	Mat src, dst;
////	//src = imread("F://Opencv_picture//44.jpg", CV_LOAD_IMAGE_COLOR);
////	src = imread("test.png");
////	//src = imread("D:/vcprojects/images/test.png");
////	if (!src.data) {
////		printf("could not load image...\n");
////		return -1;
////	}
////	char input_win[] = "input image";
////	cvtColor(src, src, CV_BGR2GRAY);
////	namedWindow(input_win, CV_WINDOW_AUTOSIZE);
////	imshow(input_win, src);
////
////	// contrast and brigthtness changes 
////	int height = src.rows;
////	int width = src.cols;
////	dst = Mat::zeros(src.size(), src.type());
////	float alpha = 1.2;
////	float beta = 30;
////
////	Mat m1;
////	float max = src.at<uchar>(0, 0);
////	float min = src.at<uchar>(0, 0);
////	src.convertTo(m1, CV_32F);
////	for (int row = 0; row < height; row++)
////	{
////		for (int col = 0; col < width; col++)
////		{
////
////			if (src.channels() == 1) {
////				float v = src.at<uchar>(row, col);
////
////				if (v > max)
////				{
////					max = v;
////				}
////
////				if (v < min)
////				{
////					min = v;
////				}
////
////			}
////		}
////	}
////	int b = 0;
////	for (int row = 0; row < height; row++) {
////		for (int col = 0; col < width; col++) {
////			if (src.channels() == 3) {
////				float b = m1.at<Vec3f>(row, col)[0];// blue
////				float g = m1.at<Vec3f>(row, col)[1]; // green
////				float r = m1.at<Vec3f>(row, col)[2]; // red
////
////				// output
////				dst.at<Vec3b>(row, col)[0] = saturate_cast<uchar>(b*alpha + beta);
////				dst.at<Vec3b>(row, col)[1] = saturate_cast<uchar>(g*alpha + beta);
////				dst.at<Vec3b>(row, col)[2] = saturate_cast<uchar>(r*alpha + beta);
////			}
////			else if (src.channels() == 1) {
////				float v = src.at<uchar>(row, col);
////				float a = ((v - min) / (max - min))*255;
////				if (a!=0)
////				{
////					b++;
////				}
////				dst.at<uchar>(row, col) = saturate_cast<uchar>(a);
////			}
////		}
////	}
////
////	char output_title[] = "contrast and brightness change demo";
////	namedWindow(output_title, CV_WINDOW_AUTOSIZE);
////	imshow(output_title, dst);
////	imwrite("test2_h.png", dst);
////	waitKey(0);
////	return 0;
////}
//
//
//
//
//#include<iostream>
//#include<fstream>
//#include<opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui_c.h>
//using namespace std;
////using namespace cv;
//
//void bbb(cv::Mat &src, cv::Mat &dst)
//{
//	cvtColor(src, src, CV_BGR2GRAY);
//	int height = src.rows;
//	int width = src.cols;
//	dst = cv::Mat::zeros(src.size(), src.type());
//	float alpha = 1.2;
//	float beta = 30;
//
//	cv::Mat m1;
//	float max = src.at<uchar>(0, 0);
//	float min = src.at<uchar>(0, 0);
//	src.convertTo(m1, CV_32F);
//	for (int row = 0; row < height; row++)
//	{
//		for (int col = 0; col < width; col++)
//		{
//
//			if (src.channels() == 1) {
//				float v = src.at<uchar>(row, col);
//
//				if (v > max)
//				{
//					max = v;
//				}
//
//				if (v < min)
//				{
//					min = v;
//				}
//
//			}
//		}
//	}
//	int b = 0;
//	for (int row = 0; row < height; row++) {
//		for (int col = 0; col < width; col++) {
//			if (src.channels() == 3) {
//				float b = m1.at<cv::Vec3f>(row, col)[0];// blue
//				float g = m1.at<cv::Vec3f>(row, col)[1]; // green
//				float r = m1.at<cv::Vec3f>(row, col)[2]; // red
//
//				// output
//				dst.at<cv::Vec3b>(row, col)[0] = cv::saturate_cast<uchar>(b*alpha + beta);
//				dst.at<cv::Vec3b>(row, col)[1] = cv::saturate_cast<uchar>(g*alpha + beta);
//				dst.at<cv::Vec3b>(row, col)[2] = cv::saturate_cast<uchar>(r*alpha + beta);
//			}
//			else if (src.channels() == 1) {
//				float v = src.at<uchar>(row, col);
//				float a = ((v - min) / (max - min)) * 255;
//				if (a != 0)
//				{
//					b++;
//				}
//				dst.at<uchar>(row, col) = cv::saturate_cast<uchar>(a);
//			}
//		}
//	}
//
//
//
//}
//
//void aaa(cv::Mat &src, cv::Mat &dst)
//{
//	float alpha = 1.2;
//	float beta = 30;
//	cv::Mat m1;
//	int height = src.rows;
//	int width = src.cols;
//	float max = src.at<uchar>(0, 0);
//	float min = src.at<uchar>(0, 0);
//	src.convertTo(m1, CV_32F);
//	for (int row = 0; row < height; row++)
//	{
//		for (int col = 0; col < width; col++)
//		{
//			/*float v = src.at<uchar>(row, col);
//
//			if (v > max)
//			{
//				max = v;
//			}
//
//			if (v < min)
//			{
//				min = v;
//			}*/
//			/*if (src.channels() == 1) {
//				float v = src.at<uchar>(row, col);
//
//				if (v > max)
//				{
//					max = v;
//				}
//
//				if (v < min)
//				{
//					min = v;
//				}
//
//			}*/
//			float v = src.at<uchar>(row, col);
//
//			if (v > max)
//			{
//				max = v;
//			}
//
//			if (v < min)
//			{
//				min = v;
//			}
//		}
//	}
//	int b = 0;
//	for (int i = 0; i < height; i++)
//	{
//		for (int j= 0; j < width; j++)
//		{
//			float v = src.at<uchar>(i, j);
//			float a = ((v - min) / (max - min)) * 255;
//			/*if (a != 0)
//			{
//				b++;
//			}*/
//			dst.at<uchar>(i, j) = cv::saturate_cast<uchar>(a);
//			//if (src.channels() == 3) {
//			//	float b = m1.at<Vec3f>(row, col)[0];// blue
//			//	float g = m1.at<Vec3f>(row, col)[1]; // green
//			//	float r = m1.at<Vec3f>(row, col)[2]; // red
//
//			//	// output
//			//	dst.at<Vec3b>(row, col)[0] = saturate_cast<uchar>(b*alpha + beta);
//			//	dst.at<Vec3b>(row, col)[1] = saturate_cast<uchar>(g*alpha + beta);
//			//	dst.at<Vec3b>(row, col)[2] = saturate_cast<uchar>(r*alpha + beta);
//			//}
//			//else if (src.channels() == 1)
//			//{
//			//	float v = src.at<uchar>(row, col);
//			//	float a = ((v - min) / (max - min)) * 255;
//			//	if (a != 0)
//			//	{
//			//		b++;
//			//	}
//			//	dst.at<uchar>(row, col) = saturate_cast<uchar>(a);
//			//}
//			/*float v = src.at<uchar>(row, col);
//			float a = ((v - min) / (max - min)) * 255;
//			if (a != 0)
//			{
//				b++;
//			}
//			dst.at<uchar>(row, col) = saturate_cast<uchar>(a);*/
//		}
//	}
//
//
//
//}
//
//
//
//int main()
//{
//	//单个图像名称
//	string name;
//	//单个图片全名地址
//	string input_image_name;
//	//单个图片保存地址
//	string save_image_name;
//	////存储图片路径
//	//string input_image_path = "C:/Users/yang/Desktop/扫描方向点间距/华静导师论文/数据集/004/Images_png/000222_13_02/";
//	////保存图片文件路径
//	//string save_image_path = "C:/Users/yang/Desktop/扫描方向点间距/华静导师论文/数据集/004/Images_png/000222_13_02_2/";
//	////图片名称列表
//	//string image_list = "C:/Users/yang/Desktop/扫描方向点间距/华静导师论文/数据集/004/Images_png/000222_13_02.txt";
//	// 创建文件流接口
//
//		//存储图片路径
//	string input_image_path = "C:/Users/yang/Desktop/扫描方向点间距/华静导师论文/数据集/004/Images_png/000530_01_01/";
//	//保存图片文件路径
//	string save_image_path = "C:/Users/yang/Desktop/扫描方向点间距/华静导师论文/数据集/004/Images_png/000530_01_01_2/";
//	//图片名称列表
//	string image_list = "C:/Users/yang/Desktop/扫描方向点间距/华静导师论文/数据集/004/Images_png/000530_01_01.txt";
//
//
//
//	ifstream str_file(image_list);
//	while (getline(str_file, name))
//	{
//		//单个图片全名
//		input_image_name = input_image_path + name;
//		cout << input_image_name << endl;
//		cv::Mat src = cv::imread(input_image_name);
//		//省去图像处理过程
//		//*****
//		cv::Mat dst;
//		dst = cv::Mat::zeros(src.size(), src.type());
//
//		bbb(src, dst);
//		//dst = src + 100;
//		save_image_name = save_image_path + name;
//		cout << save_image_name << endl;
//		cv::imwrite(save_image_name, dst);
//
//	}
//	return 0;
//}
//
//
//
//
////
//////<span style = "font-size:18px;">
//////批量读写。再添加以前的处理就OK啦
////#include<opencv2/opencv.hpp>
////#include<iostream>
////#include<time.h>
////#include<fstream>
////
////using namespace std;
////using namespace cv;
////
////void main()
////{
////	//ifstream file("C:/Users/Administrator/Desktop/date/MIT/MIT人脸库/faces/face.txt");
////	ifstream file("C:/Users/yang/Desktop/扫描方向点间距/华静导师论文/数据集/010/Images_png/批量文件名.txt");
////
////	int img_index = 0;
////
////	while (!file.eof())
////	{
////		char txt_cont[200];
////		file.getline(txt_cont, 200);
////
////		char img_file[200], save_file[200];
////		sprintf(img_file, "C:/Users/yang/Desktop/扫描方向点间距/华静导师论文/数据集/010/Images_png/000529_09_01/%s", txt_cont);
////		//sprintf(img_file, "C:/Users/Administrator/Desktop/date/MIT/MIT人脸库/faces/%s", txt_cont);
////		//sprintf(save_file, "C:/Users/Administrator/Desktop/date/MIT/MIT人脸库/save/%d.jpg", img_index);
////		sprintf(save_file, "C:/Users/yang/Desktop/扫描方向点间距/华静导师论文/数据集/010/Images_png/000529_09_01/save/%d.jpg", img_index);
////		Mat src = imread(img_file);
////
////		img_index++;
////
////		imwrite(save_file, src);
////	}
////
////}
//////< / span>
//
//
//
//
// //opencv1.cpp: 定义控制台应用程序的入口点。
//
//
////#include "stdafx.h"
////#include<iostream>  
////#include <opencv2/core/core.hpp>  
////#include <opencv2/highgui/highgui.hpp>  
////#include <cv.h>
//////#include <highgui.h>
////#include <cxcore.h>
////#include <cvaux.h>
////#include <stdlib.h>
////#include <imgproc.hpp>
////
////using namespace cv;
////
////int main()
////{
////	//IplImage *src;
////	Mat src, dst;
////	src = imread("test.png");
////	//src = cvLoadImage("pout.bmp", 1);//原图
////	IplImage *dst = cvCreateImage(cvSize(src->width, src->height), IPL_DEPTH_8U, 1);
////
////
////	std::cout << "灰度线性变换..." << std::endl;
////	double fa = 1.0, fb = 0.0;
////	while (fa >= 0)
////	{
////		for (int i = 0; i < src->height; i++)
////		{
////			for (int j = 0; j < src->width; j++)
////			{
////				CvScalar s = cvGet2D(src, i, j);
////				s.val[0] = fa * s.val[0] + fb;
////				cvSet2D(dst, i, j, s);
////			}
////		}
////		cvNamedWindow("Image", 1);//创建窗口
////		cvShowImage("Image", dst);//显示图像
////		cvWaitKey(0); //等待按键
////		cvDestroyWindow("Image");//销毁窗口
////
////		std::cin >> fa >> fb;
////	}
////
////	std::cout << "灰度拉伸..." << std::endl;
////	double x1 = 0.0, y1 = 0.0, x2 = 1.0, y2 = 1.0;
////	while (x1 >= 0)
////	{
////		for (int i = 0; i < src->height; i++)
////		{
////			for (int j = 0; j < src->width; j++)
////			{
////				CvScalar s = cvGet2D(src, i, j);
////				if (s.val[0] < x1)
////				{
////					s.val[0] = y1 / x1 * s.val[0];
////				}
////				else if (s.val[0] <= x2)
////				{
////					s.val[0] = (y2 - y1) / (x2 - x1)*(s.val[0] - x1) + y1;
////				}
////				else
////				{
////					s.val[0] = (255 - y2) / (255 - x2)*(s.val[0] - x2) + y2;
////				}
////				cvSet2D(dst, i, j, s);
////			}
////		}
////		cvNamedWindow("Image", 1);//创建窗口
////		cvShowImage("Image", dst);//显示图像
////		cvWaitKey(0); //等待按键
////		cvDestroyWindow("Image");//销毁窗口
////
////		std::cin >> x1 >> y1 >> x2 >> y2;
////	}
////
////
////	std::cout << "灰度直方图..." << std::endl;
////
////	int table[256] = { 0 };
////	int range[] = { 0, 255 };
////	std::cin >> range[0] >> range[1];
////	for (int i = 0; i < src->height; i++)
////	{
////		for (int j = 0; j < src->width; j++)
////		{
////			CvScalar s = cvGet2D(src, i, j);
////			table[(int)s.val[0]]++;
////		}
////	}
////	double max = 0;
////	int sum = 0;
////	for (int i = 0; i < 256; i++)
////	{
////		sum += table[i];
////		if (table[i] > max)max = table[i];
////		//std::cout << table[i] << std:: endl;
////	}
////	Mat histgram(256, 256, CV_8U, Scalar(255));
////	for (int i = range[0]; i <= range[1]; i++)
////	{
////		line(histgram, Point(i, 255), Point(i, (255 - table[i] * 255 / max)), Scalar(0));
////	}
////	namedWindow("histgram");
////	imshow("histgram", histgram);
////	waitKey(0);
////
////
////	std::cout << "灰度均衡直方图..." << std::endl;
////	int new_table[256] = { 0 };
////	float s_table[2] = { (float)table[0] / sum };
////	for (int i = 1; i < 256; i++)
////	{
////		s_table[i % 2] = s_table[(i - 1) % 2] + (float)table[i] / sum;
////		new_table[i] = (int)(s_table[i % 2] * 255 + 0.5);
////	}
////	for (int i = 0; i < src->height; i++)
////	{
////		for (int j = 0; j < src->width; j++)
////		{
////			CvScalar s = cvGet2D(src, i, j);
////			s.val[0] = new_table[(int)s.val[0]];
////			cvSet2D(dst, i, j, s);
////		}
////	}
////
////
////	cvNamedWindow("Image", 1);//创建窗口
////	cvShowImage("Image", dst);//显示图像
////	cvWaitKey(0); //等待按键
////	cvDestroyWindow("Image");//销毁窗口
////
////
////	cvReleaseImage(&dst); //释放图像
////}
////
