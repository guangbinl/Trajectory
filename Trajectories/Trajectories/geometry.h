#ifndef  GEOMETRY
#define  GEOMETRY
#include <math.h>
#include "Common.h"
using namespace std;

//******************高斯卷积核生成函数*************************
//第一个参数size是高斯卷积核的尺寸大小；
//第二个参数sigma是卷积核的标准差
//第三个参数返回值gaus是一个指向含有个double类型数组的指针；
// 特别注意gaus 返回值对应的内存需要自己释放
//*************************************************************
void getGaussianKernel(const int size, const double sigma, vector<double>& maxtrix);

void gaussianFilter(const vector<tagSpace>& src_vec, const int size, double sigma, vector<tagSpace>& dest_vec);

// 计算方差
double calVariance(tagSpace startTt, tagSpace endPt);

// 计算转角
double calAngle(tagSpace startPt, tagSpace curPt, tagSpace endPt);

// 密度聚类算法
void densityClustering();

// 三阶均匀B样曲线
void getBSplineCurve(vector<tagSpace>& point_vec, vector<tagSpace>&out_point_vec);

#endif
