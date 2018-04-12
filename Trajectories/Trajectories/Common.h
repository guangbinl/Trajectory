/**************************************************************************************
	相关公共函数的定义
**************************************************************************************/
#ifndef  COMMON_LIBARY
#define  COMMON_LIBARY
#include<iostream>
#include <vector>
#include <cmath>
#include <stdlib.h>
#include <io.h>
#include <string>
#include <fstream>
using namespace std;

#define PI 3.1415926

const int radius = 6378137;			// 长半轴半径
const double flatrate = 1 / 298.257222101; // 扁率

// 空间直角坐标系结构
typedef struct tagSpace {
	double x;
	double y;
	double z;
	long long datateime;
	tagSpace()
	{
		x = 0;
		y = 0;
		z = 0;
		datateime = 0;
	}
}Space;
typedef Space *pSpace;

// 大地坐标系结构
typedef struct tagGround {
	double longitude;
	double latitude;
	double height;
}Ground;
typedef Ground *pGround;


class CommonLibary
{
public:
	// 清除对应的字符
	static void Trim(string& source, char trim_char);

	// 分割所有的字段到对应的结构中
	static void split(const string &source, const string &pattern, vector<string>& result);

	static void getAllFiles(string path, vector<string>& files);
};

class CoordinateConvert		// 坐标转换
{
public:
	//度分秒转换为弧度
	static void DMS_RAD(double DMS, double *Rad);
	// 弧度转换为度分秒
	static void RAD_DMS(double Rad, double *DMS);
	// 空间直角坐标系转换为大地坐标系
	static bool Space2Ground(pSpace pS, pGround pG, double a = radius, double f = flatrate);
	// 大地坐标系转换为空间直角坐标系
	static bool Ground2Space(pGround pG, pSpace pS, double a = radius, double f = flatrate);
};
#endif // !

