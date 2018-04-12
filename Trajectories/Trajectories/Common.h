/**************************************************************************************
	��ع��������Ķ���
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

const int radius = 6378137;			// ������뾶
const double flatrate = 1 / 298.257222101; // ����

// �ռ�ֱ������ϵ�ṹ
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

// �������ϵ�ṹ
typedef struct tagGround {
	double longitude;
	double latitude;
	double height;
}Ground;
typedef Ground *pGround;


class CommonLibary
{
public:
	// �����Ӧ���ַ�
	static void Trim(string& source, char trim_char);

	// �ָ����е��ֶε���Ӧ�Ľṹ��
	static void split(const string &source, const string &pattern, vector<string>& result);

	static void getAllFiles(string path, vector<string>& files);
};

class CoordinateConvert		// ����ת��
{
public:
	//�ȷ���ת��Ϊ����
	static void DMS_RAD(double DMS, double *Rad);
	// ����ת��Ϊ�ȷ���
	static void RAD_DMS(double Rad, double *DMS);
	// �ռ�ֱ������ϵת��Ϊ�������ϵ
	static bool Space2Ground(pSpace pS, pGround pG, double a = radius, double f = flatrate);
	// �������ϵת��Ϊ�ռ�ֱ������ϵ
	static bool Ground2Space(pGround pG, pSpace pS, double a = radius, double f = flatrate);
};
#endif // !

