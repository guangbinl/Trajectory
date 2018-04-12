#ifndef  GEOMETRY
#define  GEOMETRY
#include <math.h>
#include "Common.h"
using namespace std;

//******************��˹��������ɺ���*************************
//��һ������size�Ǹ�˹����˵ĳߴ��С��
//�ڶ�������sigma�Ǿ���˵ı�׼��
//��������������ֵgaus��һ��ָ���и�double���������ָ�룻
// �ر�ע��gaus ����ֵ��Ӧ���ڴ���Ҫ�Լ��ͷ�
//*************************************************************
void getGaussianKernel(const int size, const double sigma, vector<double>& maxtrix);

void gaussianFilter(const vector<tagSpace>& src_vec, const int size, double sigma, vector<tagSpace>& dest_vec);

// ���㷽��
double calVariance(tagSpace startTt, tagSpace endPt);

// ����ת��
double calAngle(tagSpace startPt, tagSpace curPt, tagSpace endPt);

// �ܶȾ����㷨
void densityClustering();

// ���׾���B������
void getBSplineCurve(vector<tagSpace>& point_vec, vector<tagSpace>&out_point_vec);

#endif
