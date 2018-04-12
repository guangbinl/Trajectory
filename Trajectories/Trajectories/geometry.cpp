#include "geometry.h"
#include <iostream>
#include <numeric>

//******************高斯卷积核生成函数*************************
void getGaussianKernel(const int size, const double sigma, vector<double>& maxtrix)
{
	maxtrix.resize(size*size);
	const double Pi = 4.0*atan(1.0); //圆周率π赋值
	int center = size / 2;
	double sum = 0;
	for (int i = 0; i<size; i++)
	{
		for (int j = 0; j<size; j++)
		{
			maxtrix[i*size +j] = (1 / (2 * Pi*sigma*sigma))*exp(-((i - center)*(i - center) + (j - center)*(j - center)) / (2 * sigma*sigma));
			sum += maxtrix[i*size + j];
		}
	}
	for (int i = 0; i < size; i++)
	{
		for (int j = 0; j < size; j++)
		{
			maxtrix[i*size + j] /= sum;
		}
	}
	return;
}

void gaussianFilter(const vector<tagSpace>& src_vec, const int size, double sigma, vector<tagSpace>& dest_vec)
{
	vector<double> gauss_maxtrix;
	getGaussianKernel(size, sigma, gauss_maxtrix);
	if (gauss_maxtrix.empty())
		return;

	if (src_vec.empty())
		return;
	dest_vec = src_vec;

	// 讲向量内容填充进 N*N的矩阵中

	int  matrixN = size * size;

	double weightSum = 0;		// 权重总和
	/*for (unsigned i = 0; i < gauss_maxtrix.size(); i++)
	{
		weightSum += gauss_maxtrix[i];
	}*/

	int height = ceil(src_vec.size()*1.0 / matrixN);
	int width = height;
	if (height <= size/2 || width <= size/2)
		return;

	for (int j = size/2; j < height - size/2; j++)
	{
		for (int i = size/2; i < width - size/2; i++)
		{
			double sumX = 0, sumY = 0;
			int index = 0;
			weightSum = 0;
			for (int m = j - size/2; m < j + size/2 + 1; m++)
			{
				for (int n = i - size/2; n<i + size/2 + 1; n++)
				{
					if (j*width + i == 35)
						int a = 0;

					if (m*width + n >src_vec.size() - 1)
						return;

					gauss_maxtrix[(m*width + n) % matrixN] = exp(pow(src_vec[m*width + n].datateime - src_vec[j*width + i].datateime, 2)*-1 / 2);
					weightSum += gauss_maxtrix[(m*width + n) % matrixN];
					sumX += src_vec[m*width + n].x * gauss_maxtrix[(m*width + n) % matrixN];
					sumY += src_vec[m*width + n].y * gauss_maxtrix[(m*width + n) % matrixN];
				}
			}

			sumX /= weightSum;
			sumY /= weightSum;
			dest_vec[j*width + i].x = sumX;
			dest_vec[j*width + i].y = sumY;
		}
	}
}


double calVariance(tagSpace startpt, tagSpace endPt)
{
	double distance = sqrt((startpt.x - endPt.x)*(startpt.x - endPt.x) + (startpt.y - endPt.y)*(startpt.y - endPt.y));
	return distance;
}

double calAngle(tagSpace startPt, tagSpace curPt, tagSpace endPt)
{
	double delta = (curPt.x - startPt.x)*(endPt.x - curPt.x) + (curPt.y - startPt.y)*(endPt.x - curPt.y);
	double start_square = sqrt((curPt.x - startPt.x)*(curPt.x - startPt.x) + (curPt.y - startPt.y)*(curPt.y - startPt.y));
	double end_square = sqrt((endPt.x - curPt.x)*(endPt.x - curPt.x) + (endPt.x - curPt.y)*(endPt.x - curPt.y));
	
	double angle = acos(delta/(start_square*end_square));
	return angle;
}

void getBSplineCurve(vector<tagSpace>& point_vec, vector<tagSpace>&out_point_vec)
{
	if (point_vec.size() < 3)
	{
		out_point_vec = point_vec;
		return;
	}

	double dt = 1.0 / point_vec.size();

	for (size_t j = 0; j < point_vec.size() - 3; j++)
	{
		double a0 = (point_vec[j].x + 4 * point_vec[j + 1].x + point_vec[j + 2].x) / 6;
		double a1 = -(point_vec[j].x - point_vec[j + 2].x) / 2;
		double a2 = (point_vec[j + 2].x - 2 * point_vec[j + 1].x + point_vec[j].x) / 2;
		double a3 = -(point_vec[j].x - 3 * point_vec[j + 1].x + 3 * point_vec[j + 2].x - point_vec[j + 3].x)/6;


		double b0 = (point_vec[j].y + 4 * point_vec[j + 1].y + point_vec[j + 2].y) / 6;
		double b1 = -(point_vec[j].y - point_vec[j + 2].y) / 2;
		double b2 = (point_vec[j + 2].y - 2 * point_vec[j + 1].y + point_vec[j].y) / 2;
		double b3 = -(point_vec[j].y - 3 * point_vec[j + 1].y + 3 * point_vec[j + 2].y - point_vec[j + 3].y)/6;

		for (size_t i = 0; i < point_vec.size(); i++)
		{
			double t = i * dt;
			double t2 = t * t;
			double t3 = t2 * t;

			double xa = a0 + a1 * t + a2 * t2 + a3 * t3;
			double ya = b0 + b1 * t + b2 * t2 + b3 * t3;

			tagSpace pt;
			pt.x = xa;
			pt.y = ya;
			out_point_vec.push_back(pt);
		}

	}
}