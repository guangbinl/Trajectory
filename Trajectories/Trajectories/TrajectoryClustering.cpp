#include "TrajectoryClustering.h"
#include <math.h>
#include "Common.h"
#include <algorithm>

const double maxdistance = 100;
const double minPts = 10;
const double angle= 60;
const double criticalRadius = 100;
const double mindistance = 100;
const int  minPointNumber = 80;
const double speedLimt = 25;


struct sort_pair
{
	bool operator()(const std::pair<int, double> &left, const std::pair<int, double> &right) {
		return left.second < right.second;
	}
};


TrajectoryClustering::TrajectoryClustering()
{

}
TrajectoryClustering::TrajectoryClustering(map<string, map<string, GPSINFO>>& gps_info)
{
	m_gps_info.clear();
	m_gps_info = gps_info;
	getEveryGpsPointFromMap();
}
TrajectoryClustering::~TrajectoryClustering()
{

}
void TrajectoryClustering::setDataSource(map<string, map<string, GPSINFO>>& gps_info)
{
	m_gps_info = gps_info; 
	getEveryGpsPointFromMap();
}
void TrajectoryClustering::removeRedundancy()
{
	double distance = 0;
	Space pt, prev;
	vector<Space> ground_vec;

	
	map<string, vector<tagSpace>>::iterator traject_itr = m_point_vec.begin();
	for (; traject_itr != m_point_vec.end(); traject_itr++)
	{
		vector<tagSpace> delete_pt_vec;
		VectorPoints::iterator pointer_itr = traject_itr->second.begin();
		if (traject_itr->second.size()<=2)
			continue;
		prev = *traject_itr->second.begin();
		for (pointer_itr = traject_itr->second.begin() + 1; pointer_itr != traject_itr->second.end();)
		{
			pt.x = pointer_itr->x;
			pt.y = pointer_itr->y;
			pt.datateime = pointer_itr->datateime;

			// 判断速度
			double speed = sqrt(pow((pt.x - prev.x) / (pt.datateime - prev.datateime), 2) + pow((pt.y - prev.y) / (pt.datateime - prev.datateime), 2));
			if (speed > speedLimt)
				pointer_itr = traject_itr->second.erase(pointer_itr);
			else
				pointer_itr++;

			prev = pt;
		}
	}

	return;
}

void TrajectoryClustering::getEveryGpsPointFromMap()
{
	m_point_vec.clear();
	std::map<string, map<string, GPSINFO>>::iterator  file_itr = m_gps_info.begin();
	for (; file_itr != m_gps_info.end(); file_itr++)
	{
		map<string, GPSINFO>::iterator point_itr = file_itr->second.begin();
		tagSpace point;
		for (; point_itr != file_itr->second.end(); point_itr++)
		{

			point.x = atof(point_itr->second.lon.c_str()) * MESH1_E_S * MESH_MIN_UNIT;
			point.y = atof(point_itr->second.lat.c_str()) * MESH1_E_S * MESH_MIN_UNIT;
			point.z = 0;
			point.datateime = atoll(point_itr->second.dateTime.c_str());
			m_point_vec[file_itr->first].push_back(point);
		}
	}
}

void TrajectoryClustering::trajectorySmoothing()
{
	VectorPoints dest_vec;
	map<string, VectorPoints>::iterator file_itr = m_point_vec.begin();
	for (; file_itr != m_point_vec.end(); file_itr++)
	{
		gaussianFilter(file_itr->second, 5, 1, dest_vec);
		file_itr->second = dest_vec;
	}
	return;
}

void  TrajectoryClustering::densityClustering(vector<tagSpace>& pt_src, double currRadius, int minPts, vector<pair<int, vector<pair<int, double>>>>& out_point_vec)
{
	double distance = 0;
	for (size_t i=0; i < pt_src.size(); i++)
	{
		vector<pair<int, double>> currNeighbors;
		tagSpace currPt = pt_src[i];
		for (size_t j = 0; j < pt_src.size(); j++)
		{
			if (i != j)
			{
				tagSpace currTestPt = pt_src[j];
				if (currTestPt.x >= currPt.x - currRadius &&
					currTestPt.x <= currPt.x + currRadius &&
					currTestPt.y >= currPt.y - currRadius &&
					currTestPt.y <= currPt.y + currRadius
					)
				{
					double distance = sqrt(pow(currPt.x - currTestPt.x, 2) + pow(currPt.y - currTestPt.y, 2));
					if (distance <= currRadius)
					{
						pair<int, float> currPair(j, distance);
						currNeighbors.push_back(currPair);
					}
						
				}
			}
			if (currNeighbors.size() >= minPts*3)
				break;
		}

		if (currNeighbors.size() > minPointNumber) {
			vector<pair<int, double>> resizeCurrNeighbors;
			sort(currNeighbors.begin(), currNeighbors.end(), sort_pair()); // sort into ascending order
			for (int i = 0; i < minPointNumber; i++) {
				resizeCurrNeighbors.push_back(currNeighbors[i]);
			}
			currNeighbors = resizeCurrNeighbors; // update currNeighbors
		}

		out_point_vec.push_back(make_pair(i,currNeighbors));

		if (currNeighbors.size() >= 10)
		{

			// create a weighting function for currPoint after getting all of its neighbors
			std::vector<double> weights;
			double weightsSum = 0.0;
			for (size_t l = 0; l < currNeighbors.size(); l++) {
				double currDist = currNeighbors[l].second;
				double currWeight = exp(-pow(currDist, 2.0) / pow(currRadius, 2.0));
				weights.push_back(currWeight);
				weightsSum = weightsSum + currWeight;
			}
			for (size_t m = 0; m < weights.size(); m++) {
				weights[m] = weights[m] / weightsSum;
			}

			double xSum = 0.0;
			double ySum = 0.0;
			for (size_t n = 0; n < currNeighbors.size(); n++) {
				int currNeighborIndex = currNeighbors[n].first;
				tagSpace currNeighborPoint = pt_src[currNeighborIndex];
				xSum += currNeighborPoint.x * weights[n];
				ySum += currNeighborPoint.y * weights[n];
			}

			pt_src[i].x = xSum;
			pt_src[i].y = ySum;
		}
	}
}

// 生成道路中线线需要对轨迹点聚类
void TrajectoryClustering::clustring(vector<tagSpace>& pt_src, double currRadius, int minPts, vector<tagSpace>& out_point_set)
{
	vector<int> record_vec;
	vector<int> index_point_vec;
	double distance = 0;
	out_point_set.clear();
	for (size_t i = 0; i < pt_src.size(); i++)
	{
		if (find(index_point_vec.begin(), index_point_vec.end(), i) != index_point_vec.end())
			continue;

		index_point_vec.push_back(i);
		vector<pair<int, double>> currNeighbors;
		tagSpace currPt = pt_src[i];
		for (size_t j = i+1; j < pt_src.size(); j++)
		{
			if (find(index_point_vec.begin(), index_point_vec.end(),j) != index_point_vec.end()) 
				continue;
			if (i != j)
			{
				tagSpace currTestPt = pt_src[j];
				if (currTestPt.x >= currPt.x - currRadius &&
					currTestPt.x <= currPt.x + currRadius &&
					currTestPt.y >= currPt.y - currRadius &&
					currTestPt.y <= currPt.y + currRadius
					)
				{
					double distance = sqrt(pow(currPt.x - currTestPt.x, 2) + pow(currPt.y - currTestPt.y, 2));
					if (distance <= currRadius)
					{
						pair<int, float> currPair(j, distance);
						currNeighbors.push_back(currPair);
						index_point_vec.push_back(j);
					}

				}
			}
		}

		tagSpace  clusterPt = pt_src[i];

		// create a weighting function for currPoint after getting all of its neighbors
		std::vector<double> weights;
		double weightsSum = 0.0;
		for (size_t l = 0; l < currNeighbors.size(); l++) {
			double currDist = currNeighbors[l].second;
			double currWeight = exp(-pow(currDist, 2.0) / pow(currRadius, 2.0));
			weights.push_back(currWeight);
			weightsSum = weightsSum + currWeight;
		}
		for (size_t m = 0; m < weights.size(); m++) {
			weights[m] = weights[m] / weightsSum;
		}

		double xSum = 0.0;
		double ySum = 0.0;
		if (currNeighbors.size())
		{
			for (size_t n = 0; n < currNeighbors.size(); n++) {
				int currNeighborIndex = currNeighbors[n].first;
				tagSpace currNeighborPoint = pt_src[currNeighborIndex];
				xSum += currNeighborPoint.x * weights[n];
				ySum += currNeighborPoint.y * weights[n];
			}

			clusterPt.x = xSum;
			clusterPt.y = ySum;
		}
		
		out_point_set.push_back(clusterPt);
	}
}
void TrajectoryClustering::clustringSlit(vector<tagSpace>& point_vec, double angle, double distance ,vector<vector<tagSpace>>& out_point_set)
{
	vector<tagSpace> pt_vec;
	if (point_vec.size() < 3)
	{
		pt_vec = point_vec;
		out_point_set.push_back(pt_vec);
		return;
	}

	vector<pair<int, pair<double, double>>> point_index_angle_vec;
	tagSpace startPt, curPt, endPt;
	// 计算距离-角度
	for(size_t i = 0; i < point_vec.size()-2; i++)
	{
		startPt = point_vec[i];
		curPt = point_vec[i + 1];
		endPt = point_vec[i + 2];
		double curAngle = calAngle(startPt, curPt, endPt);
		double curdistance = calVariance(curPt, endPt);
		point_index_angle_vec.push_back(make_pair(i + 2, make_pair(curdistance, curAngle)));
	}

	// 距离-角度分组
	pt_vec.push_back(point_vec[0]);
	pt_vec.push_back(point_vec[1]);
	for (size_t i = 0; i < point_index_angle_vec.size(); i++)
	{
		int index = point_index_angle_vec[i].first;
		double curdistance = point_index_angle_vec[i].second.first;
		double curAngle = point_index_angle_vec[i].second.second;
		if (curAngle < angle  && curdistance < distance)
		{
			pt_vec.push_back(point_vec[index]);
		}
		else
		{
			out_point_set.push_back(pt_vec);
			pt_vec.clear();
			pt_vec.push_back(point_vec[i]);
		}
	}
	out_point_set.push_back(pt_vec);
}

void TrajectoryClustering::singleClustering()
{
	map<string, VectorPoints>::iterator traject_itr = m_point_vec.begin();
	map<string, vector<pair<int, vector<pair<int, double>>>>>  traject_map;
	for (; traject_itr != m_point_vec.end(); traject_itr++)
	{
		densityClustering(traject_itr->second, criticalRadius, minPointNumber, traject_map[traject_itr->first]);
	}
	return;
}

void TrajectoryClustering::prevDealPoint(vector<tagSpace>& point_vec, double currRaidus)
{
	vector<tagSpace> out_point_vec;
	vector<tagSpace>::iterator pt_itr = point_vec.begin();
	tagSpace startPt, currPt,endPt;
	startPt = *pt_itr;
	out_point_vec.push_back(startPt);
	for (pt_itr = point_vec.begin() + 1; pt_itr != point_vec.end();pt_itr++)
	{
		endPt = *pt_itr;
		if (calVariance(startPt, endPt) > 2*currRaidus)
		{
			currPt.x = (startPt.x + endPt.x) / 2;
			currPt.y = (startPt.y + endPt.y) / 2;
			currPt.datateime = (startPt.datateime + endPt.datateime) / 2;
			out_point_vec.push_back(currPt);
		}
		out_point_vec.push_back(endPt);
		startPt = endPt;
	}
	point_vec = out_point_vec;
	return;
}


void TrajectoryClustering::roadLineGeneration()
{
	map<string, VectorPoints>::iterator file_itr = m_point_vec.begin();
	for (; file_itr != m_point_vec.end(); file_itr++)
	{
		vector<tagSpace> out_point_vec;
		// 聚类
		clustring(file_itr->second, criticalRadius, minPts, out_point_vec);


		// 插点
		prevDealPoint(out_point_vec, criticalRadius);
		
		// 切割
		vector<vector<tagSpace>> cluster_pt_vec;
		clustringSlit(out_point_vec, PI / 3,  maxdistance, cluster_pt_vec);

		//拟合
		// B样条曲线计算
		out_point_vec.clear();
		vector<tagSpace> temp_point_vec;
		for (size_t j = 0; j < cluster_pt_vec.size(); j++)
		{
			temp_point_vec.clear();
			getBSplineCurve(cluster_pt_vec[j], temp_point_vec);
			out_point_vec.insert(out_point_vec.end(), temp_point_vec.begin(), temp_point_vec.end());
		}

		file_itr->second = out_point_vec;
	}
	return;
}
