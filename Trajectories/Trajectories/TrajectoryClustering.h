#ifndef TRAJECTORY_CLUSTER
#define TRAJECTORY_CLUSTER
#include "geometry.h"
#include "GPSinfoDeal.h"
class TrajectoryClustering
{
public:
	TrajectoryClustering();
	TrajectoryClustering(map<string, map<string, GPSINFO>>& gps_info);
	~TrajectoryClustering();
	/*************************
	针对单条轨迹, 进行去除冗余、聚类、平滑
	****************************/
	// 去除冗余的点
	void removeRedundancy();

	// 轨迹点聚类,主要用于去除停滞点
	void singleClustering();
	
	/*void  splitClusterPoint(map<string, vector<pair<int, vector<int>>>>&traject_map, map<string, vector<vector<int>>>& out_traject_center_map);*/

	// 轨迹平滑
	void trajectorySmoothing();



	// 生成道路中心线
	void roadLineGeneration();

	// 生成道路中线线需要对轨迹点聚类
	void clustring(vector<tagSpace>& pt_src, double radius, int minPts, vector<tagSpace>& out_point_set);

	void clustringSlit(vector<tagSpace>& point_vec, double angle, double distance, vector<vector<tagSpace>>& out_point_set);

	// 道路预处理
	void prevDealPoint(vector<tagSpace>& point_set, double distance);


	/*****************************************
	多条轨迹进行分割-聚类-融合
	******************************************/
	// 轨迹切割分段
	void trajectorySplit();

	// 轨迹聚类
	void trajectoryClustering();

	// 多条轨迹融合
	void trajectoryMerge();

	// 路口检测
	void intersectionDetect();


	// 设置数据源
	void setDataSource(map<string, map<string, GPSINFO>>& gps_info);


	// 结构转换
	void getEveryGpsPointFromMap();
	
	map<string, VectorPoints> getPointVector(){ return m_point_vec; }


protected:
	// 输出结构  pair(中心点索引号----》邻域所有符合条件的节点的索引号) 所有的索引号针对输入变量pt_src 密度聚类算法，主要去除停滞点
	void  densityClustering(vector<tagSpace>& pt_src, double radius, int minPts, vector<pair<int, vector<pair<int, double>>>>& out_point_set);
private:
	// 存储结构定义
	map<string, map<string, GPSINFO>> m_gps_info;
	map<string, VectorPoints>m_point_vec;
};
#endif // !TRAJECTORY_CLUSTER