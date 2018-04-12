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
	��Ե����켣, ����ȥ�����ࡢ���ࡢƽ��
	****************************/
	// ȥ������ĵ�
	void removeRedundancy();

	// �켣�����,��Ҫ����ȥ��ͣ�͵�
	void singleClustering();
	
	/*void  splitClusterPoint(map<string, vector<pair<int, vector<int>>>>&traject_map, map<string, vector<vector<int>>>& out_traject_center_map);*/

	// �켣ƽ��
	void trajectorySmoothing();



	// ���ɵ�·������
	void roadLineGeneration();

	// ���ɵ�·��������Ҫ�Թ켣�����
	void clustring(vector<tagSpace>& pt_src, double radius, int minPts, vector<tagSpace>& out_point_set);

	void clustringSlit(vector<tagSpace>& point_vec, double angle, double distance, vector<vector<tagSpace>>& out_point_set);

	// ��·Ԥ����
	void prevDealPoint(vector<tagSpace>& point_set, double distance);


	/*****************************************
	�����켣���зָ�-����-�ں�
	******************************************/
	// �켣�и�ֶ�
	void trajectorySplit();

	// �켣����
	void trajectoryClustering();

	// �����켣�ں�
	void trajectoryMerge();

	// ·�ڼ��
	void intersectionDetect();


	// ��������Դ
	void setDataSource(map<string, map<string, GPSINFO>>& gps_info);


	// �ṹת��
	void getEveryGpsPointFromMap();
	
	map<string, VectorPoints> getPointVector(){ return m_point_vec; }


protected:
	// ����ṹ  pair(���ĵ�������----���������з��������Ľڵ��������) ���е�����������������pt_src �ܶȾ����㷨����Ҫȥ��ͣ�͵�
	void  densityClustering(vector<tagSpace>& pt_src, double radius, int minPts, vector<pair<int, vector<pair<int, double>>>>& out_point_set);
private:
	// �洢�ṹ����
	map<string, map<string, GPSINFO>> m_gps_info;
	map<string, VectorPoints>m_point_vec;
};
#endif // !TRAJECTORY_CLUSTER