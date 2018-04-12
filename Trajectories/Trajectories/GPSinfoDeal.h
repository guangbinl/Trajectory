#ifndef  GPS_INFO_DEAL
#define  GPS_INFO_DEAL
#include "Common.h"
#include "GpsDataStruct.h"
#include <map>
#include <string>

#define    MESH1_E_S                    3600					  // 1��Mesh���:�뵥λ(1��)
#define    MESH_MIN_UNIT                128                       // ��С��γ���뵥λ�ķ�ĸ
#define    UNIT_BL					(3600*128)

typedef vector<tagSpace>  VectorPoints;
class GpsinfoDeal
{
public:
	bool ReadGpsInfo(string&  gps_files_path);
	void GetRoadName(const string& gps_file_full_path, string& result);
	void DealWithDateTime(string& datetime);
	void getAllGpsPoint(VectorPoints& point_vec);
	void getEveryGpsPoint(map<string, VectorPoints>& point_vec);
	map<string, map<string, GPSINFO>> getGpsPointMap(){ return m_gps_info; }
private:
	//to store gps info
	std::map<string, map<string, GPSINFO>> m_gps_info;
};


#endif // !1



