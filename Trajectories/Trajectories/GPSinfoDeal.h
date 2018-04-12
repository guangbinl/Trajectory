#ifndef  GPS_INFO_DEAL
#define  GPS_INFO_DEAL
#include "Common.h"
#include "GpsDataStruct.h"
#include <map>
#include <string>

#define    MESH1_E_S                    3600					  // 1次Mesh宽度:秒单位(1秒)
#define    MESH_MIN_UNIT                128                       // 最小经纬度秒单位的分母
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



