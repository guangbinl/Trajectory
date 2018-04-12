#include "GPSinfoDeal.h"
#include "geometry.h"
#include "TrajectoryClustering.h"

int main()
{
	GpsinfoDeal gpsinfo;
	string path = "D:\\software\\T-drive Taxi Trajectories\\release\\taxi_log_2008_by_id";
	gpsinfo.ReadGpsInfo(path);

	map<string, VectorPoints> point_vec_map;
	gpsinfo.getEveryGpsPoint(point_vec_map);

	TrajectoryClustering obj(gpsinfo.getGpsPointMap());
	obj.removeRedundancy();
	obj.singleClustering();
	obj.trajectorySmoothing();

	obj.roadLineGeneration();

	point_vec_map.clear();
	point_vec_map = obj.getPointVector();

	
	string outpath = "D:\\data\\test.txt";
	ofstream outfile;
	outfile.open(outpath);
	
	outfile << std::fixed;
	VectorPoints point_vec;
	map<string, VectorPoints>::iterator traject_itr = point_vec_map.begin();
	for (; traject_itr != point_vec_map.end(); traject_itr++)
	{
		point_vec = traject_itr->second;
		for (unsigned i = 0; i < point_vec.size(); i++)
		{
			outfile << point_vec[i].x / (MESH1_E_S * MESH_MIN_UNIT) << "," << point_vec[i].y / (MESH1_E_S * MESH_MIN_UNIT) << "," << point_vec[i].z << " ";
		}
		outfile << endl;
	}
	outfile.close();
}