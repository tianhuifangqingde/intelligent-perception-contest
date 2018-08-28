#ifndef _GPS_ECEF_H
#define _GPS_ECEF_H

#include <stdio.h>
#include <cmath>
//XY坐标转GPS经纬度
static double NAV_EQUATORIAL_RADIUS = (6378.137 * 1000.0);			    // meters
static double NAV_FLATTENING = 1.0 / 298.257223563;			    // WGS-84
static double pi = 3.14159265;
static double DEG_TO_RAD = pi / 180.0f;
static double doubleNAV_E_2 = NAV_FLATTENING * (2.0 - NAV_FLATTENING);
static double NAV_E_2 = NAV_FLATTENING * (2.0 - NAV_FLATTENING);

typedef struct Radius
{
	double r1;
	double r2;
}Radius;

typedef struct Pos
{
	double x;
	double y;
	double height;
}Pos;

//XY坐标转GPS经纬度
struct Radius CalcEarthRadius(double lat)
{
	Radius radius;
	static double sinLat2 = 0;
	sinLat2 = sin(lat * (double)DEG_TO_RAD);
	sinLat2 = sinLat2 * sinLat2;
	radius.r1 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD * ((double)1.0 - (double)NAV_E_2) /(double)pow((double)1.0 - ((double)NAV_E_2 * sinLat2), ((double)3.0 / (double)2.0));
	radius.r2 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD / sqrt((double)1.0 - ((double)NAV_E_2 * sinLat2)) * cos(lat * (double)DEG_TO_RAD);
	return radius;
}

//void CalcGlobalLocation(double posNorth, double posEast, double height,double r1,double r2)
//{
//	GPS_W_F = posNorth / (double)(r1 + 0.1) + local_Lat;
//	GPS_J_F = posEast / (double)(r2 + 0.1) + local_Lon;
//	GPS_H_F = height;
//}

struct Pos Offset(double GPS_Lat_now,double GPS_Lon_now,double GPS_Lat_init,double GPS_Lon_init,double r1,double r2)
{
	Pos pos;
	pos.y = (float)((GPS_Lat_now - GPS_Lat_init)*(double)(r1 + 0.1));
	pos.x = (float)((GPS_Lon_now - GPS_Lon_init)*(double)(r2 + 0.1));
	return pos;
}



//经纬高转ECEF
void GPStoECEF(double lon, double lat, double height,double ECEF_X, double ECEF_Y, double ECEF_Z)
{
	lon = lon / 180 * pi;
	lat = lat / 180 * pi;
	double setaparametera = 6378137.0;
	double setaparametere2 = NAV_E_2;
	double radarrn = setaparametera / sqrt(1 - setaparametere2 * sin(lat) * sin(lat));
	ECEF_X = (radarrn + height) * cos(lat) * cos(lon);
	ECEF_Y = (radarrn + height) * cos(lat) * sin(lon);
	ECEF_Z = (radarrn * (1 - setaparametere2) + height) *sin(lat);
}


#endif
