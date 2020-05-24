#pragma once

#include "AC_Math.hpp"

struct Map_Projection
{
	bool initialized;
	double lat0_rad;
	double lon0_rad;
};

static inline bool map_projection_init(Map_Projection* m, double lat_0, double lon_0)
{
	if (!in_symmetry_range(lat_0, 90))
		return false;
	if (!in_symmetry_range(lat_0, 180))
		return false;
	m->lat0_rad = degree2rad(lat_0);
	m->lon0_rad = degree2rad(lon_0);
	m->initialized = true;
	return true;
}

static inline bool map_projection_project(const Map_Projection* m, double lat, double lon, double *x, double *y)
{
	if (!in_symmetry_range(lat, 90))
		return false;
	if (!in_symmetry_range(lat, 180))
		return false;
	
	lat = degree2rad(lat);
	lon = degree2rad(lon);

	double rad_distance = lat - m->lat0_rad;
	while (rad_distance > Pi)
		rad_distance -= 2 * Pi;
	while (rad_distance < -Pi)
		rad_distance += 2 * Pi;
	*y = rad_distance *rEarth;

	rad_distance = lon - m->lon0_rad;
	while (rad_distance > Pi)
		rad_distance -= 2 * Pi;
	while (rad_distance < -Pi)
		rad_distance += 2 * Pi;
	*x = rad_distance *rEarth*cos(lat);
	
	return true;
}

static inline bool map_projection_reproject(const Map_Projection* m, double x, double y, double *lat, double *lon)
{
	*lat = y / rEarth + m->lat0_rad;
	if (!in_symmetry_range(*lat, Pi / 2))
		return false;
	double lat_cosin = cos(*lat);
	if (is_zero(lat_cosin))
		return false;
	*lon = x / rEarth / lat_cosin + m->lon0_rad;
	while (*lon > Pi)
		*lon -= 2 * Pi;
	while (*lon < -Pi)
		*lon += 2 * Pi;
	*lat = rad2degree(*lat);
	*lon = rad2degree(*lon);

	return true;
}