/*
 *	Smala Map component
 *
 *	The copyright holders for the contents of this file are:
 *		Ecole Nationale de l'Aviation Civile, France (2021)
 *	See file "license.terms" for the rights and conditions
 *	defined by copyright holders.
 *
 *
 *	Contributors:
 *		Mathieu Magnaudet <mathieu.magnaudet@enac.fr>
 *
 */

#include "coords-utils.h"
#include <cmath>
#include <iostream>
/* 
 * conversion functions are taken from
 *  https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
 */
#define RADIUS 6378137.0

// zoom level to resolution, in meters per pixel for Web Mercator projection
// https://geoservices.ign.fr/documentation/services/api-et-services-ogc/images-tuilees-wmts-ogc
double m_per_px[22] = { 156543.0339280410,
                        78271.5169640205,
                        39135.7584820102,
                        19567.8792410051,
                        9783.9396205026,
                        4891.9698102513,
                        2445.9849051256,
                        1222.992452628,
                        611.4962262814,
                        305.7481131407,
                        152.8740565704,
                        76.4370282852,
                        38.2185141426,
                        19.1092570713,
                        9.5546285356,
                        4.7773142678,
                        2.3886571339,
                        1.1943285670,
                        0.5971642835,
                        0.2985821417,
                        0.1492910709,
                        0.0746455354 };

double get_resolution (int zoom) {
  return m_per_px [zoom];
}

double get_x0 () { return -20037508; }
double get_y0 () { return 20037508; }

static double to_radians (double deg)
{
	return deg * (M_PI/180);
}

static double to_degrees (double rad)
{
	return rad * (180/M_PI);
}
double lat2my (double lat) {
  return log (tan (M_PI / 4 + to_radians (lat) / 2)) * RADIUS;
}

double my2lat (double my) {
  return to_degrees (atan (exp(my / RADIUS)) * 2 - M_PI/2);
}

double lon2mx (double lon) {
	return to_radians (lon) * RADIUS;
}

double mx2lon (double mx) {
  return to_degrees (mx / RADIUS);
}

double mx2px (double mx, int zoom) {
  return (mx + get_x0 ())/ get_resolution (zoom);
}

double px2mx (double px, int zoom) {
  return px * get_resolution (zoom) - get_x0 ();
}

double my2py (double my, int zoom) {
  return (my + get_y0 ())/ get_resolution (zoom);
}

double py2my (double py, int zoom) {
  return py * get_resolution (zoom) - get_y0 ();
}

double lat2py (double lat, int zoom)
{
  return my2py (lat2my (lat), zoom);
}

double py2lat (double py, int zoom)
{
  return my2lat (py2my (py, zoom));
}

double lon2px (double lon, int zoom)
{
  return mx2px (lon2mx (lon), zoom);
}

double px2lon (double px, int zoom)
{ 
  return mx2lon (px2mx (px, zoom));
}

int lat2tiley (double lat, int z)
{
  double latrad = lat * M_PI/180.0;
  int n = pow (2, z);
	return (int)((1.0 - asinh(tan(latrad)) / M_PI) / 2.0 * n);
}

int lon2tilex (double lon, int z)
{
  int n = pow (2, z);
  return (int)((lon + 180.0) / 360.0 * n);
}

double tiley2lat (int y, int z)
{
  double n = M_PI - 2.0 * M_PI * y / (double)(1 << z);
	return 180.0 / M_PI * atan(0.5 * (exp(n) - exp(-n)));
}

double tilex2lon (int x, int z)
{
  return x / (double)(1 << z) * 360.0 - 180;
}
