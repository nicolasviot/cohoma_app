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

double lat2my (double lat);
double my2lat (double my);
double lon2mx (double lon);
double mx2lon (double mx);
double mx2px (double mx, int zoom);
double px2mx (double px, int zoom);
double my2py (double my, int zoom);
double py2my (double y, int zoom);
double lat2py (double lat, int zoom);
double py2lat (double py, int zoom);
double lon2px (double lon, int zoom);
double px2lon (double px, int zoom);
int lat2tiley (double lat, int z);
double tiley2lat (int y, int z);
int lon2tilex (double lon, int z);
double tilex2lon (int x, int z);
double get_resolution (int zoom);
double get_x0 ();
double get_y0 ();
