use core
use gui
use base

_native_code_
%{
    #include <iostream>
%}


_define_
PointModel (double _lat, double _lon, double _alt)
{
	Double lat (_lat)
	Double lon (_lon)
	Double alt (_alt)

	//print ("Model of point: lat = " + lat + " -- lon = " + lon + "\n")

	//Double dx_in_map (0)
	//Double dy_in_map (0)
}