use core
use gui
use base

_native_code_
%{
    #include <iostream>
%}


_define_
ChatMessageModel (string _sender, int _text, string _type, double _lat, double _lon, double _alt)
{
	Int sender (_sender)
	/*
	uint8 SENDER_UNKNOWN = 0
	uint8 SENDER_OT = 1
	uint8 SENDER_OG1 = 2
	uint8 SENDER_OG2 = 3
	uint8 SENDER_OG3 = 4
	*/
	String text (_text)
	
	String type (_type)
	/*
	uint8 TYPE_UNKNOWN = 0
	uint8 TYPE_MESSAGE = 1
	uint8 TYPE_TRAP = 2
	uint8 TYPE_LOCATION = 3
	*/
	
	Double lat (_lat)
	Double lon (_lon)
	Double alt (_alt)

	print ("Model of chat message sender (" + _sender + ") '" + _text + "\n")

	Double dx_in_map (0)
	Double dy_in_map (0)
}