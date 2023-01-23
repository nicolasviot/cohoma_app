use core
use gui
use base

/*_native_code_
%{
    #include <iostream>
	using namespace std;
%}*/


_define_
SubLayerModel (string _name)
{
	String name (_name)
	Bool is_visible (true)

	print ("Model of (sub) layer '" + name + "'\n")
}