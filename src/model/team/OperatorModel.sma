use core
use gui
use base

_native_code_
%{
    #include <iostream>

%}

_define_
OperatorModel (Process _context, int _uid, int _type, string _code, string _title, string _name, int _color)
{
    //context aka _context

    Int uid (_uid)
    Int type (_type)
    String code (_code)
    String title (_title)
    String name (_name)
    Int color (_color)

    List robots // = satellites / = vehicles

    print ("New model of Operator (" + uid + ") type: " + code + " (" + type + ") title: " + title + " named " + name + "\n")
}