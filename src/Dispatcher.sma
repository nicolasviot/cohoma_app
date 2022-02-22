use core
use base
use display
use gui

/*
 	TextProperty _msg;
    IntProperty _robot_id;
    DoubleProperty _latitude;
    DoubleProperty _longitude;
    IntProperty _battery_percentage;
    DoubleProperty _battery_voltage;
    DoubleProperty _altitude_msl;
    DoubleProperty _compass_heading;
    BoolProperty _emergency_stop;
    BoolProperty _failsafe;
    IntProperty _operation_mode;
    */

_define_
Dispatcher (Process input, Process _list) {


Int _battery_percentage(0)
Double _battery_voltage(0)
Double _altitude_msl(0)
Double _compass_heading(0)
Bool _emergency_stop(0)
Bool _failsafe(0)
Int _operation_mode(0)

input.battery_percentage =:> _battery_percentage
input.battery_voltage =:> _battery_voltage
input.altitude_msl =:> _altitude_msl
input.compass_heading =:> _compass_heading
input.emergency_stop =:> _emergency_stop
input.failsafe =:> _failsafe
input.operation_mode =:> _operation_mode
LogPrinter lp("test compass")
msg aka input

input.compass_heading =:> lp.input


	my_list aka _list

	Int _robot_id(0)
	Double _latitude(0)
	Double _longitude(0)
	Double _compass_heading(0)
	input.robot_id  =:> _robot_id
	input.latitude =:> _latitude
	input.longitude =:> _longitude
	input.altitude_msl =:> _altitude_msl
	input.battery_voltage =:> _battery_voltage

//TODO garantir synchronicité
	_robot_id -> (this){
		this.my_list.[$this._robot_id].lat = this._latitude
		this.my_list.[$this._robot_id].lon	= this._longitude
		this.my_list.[$this._robot_id].altitude_msl = this._altitude_msl
		this.my_list.[$this._robot_id].battery_voltage = this._battery_voltage
		this.my_list.[$this._robot_id].heading_rot = this.msg.compass_heading
	}

}