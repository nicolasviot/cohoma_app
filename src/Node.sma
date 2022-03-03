use core
use gui
use base
import Waypoints

_define_
Node(Process map, double _lat, double _lon, double _alt, int _isPPO, string _label, int _id, Process _manager_id){


Double lat(_lat)
Double lon(_lon)
Double alt(_alt)
Bool isPPO(_isPPO)
String label(_label)
Int id(_id)
Spike pressed

manager aka _manager_id
Waypoints wpt(map, $lat, $lon, 140, 140, 140)

lat =:> wpt.lat
lon =:> wpt.lon

AssignmentSequence export_id (1){
	id =: manager.selected_id
} 
wpt.c.press -> export_id


}