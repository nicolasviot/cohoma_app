use core
use base
use gui
use display

import Strip

_native_code_
%{
   #include <iostream>
   using namespace std;

   #include "core/control/native_action.h" // FIXME: error: use of undeclared identifier 'get_native_user_data'
%}


_action_
collection_action_robots (list l, Process c)
%{
   Process *self = (Process*) get_native_user_data(c);

   for (auto robot : l) {
      //double w = ((AbstractProperty*)e->find_child("width"))->get_double_value ();
      //((DoubleProperty*)e->find_child("width"))->set_value (w + 5, true);

      string title = static_cast<TextProperty*>(self->find_child("model/title"))->get_value();
      string robot_title = static_cast<TextProperty*>(robot->find_child("title"))->get_value();
      cout << "Operator '" << title << "': robot = " << title << endl;
   }
%}


_define_
//Operator (Process _context, Process _model, Process _svg)
Operator (Process _context, Process _model)
{
   //context aka _context
   model aka _model

   Double height ($_context.OPERATOR_HEADER_HEIGHT + 5)

   Translation tr (0, 0)
   y aka tr.ty

   Component bg {
      OutlineWidth _ (1)
      OutlineColor black (#000000)
      NoFill _
      Rectangle bg (0, 0, $_context.LEFT_PANEL_WIDTH, 0, 5, 5)
      height =:> bg.height
   }

   Component header {
      NoOutline _

      FillColor _ ($_model.color)
      Rectangle r1 (1, 1, 147, $_context.OPERATOR_HEADER_HEIGHT, 0, 0)

      FillColor _ (#AAAAAA)
      Rectangle r2 (148, 1, $_context.LEFT_PANEL_WIDTH - 152, $_context.OPERATOR_HEADER_HEIGHT, 0, 0)

      //FontWeight _ (DJN_BOLD)
      //FontWeight _ (DJN_NORMAL)
      FontSize _ (5, 15) // 5 = pixel
      
      FillColor white (#FFFFFF)
      Text txt_title (5, 18, toString(_model.title))

      FillColor black (#000000)
      Text txt_name (152, 18, toString(_model.name))
   }

   //print ("New view of Operator (" + _model.uid + ") type: " + _model.code + " (" + _model.type + ") title: " + _model.title + " named " + _model.name + "\n")
   //print ("New view of Operator (" + _model.uid + ") type: " + _model.code + " (" + _model.type + ") title: " + _model.title + " named " + _model.name + " with " +  _model.robots.size + " robots\n")

   Translation _ (0, $_context.OPERATOR_HEADER_HEIGHT + 5)

   int i = 0
   for robot : _model.robots {
      print (_model.title + " has robot " + robot.title + "\n")
      //_model =: robot.ref_operator
      setRef (robot.ref_operator, model)

      Strip strip (_context, robot, i)

      
      height + 5 + _context.VEHICLE_STRIP_HEIGHT =: height
      
      i = i + 1
   }

   NativeCollectionAction nca_robots (collection_action_robots, _model.robots, 1)
   _model.robots -> nca_robots

   Spike show_dropzone
   Spike hide_dropzone
   //dropzone
   
   FSM dropable{
      State idle
      State accept_drop{
         OutlineWidth _ (1)
         OutlineColor black (#00FF00)
         FillOpacity _ (0.1)
         FillColor _ (#00FF00)
         Rectangle bg (0, 0, $_context.LEFT_PANEL_WIDTH, 0, 5, 5)
         height =:> bg.height
      }
      idle -> accept_drop (show_dropzone)
      accept_drop -> idle (hide_dropzone)
   }

   //|-> show_dropzone
   //g << clone (_svg.Strip)

   //heading.output + "°" =:> g.central.data.heading.heading_text.text
   //alt_msl.output + "m" =:> g.central.data.altitude.altitude_text.text 
   //model.name =:> g.id.text
   //model.status =:> g.left.status.mode.mode_text.text
   //model.color =:> g.strip_color.fill.value

   //g.background.press -> model.start_locate
   //g.background.release -> model.stop_locate

}

