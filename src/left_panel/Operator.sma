use core
use base
use gui
use display

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

   Double height ($_context.OPERATOR_HEADER_HEIGHT + 10)

   Translation tr (0, 0)
   y aka tr.ty

   NoFill _
   Rectangle bg (0, 0, $_context.LEFT_PANEL_WIDTH, 0, 0, 0)
   height =:> bg.height

   Component header {
      NoOutline _

      FillColor _ ($_model.color)
      Rectangle r1 (0, 0, 147, $_context.OPERATOR_HEADER_HEIGHT, 0, 0)

      FillColor _ (#AAAAAA)
      Rectangle r2 (147, 0, $_context.LEFT_PANEL_WIDTH - 147, $_context.OPERATOR_HEADER_HEIGHT, 0, 0)

      //FontWeight _ (DJN_BOLD)
      //FontWeight _ (DJN_NORMAL)
      FontSize _ (5, 18) // 5 = pixel
      
      FillColor white (#FFFFFF)
      Text txt_title (5, 25, toString(_model.title))

      FillColor black (#000000)
      Text txt_name (152, 25, toString(_model.name))
   }

   print ("New view of Operator (" + _model.uid + ") type: " + _model.code + " (" + _model.type + ") title: " + _model.title + " named " + _model.name + "\n")
   //print ("New view of Operator (" + _model.uid + ") type: " + _model.code + " (" + _model.type + ") title: " + _model.title + " named " + _model.name + " with " +  _model.robots.size + " robots\n")


   FontSize _ (5, 18) // 5 = pixel
   FillColor _ (#000000)

   // FIXME: doesn't work
   //int i = 0
   /*for robot : _model.robots {
      height + 20 =: height
      
      //Text txt_robot (5, 25 + i * 20, toString(robot.title))
      Text txt_robot (5, 25 + 20, toString(robot.title))
      //i += 1
   }*/

   NativeCollectionAction nca_robots (collection_action_robots, _model.robots, 1)
   _model.robots -> nca_robots


   //g << clone (_svg.Strip)

   //heading.output + "°" =:> g.central.data.heading.heading_text.text
   //alt_msl.output + "m" =:> g.central.data.altitude.altitude_text.text 
   //model.name =:> g.id.text
   //model.status =:> g.left.status.mode.mode_text.text
   //model.color =:> g.strip_color.fill.value

   //g.background.press -> model.start_locate
   //g.background.release -> model.stop_locate

}

