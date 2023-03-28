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
   context aka _context

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

   print ("New view of Operator (" + _model.uid + ") type: " + _model.type + " code " + _model.code + " title: " + _model.title + " named " + _model.name + "\n")
   //print ("New view of Operator (" + _model.uid + ") type: " + _model.type + " code" + _model.code + " title: " + _model.title + " named " + _model.name + " with " +  _model.robots.size + " robots\n")


   Component strips{
      Translation _ (0, $_context.OPERATOR_HEADER_HEIGHT + 5)

      int i = 0
      for robot : _model.robots {
         print (_model.title + " has robot " + robot.title + "\n")
         setRef (robot.ref_operator, model)
         Strip strip (_context, robot, i)
         height + 5 + _context.VEHICLE_STRIP_HEIGHT =: height
         i = i + 1
      }
   }

    NativeCollectionAction nca_robots (collection_action_robots, _model.robots, 1)
   _model.robots -> nca_robots
   

   Bool accept_drop (0)
   Spike drop_trigger
   
   FSM drop_zone{
      State hide
      State show_drop{
         OutlineWidth _ (1)
         OutlineColor black (#00FF00)
         FillOpacity _ (0.1)
         FillColor _ (#00FF00)
         Rectangle zone (0, 0, $_context.LEFT_PANEL_WIDTH, 0, 5, 5)
         height =:> zone.height
      }
      hide -> show_drop (accept_drop.true)
      show_drop -> hide (accept_drop.false)
   }
   drop_zone.show_drop.zone.release -> drop_trigger

   //formulaire confirmation de changement opérateur
   form_svg = load_from_XML("res/svg/allocation-robot-confirmation.svg")
   //Confirmation du changement
   FSM show_confirmation{
         State idle{}
         State show{
            form << form_svg.form
            model.title =: form.op_name.text
         }
         idle -> show (drop_trigger)
         show-> idle (show.form.btn_oui.press)
         show -> idle (show.form.btn_non.press)
   }

   show_confirmation.show.form.btn_oui.press -> na_transfer_robot : (this){
       _ref_vehicle_model = getRef (this.context.model_of_dragged_strip)
      
        //Ca marche vraiment pas comme je crois...
              //Process _prev_operator = _ref_vehicle_model.ref_operator
              //Process _new_operator = this.model
               print ("je confirme que le robot " + _ref_vehicle_model.code + " soit alloué à l'opérateur " + this.model.title + "\n")
               //TODO : faire les changements sur les modèles..
               //trouver infos sur ProcessCollection
   }


   //attention obligé de stocker contexte comme un enfant... sinon marche pas.
   drop_trigger -> na_find_dropped_strip_model:(this) {
      _ref_vehicle_model = getRef (this.context.model_of_dragged_strip)
      if (&_ref_vehicle_model != null) {
         print ("j'ai droppé le strip " + _ref_vehicle_model.code + " sur l'opérateur " + this.model.title + "\n")
         //set form values
         //TODO ca marche pas comme ça on dirait...
        this.show_confirmation.show.form.robot_name.text = _ref_vehicle_model.code
      }
   }
}

