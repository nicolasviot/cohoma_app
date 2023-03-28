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

_define_
//Operator (Process _context, Process _model, Process _svg)
Operator (Process _context, Process _model)
{
   //context aka _context
   model aka _model
   context aka _context

   Double height ($_context.OPERATOR_HEADER_HEIGHT + 5)

   //compute height according to number of strips

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

      TextPrinter tp
   Double nb_robots (0)
   model.robots.add -> model_changed:(this){
      int i = 0
         for robot : this.model.robots {
            i=i+1
         }
      this.nb_robots = i
   }
   model.robots.rm -> model_changed

   "il y a " + nb_robots =:> tp.input





   Bool accept_drop (0)
   Spike drop_trigger

   FSM drop_zone{
      State hide
      State show_drop{
         OutlineWidth out_w (0.5)
         OutlineColor black (#00FF00)
         FillOpacity fill_op (0.1)
         FillColor _ (#00FF00)
         Rectangle zone (0, 0, $_context.LEFT_PANEL_WIDTH, 0, 5, 5)
         height =:> zone.height
         1000000 =: zone.z

         FSM hover_FSM{
            State idle{
               0.1 =: fill_op.a
               0.5 =: out_w.width
            }
            State hover{
               0.2 =: fill_op.a
               2 =: out_w.width
            }

            idle -> hover (zone.enter)
            hover -> idle (zone.leave)
            hover -> idle (accept_drop.false)
         }
      }
      hide -> show_drop (accept_drop.true)
      show_drop -> hide (accept_drop.false)
   }
   drop_zone.show_drop.zone.release -> drop_trigger

   ////////////////
   //formulaire confirmation de changement opérateur
   form_svg = load_from_XML("res/svg/allocation-robot-confirmation.svg")
   //Confirmation du changement
   FSM show_confirmation_form{
         State idle{}
         State show{
            form << form_svg.form
            model.title =: form.op_name.text
         }
         idle -> show (drop_trigger)
         show-> idle (show.form.btn_oui.press)
         show -> idle (show.form.btn_non.press)
   }

   //action lors du drop pour afficher la confirmation
   drop_trigger -> na_find_dropped_strip_model:(this) {
      _dropped_strip = getRef(this.context.dragged_strip)
      //_ref_vehicle_model = _dropped_strip.model
      if (&_dropped_strip != null) {
         this.show_confirmation_form.show.form.robot_name.text = toString(_dropped_strip.model.code)
      }
   }

   //action lorsque l'utilisateur choisit oui et confirme la nouvelle affectation.
   show_confirmation_form.show.form.btn_oui.press -> na_transfer_robot : (this){
         _dropped_strip = getRef(this.context.dragged_strip)

         //remove robot from previous operator processcollector
         _prev_operator = getRef(_dropped_strip.model.ref_operator)
         setRef(_prev_operator.robots.rm , _dropped_strip.model)
        
         //add the robot to the new operator and set ref to operator
         setRef(this.model.robots.add , _dropped_strip.model)
         setRef(_dropped_strip.model.ref_operator, this.model)

         //change the strip to the new parent
         addChildrenTo this.strips{
            _dropped_strip
         }

         print ("je confirme que le robot " + _dropped_strip.model.code + " est alloué à l'opérateur " + this.model.title + "\n")

    //     for ()
         //SEND MESSAGE WITH NEW ALLOCATION TO ROS TOPIC
   }
}

