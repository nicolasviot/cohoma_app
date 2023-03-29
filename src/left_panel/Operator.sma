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
   //print ("New view of Operator (" + _model.uid + ") type: " + _model.type + " code " + _model.code + " title: " + _model.title + " named " + _model.name + "\n")

   Component strips_container{
      Translation _ (0, $_context.OPERATOR_HEADER_HEIGHT + 5)
       int i = 0
      List strips {
        
         for robot : model.robots {
            //print (_model.title + " has robot " + robot.title + "\n")
            setRef (robot.ref_operator, model)
            Strip strip (_context, robot, i)
            i = i + 1
         }
      }
   }
   strips aka strips_container.strips
   
   $_context.OPERATOR_HEADER_HEIGHT + 5 + (strips.size) * (5 + _context.VEHICLE_STRIP_HEIGHT) =:> height
  
   //update height and strip position when model changes
   strips.size -> update_strip_index: (this){
      for (int i = 1; i <= this.strips.size; i++) {
         this.strips.[i].index = i-1
      }
   }

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
         10 =: zone.z

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

   AssignmentSequence set_operator_drop (1){
      this =: context.dropped_operator
   }

   drop_trigger -> set_operator_drop

}

