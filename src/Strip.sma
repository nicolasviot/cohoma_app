use core
use gui
use display

_define_
Strip (string name, Process frame){

   Translation t (0, - 100)
   Double battery_voltage (0)
   DoubleFormatter b_volt (0, 1)
   battery_voltage =:> b_volt.input
   Double altitude_msl(0)
   DoubleFormatter alt_msl (0, 1)
   altitude_msl =:> alt_msl.input

   svg = loadFromXML ("res/svg/strip.svg")
   g << svg.Strip
   g.l1.l2.Aircraft_x5F_name_16_.text = name

   b_volt.output =:> this.g.l1.l2.Left_x5F_part_9_.Energy_9_.widget_9_.value_15_.text
  /* battery_voltage -> (this){
      this.g.l1.l2.Left_x5F_part_9_.Energy_9_.widget_9_.value_15_.text = this.battery_voltage
   }
*/
   alt_msl.output =:> this.g.l1.l2.Central_x5F_part_9_.fly_data_9_.Altitude_9_.textfield_37_.data_46_.text
   FSM drag {
      State idle{

      }
      State dragging{
         frame.move.x - 75 =:> t.tx
         frame.move.y - 182=:> t.ty
      }
      idle-> dragging (g.l1.l2.Aircraft_x5F_name_16_.press)
      dragging -> idle (g.l1.l2.Aircraft_x5F_name_16_.release)
   }
}

