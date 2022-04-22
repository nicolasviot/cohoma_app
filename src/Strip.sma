use core
use gui
use display

_define_
Strip (string _name, Process frame){

   
   Double battery_voltage(24)
   Int battery_percentage(75)
   Double altitude_msl(0)
   Double heading_rot(0)
   Bool emergency_stop(0)
   Bool failsafe(0)
   Int operation_mode(0)
   String name(_name)


   DoubleFormatter b_volt (0, 1)
   battery_voltage =:> b_volt.input
   DoubleFormatter b_volt_per (0, 1)
   battery_percentage =:> b_volt_per.input
   DoubleFormatter alt_msl (0, 1)
   altitude_msl =:> alt_msl.input
   DoubleFormatter heading(0, 1)
   heading_rot =:> heading.input
   String emergency_stop_str("")
   emergency_stop?"true":"false" =:> emergency_stop_str
   String failsafe_str("")
   failsafe?"true":"false" =:> failsafe_str
   DoubleFormatter operation_mode_format(0, 1)
   operation_mode =:> operation_mode_format.input
   Translation t (0, 0)

   Double parent_tx(0)
   Double parent_ty(0)
   

   svg = loadFromXML ("res/svg/stripV2.svg")
   g << svg.Strip

   heading.output + "°" =:> g.central.data.heading.heading_text.text
   alt_msl.output + "m" =:> g.central.data.altitude.altitude_text.text 
   b_volt_per.output + "%" =:> g.central.battery_gauge.battery_percentage.text
   battery_percentage * 46/100 =:> g.central.battery_gauge.battery_rect.width 
   b_volt.output + "V" =:> g.left.energy.gauge.energy_text.text
   battery_percentage * 60/100  =:> g.left.energy.gauge.energy_rect.width
   (_name=="DRONE")?"aérien":"terrestre" =:> g.type.text
   name =:> g.id.text

   FSM drag {
      State idle{

      }
      State dragging{
         frame.move.x - 20 - parent_tx =:> t.tx
         frame.move.y - 20 - parent_ty =:> t.ty
      }
      idle-> dragging (g.background.press)
      dragging -> idle (g.background.release)
   }
}

