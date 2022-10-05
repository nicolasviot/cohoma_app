use core
use gui
use display

_define_
Strip (string _name, Process frame, Process satellite, Process _svg)
{
   
   Double battery_voltage(24)
   Int battery_percentage(75)
   Double altitude_msl(0)
   Double heading_rot(0)
   Bool emergency_stop(0)
   Bool failsafe(0)
   Int operation_mode(0)
   String name(_name)
   Int color (#0F0F0F)


   //link with model (vehicule component)
   satellite.battery_voltage => battery_voltage
   satellite.battery_percentage =:> battery_percentage
   satellite.altitude_msl =:> altitude_msl
   satellite.heading_rot =:> heading_rot
   satellite.emergency_stop =:> emergency_stop
   satellite.failsafe =:> failsafe
   satellite.operation_mode =:> operation_mode
   satellite.color =:> color
   

   DoubleFormatter b_volt (0, 1)
   battery_voltage =:> b_volt.input

   DoubleFormatter b_volt_per (0, 1)
   battery_percentage =:> b_volt_per.input

   DoubleFormatter alt_msl (0, 1)
   altitude_msl =:> alt_msl.input

   DoubleFormatter heading(0, 1)
   heading_rot =:> heading.input

   String emergency_stop_str("")
   emergency_stop ? "true" : "false" =:> emergency_stop_str

   String failsafe_str("")
   failsafe ? "true" : "false" =:> failsafe_str

   String status("")
   /* OPERATING_MODE_UNKNOWN = 0            # Default value
   OPERATING_MODE_MANUAL = 1             # Operated by security pilot
   OPERATING_MODE_TELEOPERATION = 2      # Operated by remote operator
   OPERATING_MODE_AUTONOMOUS = 3 
   */
   (operation_mode == 1 ) ? "Manual" : ((operation_mode == 2 )? "TeleOP" : ((operation_mode == 3 )? "Auto" : "???")) =:> status


   Translation t (0, 0)

   Double parent_tx(0)
   Double parent_ty(0)
   

   g << clone (_svg.Strip)

   heading.output + "°" =:> g.central.data.heading.heading_text.text
   alt_msl.output + "m" =:> g.central.data.altitude.altitude_text.text 
   b_volt_per.output + "%" =:> g.central.battery_gauge.battery_percentage.text
   battery_percentage * 46/100 =:> g.central.battery_gauge.battery_rect.width 
   b_volt.output + "V" =:> g.left.energy.gauge.energy_text.text
   battery_percentage * 60/100  =:> g.left.energy.gauge.energy_rect.width
   (_name=="DRONE")?"aérien":"terrestre" =:> g.type.text
   name =:> g.id.text
   status =:> g.left.status.mode.mode_text.text
   color =:> g.strip_color.fill.value

   //link status
   Spike data_in
   battery_voltage -> data_in
   heading_rot -> data_in
   //for debugging
   //g.id.press -> data_in

   //for status link element
   Int link_ON_color (#00E600)
   Int link_OFF_color (#FA3004) //#CDCCC6

   FSM link_status_FSM{
      State disconnected{
        link_OFF_color =: g.left.link.link_rect.fill.value
      }
      State connected{
        link_ON_color =: g.left.link.link_rect.fill.value
        Timer status_timer (4000) //wait 4seconds
        Clock timer (100)
        Incr ellapsedIncr (0)

        AssignmentSequence reset_color (1){
            0 =: ellapsedIncr.state
        }

        data_in -> reset_color
        data_in -> status_timer.reset
        |-> reset_color
        
        timer.tick -> ellapsedIncr
        227 - ((ellapsedIncr.state * 100) / 4000) *  23 =:> g.left.link.link_rect.fill.g
        106 + ((ellapsedIncr.state * 100) / 4000) *  100 =:> g.left.link.link_rect.fill.r
        68 + ((ellapsedIncr.state * 100) / 4000) *  130 =:> g.left.link.link_rect.fill.b
      }

      disconnected -> connected (data_in)
      connected -> disconnected (connected.status_timer.end) 
   }

   g.background.press -> satellite.startAnim
   g.background.release -> satellite.stopAnim

}

