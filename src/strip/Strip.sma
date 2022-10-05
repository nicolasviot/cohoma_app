use core
use gui
use display

_define_
Strip (Process _model, Process _svg)
{
   model aka _model

   //link with model   
   DoubleFormatter b_volt (0, 1)
   model.battery_voltage =:> b_volt.input

   DoubleFormatter b_volt_per (0, 1)
   model.battery_percentage =:> b_volt_per.input

   DoubleFormatter alt_msl (0, 1)
   model.altitude_msl =:> alt_msl.input

   DoubleFormatter heading(0, 1)
   model.heading_rot =:> heading.input

   String emergency_stop_str("")
   model.emergency_stop ? "true" : "false" =:> emergency_stop_str

   String failsafe_str("")
   model.failsafe ? "true" : "false" =:> failsafe_str


   Translation t (0, 0)

   Double parent_tx(0)
   Double parent_ty(0)
   

   g << clone (_svg.Strip)

   heading.output + "°" =:> g.central.data.heading.heading_text.text
   alt_msl.output + "m" =:> g.central.data.altitude.altitude_text.text 
   b_volt_per.output + "%" =:> g.central.battery_gauge.battery_percentage.text
   model.battery_percentage * 46/100 =:> g.central.battery_gauge.battery_rect.width 
   b_volt.output + "V" =:> g.left.energy.gauge.energy_text.text
   model.battery_percentage * 60/100  =:> g.left.energy.gauge.energy_rect.width
   (model.type == "drone") ? "aérien" : "terrestre" =:> g.type.text
   model.name =:> g.id.text
   model.status =:> g.left.status.mode.mode_text.text
   model.color =:> g.strip_color.fill.value

   //link status
   Spike data_in
   model.battery_voltage -> data_in
   model.heading_rot -> data_in
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

   // FIXME
   g.background.press -> model.start_locate
   g.background.release -> model.stop_locate

}

