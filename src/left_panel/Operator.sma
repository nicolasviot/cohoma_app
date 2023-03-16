use core
use gui
use display

/*_native_code_
%{
   #include <iostream>
%}*/


_define_
//Operator (Process _context, Process _model, int _index, Process _svg)
Operator (Process _context, Process _model)
{
   //context aka _context
   model aka _model

   Double height (0)

   FillColor _ (#0000FF)
   Rectangle fake (0, 120, 100, 100, 0, 0)

   //Translation tr (_index * $_context.STRIP_WIDTH, 0)

   //g << clone (_svg.Strip)

   //heading.output + "°" =:> g.central.data.heading.heading_text.text
   //alt_msl.output + "m" =:> g.central.data.altitude.altitude_text.text 
   //(model.type == "drone") ? "aérien" : "terrestre" =:> g.type.text
   //model.name =:> g.id.text
   //model.status =:> g.left.status.mode.mode_text.text
   //model.color =:> g.strip_color.fill.value

   //g.background.press -> model.start_locate
   //g.background.release -> model.stop_locate

}

